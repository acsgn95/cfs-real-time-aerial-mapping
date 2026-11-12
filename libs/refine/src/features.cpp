// Feature front end (ADR-019 section 3, steps 1 and 2): SIFT, matching with
// the ratio test and an essential-matrix RANSAC, tracks.

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>

#include "orthostream/refine/refine.hpp"

namespace orthostream::refine
{
namespace
{

class SiftExtractor : public FeatureExtractor
{
public:
    // Precise upscaling: with the default upscaling of the first octave,
    // OpenCV's keypoints are shifted by +0.25 px in x and y. The shift is
    // the same in every image, but images of opposite flight lines are
    // turned by 180 degrees, so tie points between them disagree by 0.5 px
    // and the adjustment turns the cameras to compensate (found with the
    // rendered-plane test: 0.065 deg error without, see test_refine.cpp).
    explicit SiftExtractor(int max_features)
        : sift_(cv::SIFT::create(max_features, 3, 0.04, 10, 1.6, /*enable_precise_upscale=*/true))
    {
    }

    Features extract(const GreyImage &image) override
    {
        if (image.data == nullptr || image.width <= 0 || image.height <= 0)
        {
            throw RefineError("empty image");
        }
        const cv::Mat             img(image.height, image.width, CV_8UC1, const_cast<std::uint8_t *>(image.data),
                                      image.stride);
        std::vector<cv::KeyPoint> kps;
        cv::Mat                   desc;
        sift_->detectAndCompute(img, cv::noArray(), kps, desc);
        Features f;
        f.descriptor_size = desc.cols;
        f.keypoints.reserve(kps.size());
        f.descriptors.resize(kps.size() * static_cast<std::size_t>(desc.cols));
        for (std::size_t i = 0; i < kps.size(); ++i)
        {
            f.keypoints.emplace_back(kps[i].pt.x, kps[i].pt.y);
            const float *row = desc.ptr<float>(static_cast<int>(i));
            std::copy(row, row + desc.cols, f.descriptors.begin() + static_cast<std::ptrdiff_t>(i * desc.cols));
        }
        return f;
    }

    std::string name() const override { return "sift"; }

private:
    cv::Ptr<cv::SIFT> sift_;
};

cv::Mat descriptorMat(const Features &f)
{
    return cv::Mat(static_cast<int>(f.keypoints.size()), f.descriptor_size, CV_32F,
                   const_cast<float *>(f.descriptors.data()));
}

struct DisjointSet
{
    explicit DisjointSet(std::size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), std::size_t { 0 }); }
    std::size_t find(std::size_t x)
    {
        while (parent[x] != x)
        {
            parent[x] = parent[parent[x]];
            x         = parent[x];
        }
        return x;
    }
    void unite(std::size_t a, std::size_t b) { parent[find(a)] = find(b); }
    std::vector<std::size_t> parent;
};

}  // namespace

std::unique_ptr<FeatureExtractor> makeSiftExtractor(int max_features)
{
    return std::make_unique<SiftExtractor>(max_features);
}

PairMatches matchPair(const Features &a, const Features &b, const geo::CameraModel &camera,
                      const RefineConfig &config)
{
    PairMatches pm;
    if (a.keypoints.size() < 8 || b.keypoints.size() < 8 || a.descriptor_size != b.descriptor_size)
    {
        return pm;
    }
    cv::FlannBasedMatcher                matcher;
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(descriptorMat(a), descriptorMat(b), knn, 2);
    std::vector<cv::Point2d>                       pa, pb;
    std::vector<std::pair<std::size_t, std::size_t>> candidates;
    for (const auto &m : knn)
    {
        if (m.size() == 2 && m[0].distance < config.ratio_test * m[1].distance)
        {
            const std::size_t ia = static_cast<std::size_t>(m[0].queryIdx);
            const std::size_t ib = static_cast<std::size_t>(m[0].trainIdx);
            candidates.emplace_back(ia, ib);
            pa.emplace_back((a.keypoints[ia].x() - camera.cx) / camera.fx, (a.keypoints[ia].y() - camera.cy) / camera.fy);
            pb.emplace_back((b.keypoints[ib].x() - camera.cx) / camera.fx, (b.keypoints[ib].y() - camera.cy) / camera.fy);
        }
    }
    pm.candidates = candidates.size();
    if (candidates.size() < 8)
    {
        return pm;
    }
    cv::Mat mask;
    const cv::Mat E = cv::findEssentialMat(pa, pb, 1.0, cv::Point2d(0.0, 0.0), cv::RANSAC, 0.999,
                                          config.ransac_threshold_px / camera.fx, mask);
    if (E.empty())
    {
        return pm;
    }
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
        if (mask.at<std::uint8_t>(static_cast<int>(i)) != 0)
        {
            pm.inliers.push_back(candidates[i]);
        }
    }
    return pm;
}

std::vector<Track> buildTracks(const std::vector<std::vector<Eigen::Vector2d>> &keypoints,
                               const std::vector<PairMatches>                  &matches)
{
    std::vector<std::size_t> offset(keypoints.size() + 1, 0);
    for (std::size_t i = 0; i < keypoints.size(); ++i)
    {
        offset[i + 1] = offset[i] + keypoints[i].size();
    }
    DisjointSet       sets(offset.back());
    std::vector<bool> used(offset.back(), false);
    for (const PairMatches &pm : matches)
    {
        if (pm.image_a >= keypoints.size() || pm.image_b >= keypoints.size())
        {
            throw RefineError("match of an unknown image");
        }
        for (const auto &[ia, ib] : pm.inliers)
        {
            if (ia >= keypoints[pm.image_a].size() || ib >= keypoints[pm.image_b].size())
            {
                throw RefineError("match of an unknown keypoint");
            }
            const std::size_t na = offset[pm.image_a] + ia, nb = offset[pm.image_b] + ib;
            sets.unite(na, nb);
            used[na] = used[nb] = true;
        }
    }
    std::unordered_map<std::size_t, Track> groups;
    for (std::size_t image = 0; image < keypoints.size(); ++image)
    {
        for (std::size_t k = 0; k < keypoints[image].size(); ++k)
        {
            const std::size_t node = offset[image] + k;
            if (used[node])
            {
                groups[sets.find(node)].push_back({ image, keypoints[image][k] });
            }
        }
    }
    std::vector<Track> tracks;
    for (auto &[root, track] : groups)
    {
        (void)root;
        // Observations are in image order, so a repeated image is adjacent.
        bool consistent = track.size() >= 2;
        for (std::size_t i = 1; i < track.size() && consistent; ++i)
        {
            consistent = track[i].image != track[i - 1].image;
        }
        if (consistent)
        {
            tracks.push_back(std::move(track));
        }
    }
    // Deterministic order independent of the hash map.
    std::sort(tracks.begin(), tracks.end(), [](const Track &x, const Track &y) {
        if (x.front().image != y.front().image)
        {
            return x.front().image < y.front().image;
        }
        if (x.front().pixel.x() != y.front().pixel.x())
        {
            return x.front().pixel.x() < y.front().pixel.x();
        }
        return x.front().pixel.y() < y.front().pixel.y();
    });
    return tracks;
}

}  // namespace orthostream::refine
