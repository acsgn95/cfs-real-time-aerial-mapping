// One refinement window (ADR-019 section 3): features, matching, tracks,
// adjustment and the checks of section 5.

#include <chrono>
#include <cmath>
#include <sstream>

#include <opencv2/core.hpp>

#include "orthostream/refine/refine.hpp"

namespace orthostream::refine
{
namespace
{

using Clock = std::chrono::steady_clock;

double since(Clock::time_point t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

}  // namespace

struct Refiner::Impl
{
    geo::CameraModel                  camera;
    RefineConfig                      config;
    std::unique_ptr<FeatureExtractor> extractor;
    std::vector<Features>             features;
    std::vector<Eigen::Isometry3d>    priors;
    std::vector<PairMatches>          matches;
    double                            features_s = 0.0;
    double                            matching_s = 0.0;
};

Refiner::Refiner(const geo::CameraModel &camera, const RefineConfig &config,
                 std::unique_ptr<FeatureExtractor> extractor)
    : impl_(std::make_unique<Impl>())
{
    impl_->camera    = camera;
    impl_->config    = config;
    impl_->extractor = extractor ? std::move(extractor) : makeSiftExtractor(config.max_features);
}

Refiner::~Refiner() = default;

std::size_t Refiner::addImage(const GreyImage &image, const Eigen::Isometry3d &T_W_C_prior)
{
    if (image.width != impl_->camera.width || image.height != impl_->camera.height)
    {
        throw RefineError("image size differs from the camera");
    }
    const auto t0 = Clock::now();
    impl_->features.push_back(impl_->extractor->extract(image));
    impl_->priors.push_back(T_W_C_prior);
    impl_->features_s += since(t0);
    return impl_->features.size() - 1;
}

void Refiner::addPairs(const std::vector<std::pair<std::size_t, std::size_t>> &pairs)
{
    const auto t0 = Clock::now();
    for (const auto &[a, b] : pairs)
    {
        if (a >= impl_->features.size() || b >= impl_->features.size() || a == b)
        {
            throw RefineError("invalid image pair");
        }
    }
    std::vector<PairMatches> found(pairs.size());
    cv::parallel_for_(cv::Range(0, static_cast<int>(pairs.size())), [&](const cv::Range &r) {
        for (int p = r.start; p < r.end; ++p)
        {
            const auto [a, b] = pairs[static_cast<std::size_t>(p)];
            PairMatches pm    = matchPair(impl_->features[a], impl_->features[b], impl_->camera, impl_->config);
            pm.image_a        = a;
            pm.image_b        = b;
            found[static_cast<std::size_t>(p)] = std::move(pm);
        }
    });
    for (PairMatches &pm : found)
    {
        impl_->matches.push_back(std::move(pm));
    }
    impl_->matching_s += since(t0);
}

std::size_t Refiner::images() const
{
    return impl_->features.size();
}

const Features &Refiner::features(std::size_t image) const
{
    return impl_->features.at(image);
}

RefineResult Refiner::solve()
{
    RefineResult r;
    r.T_W_C       = impl_->priors;
    r.features_s  = impl_->features_s;
    r.matching_s  = impl_->matching_s;
    r.pairs_tried = impl_->matches.size();

    std::vector<PairMatches> used;
    for (const PairMatches &pm : impl_->matches)
    {
        if (pm.inliers.size() >= static_cast<std::size_t>(impl_->config.min_pair_inliers))
        {
            used.push_back(pm);
        }
    }
    r.pairs_used = used.size();
    std::vector<std::vector<Eigen::Vector2d>> keypoints;
    for (const Features &f : impl_->features)
    {
        keypoints.push_back(f.keypoints);
    }
    const std::vector<Track> tracks = buildTracks(keypoints, used);
    r.tracks                        = tracks.size();

    const auto t0 = Clock::now();
    try
    {
        r.adjustment = adjust(impl_->camera, impl_->priors, tracks, impl_->config);
    }
    catch (const RefineError &e)
    {
        r.adjustment_s = since(t0);
        r.reason       = e.what();
        return r;
    }
    r.adjustment_s = since(t0);

    // Checks (ADR-019 section 5).
    const RefineConfig     &c = impl_->config;
    const AdjustmentResult &a = r.adjustment;
    std::ostringstream      why;
    std::size_t             weak = 0;
    for (std::size_t n : a.observations_per_image)
    {
        weak += n < c.min_image_observations ? 1 : 0;
    }
    const double position_sigma = c.precise_position ? c.precise_position_sigma : c.position_sigma;
    if (weak > 0)
    {
        why << weak << " image(s) with fewer than " << c.min_image_observations << " observations";
    }
    else if (a.rms_after_px > c.max_reprojection_rms_px)
    {
        why << "reprojection RMS " << a.rms_after_px << " px";
    }
    else if (a.max_rotation_correction_deg > c.max_correction_sigmas * c.attitude_sigma_deg)
    {
        why << "attitude correction " << a.max_rotation_correction_deg << " deg";
    }
    else if (a.max_position_correction_m > c.max_correction_sigmas * position_sigma)
    {
        why << "position correction " << a.max_position_correction_m << " m";
    }
    r.reason = why.str();
    if (r.reason.empty())
    {
        r.source = PoseSource::refined;
        r.T_W_C  = a.T_W_C;
    }
    return r;
}

}  // namespace orthostream::refine
