#include <cmath>
#include <random>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "orthostream/refine/refine.hpp"

namespace
{

using namespace orthostream;
using refine::Observation;
using refine::Track;

constexpr double kDeg = M_PI / 180.0;

geo::CameraModel camera(int width, int height, double f)
{
    geo::CameraModel c;
    c.width  = width;
    c.height = height;
    c.fx = c.fy = f;
    c.cx        = 0.5 * (width - 1);
    c.cy        = 0.5 * (height - 1);
    return c;
}

/// Nadir camera over (x, y) at height h, flying along +x (forward) or -x.
Eigen::Isometry3d nadir(double x, double y, double h, bool forward)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const double      s = forward ? 1.0 : -1.0;
    T.linear() << s, 0, 0, 0, -s, 0, 0, 0, -1;
    T.translation() = Eigen::Vector3d(x, y, h);
    return T;
}

Eigen::Matrix3d rotation(const Eigen::Vector3d &axis_angle)
{
    const double a = axis_angle.norm();
    return a > 0 ? Eigen::AngleAxisd(a, axis_angle / a).toRotationMatrix() : Eigen::Matrix3d::Identity();
}

double angleDeg(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B)
{
    return std::acos(std::clamp(((A.transpose() * B).trace() - 1.0) / 2.0, -1.0, 1.0)) / kDeg;
}

/// Two flight lines in opposite directions, six images each.
std::vector<Eigen::Isometry3d> block(double h)
{
    std::vector<Eigen::Isometry3d> poses;
    for (int i = 0; i < 6; ++i)
    {
        poses.push_back(nadir(20.0 * i, 0.0, h, true));
    }
    for (int i = 5; i >= 0; --i)
    {
        poses.push_back(nadir(20.0 * i, 45.0, h, false));
    }
    return poses;
}

/// Priors: the true attitude with a common mounting error (boresight) and a
/// random error per image; positions optionally perturbed.
std::vector<Eigen::Isometry3d> perturbed(const std::vector<Eigen::Isometry3d> &truth, const Eigen::Matrix3d &B,
                                         double random_deg, double position_noise, std::mt19937 &rng)
{
    std::normal_distribution<double> n(0.0, 1.0);
    std::vector<Eigen::Isometry3d>   priors;
    for (const auto &T : truth)
    {
        Eigen::Isometry3d P = T;
        const Eigen::Vector3d e(n(rng), n(rng), n(rng));
        // truth = prior * B  ->  prior = truth * B^T, plus a random error.
        P.linear() = T.linear() * B.transpose() * rotation(e * random_deg * kDeg);
        P.translation() += position_noise * Eigen::Vector3d(n(rng), n(rng), n(rng));
        priors.push_back(P);
    }
    return priors;
}

std::vector<Track> observe(const geo::CameraModel &cam, const std::vector<Eigen::Isometry3d> &truth,
                           std::size_t points, double noise_px, std::mt19937 &rng)
{
    std::uniform_real_distribution<double> ux(-30.0, 130.0), uy(-30.0, 75.0);
    std::normal_distribution<double>       n(0.0, noise_px);
    std::vector<Track>                     tracks;
    for (std::size_t j = 0; j < points; ++j)
    {
        const double          x = ux(rng), y = uy(rng);
        const Eigen::Vector3d X(x, y, 6.0 * std::sin(x / 25.0) * std::cos(y / 30.0));
        Track                 t;
        for (std::size_t i = 0; i < truth.size(); ++i)
        {
            const Eigen::Vector3d p = truth[i].inverse() * X;
            const double          u = cam.fx * p.x() / p.z() + cam.cx + n(rng);
            const double          v = cam.fy * p.y() / p.z() + cam.cy + n(rng);
            if (p.z() > 0 && u >= 0 && v >= 0 && u <= cam.width - 1 && v <= cam.height - 1)
            {
                t.push_back({ i, { u, v } });
            }
        }
        if (t.size() >= 2)
        {
            tracks.push_back(t);
        }
    }
    return tracks;
}

TEST(Tracks, ChainsMatchesAndDropsInconsistentTracks)
{
    const std::vector<std::vector<Eigen::Vector2d>> kp { { { 0, 0 }, { 1, 1 } }, { { 2, 2 }, { 3, 3 } },
                                                         { { 4, 4 }, { 5, 5 } } };
    refine::PairMatches ab { 0, 1, 0, { { 0, 0 }, { 1, 1 } } };
    refine::PairMatches bc { 1, 2, 0, { { 0, 0 } } };
    // Keypoint 1 of image 0 also matches keypoint 0 of image 1, which is
    // already in another track: both tracks join and contain image 1 twice.
    refine::PairMatches conflict { 0, 1, 0, { { 1, 0 } } };

    const auto clean = refine::buildTracks(kp, { ab, bc });
    ASSERT_EQ(clean.size(), 2u);
    EXPECT_EQ(clean[0].size(), 3u);  // 0/0 - 1/0 - 2/0
    EXPECT_EQ(clean[1].size(), 2u);  // 0/1 - 1/1

    const auto with_conflict = refine::buildTracks(kp, { ab, bc, conflict });
    EXPECT_TRUE(with_conflict.empty());
}

TEST(Adjust, RecoversAttitudeWithPrecisePositions)
{
    std::mt19937           rng(7);
    const auto             cam   = camera(1600, 1067, 2000.0);
    const auto             truth = block(150.0);
    const Eigen::Matrix3d  B     = rotation(Eigen::Vector3d(0.6, -1.0, 0.4).normalized() * 1.3 * kDeg);
    const auto             prior = perturbed(truth, B, 0.4, 0.0, rng);
    const auto             tracks = observe(cam, truth, 3000, 0.3, rng);

    refine::RefineConfig c;
    c.precise_position = true;
    const auto r       = refine::adjust(cam, prior, tracks, c);

    EXPECT_GT(r.rms_before_px, 10.0);
    EXPECT_LT(r.rms_after_px, 0.5);
    double worst_before = 0.0, worst_after = 0.0;
    for (std::size_t i = 0; i < truth.size(); ++i)
    {
        worst_before = std::max(worst_before, angleDeg(prior[i].linear(), truth[i].linear()));
        worst_after  = std::max(worst_after, angleDeg(r.T_W_C[i].linear(), truth[i].linear()));
        EXPECT_LT((r.T_W_C[i].translation() - truth[i].translation()).norm(), 0.05);
    }
    EXPECT_GT(worst_before, 1.0);
    EXPECT_LT(worst_after, 0.02);
    // The shared rotation takes the mounting error and the mean of the
    // random errors (0.4 deg / sqrt(12) per axis, about 0.2 deg in all).
    EXPECT_LT(angleDeg(r.boresight, B), 0.5);
}

TEST(Adjust, MakesPosesConsistentWithNoisyPositions)
{
    std::mt19937          rng(11);
    const auto            cam    = camera(1600, 1067, 2000.0);
    const auto            truth  = block(150.0);
    const Eigen::Matrix3d B      = rotation(Eigen::Vector3d(1.0, 0.0, 0.0) * 1.0 * kDeg);
    const auto            prior  = perturbed(truth, B, 0.5, 0.5, rng);
    const auto            tracks = observe(cam, truth, 3000, 0.3, rng);

    refine::RefineConfig c;
    c.precise_position = false;
    c.position_sigma   = 0.5;
    const auto r       = refine::adjust(cam, prior, tracks, c);
    EXPECT_GT(r.rms_before_px, 10.0);
    EXPECT_LT(r.rms_after_px, 0.5);
}

TEST(Adjust, RejectsDistortedCamera)
{
    auto cam = camera(100, 100, 100.0);
    cam.k1   = 0.1;
    EXPECT_THROW(refine::adjust(cam, { Eigen::Isometry3d::Identity() }, {}, {}), refine::RefineError);
}

/// Renders a textured ground plane (z = 0) seen by a camera.
cv::Mat render(const cv::Mat &texture, double metres_per_texel, const geo::CameraModel &cam,
               const Eigen::Isometry3d &T_W_C)
{
    // Pixel = K [r1 r2 t] (X, Y, 1) for plane points; texture texel (s, t)
    // is the plane point (s, t) * metres_per_texel.
    const Eigen::Isometry3d T_C_W = T_W_C.inverse();
    Eigen::Matrix3d         K;
    K << cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1;
    Eigen::Matrix3d M;
    M.col(0) = T_C_W.linear().col(0);
    M.col(1) = T_C_W.linear().col(1);
    M.col(2) = T_C_W.translation();
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    S(0, 0) = S(1, 1) = metres_per_texel;
    const Eigen::Matrix3d H = K * M * S;
    cv::Mat               Hm(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            Hm.at<double>(r, c) = H(r, c);
        }
    }
    cv::Mat out;
    cv::warpPerspective(texture, out, Hm, cv::Size(cam.width, cam.height), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                        cv::Scalar(0));
    return out;
}

TEST(Refiner, RefinesRenderedImagesOfAPlane)
{
    // Texture with structure at several scales over 200 m x 120 m.
    std::mt19937 rng(3);
    cv::Mat      texture(1200, 2000, CV_32F, cv::Scalar(0));
    for (int scale : { 3, 9, 27 })
    {
        cv::Mat noise(texture.size(), CV_32F);
        cv::randn(noise, 0.0, 1.0);
        cv::GaussianBlur(noise, noise, cv::Size(0, 0), scale);
        cv::normalize(noise, noise, 0.0, 1.0, cv::NORM_MINMAX);
        texture += noise;
    }
    cv::normalize(texture, texture, 0, 255, cv::NORM_MINMAX);
    texture.convertTo(texture, CV_8U);
    const double metres_per_texel = 0.1;  // texture origin at the plane origin

    const auto cam = camera(1024, 683, 900.0);
    std::vector<Eigen::Isometry3d> truth;
    for (int i = 0; i < 4; ++i)
    {
        truth.push_back(nadir(60.0 + 15.0 * i, 45.0, 100.0, true));
    }
    for (int i = 3; i >= 0; --i)
    {
        truth.push_back(nadir(60.0 + 15.0 * i, 75.0, 100.0, false));
    }
    const Eigen::Matrix3d B     = rotation(Eigen::Vector3d(0.0, 1.0, 0.5).normalized() * 1.2 * kDeg);
    const auto            prior = perturbed(truth, B, 0.3, 0.0, rng);

    refine::RefineConfig c;
    c.precise_position       = true;
    c.min_image_observations = 30;
    refine::Refiner      refiner(cam, c);
    std::vector<cv::Mat> images;
    for (std::size_t i = 0; i < truth.size(); ++i)
    {
        images.push_back(render(texture, metres_per_texel, cam, truth[i]));
        refiner.addImage({ images.back().data, cam.width, cam.height, images.back().step }, prior[i]);
        EXPECT_GT(refiner.features(i).keypoints.size(), 500u);
    }
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    for (std::size_t a = 0; a < truth.size(); ++a)
    {
        for (std::size_t b = a + 1; b < truth.size(); ++b)
        {
            pairs.emplace_back(a, b);
        }
    }
    refiner.addPairs(pairs);
    const refine::RefineResult r = refiner.solve();
    ASSERT_EQ(r.source, refine::PoseSource::refined) << r.reason;
    EXPECT_LT(r.adjustment.rms_after_px, 1.0);
    for (std::size_t i = 0; i < truth.size(); ++i)
    {
        // 0.065 deg with OpenCV's default SIFT upscaling (0.25 px keypoint
        // shift, see features.cpp); about 0.002 deg with precise upscaling.
        EXPECT_LT(angleDeg(r.T_W_C[i].linear(), truth[i].linear()), 0.01) << "image " << i;
    }
}

}  // namespace
