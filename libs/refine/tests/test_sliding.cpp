// Sliding-window refinement and its C interface on rendered images of a
// textured plane: two flight lines in opposite directions, batches arriving
// one by one as from the selection.

#include <cmath>
#include <random>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "orthostream/geo/frames.hpp"
#include "orthostream/refine/sliding.hpp"
#include "refine_c_api.h"

extern "C" int ost_refine_c_api_usage(void);

namespace
{

using namespace orthostream;
constexpr double kDeg = M_PI / 180.0;

geo::CameraModel camera()
{
    geo::CameraModel c;
    c.width  = 1024;
    c.height = 683;
    c.fx = c.fy = 900.0;
    c.cx        = 0.5 * (c.width - 1);
    c.cy        = 0.5 * (c.height - 1);
    return c;
}

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

cv::Mat texture()
{
    std::mt19937 rng(3);
    cv::Mat      t(1500, 2600, CV_32F, cv::Scalar(0));
    cv::theRNG().state = 12345;
    for (int scale : { 3, 9, 27 })
    {
        cv::Mat noise(t.size(), CV_32F);
        cv::randn(noise, 0.0, 1.0);
        cv::GaussianBlur(noise, noise, cv::Size(0, 0), scale);
        cv::normalize(noise, noise, 0.0, 1.0, cv::NORM_MINMAX);
        t += noise;
    }
    cv::normalize(t, t, 0, 255, cv::NORM_MINMAX);
    t.convertTo(t, CV_8U);
    return t;
}

/// The textured plane z = 0 (0.1 m per texel) seen by a camera.
cv::Mat render(const cv::Mat &tex, const geo::CameraModel &cam, const Eigen::Isometry3d &T_W_C)
{
    const Eigen::Isometry3d T_C_W = T_W_C.inverse();
    Eigen::Matrix3d         K;
    K << cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1;
    Eigen::Matrix3d M;
    M.col(0) = T_C_W.linear().col(0);
    M.col(1) = T_C_W.linear().col(1);
    M.col(2) = T_C_W.translation();
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    S(0, 0) = S(1, 1) = 0.1;
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
    cv::warpPerspective(tex, out, Hm, cv::Size(cam.width, cam.height), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                        cv::Scalar(0));
    return out;
}

/// Line 1 along +x at y = 50, then line 2 back along -x at y = 80; 1.5 s
/// between images, 20 s for the turn.
struct Flight
{
    std::vector<Eigen::Isometry3d> truth, prior;
    std::vector<std::int64_t>      time_ns;
    std::vector<cv::Mat>           images;
};

Flight makeFlight(const geo::CameraModel &cam, const Eigen::Matrix3d &B)
{
    const cv::Mat tex = texture();
    Flight        f;
    std::mt19937  rng(5);
    std::normal_distribution<double> n(0.0, 1.0);
    std::int64_t  t = 0;
    for (int i = 0; i < 8; ++i, t += 1500000000)
    {
        f.truth.push_back(nadir(60.0 + 18.0 * i, 50.0, 100.0, true));
        f.time_ns.push_back(t);
    }
    t += 20000000000;
    for (int i = 7; i >= 0; --i, t += 1500000000)
    {
        f.truth.push_back(nadir(60.0 + 18.0 * i, 80.0, 100.0, false));
        f.time_ns.push_back(t);
    }
    for (const auto &T : f.truth)
    {
        Eigen::Isometry3d P = T;
        P.linear()          = T.linear() * B.transpose() * rotation(Eigen::Vector3d(n(rng), n(rng), n(rng)) * 0.3 * kDeg);
        f.prior.push_back(P);
        f.images.push_back(render(tex, cam, T));
    }
    return f;
}

refine::SlidingConfig config()
{
    refine::SlidingConfig c;
    c.refine.precise_position       = true;
    c.refine.min_image_observations = 30;
    c.max_window_images             = 10;
    c.max_points                    = 2000;
    c.boresight_settle_batches      = 3;
    c.max_wait_s                    = 60.0;
    return c;
}

}  // namespace

TEST(Sliding, RefinesBatchesAsTheyArrive)
{
    const auto            cam = camera();
    const Eigen::Matrix3d B   = rotation(Eigen::Vector3d(0.0, 1.0, 0.5).normalized() * 1.2 * kDeg);
    const Flight          f   = makeFlight(cam, B);
    refine::SlidingRefiner s(cam, config());

    // Batch k: keyframe k with its neighbours on the same line, added when
    // the next image has arrived (delayed source views).
    std::vector<refine::SlidingResult> results;
    for (std::size_t i = 0; i < f.truth.size(); ++i)
    {
        refine::SlidingImage im;
        im.id      = 100 + i;
        im.time_ns = f.time_ns[i];
        im.image   = { f.images[i].data, cam.width, cam.height, f.images[i].step };
        im.T_W_C   = f.prior[i];
        s.addImage(im);
        EXPECT_TRUE(s.hasImage(100 + i));
        if (i >= 2 && i != 8)
        {
            const std::size_t k = i - 1;  // keyframe with neighbours k-1 and k+1
            if (k % 8 != 0)
            {
                refine::SlidingBatch b;
                b.id       = k;
                b.keyframe = 100 + k;
                b.images   = { 100 + k, 100 + k - 1, 100 + k + 1 };
                b.ground_z = 0.0;
                s.addBatch(b);
            }
        }
        for (auto &r : s.poll(f.time_ns[i]))
        {
            results.push_back(std::move(r));
        }
    }
    for (auto &r : s.poll(f.time_ns.back(), true))
    {
        results.push_back(std::move(r));
    }

    ASSERT_EQ(results.size(), 12u);  // keyframes 1-6 and 9-14
    for (const auto &r : results)
    {
        ASSERT_EQ(r.source, refine::PoseSource::refined) << "batch " << r.batch_id << ": " << r.reason;
        const std::size_t k = r.batch_id;
        EXPECT_LT(angleDeg(r.T_W_C[0].linear(), f.truth[k].linear()), 0.02) << "batch " << k;
        if (k < 8)
        {
            // Line 1 waits for line 2; the last batches of line 2 are flushed.
            EXPECT_TRUE(r.covered) << "batch " << k;
            EXPECT_GT(r.waited_s, 15.0);
        }
    }
    EXPECT_TRUE(results.back().boresight_held);
    const auto st = s.stats();
    EXPECT_EQ(st.refined_batches, 12u);
    EXPECT_EQ(st.pending_batches, 0u);
}

TEST(Sliding, RejectsInvalidInput)
{
    const auto             cam = camera();
    refine::SlidingRefiner s(cam, config());
    std::vector<std::uint8_t> small(10 * 10, 0);
    refine::SlidingImage   im;
    im.image = { small.data(), 10, 10, 10 };
    EXPECT_THROW(s.addImage(im), refine::RefineError);
    refine::SlidingBatch b;
    b.keyframe = 1;
    b.images   = { 1 };
    EXPECT_THROW(s.addBatch(b), refine::RefineError);  // unknown image
}

TEST(RefineCApi, SlidingWindowThroughTheCInterface)
{
    EXPECT_EQ(ost_refine_c_api_usage(), 0);

    // The same flight, placed in ECEF with a local frame at 46.8 N, 9.8 E.
    const auto            cam = camera();
    const Eigen::Matrix3d B   = rotation(Eigen::Vector3d(1.0, 0.0, 0.0) * 1.0 * kDeg);
    const Flight          f   = makeFlight(cam, B);
    const double          lat = 46.8 * kDeg, lon = 9.8 * kDeg, h0 = 1500.0;
    const double          a = 6378137.0, e2 = (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563);
    const double          N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    const Eigen::Vector3d origin((N + h0) * std::cos(lat) * std::cos(lon), (N + h0) * std::cos(lat) * std::sin(lon),
                                 (N * (1.0 - e2) + h0) * std::sin(lat));
    const geo::LocalFrame L(origin, lat, lon);
    auto pose = [&](const Eigen::Isometry3d &T_L_C) {
        const Eigen::Isometry3d T = L.ecefFromLocal() * T_L_C;
        OST_Refine_Pose_t       p;
        const Eigen::Quaterniond q(T.linear());
        for (int k = 0; k < 3; ++k)
        {
            p.position_ecef[k] = T.translation()[k];
        }
        p.q_ecef_from_camera[0] = q.w();
        p.q_ecef_from_camera[1] = q.x();
        p.q_ecef_from_camera[2] = q.y();
        p.q_ecef_from_camera[3] = q.z();
        return p;
    };

    OST_Refine_Config_t c;
    OST_Refine_DefaultConfig(&c);
    c.precise_position         = 1;
    c.min_image_observations   = 30;
    c.max_window_images        = 10;
    c.max_points               = 2000;
    c.boresight_settle_batches = 3;
    c.max_wait_s               = 60.0;
    const OST_Refine_Camera_t camc { cam.width, cam.height, cam.fx, cam.fy, cam.cx, cam.cy };
    OST_Refine_t             *h = nullptr;
    ASSERT_EQ(OST_Refine_Create(&c, &camc, &h), OST_SUCCESS);

    std::vector<OST_Refine_Result_t> results;
    OST_Refine_Result_t              buf[2];
    for (std::size_t i = 0; i < f.truth.size(); ++i)
    {
        int has = 1;
        ASSERT_EQ(OST_Refine_HasImage(h, i, &has), OST_SUCCESS);
        EXPECT_EQ(has, 0);
        OST_Refine_Image_t im {};
        im.id             = i;
        im.capture_utc_ns = 1700000000000000000LL + f.time_ns[i];
        im.grey           = f.images[i].data;
        im.width          = cam.width;
        im.height         = cam.height;
        im.stride         = f.images[i].step;
        im.pose           = pose(f.prior[i]);
        ASSERT_EQ(OST_Refine_AddImage(h, &im), OST_SUCCESS) << OST_Refine_LastError(h);
        if (i >= 2 && i != 8 && (i - 1) % 8 != 0)
        {
            const uint64_t ids[3] = { i - 1, i - 2, i };
            // Plane z = 0 of the local frame: ellipsoidal height h0.
            ASSERT_EQ(OST_Refine_AddBatch(h, i - 1, ids, 3, h0), OST_SUCCESS) << OST_Refine_LastError(h);
        }
        uint32_t n = 0;
        ASSERT_EQ(OST_Refine_Poll(h, 1700000000000000000LL + f.time_ns[i], 0, buf, 2, &n), OST_SUCCESS);
        results.insert(results.end(), buf, buf + n);
    }
    uint32_t n = 0;
    do
    {
        ASSERT_EQ(OST_Refine_Poll(h, 1700000000000000000LL + f.time_ns.back(), 1, buf, 2, &n), OST_SUCCESS);
        results.insert(results.end(), buf, buf + n);
    } while (n > 0);

    ASSERT_EQ(results.size(), 12u);
    for (const auto &r : results)
    {
        ASSERT_EQ(r.refined, 1) << "batch " << r.batch_id << ": " << r.reason;
        ASSERT_EQ(r.count, 3u);
        const auto &p = r.poses[0];
        const Eigen::Matrix3d R_E_C =
            Eigen::Quaterniond(p.q_ecef_from_camera[0], p.q_ecef_from_camera[1], p.q_ecef_from_camera[2],
                               p.q_ecef_from_camera[3])
                .toRotationMatrix();
        const Eigen::Matrix3d truth = L.ecefFromLocal().linear() * f.truth[r.batch_id].linear();
        EXPECT_LT(angleDeg(R_E_C, truth), 0.02) << "batch " << r.batch_id;
    }
    OST_Refine_Stats_t st;
    ASSERT_EQ(OST_Refine_GetStats(h, &st), OST_SUCCESS);
    EXPECT_EQ(st.refined_batches, 12u);
    EXPECT_EQ(st.queued_results, 0u);

    // Errors: unknown image in a batch, oversized batch.
    const uint64_t bad[1] = { 999 };
    EXPECT_EQ(OST_Refine_AddBatch(h, 50, bad, 1, h0), OST_ERR_INVALID_ARGUMENT);
    EXPECT_STRNE(OST_Refine_LastError(h), "");
    EXPECT_EQ(OST_Refine_AddBatch(h, 51, bad, OST_REFINE_MAX_BATCH_IMAGES + 1, h0), OST_ERR_INVALID_ARGUMENT);
    OST_Refine_Destroy(h);
}
