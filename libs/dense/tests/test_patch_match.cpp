// PatchMatch on a synthetic scene with known geometry: a textured, sloped
// plane seen by a nadir reference camera and four source cameras.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "orthostream/dense/patch_match.hpp"

#ifdef OST_DENSE_TEST_CUDA
#include <cuda_runtime_api.h>
#endif

namespace orthostream::dense
{
namespace
{

// Ground plane n . p = d in the working frame: slope 0.2 east, 0.1 north.
const Eigen::Vector3d kPlaneNormal = Eigen::Vector3d(-0.2, -0.1, 1.0).normalized();
constexpr double      kPlaneD      = 0.0;

geo::CameraModel testCamera()
{
    geo::CameraModel c;
    c.width  = 320;
    c.height = 240;
    c.fx     = 300.0;
    c.fy     = 300.0;
    c.cx     = 159.5;
    c.cy     = 119.5;
    return c;
}

// Nadir camera at a position: x right = east, y down = south, z = down.
Eigen::Isometry3d nadirPose(const Eigen::Vector3d &centre)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    T.translation() = centre;
    return T;
}

double hash01(int x, int y)
{
    unsigned h = static_cast<unsigned>(x) * 374761393u + static_cast<unsigned>(y) * 668265263u;
    h          = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<double>(h ^ (h >> 16)) / 4294967295.0;
}

// Continuous texture on the ground: value noise at 1 m plus smooth waves.
double texture(double x, double y)
{
    const double fx = std::floor(x), fy = std::floor(y);
    const double tx = x - fx, ty = y - fy;
    const int    ix = static_cast<int>(fx), iy = static_cast<int>(fy);
    const double n  = (1 - tx) * (1 - ty) * hash01(ix, iy) + tx * (1 - ty) * hash01(ix + 1, iy)
                   + (1 - tx) * ty * hash01(ix, iy + 1) + tx * ty * hash01(ix + 1, iy + 1);
    return 40.0 + 170.0 * n + 30.0 * std::sin(0.31 * x) * std::cos(0.23 * y);
}

// Depth (z in the camera) of the plane at a pixel, and the ground point.
double planeDepth(const geo::CameraModel &cam, const Eigen::Isometry3d &T_L_C, double u, double v,
                  Eigen::Vector3d *ground = nullptr)
{
    const Eigen::Vector3d ray = T_L_C.linear() * Eigen::Vector3d((u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0);
    const double          t   = (kPlaneD - kPlaneNormal.dot(T_L_C.translation())) / kPlaneNormal.dot(ray);
    if (ground != nullptr)
    {
        *ground = T_L_C.translation() + t * ray;
    }
    return t;  // ray has z = 1 in the camera frame
}

std::vector<std::uint8_t> render(const geo::CameraModel &cam, const Eigen::Isometry3d &T_L_C)
{
    std::vector<std::uint8_t> img(static_cast<std::size_t>(cam.width) * cam.height);
    for (int v = 0; v < cam.height; ++v)
    {
        for (int u = 0; u < cam.width; ++u)
        {
            double sum = 0.0;  // 3 x 3 supersampling against aliasing
            for (int sv = -1; sv <= 1; ++sv)
            {
                for (int su = -1; su <= 1; ++su)
                {
                    Eigen::Vector3d g;
                    planeDepth(cam, T_L_C, u + su / 3.0, v + sv / 3.0, &g);
                    sum += texture(g.x(), g.y());
                }
            }
            img[static_cast<std::size_t>(v) * cam.width + u] =
                static_cast<std::uint8_t>(std::clamp(std::lround(sum / 9.0), 0L, 255L));
        }
    }
    return img;
}

bool gpuAvailable()
{
#ifdef OST_DENSE_TEST_CUDA
    int count = 0;
    return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
#else
    return false;
#endif
}

TEST(PatchMatch, RecoversASlopedPlane)
{
    if (!patchMatchAvailable() || !gpuAvailable())
    {
        GTEST_SKIP() << "PatchMatch needs COLMAP and a CUDA GPU";
    }
    const geo::CameraModel cam = testCamera();
    const double           H   = 100.0;

    DepthBatch batch;
    batch.camera          = cam;
    batch.depth_min       = 50.0;
    batch.depth_max       = 200.0;
    batch.reference.T_L_C = nadirPose({ 0.0, 0.0, H });
    std::vector<std::vector<std::uint8_t>> images;
    images.push_back(render(cam, batch.reference.T_L_C));
    for (const Eigen::Vector3d c : { Eigen::Vector3d(20, 0, H), Eigen::Vector3d(-20, 0, H),
                                     Eigen::Vector3d(0, 15, H), Eigen::Vector3d(0, -15, H) })
    {
        BatchView v;
        v.T_L_C = nadirPose(c);
        images.push_back(render(cam, v.T_L_C));
        batch.sources.push_back(v);
    }
    batch.reference.image = { images[0].data(), static_cast<std::size_t>(cam.width) };
    for (std::size_t i = 0; i < batch.sources.size(); ++i)
    {
        batch.sources[i].image = { images[i + 1].data(), static_cast<std::size_t>(cam.width) };
    }

    const std::size_t  n = static_cast<std::size_t>(cam.width) * cam.height;
    std::vector<float> depth(n), normals(3 * n), confidence(n);
    PatchMatchEstimator estimator(PatchMatchConfig {});
    estimator.estimate(batch, { depth.data(), normals.data(), confidence.data() });

    // Central region, away from the border where fewer sources overlap.
    std::vector<double> rel_err;
    int                 valid = 0, total = 0;
    const Eigen::Vector3d n_C = batch.reference.T_L_C.linear().transpose() * kPlaneNormal;
    double                max_normal_angle = 0.0;
    for (int v = 40; v < cam.height - 40; ++v)
    {
        for (int u = 50; u < cam.width - 50; ++u)
        {
            const std::size_t i = static_cast<std::size_t>(v) * cam.width + u;
            ++total;
            if (depth[i] <= 0.0f)
            {
                continue;
            }
            ++valid;
            const double expected = planeDepth(cam, batch.reference.T_L_C, u, v);
            rel_err.push_back(std::abs(depth[i] - expected) / expected);
            const Eigen::Vector3d est(normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]);
            // Normals face the camera; the plane normal of the test points up.
            max_normal_angle = std::max(max_normal_angle, std::acos(std::min(1.0, std::abs(est.dot(n_C)))));
            EXPECT_GT(confidence[i], 0.0f);
            EXPECT_LE(confidence[i], 1.0f);
        }
    }
    ASSERT_GT(valid, 0);
    std::nth_element(rel_err.begin(), rel_err.begin() + rel_err.size() / 2, rel_err.end());
    const double median = rel_err[rel_err.size() / 2];
    RecordProperty("valid_fraction", std::to_string(static_cast<double>(valid) / total));
    RecordProperty("median_relative_depth_error", std::to_string(median));
    EXPECT_GT(static_cast<double>(valid) / total, 0.8);
    EXPECT_LT(median, 0.005);  // 0.5 % of the depth, 0.5 m at 100 m
    (void)max_normal_angle;
}

TEST(DepthBatch, RejectsInvalidBatches)
{
    std::vector<std::uint8_t> img(320 * 240);
    std::vector<float>        d(320 * 240), nrm(3 * 320 * 240), c(320 * 240);
    DepthBatch                batch;
    batch.camera          = testCamera();
    batch.depth_min       = 50.0;
    batch.depth_max       = 200.0;
    batch.reference.image = { img.data(), 320 };
    const DepthOutput out { d.data(), nrm.data(), c.data() };

    EXPECT_THROW(validateBatch(batch, out), DenseError);  // no sources
    BatchView source;
    source.image = { img.data(), 320 };
    batch.sources.push_back(source);
    EXPECT_NO_THROW(validateBatch(batch, out));

    DepthBatch distorted = batch;
    distorted.camera.k1  = -0.1;
    EXPECT_THROW(validateBatch(distorted, out), DenseError);

    DepthBatch bad_range = batch;
    bad_range.depth_max  = 10.0;
    EXPECT_THROW(validateBatch(bad_range, out), DenseError);

    EXPECT_THROW(validateBatch(batch, { d.data(), nullptr, c.data() }), DenseError);
}

TEST(PatchMatch, GeometricConsistencyIsNotImplementedYet)
{
    if (!patchMatchAvailable())
    {
        EXPECT_THROW(PatchMatchEstimator(PatchMatchConfig {}), NotAvailableError);
        return;
    }
    PatchMatchConfig config;
    config.geom_consistency = true;
    EXPECT_THROW(PatchMatchEstimator { config }, DenseError);
}

}  // namespace
}  // namespace orthostream::dense
