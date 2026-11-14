// The C interface on a synthetic scene: flat textured ground at a known
// ellipsoidal height, a flat elevation model of that height (EPSG:4979),
// nadir cameras posed in ECEF.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <gdal.h>
#include <gtest/gtest.h>

#include "dense_c_api.h"
#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/frames.hpp"

extern "C" int ost_dense_c_api_usage(void);

namespace
{

namespace fs  = std::filesystem;
namespace geo = orthostream::geo;

constexpr double kPi     = 3.14159265358979323846;
constexpr double kGround = 1500.0;  // ellipsoidal height of the ground, m
constexpr double kHeight = 100.0;   // camera height above the ground, m
constexpr int    kW = 320, kH = 240;

double hash01(int x, int y)
{
    unsigned h = static_cast<unsigned>(x) * 374761393u + static_cast<unsigned>(y) * 668265263u;
    h          = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<double>(h ^ (h >> 16)) / 4294967295.0;
}

double texture(double x, double y)
{
    const double fx = std::floor(x), fy = std::floor(y);
    const double tx = x - fx, ty = y - fy;
    const int    ix = static_cast<int>(fx), iy = static_cast<int>(fy);
    const double n  = (1 - tx) * (1 - ty) * hash01(ix, iy) + tx * (1 - ty) * hash01(ix + 1, iy)
                   + (1 - tx) * ty * hash01(ix, iy + 1) + tx * ty * hash01(ix + 1, iy + 1);
    return 40.0 + 170.0 * n + 30.0 * std::sin(0.31 * x) * std::cos(0.23 * y);
}

// Flat elevation model of the given height over lon 9.7-9.9, lat 46.7-46.9.
std::string writeFlatDem(const fs::path &dir, double height)
{
    GDALAllRegister();
    const std::string path  = (dir / "flat.tif").string();
    GDALDatasetH      ds    = GDALCreate(GDALGetDriverByName("GTiff"), path.c_str(), 200, 200, 1, GDT_Float32, nullptr);
    double            gt[6] = { 9.7, 0.001, 0.0, 46.9, 0.0, -0.001 };
    GDALSetGeoTransform(ds, gt);
    std::vector<float> v(200 * 200, static_cast<float>(height));
    EXPECT_EQ(GDALRasterIO(GDALGetRasterBand(ds, 1), GF_Write, 0, 0, 200, 200, v.data(), 200, 200, GDT_Float32, 0, 0),
              CE_None);
    GDALClose(ds);
    return path;
}

// Ground frame G: east-north-up on the ground at a geodetic position;
// cameras look straight down from kHeight.
struct Scene
{
    explicit Scene(double lon_deg = 9.8)
        : geodetic(OST_DENSE_TEST_PROJ_DATA_DIR),
          origin(geodetic.geodeticToEcef({ 46.8 * kPi / 180.0, lon_deg * kPi / 180.0, kGround })),
          G(origin, 46.8 * kPi / 180.0, lon_deg * kPi / 180.0)
    {
        camera.width  = kW;
        camera.height = kH;
        camera.fx = camera.fy = 300.0;
        camera.cx             = 159.5;
        camera.cy             = 119.5;
    }

    Eigen::Isometry3d poseInGround(double x, double y) const
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() << 1, 0, 0, 0, -1, 0, 0, 0, -1;  // x east, y south, z down
        T.translation() = Eigen::Vector3d(x, y, kHeight);
        return T;
    }

    std::vector<std::uint8_t> render(const Eigen::Isometry3d &T_G_C) const
    {
        std::vector<std::uint8_t> img(static_cast<std::size_t>(kW) * kH);
        for (int v = 0; v < kH; ++v)
        {
            for (int u = 0; u < kW; ++u)
            {
                double sum = 0.0;
                for (int sv = -1; sv <= 1; ++sv)
                {
                    for (int su = -1; su <= 1; ++su)
                    {
                        const Eigen::Vector3d r = T_G_C.linear()
                                                  * Eigen::Vector3d((u + su / 3.0 - camera.cx) / camera.fx,
                                                                    (v + sv / 3.0 - camera.cy) / camera.fy, 1.0);
                        const Eigen::Vector3d g = T_G_C.translation() + r * (-T_G_C.translation().z() / r.z());
                        sum += texture(g.x(), g.y());
                    }
                }
                img[static_cast<std::size_t>(v) * kW + u] =
                    static_cast<std::uint8_t>(std::clamp(std::lround(sum / 9.0), 0L, 255L));
            }
        }
        return img;
    }

    OST_Dense_View_t view(double x, double y, std::vector<std::uint8_t> &img) const
    {
        const Eigen::Isometry3d  T_G_C = poseInGround(x, y);
        const Eigen::Isometry3d  T_E_C = G.ecefFromLocal() * T_G_C;
        const Eigen::Quaterniond q(T_E_C.linear());
        img = render(T_G_C);
        OST_Dense_View_t v {};
        v.grey   = img.data();
        v.stride = kW;
        for (int i = 0; i < 3; ++i)
        {
            v.position_ecef[i] = T_E_C.translation()[i];
        }
        v.q_ecef_from_camera[0] = q.w();
        v.q_ecef_from_camera[1] = q.x();
        v.q_ecef_from_camera[2] = q.y();
        v.q_ecef_from_camera[3] = q.z();
        return v;
    }

    geo::GeodeticConverter geodetic;
    Eigen::Vector3d        origin;
    geo::LocalFrame        G;
    OST_Dense_Camera_t     camera {};
};

struct Outcome
{
    OST_Status_t       status = OST_ERR_INTERNAL;
    OST_Dense_Result_t result {};
    std::vector<float> depth = std::vector<float>(kW * kH), confidence = std::vector<float>(kW * kH);
    double             central_valid  = 0.0;
    double             central_median = 1e9;  ///< median |depth - kHeight|
};

Outcome estimate(OST_Dense_t *h, const Scene &s, double plane_height)
{
    std::vector<std::uint8_t>     ref_img, a_img, b_img;
    const OST_Dense_View_t        ref     = s.view(0, 0, ref_img);
    const OST_Dense_View_t        src[2]  = { s.view(60, 0, a_img), s.view(20, 0, b_img) };
    Outcome                       r;
    const OST_Dense_Output_t      out { r.depth.data(), r.confidence.data(), nullptr };
    r.status = OST_Dense_Estimate(h, &s.camera, &ref, src, 2, plane_height, &out, &r.result);
    std::vector<double> err;
    int                 total = 0;
    for (int v = 40; v < kH - 40; ++v)
    {
        for (int u = 60; u < kW - 60; ++u)
        {
            ++total;
            const float d = r.depth[static_cast<std::size_t>(v) * kW + u];
            if (d > 0.0f)
            {
                err.push_back(std::abs(d - kHeight));
            }
        }
    }
    r.central_valid = static_cast<double>(err.size()) / total;
    if (!err.empty())
    {
        std::nth_element(err.begin(), err.begin() + static_cast<std::ptrdiff_t>(err.size() / 2), err.end());
        r.central_median = err[err.size() / 2];
    }
    return r;
}

class DenseCApi : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dir_ = fs::temp_directory_path() / ("ost_dense_c_api_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
        fs::create_directories(dir_);
    }
    void TearDown() override { fs::remove_all(dir_); }

    OST_Dense_t *create(double dem_height, double gate = 30.0)
    {
        dem_ = writeFlatDem(dir_, dem_height);
        OST_Dense_Config_t c;
        OST_Dense_DefaultConfig(&c);
        c.terrain_radius_m = 3000.0;
        c.terrain_gate_m   = gate;
        const OST_Dense_Terrain_t t { dem_.c_str(), "EPSG:4979", OST_DENSE_TEST_PROJ_DATA_DIR, "" };
        OST_Dense_t              *h = nullptr;
        EXPECT_EQ(OST_Dense_Create(&c, &t, &h), OST_SUCCESS);
        return h;
    }

    fs::path    dir_;
    std::string dem_;
};

}  // namespace

TEST(DenseCApiUsage, CompilesAsC99WithTheDefaultsOfAdr005)
{
    EXPECT_EQ(ost_dense_c_api_usage(), 0);
}

TEST_F(DenseCApi, RecoversFlatGroundInTheBatchFrame)
{
    OST_Dense_t *h = create(kGround);
    ASSERT_NE(h, nullptr);
    const Scene s;
    const Outcome   r = estimate(h, s, kGround);
    ASSERT_EQ(r.status, OST_SUCCESS) << OST_Dense_LastError(h);
    EXPECT_EQ(r.result.terrain_available, 1);
    EXPECT_EQ(r.result.gate_applied, 1);
    EXPECT_EQ(r.result.terrain_loads, 1u);
    EXPECT_EQ(r.result.source_index, 1);  // 20 m of 100 m: ratio 0.2
    EXPECT_NEAR(r.result.height_m, kGround + kHeight, 1e-3);
    const Eigen::Vector3d camera_E = s.G.toEcef({ 0, 0, kHeight });
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(r.result.origin_ecef[i], camera_E[i], 1e-6);
    }
    EXPECT_NEAR(r.result.depth_min_m, kHeight - 50.0, 1.0);  // terrain +- 50 m margin
    EXPECT_NEAR(r.result.depth_max_m, kHeight + 50.0, 1.0);
    EXPECT_GT(r.central_valid, 0.7);
    EXPECT_LT(r.central_median, 0.5);
    EXPECT_GT(r.result.valid_pixels, static_cast<uint32_t>(0.5 * kW * kH));
    EXPECT_GT(r.result.estimate_s, 0.0);
    OST_Dense_Destroy(h);
}

TEST_F(DenseCApi, TheTerrainGateRemovesDepthFarFromTheTerrain)
{
    // The model says 40 m higher than the ground seen: every depth is gated.
    OST_Dense_t *h = create(kGround + 40.0);
    ASSERT_NE(h, nullptr);
    const Outcome r = estimate(h, Scene(), kGround);
    ASSERT_EQ(r.status, OST_SUCCESS) << OST_Dense_LastError(h);
    EXPECT_EQ(r.result.gate_applied, 1);
    EXPECT_GT(r.result.gated_pixels, 0u);
    EXPECT_EQ(r.result.valid_pixels, 0u);
    OST_Dense_Destroy(h);

    // Without the gate the same depths stay.
    h = create(kGround + 40.0, 0.0);
    ASSERT_NE(h, nullptr);
    const Outcome open = estimate(h, Scene(), kGround);
    ASSERT_EQ(open.status, OST_SUCCESS);
    EXPECT_EQ(open.result.gate_applied, 0);
    EXPECT_GT(open.central_valid, 0.7);
    OST_Dense_Destroy(h);
}

TEST_F(DenseCApi, WithoutTerrainTheRangeFollowsThePlaneHeight)
{
    OST_Dense_t *h = create(kGround);
    ASSERT_NE(h, nullptr);
    const Outcome r = estimate(h, Scene(10.5), kGround);  // outside the model
    ASSERT_EQ(r.status, OST_SUCCESS) << OST_Dense_LastError(h);
    EXPECT_EQ(r.result.terrain_available, 0);
    EXPECT_EQ(r.result.gate_applied, 0);
    EXPECT_NEAR(r.result.depth_min_m, kHeight - 50.0, 1.0);
    EXPECT_GT(r.central_valid, 0.7);
    EXPECT_LT(r.central_median, 0.5);
    OST_Dense_Destroy(h);
}

TEST_F(DenseCApi, RejectsInvalidInput)
{
    OST_Dense_Config_t c;
    OST_Dense_DefaultConfig(&c);
    const std::string         missing = (dir_ / "none.tif").string();
    const OST_Dense_Terrain_t bad { missing.c_str(), "EPSG:4979", OST_DENSE_TEST_PROJ_DATA_DIR, "" };
    OST_Dense_t              *h = nullptr;
    EXPECT_EQ(OST_Dense_Create(&c, &bad, &h), OST_ERR_IO);
    EXPECT_EQ(h, nullptr);
    if (!OST_Dense_PatchMatchAvailable())
    {
        OST_Dense_Config_t pm = c;
        pm.method             = OST_DENSE_METHOD_PATCHMATCH;
        const std::string dem = writeFlatDem(dir_, kGround);
        const OST_Dense_Terrain_t t { dem.c_str(), "EPSG:4979", OST_DENSE_TEST_PROJ_DATA_DIR, "" };
        EXPECT_EQ(OST_Dense_Create(&pm, &t, &h), OST_ERR_UNSUPPORTED);
    }

    h = create(kGround);
    ASSERT_NE(h, nullptr);
    const Scene               s;
    std::vector<std::uint8_t> a, b;
    OST_Dense_View_t          ref = s.view(0, 0, a);
    const OST_Dense_View_t    src = s.view(20, 0, b);
    std::vector<float>        d(kW * kH), conf(kW * kH);
    const OST_Dense_Output_t  out { d.data(), conf.data(), nullptr };
    OST_Dense_Result_t        res;
    EXPECT_EQ(OST_Dense_Estimate(h, &s.camera, &ref, &src, 0, kGround, &out, &res), OST_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(OST_Dense_Estimate(h, &s.camera, &ref, &src, 1, kGround + 200.0, &out, &res), OST_ERR_INVALID_ARGUMENT);
    EXPECT_STRNE(OST_Dense_LastError(h), "");
    ref.q_ecef_from_camera[0] = 2.0;
    EXPECT_EQ(OST_Dense_Estimate(h, &s.camera, &ref, &src, 1, kGround, &out, &res), OST_ERR_INVALID_ARGUMENT);
    OST_Dense_Destroy(h);
}
