// Tests of the incremental selector and its C interface (ADR-006).

#include <cmath>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

#include <gdal.h>
#include <gtest/gtest.h>

#include "orthostream/selection/selector.hpp"
#include "selection_c_api.h"
#include "test_support.hpp"

extern "C" int ost_selection_c_api_usage(void);

namespace fs  = std::filesystem;
namespace sel = orthostream::selection;
namespace geo = orthostream::geo;

namespace
{

constexpr double kDeg    = 3.14159265358979323846 / 180.0;
constexpr double kGround = 1500.0;  // ellipsoidal height of the flat terrain
constexpr double kAgl    = 250.0;

/// Flat terrain at kGround around (46.8 N, 9.8 E); the flight is given in
/// the ENU frame at the first camera position.
class Scene
{
public:
    Scene()
        : geodetic_(OST_SELECTION_TEST_PROJ_DATA_DIR),
          origin_(geodetic_.geodeticToEcef({46.8 * kDeg, 9.8 * kDeg, kGround + kAgl})),
          frame_(origin_, 46.8 * kDeg, 9.8 * kDeg)
    {
    }

    sel::SurfaceFunction surface(bool hole = false) const
    {
        return [this, hole](const Eigen::Vector3d &p) -> std::optional<Eigen::Vector3d> {
            geo::Geodetic g = geodetic_.ecefToGeodetic(p);
            if (hole && frame_.toLocal(p).y() > 400.0)
            {
                return std::nullopt;
            }
            g.height_m = kGround;
            return geodetic_.geodeticToEcef(g);
        };
    }

    sel::GeodeticFunction geodeticFn() const
    {
        return [this](const Eigen::Vector3d &p) { return geodetic_.ecefToGeodetic(p); };
    }

    /// Camera pose in ECEF from a nadir pose at (x, y) of the ENU frame,
    /// kAgl above the terrain (the frame origin is at that height).
    Eigen::Isometry3d pose(double x, double y, double yaw = 0.0) const
    {
        return frame_.ecefFromLocal() * sel::test::nadir({x, y, 0.0}, yaw);
    }

    const geo::GeodeticConverter &geodetic() const { return geodetic_; }

private:
    geo::GeodeticConverter geodetic_;
    Eigen::Vector3d        origin_;
    geo::LocalFrame        frame_;
};

/// Two lines: north along x = 0, then south along x = 120 (25 m steps),
/// with two images in the turn.
std::vector<Eigen::Vector2d> twoLines()
{
    std::vector<Eigen::Vector2d> p;
    for (int i = 0; i <= 24; ++i)
    {
        p.emplace_back(0.0, 25.0 * i);
    }
    p.emplace_back(60.0, 640.0);
    p.emplace_back(120.0, 620.0);
    for (int i = 24; i >= 0; --i)
    {
        p.emplace_back(120.0, 25.0 * i);
    }
    return p;
}

}  // namespace

TEST(Selector, KeyframesSourcesAndLines)
{
    const Scene         scene;
    sel::SelectorConfig config;
    sel::Selector       s(sel::test::camera(), config, scene.surface(), scene.geodeticFn());

    const auto                points = twoLines();
    std::vector<std::uint64_t> keyframes;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const sel::Decision d = s.add(i, scene.pose(points[i].x(), points[i].y()));
        ASSERT_TRUE(d.footprint) << i;
        EXPECT_NEAR(d.plane_height, kGround, 0.05) << i;
        EXPECT_NEAR(d.width_m, 2.0 * kAgl, 1.0);
        EXPECT_NEAR(d.height_m, kAgl, 1.0);
        EXPECT_LT(d.tilt_deg, 0.01);
        if (!d.keyframe)
        {
            continue;
        }
        keyframes.push_back(i);
        if (d.overlap_last >= 0.0)
        {
            EXPECT_LT(d.overlap_last, config.keyframe.target_overlap);
            EXPECT_GT(d.overlap_last, 0.5);
        }
        for (const sel::SourceView &v : d.sources)
        {
            EXPECT_LT(v.id, i) << "causal";
        }
        if (i == 0)
        {
            EXPECT_TRUE(d.sources.empty());
        }
        else if (i >= 30)
        {
            // Second line: one source of the first line.
            EXPECT_EQ(d.sources.size(), 5u);
            EXPECT_TRUE(std::any_of(d.sources.begin(), d.sources.end(),
                                    [](const sel::SourceView &v) { return v.other_line; }));
        }
    }
    EXPECT_EQ(keyframes.front(), 0u);
    EXPECT_GE(keyframes.size(), 12u);
    const sel::SelectorStats st = s.stats();
    EXPECT_EQ(st.images, points.size());
    EXPECT_EQ(st.footprints, points.size());
    EXPECT_GE(st.flight_lines, 3);
    EXPECT_EQ(st.pool, points.size());
}

TEST(Selector, ImagesWithoutTerrainStayOutOfThePool)
{
    const Scene   scene;
    sel::Selector s(sel::test::camera(), {}, scene.surface(true), scene.geodeticFn());
    EXPECT_TRUE(s.add(1, scene.pose(0.0, 0.0)).footprint);
    const sel::Decision d = s.add(2, scene.pose(0.0, 450.0));
    EXPECT_FALSE(d.footprint);
    EXPECT_FALSE(d.keyframe);
    EXPECT_EQ(s.stats().pool, 1u);
}

TEST(Selector, EvictsByDistanceCountAndAge)
{
    const Scene         scene;
    sel::SelectorConfig config;
    config.pool_distance_m = 260.0;
    config.max_pool        = 5;
    sel::Selector s(sel::test::camera(), config, scene.surface(), scene.geodeticFn());
    for (std::uint64_t i = 0; i < 4; ++i)
    {
        s.add(i, scene.pose(0.0, 25.0 * static_cast<double>(i)));
    }
    EXPECT_TRUE(s.evict().empty());
    s.add(4, scene.pose(0.0, 300.0));
    // Footprint centres at y = 0 and 25 are farther than 260 m from y = 300.
    EXPECT_EQ(s.evict(), (std::vector<std::uint64_t>{0, 1}));
    for (std::uint64_t i = 5; i < 9; ++i)
    {
        s.add(i, scene.pose(10.0 * static_cast<double>(i), 300.0));
    }
    // Image 2 (0, 50) is 262 m from (80, 300); then 6 remain, one more than
    // max_pool: the oldest, 3, leaves.
    EXPECT_EQ(s.evict(), (std::vector<std::uint64_t>{2, 3}));
    EXPECT_LE(s.stats().pool, 5u);
    const auto oldest = s.evictOldest();
    ASSERT_TRUE(oldest.has_value());
    EXPECT_TRUE(s.remove(8));
    EXPECT_FALSE(s.remove(8));
    s.reset(sel::test::camera());
    EXPECT_EQ(s.stats().pool, 0u);
    EXPECT_FALSE(s.hasFrame());
}

namespace
{

/// Flat DEM with ellipsoidal heights (EPSG:4979) at kGround around the scene.
std::string writeFlatDem(const fs::path &dir)
{
    GDALAllRegister();
    const std::string path = (dir / "flat.tif").string();
    GDALDatasetH      ds   = GDALCreate(GDALGetDriverByName("GTiff"), path.c_str(), 200, 200, 1, GDT_Float32, nullptr);
    double            gt[6] = {9.7, 0.001, 0.0, 46.9, 0.0, -0.001};
    GDALSetGeoTransform(ds, gt);
    std::vector<float> v(200 * 200, static_cast<float>(kGround));
    EXPECT_EQ(GDALRasterIO(GDALGetRasterBand(ds, 1), GF_Write, 0, 0, 200, 200, v.data(), 200, 200, GDT_Float32, 0, 0),
              CE_None);
    GDALClose(ds);
    return path;
}

}  // namespace

TEST(SelectCApi, SelectsWithADemFile)
{
    EXPECT_EQ(ost_selection_c_api_usage(), 0);

    const fs::path dir = fs::temp_directory_path() / "ost_select_c_api";
    fs::create_directories(dir);
    const std::string    dem = writeFlatDem(dir);
    const Scene          scene;
    OST_Select_Config_t  c;
    OST_Select_DefaultConfig(&c);
    EXPECT_EQ(c.max_sources, 5);
    EXPECT_DOUBLE_EQ(c.target_overlap, 0.7);
    c.terrain_radius_m = 3000.0;
    OST_Select_Terrain_t t{dem.c_str(), "EPSG:4979", OST_SELECTION_TEST_PROJ_DATA_DIR, ""};
    OST_Geo_Camera_t     cam{};
    const geo::CameraModel m = sel::test::camera();
    cam.width = m.width, cam.height = m.height, cam.fx = m.fx, cam.fy = m.fy, cam.cx = m.cx, cam.cy = m.cy;

    OST_Select_t *h = nullptr;
    OST_Select_Terrain_t missing = t;
    missing.dem_path             = "/nonexistent/dem.tif";
    EXPECT_EQ(OST_Select_Create(&c, &missing, &cam, &h), OST_ERR_IO);
    ASSERT_EQ(OST_Select_Create(&c, &t, &cam, &h), OST_SUCCESS);

    const auto points = twoLines();
    uint32_t   keyframes = 0, with_other = 0;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const Eigen::Isometry3d  T = scene.pose(points[i].x(), points[i].y());
        const Eigen::Quaterniond q(T.linear());
        const double             p[3] = {T.translation().x(), T.translation().y(), T.translation().z()};
        const double             qq[4] = {q.w(), q.x(), q.y(), q.z()};
        OST_Select_Decision_t    d;
        ASSERT_EQ(OST_Select_AddImage(h, 100 + i, p, qq, &d), OST_SUCCESS);
        EXPECT_EQ(d.terrain_available, 1);
        ASSERT_EQ(d.footprint, 1);
        EXPECT_NEAR(d.plane_height_m, kGround, 0.05);
        if (d.keyframe)
        {
            ++keyframes;
            for (uint32_t k = 0; k < d.source_count; ++k)
            {
                EXPECT_LT(d.sources[k].id, 100 + i);
                with_other += d.sources[k].other_line;
            }
        }
        uint64_t ids[8];
        uint32_t n = 0;
        ASSERT_EQ(OST_Select_Evict(h, ids, 8, &n), OST_SUCCESS);
    }
    EXPECT_GE(keyframes, 12u);
    EXPECT_GT(with_other, 0u);

    OST_Select_Stats_t st;
    ASSERT_EQ(OST_Select_GetStats(h, &st), OST_SUCCESS);
    EXPECT_EQ(st.terrain_loads, 1u);
    EXPECT_EQ(st.terrain_loaded, 1);
    EXPECT_EQ(st.images, points.size());

    uint64_t id    = 0;
    int      found = 0;
    ASSERT_EQ(OST_Select_EvictOldest(h, &id, &found), OST_SUCCESS);
    EXPECT_EQ(found, 1);
    EXPECT_GE(id, 100u);  // the first images left the pool by distance
    ASSERT_EQ(OST_Select_Reset(h, &cam), OST_SUCCESS);
    ASSERT_EQ(OST_Select_GetStats(h, &st), OST_SUCCESS);
    EXPECT_EQ(st.pool, 0u);
    const double bad_q[4] = {2, 0, 0, 0};
    const double p0[3]    = {0, 0, 0};
    OST_Select_Decision_t d;
    EXPECT_EQ(OST_Select_AddImage(h, 1, p0, bad_q, &d), OST_ERR_INVALID_ARGUMENT);
    EXPECT_STRNE(OST_Select_LastError(h), "");
    OST_Select_Destroy(h);
    fs::remove_all(dir);
}
