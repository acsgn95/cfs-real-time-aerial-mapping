#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "map_products_c_api.h"
#include "orthostream/map_products/dsm.hpp"
#include "orthostream/map_products/point_set.hpp"
#include "test_support.hpp"

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;
using test::flatScene;
using test::Scene;

// Two overlapping keyframes over a flat surface, with noise and outliers,
// so that cells get several observations and the k-sigma test rejects some.
std::vector<Scene> noisyScenes(const geo::GeodeticConverter &conv)
{
    std::vector<Scene> scenes { flatScene(conv, 400, 300, 300.0), flatScene(conv, 400, 300, 300.0, 40.0, 10.0) };
    std::mt19937                     rng(7);
    std::normal_distribution<float>  noise(0.0f, 0.1f);
    std::uniform_int_distribution<>  pick(0, 99);
    for (Scene &s : scenes)
    {
        for (std::size_t i = 0; i < s.depth.size(); ++i)
        {
            s.depth[i] += pick(rng) == 0 ? 8.0f : noise(rng);  // 1 % outliers
            s.confidence[i] = 0.5f + 0.5f * static_cast<float>(i % 7) / 6.0f;
        }
    }
    return scenes;
}

TEST(DsmFusion, PointSetsGiveTheSameDsmAsDepthMaps)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:32632", OST_MP_TEST_PROJ_DATA_DIR);
    DsmFusionConfig              config;
    config.grid.cell_size  = 2.0;
    config.grid.tile_cells = 64;
    config.min_sigma       = 0.05;
    DsmFusion       from_depth(config, frame), from_points(config, frame);
    PointSetBuilder builder(frame, config.grid, 0.0);

    std::size_t rejected = 0;
    for (const Scene &s : noisyScenes(conv))
    {
        const DepthMapView    map { s.depth.data(), s.confidence.data() };
        const FusionStats     a = from_depth.addDepthMap(map, s.camera, s.T_E_C);
        std::vector<PointRecord> points(s.depth.size());
        std::vector<TileSpan>    tiles(16);
        const PointSetStats      st = builder.build(map, s.camera, {}, s.T_E_C, points.data(), points.size(),
                                                    tiles.data(), tiles.size());
        const FusionStats b = from_points.addPoints({ points.data(), st.points, st.origin, tiles.data(), st.tiles });
        EXPECT_EQ(b.points, a.points);
        EXPECT_EQ(b.added, a.added);
        EXPECT_EQ(b.rejected, a.rejected);
        EXPECT_EQ(b.tiles.size(), a.tiles.size());
        rejected += a.rejected;
    }
    EXPECT_GT(rejected, 0u);  // the test exercises the rejection

    // Float storage of the relative coordinates (about 0.03 mm here) moves
    // a point that lies that close to a cell edge into the neighbouring
    // cell; such cells differ, all others are identical.
    ASSERT_EQ(from_points.tiles().size(), from_depth.tiles().size());
    std::size_t filled = 0, differ = 0;
    double      worst  = 0.0;
    for (const auto &[id, ta] : from_depth.tiles())
    {
        const DsmTile *tb = from_points.tile(id);
        ASSERT_NE(tb, nullptr);
        EXPECT_EQ(tb->referenceHeight(), ta->referenceHeight());
        for (int r = 0; r < ta->cells(); ++r)
        {
            for (int c = 0; c < ta->cells(); ++c)
            {
                filled += ta->observations(c, r) > 0;
                if (tb->observations(c, r) != ta->observations(c, r))
                {
                    ++differ;
                }
                else if (const auto h = ta->height(c, r))
                {
                    worst = std::max(worst, std::abs(*tb->height(c, r) - *h));
                }
            }
        }
    }
    std::printf("cells %zu, differing %zu, largest height difference elsewhere %.2e m\n", filled, differ, worst);
    EXPECT_LT(static_cast<double>(differ), 1e-3 * static_cast<double>(filled));
    EXPECT_LT(worst, 1e-3);
}

struct Products
{
    fs::path            dir;
    OST_Map_Products_t *h = nullptr;

    explicit Products(const std::string &name) : dir(fs::path(::testing::TempDir()) / name)
    {
        fs::remove_all(dir);
        const std::string        out = dir.string();
        OST_Map_ProductsConfig_t c;
        OST_Map_ProductsDefaultConfig(&c);
        c.projected_crs       = "EPSG:32632";
        c.vertical_crs        = "";
        c.proj_data_dir       = OST_MP_TEST_PROJ_DATA_DIR;
        c.output_dir          = out.c_str();
        c.cell_size_m         = 2.0;
        c.tile_cells          = 64;
        c.ortho_pixel_m       = 0.5;
        c.surface_smoothing_m = 0.0;
        EXPECT_EQ(OST_Map_ProductsCreate(&c, &h), OST_SUCCESS);
    }
    ~Products() { OST_Map_ProductsDestroy(h); }
};

TEST(MapProductsCApi, FusesRendersPublishesAndFinalises)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const Scene                  s = flatScene(conv);
    Products                     p("ost_map_products_c_api");
    ASSERT_NE(p.h, nullptr);

    // Point set through the georeferencing interface.
    OST_Map_Georef_t      *g = nullptr;
    OST_Map_GeorefConfig_t gc;
    OST_Map_GeorefDefaultConfig(&gc);
    gc.projected_crs = "EPSG:32632";
    gc.proj_data_dir = OST_MP_TEST_PROJ_DATA_DIR;
    gc.cell_size_m   = 2.0;
    gc.tile_cells    = 64;
    ASSERT_EQ(OST_Map_GeorefCreate(&gc, &g), OST_SUCCESS);
    OST_Map_DepthMap_t map {};
    map.camera     = { s.camera.width, s.camera.height, s.camera.fx, s.camera.fy, s.camera.cx, s.camera.cy };
    map.depth      = s.depth.data();
    map.confidence = s.confidence.data();
    OST_Map_Pose_t           pose {};
    const Eigen::Quaterniond q(s.T_E_C.linear());
    const double             qv[4] = { q.w(), q.x(), q.y(), q.z() };
    for (int i = 0; i < 4; ++i)
    {
        pose.q_ecef_from_camera[i] = qv[i];
    }
    for (int i = 0; i < 3; ++i)
    {
        pose.position_ecef[i] = s.T_E_C.translation()[i];
    }
    std::vector<OST_Map_Point_t>    points(s.depth.size());
    std::vector<OST_Map_TileSpan_t> spans(16);
    OST_Map_GeorefResult_t          gr;
    ASSERT_EQ(OST_Map_GeorefConvert(g, &map, nullptr, &pose, points.data(), static_cast<uint32_t>(points.size()),
                                    spans.data(), 16, &gr),
              OST_SUCCESS);
    OST_Map_GeorefDestroy(g);

    const OST_Map_PointSet_t set { points.data(), gr.points, gr.tile_count, spans.data(),
                                   { gr.origin[0], gr.origin[1], gr.origin[2] } };
    std::vector<OST_Map_TileSpan_t> touched(16);
    OST_Map_FusionResult_t          fr;
    ASSERT_EQ(OST_Map_ProductsAddPoints(p.h, &set, touched.data(), 16, &fr), OST_SUCCESS)
        << OST_Map_ProductsLastError(p.h);
    EXPECT_EQ(fr.points, s.depth.size());
    EXPECT_EQ(fr.added, fr.points);
    EXPECT_EQ(fr.tile_count, gr.tile_count);
    EXPECT_NEAR(fr.height_median, 2000.0, 0.01);
    EXPECT_LT(fr.extent[0], fr.extent[2]);

    // Orthomosaic from a grey keyframe image of the same (distortion-free) camera.
    const std::vector<std::uint8_t> rgb(static_cast<std::size_t>(3 * s.camera.width * s.camera.height), 120);
    const OST_Map_RgbImage_t        img { rgb.data(), s.camera.width, s.camera.height,
                                          static_cast<size_t>(3 * s.camera.width) };
    const OST_Map_FullCamera_t      cam { s.camera.width, s.camera.height, s.camera.fx, s.camera.fy,
                                          s.camera.cx, s.camera.cy, 0, 0, 0, 0, 0 };
    OST_Map_RenderResult_t          rr;
    ASSERT_EQ(OST_Map_ProductsRender(p.h, &img, &cam, &pose, fr.height_median, fr.extent, touched.data(), 16, &rr),
              OST_SUCCESS)
        << OST_Map_ProductsLastError(p.h);
    EXPECT_GT(rr.pixels, 100000u);  // about 333 m x 250 m at 0.5 m
    EXPECT_GE(rr.tile_count, 4u);

    OST_Map_ProductsStats_t st;
    ASSERT_EQ(OST_Map_ProductsGetStats(p.h, &st), OST_SUCCESS);
    EXPECT_EQ(st.active_tiles, rr.tile_count > fr.tile_count ? rr.tile_count : fr.tile_count);
    EXPECT_EQ(st.changed_tiles, st.active_tiles);
    EXPECT_GT(st.memory_bytes, 0u);

    OST_Map_WriteResult_t wr;
    ASSERT_EQ(OST_Map_ProductsPublish(p.h, &wr), OST_SUCCESS) << OST_Map_ProductsLastError(p.h);
    EXPECT_EQ(wr.tiles, fr.tile_count);
    EXPECT_TRUE(fs::exists(p.dir / "dsm.vrt"));
    EXPECT_TRUE(fs::exists(p.dir / "ortho.vrt"));
    ASSERT_EQ(OST_Map_ProductsGetStats(p.h, &st), OST_SUCCESS);
    EXPECT_EQ(st.changed_tiles, 0u);

    // The vehicle stays close: nothing is finalised.
    ASSERT_EQ(OST_Map_ProductsUpdate(p.h, pose.position_ecef, &wr), OST_SUCCESS);
    EXPECT_EQ(wr.tiles, 0u);
    // Far away: everything is finalised and evicted.
    const double far[3] = { pose.position_ecef[0] + 5000.0, pose.position_ecef[1], pose.position_ecef[2] };
    ASSERT_EQ(OST_Map_ProductsUpdate(p.h, far, &wr), OST_SUCCESS);
    EXPECT_EQ(wr.tiles, st.active_tiles);
    EXPECT_EQ(wr.active_tiles, 0u);
    std::size_t states = 0;
    for (const auto &e : fs::directory_iterator(p.dir / "state"))
    {
        states += e.path().extension() == ".state";
    }
    EXPECT_GE(states, st.active_tiles);
    // Their products wait for the next publication; they were published
    // above in the same version, so nothing is written again.
    OST_Map_ProductsStats_t after;
    ASSERT_EQ(OST_Map_ProductsGetStats(p.h, &after), OST_SUCCESS);
    EXPECT_EQ(after.deferred_tiles, st.active_tiles);
    EXPECT_EQ(after.memory_bytes, st.memory_bytes);
    ASSERT_EQ(OST_Map_ProductsPublish(p.h, &wr), OST_SUCCESS);
    EXPECT_EQ(wr.tiles, 0u);
    ASSERT_EQ(OST_Map_ProductsGetStats(p.h, &after), OST_SUCCESS);
    EXPECT_EQ(after.deferred_tiles, 0u);
    EXPECT_EQ(after.memory_bytes, 0u);

    // A later keyframe over the same tiles reloads them.
    ASSERT_EQ(OST_Map_ProductsAddPoints(p.h, &set, touched.data(), 16, &fr), OST_SUCCESS);
    ASSERT_EQ(OST_Map_ProductsFinaliseAll(p.h, &wr), OST_SUCCESS);
    EXPECT_EQ(wr.tiles, fr.tile_count);
    EXPECT_EQ(wr.active_tiles, 0u);

    // Errors keep their text.
    OST_Map_Pose_t bad = pose;
    bad.q_ecef_from_camera[0] = 3.0;
    EXPECT_EQ(OST_Map_ProductsRender(p.h, &img, &cam, &bad, 2000.0, nullptr, nullptr, 0, &rr),
              OST_ERR_INVALID_ARGUMENT);
    EXPECT_NE(std::string(OST_Map_ProductsLastError(p.h)).find("quaternion"), std::string::npos);
}

// Point set of the flat scene through the georeferencing interface.
struct FlatPointSet
{
    std::vector<OST_Map_Point_t>    points;
    std::vector<OST_Map_TileSpan_t> spans = std::vector<OST_Map_TileSpan_t>(16);
    OST_Map_GeorefResult_t          r {};
    OST_Map_Pose_t                  pose {};

    explicit FlatPointSet(const Scene &s)
    {
        OST_Map_Georef_t      *g = nullptr;
        OST_Map_GeorefConfig_t gc;
        OST_Map_GeorefDefaultConfig(&gc);
        gc.projected_crs = "EPSG:32632";
        gc.proj_data_dir = OST_MP_TEST_PROJ_DATA_DIR;
        gc.cell_size_m   = 2.0;
        gc.tile_cells    = 64;
        EXPECT_EQ(OST_Map_GeorefCreate(&gc, &g), OST_SUCCESS);
        OST_Map_DepthMap_t map {};
        map.camera     = { s.camera.width, s.camera.height, s.camera.fx, s.camera.fy, s.camera.cx, s.camera.cy };
        map.depth      = s.depth.data();
        map.confidence = s.confidence.data();
        const Eigen::Quaterniond q(s.T_E_C.linear());
        pose.q_ecef_from_camera[0] = q.w(), pose.q_ecef_from_camera[1] = q.x();
        pose.q_ecef_from_camera[2] = q.y(), pose.q_ecef_from_camera[3] = q.z();
        for (int i = 0; i < 3; ++i)
        {
            pose.position_ecef[i] = s.T_E_C.translation()[i];
        }
        points.resize(s.depth.size());
        EXPECT_EQ(OST_Map_GeorefConvert(g, &map, nullptr, &pose, points.data(), static_cast<uint32_t>(points.size()),
                                        spans.data(), 16, &r),
                  OST_SUCCESS);
        OST_Map_GeorefDestroy(g);
    }
    OST_Map_PointSet_t set() const
    {
        return { points.data(), r.points, r.tile_count, spans.data(), { r.origin[0], r.origin[1], r.origin[2] } };
    }
};

TEST(MapProductsCApi, SnapshotsAreWrittenOnAnotherTaskWithoutReplacingNewerTiles)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    FlatPointSet                 flat(flatScene(conv));
    Products                     p("ost_map_products_snapshot");
    ASSERT_NE(p.h, nullptr);
    const OST_Map_PointSet_t set = flat.set();
    OST_Map_FusionResult_t   fr;
    OST_Map_WriteResult_t    wr;

    // Nothing changed: no snapshot.
    OST_Map_Snapshot_t *snap  = nullptr;
    uint32_t            tiles = 0;
    ASSERT_EQ(OST_Map_ProductsSnapshot(p.h, &snap, &tiles), OST_SUCCESS);
    EXPECT_EQ(snap, nullptr);

    // A snapshot written on another task while the owner keeps fusing.
    ASSERT_EQ(OST_Map_ProductsAddPoints(p.h, &set, nullptr, 0, &fr), OST_SUCCESS);
    ASSERT_EQ(OST_Map_ProductsSnapshot(p.h, &snap, &tiles), OST_SUCCESS);
    ASSERT_NE(snap, nullptr);
    EXPECT_EQ(tiles, fr.tile_count);
    OST_Status_t written = OST_ERR_INTERNAL;
    std::thread  publisher([&] { written = OST_Map_ProductsWriteSnapshot(p.h, snap, &wr); });
    for (int i = 0; i < 3; ++i)
    {
        ASSERT_EQ(OST_Map_ProductsAddPoints(p.h, &set, nullptr, 0, &fr), OST_SUCCESS);
    }
    publisher.join();
    ASSERT_EQ(written, OST_SUCCESS) << OST_Map_ProductsPublishError(p.h);
    EXPECT_EQ(wr.tiles, tiles);
    EXPECT_TRUE(fs::exists(p.dir / "dsm.vrt"));

    // An older copy taken before the tiles were finalised is not written
    // over the finalised files.
    ASSERT_EQ(OST_Map_ProductsSnapshot(p.h, &snap, &tiles), OST_SUCCESS);
    ASSERT_NE(snap, nullptr);
    ASSERT_EQ(OST_Map_ProductsAddPoints(p.h, &set, nullptr, 0, &fr), OST_SUCCESS);
    ASSERT_EQ(OST_Map_ProductsFinaliseAll(p.h, &wr), OST_SUCCESS);
    EXPECT_EQ(wr.tiles, fr.tile_count);
    std::map<std::string, fs::file_time_type> stamps;
    for (const auto &e : fs::directory_iterator(p.dir / "dsm"))
    {
        stamps[e.path().string()] = fs::last_write_time(e.path());
    }
    ASSERT_EQ(OST_Map_ProductsWriteSnapshot(p.h, snap, &wr), OST_SUCCESS);
    EXPECT_EQ(wr.tiles, 0u);
    for (const auto &[path, stamp] : stamps)
    {
        EXPECT_EQ(fs::last_write_time(path), stamp) << path;
    }

    // NULL snapshot: nothing to do.
    EXPECT_EQ(OST_Map_ProductsWriteSnapshot(p.h, nullptr, &wr), OST_SUCCESS);
}

TEST(MapProductsCApi, CreateChecksTheConfiguration)
{
    OST_Map_ProductsConfig_t c;
    OST_Map_Products_t      *h = nullptr;
    OST_Map_ProductsDefaultConfig(&c);
    c.proj_data_dir = OST_MP_TEST_PROJ_DATA_DIR;
    EXPECT_EQ(OST_Map_ProductsCreate(&c, &h), OST_ERR_INVALID_ARGUMENT);  // no output directory
    const std::string out = (fs::path(::testing::TempDir()) / "ost_map_products_bad").string();
    c.output_dir          = out.c_str();
    c.projected_crs       = "EPSG:999999";
    EXPECT_EQ(OST_Map_ProductsCreate(&c, &h), OST_ERR_IO);
    EXPECT_EQ(h, nullptr);
}

}  // namespace
}  // namespace orthostream::map_products
