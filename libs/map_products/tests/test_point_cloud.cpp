#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "laz_reader.hpp"
#include "orthostream/map_products/point_cloud.hpp"
#include "test_support.hpp"

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;
using test::flatScene;
using test::hasKey;
using test::LazFile;
using test::LazPoint;
using test::readLaz;
using test::Scene;

TEST(PointCloudWriter, WritesOneFilePerTile)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:32632", OST_MP_TEST_PROJ_DATA_DIR);
    const Scene                  s   = flatScene(conv);
    const fs::path               dir = fs::path(::testing::TempDir()) / "ost_map_products_points";
    fs::remove_all(dir);
    PointCloudWriterConfig config;
    config.directory       = dir.string();
    config.projected_crs   = "EPSG:32632";
    config.proj_data_dir   = OST_MP_TEST_PROJ_DATA_DIR;
    config.grid.cell_size  = 2.0;
    config.grid.tile_cells = 64;  // 128 m tiles
    PointCloudWriter writer(config, frame);

    const std::vector<std::uint8_t> img(static_cast<std::size_t>(3 * 80 * 60), 0);
    std::vector<std::uint8_t>       coloured = img;
    for (std::size_t i = 0; i < coloured.size(); i += 3)
    {
        coloured[i] = 10, coloured[i + 1] = 20, coloured[i + 2] = 30;
    }
    const PointCloudStats stats =
        writer.write(70001, { s.depth.data(), s.confidence.data() }, s.camera, { coloured.data(), 80, 60, 240 },
                     s.camera, s.T_E_C);
    EXPECT_EQ(stats.points, s.depth.size());
    EXPECT_GE(stats.files.size(), 4u);  // a 333 m x 250 m footprint over 128 m tiles

    std::size_t total = 0;
    const double T = 128.0;
    for (const std::string &path : stats.files)
    {
        EXPECT_FALSE(fs::exists(path + ".tmp"));
        const LazFile f = readLaz(path);
        EXPECT_TRUE(hasKey(f.geokeys, 3072, 32632));
        EXPECT_FALSE(hasKey(f.geokeys, 4096, 5729));
        ASSERT_FALSE(f.points.empty());
        const double west = std::floor(f.points[0].x / T) * T, south = std::floor(f.points[0].y / T) * T;
        for (const LazPoint &p : f.points)
        {
            EXPECT_NEAR(p.z, 2000.0, 0.003);  // ellipsoidal, millimetre resolution
            EXPECT_GE(p.x, west);
            EXPECT_LT(p.x, west + T);
            EXPECT_GE(p.y, south);
            EXPECT_LT(p.y, south + T);
            EXPECT_EQ(p.intensity, 49151);  // 0.75 x 65535
            EXPECT_EQ(p.source, 70001 & 0xFFFF);
            EXPECT_EQ(p.r, 10 * 257);
            EXPECT_EQ(p.b, 30 * 257);
        }
        EXPECT_LE(f.min_x, f.max_x);
        total += f.points.size();
    }
    EXPECT_EQ(total, s.depth.size());
}

TEST(PointCloudWriter, OrthometricHeights)
{
#ifndef OST_MP_TEST_GRID_DIR
    GTEST_SKIP() << "needs the LHN95 geoid grid (scripts/fetch_proj_grids.sh)";
#else
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:2056", OST_MP_TEST_PROJ_DATA_DIR, { OST_MP_TEST_GRID_DIR });
    const Scene                  s   = flatScene(conv);
    const fs::path               dir = fs::path(::testing::TempDir()) / "ost_map_products_points_lhn95";
    fs::remove_all(dir);
    PointCloudWriterConfig config;
    config.directory     = dir.string();
    config.projected_crs = "EPSG:2056";
    config.vertical_crs  = "EPSG:5729";
    config.proj_data_dir = OST_MP_TEST_PROJ_DATA_DIR;
    config.grid_dirs     = { OST_MP_TEST_GRID_DIR };
    config.min_confidence = 0.8f;  // above the scene's 0.75: nothing
    {
        PointCloudWriter writer(config, frame);
        const std::vector<std::uint8_t> img(static_cast<std::size_t>(3 * 80 * 60), 0);
        EXPECT_TRUE(writer.write(1, { s.depth.data(), s.confidence.data() }, s.camera, { img.data(), 80, 60, 240 },
                                 s.camera, s.T_E_C)
                        .files.empty());
    }
    config.min_confidence = 0.0f;
    PointCloudWriter                writer(config, frame);
    const std::vector<std::uint8_t> img(static_cast<std::size_t>(3 * 80 * 60), 0);
    const PointCloudStats stats = writer.write(2, { s.depth.data(), s.confidence.data() }, s.camera,
                                               { img.data(), 80, 60, 240 }, s.camera, s.T_E_C);
    ASSERT_FALSE(stats.files.empty());
    const LazFile f = readLaz(stats.files.front());
    EXPECT_TRUE(hasKey(f.geokeys, 3072, 2056));
    EXPECT_TRUE(hasKey(f.geokeys, 4096, 5729));
    // The same exact LHN95 height as the DSM writer test: about 50 m below.
    const geo::CrsTransformer to_lhn95("EPSG:4978", "EPSG:2056+5729", OST_MP_TEST_PROJ_DATA_DIR, { OST_MP_TEST_GRID_DIR });
    for (std::size_t i = 0; i < f.points.size(); i += 97)
    {
        const LazPoint &p     = f.points[i];
        const double    exact = to_lhn95.forward(frame.toEcef({ p.x, p.y, 2000.0 })).z();
        EXPECT_NEAR(p.z, exact, 0.003);
    }
    EXPECT_NEAR(f.points.front().z, 1950.0, 5.0);
#endif
}

}  // namespace
}  // namespace orthostream::map_products
