#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "laz_reader.hpp"
#include "map_products_c_api.h"
#include "test_support.hpp"

extern "C" int ost_map_c_api_usage(void);

namespace
{

namespace fs  = std::filesystem;
namespace geo = orthostream::geo;
namespace mpt = orthostream::map_products::test;

TEST(MapCApiUsage, CompilesAsC99WithTheDefaultsOfAdr007)
{
    EXPECT_EQ(ost_map_c_api_usage(), 0);
}

struct Fixture
{
    mpt::Scene          scene;
    OST_Map_DepthMap_t  map {};
    OST_Map_Pose_t      pose {};
    std::vector<std::uint8_t> image;

    Fixture()
    {
        const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
        scene             = mpt::flatScene(conv);
        map.camera.width  = scene.camera.width;
        map.camera.height = scene.camera.height;
        map.camera.fx     = scene.camera.fx;
        map.camera.fy     = scene.camera.fy;
        map.camera.cx     = scene.camera.cx;
        map.camera.cy     = scene.camera.cy;
        map.depth         = scene.depth.data();
        map.confidence    = scene.confidence.data();
        const Eigen::Quaterniond q(scene.T_E_C.linear());
        pose.q_ecef_from_camera[0] = q.w();
        pose.q_ecef_from_camera[1] = q.x();
        pose.q_ecef_from_camera[2] = q.y();
        pose.q_ecef_from_camera[3] = q.z();
        for (int i = 0; i < 3; ++i)
        {
            pose.position_ecef[i] = scene.T_E_C.translation()[i];
        }
        image.assign(scene.depth.size() * 3, 50);
    }
};

OST_Map_GeorefConfig_t config(const std::string &out)
{
    OST_Map_GeorefConfig_t c;
    OST_Map_GeorefDefaultConfig(&c);
    c.projected_crs = "EPSG:32632";
    c.vertical_crs  = "";
    c.proj_data_dir = OST_MP_TEST_PROJ_DATA_DIR;
    c.output_dir    = out.c_str();
    c.cell_size_m   = 2.0;
    c.tile_cells    = 64;
    return c;
}

TEST(MapCApi, CreateChecksTheConfiguration)
{
    OST_Map_Georef_t      *h = nullptr;
    OST_Map_GeorefConfig_t c = config("");
    c.tile_cells             = 0;
    EXPECT_EQ(OST_Map_GeorefCreate(&c, &h), OST_ERR_INVALID_ARGUMENT);
    c                = config("");
    c.min_confidence = 1.5;
    EXPECT_EQ(OST_Map_GeorefCreate(&c, &h), OST_ERR_INVALID_ARGUMENT);
    c               = config("");
    c.projected_crs = "EPSG:999999";
    EXPECT_EQ(OST_Map_GeorefCreate(&c, &h), OST_ERR_IO);
    EXPECT_EQ(h, nullptr);
}

TEST(MapCApi, ConvertsADepthMapAndWritesItsPointCloud)
{
    const fs::path dir = fs::path(::testing::TempDir()) / "ost_map_c_api";
    fs::remove_all(dir);
    const std::string      out = dir.string();
    const OST_Map_GeorefConfig_t c = config(out);
    OST_Map_Georef_t      *h   = nullptr;
    ASSERT_EQ(OST_Map_GeorefCreate(&c, &h), OST_SUCCESS);
    Fixture f;

    uint32_t count = 0;
    ASSERT_EQ(OST_Map_GeorefCount(h, &f.map, &count), OST_SUCCESS);
    ASSERT_EQ(count, f.scene.depth.size());

    std::vector<OST_Map_Point_t>    points(count);
    std::vector<OST_Map_TileSpan_t> tiles(16);
    OST_Map_GeorefResult_t          r;
    const OST_Map_Image_t           img { f.image.data(), static_cast<size_t>(3 * f.map.camera.width), 3 };
    ASSERT_EQ(OST_Map_GeorefConvert(h, &f.map, &img, &f.pose, points.data(), count, tiles.data(), 16, &r),
              OST_SUCCESS)
        << OST_Map_GeorefLastError(h);
    EXPECT_EQ(r.points, count);
    EXPECT_GE(r.tile_count, 4u);
    EXPECT_NEAR(r.height_min, 2000.0, 0.01);
    EXPECT_NEAR(r.origin[2], 2250.0, 0.6);  // the camera centre, rounded
    uint32_t total = 0;
    for (uint32_t k = 0; k < r.tile_count; ++k)
    {
        EXPECT_EQ(tiles[k].first, total);
        total += tiles[k].count;
    }
    EXPECT_EQ(total, count);
    EXPECT_EQ(points[0].rgb[1], 50);
    EXPECT_EQ(points[0].weight, 0.75f);

    const OST_Map_PointSet_t set { points.data(), r.points, r.tile_count, tiles.data(),
                                   { r.origin[0], r.origin[1], r.origin[2] } };
    OST_Map_LazResult_t      laz;
    ASSERT_EQ(OST_Map_GeorefWriteLaz(h, 9, &set, &laz), OST_SUCCESS) << OST_Map_GeorefLastError(h);
    EXPECT_EQ(laz.files, r.tile_count);
    EXPECT_EQ(laz.points, count);
    std::size_t in_files = 0;
    for (const auto &e : fs::directory_iterator(dir / "points"))
    {
        EXPECT_EQ(e.path().extension(), ".laz");
        const mpt::LazFile file = mpt::readLaz(e.path().string());
        in_files += file.points.size();
        EXPECT_NEAR(file.points.front().z, 2000.0, 0.003);
    }
    EXPECT_EQ(in_files, count);

    // Errors keep their text.
    OST_Map_Pose_t bad = f.pose;
    bad.q_ecef_from_camera[0] *= 2.0;
    EXPECT_EQ(OST_Map_GeorefConvert(h, &f.map, &img, &bad, points.data(), count, tiles.data(), 16, &r),
              OST_ERR_INVALID_ARGUMENT);
    EXPECT_NE(std::string(OST_Map_GeorefLastError(h)).find("quaternion"), std::string::npos);
    EXPECT_EQ(OST_Map_GeorefConvert(h, &f.map, &img, &f.pose, points.data(), count - 1, tiles.data(), 16, &r),
              OST_ERR_INVALID_ARGUMENT);
    EXPECT_NE(std::string(OST_Map_GeorefLastError(h)).find("capacity"), std::string::npos);
    EXPECT_EQ(OST_Map_GeorefConvert(h, &f.map, &img, &f.pose, points.data(), count, tiles.data(), 2, &r),
              OST_ERR_INVALID_ARGUMENT);
    OST_Map_TileSpan_t       wrong = tiles[0];
    wrong.count                    = count + 1;
    const OST_Map_PointSet_t broken { points.data(), count, 1, &wrong, { r.origin[0], r.origin[1], r.origin[2] } };
    EXPECT_EQ(OST_Map_GeorefWriteLaz(h, 9, &broken, &laz), OST_ERR_INVALID_ARGUMENT);
    OST_Map_GeorefDestroy(h);

    // Without an output directory there is no point cloud.
    const OST_Map_GeorefConfig_t none = config("");
    ASSERT_EQ(OST_Map_GeorefCreate(&none, &h), OST_SUCCESS);
    EXPECT_EQ(OST_Map_GeorefWriteLaz(h, 9, &set, &laz), OST_ERR_UNSUPPORTED);
    OST_Map_GeorefDestroy(h);
}

}  // namespace
