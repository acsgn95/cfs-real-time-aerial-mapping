#include <cmath>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "laz_reader.hpp"
#include "orthostream/map_products/point_cloud.hpp"
#include "orthostream/map_products/point_set.hpp"
#include "test_support.hpp"

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;
using test::flatScene;
using test::Scene;

constexpr double kTile = 128.0;  // 2 m cells, 64 per tile

TileGridConfig grid()
{
    TileGridConfig g;
    g.cell_size  = 2.0;
    g.tile_cells = 64;
    return g;
}

// RGB image whose colour is the pixel position: red u, green v.
std::vector<std::uint8_t> pixelImage(const geo::CameraModel &cam)
{
    std::vector<std::uint8_t> img(static_cast<std::size_t>(3 * cam.width * cam.height));
    for (int v = 0; v < cam.height; ++v)
    {
        for (int u = 0; u < cam.width; ++u)
        {
            std::uint8_t *px = &img[static_cast<std::size_t>(3 * (v * cam.width + u))];
            px[0] = static_cast<std::uint8_t>(u), px[1] = static_cast<std::uint8_t>(v), px[2] = 7;
        }
    }
    return img;
}

TEST(PointSetBuilder, GivesTheExactPointsSortedByTile)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:32632", OST_MP_TEST_PROJ_DATA_DIR);
    Scene                        s = flatScene(conv);
    s.depth[5]                     = 0.0f;  // no depth
    s.confidence[6]                = 0.0f;  // no confidence
    PointSetBuilder builder(frame, grid(), 0.0);
    ASSERT_EQ(builder.count({ s.depth.data(), s.confidence.data() }, s.camera), s.depth.size() - 2);

    const std::vector<std::uint8_t> img = pixelImage(s.camera);
    std::vector<PointRecord>        points(s.depth.size());
    std::vector<TileSpan>           tiles(16);
    const PointSetStats stats = builder.build({ s.depth.data(), s.confidence.data() }, s.camera,
                                              { img.data(), static_cast<std::size_t>(3 * s.camera.width), 3 },
                                              s.T_E_C, points.data(), points.size(), tiles.data(), tiles.size());
    ASSERT_EQ(stats.points, s.depth.size() - 2);
    EXPECT_GE(stats.tiles, 4u);  // a 333 m x 250 m footprint over 128 m tiles
    const Eigen::Vector3d centre = frame.fromEcef(s.T_E_C.translation());
    EXPECT_EQ(stats.origin, Eigen::Vector3d(std::round(centre.x()), std::round(centre.y()), std::round(centre.z())));
    EXPECT_NEAR(stats.height_min, 2000.0, 0.01);
    EXPECT_NEAR(stats.height_max, 2000.0, 0.01);

    std::uint32_t next = 0;
    for (std::size_t k = 0; k < stats.tiles; ++k)
    {
        const TileSpan &t = tiles[k];
        EXPECT_EQ(t.first, next);
        next += t.count;
        if (k > 0)
        {
            EXPECT_TRUE(tiles[k - 1].id < t.id);
        }
        int last_pixel = -1;
        for (std::uint32_t j = t.first; j < t.first + t.count; ++j)
        {
            const PointRecord &r = points[j];
            const int          u = r.rgb[0], v = r.rgb[1];
            EXPECT_EQ(r.rgb[2], 7);
            EXPECT_GT(v * s.camera.width + u, last_pixel);  // pixel order within a tile
            last_pixel = v * s.camera.width + u;
            const std::size_t     i = static_cast<std::size_t>(v * s.camera.width + u);
            const Eigen::Vector3d p_C(s.depth[i] * (u - s.camera.cx) / s.camera.fx,
                                      s.depth[i] * (v - s.camera.cy) / s.camera.fy, s.depth[i]);
            const Eigen::Vector3d exact = frame.fromEcef(s.T_E_C * p_C);
            const Eigen::Vector3d got   = stats.origin + Eigen::Vector3d(r.east, r.north, r.height);
            EXPECT_LT((got - exact).norm(), 2e-4);  // interpolation and float storage
            EXPECT_EQ(std::floor(got.x() / kTile), t.id.ix);
            EXPECT_EQ(std::floor(got.y() / kTile), t.id.iy);
            EXPECT_EQ(r.weight, 0.75f);
        }
    }
    EXPECT_EQ(next, stats.points);
}

TEST(PointSetBuilder, GreyImagesMinimumConfidenceAndSmallBuffers)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:32632", OST_MP_TEST_PROJ_DATA_DIR);
    const Scene                  s = flatScene(conv);
    const std::vector<std::uint8_t> grey(s.depth.size(), 99);
    std::vector<PointRecord>        points(s.depth.size());
    std::vector<TileSpan>           tiles(16);
    const DepthMapView              map { s.depth.data(), s.confidence.data() };

    PointSetBuilder builder(frame, grid(), 0.0);
    builder.build(map, s.camera, { grey.data(), static_cast<std::size_t>(s.camera.width), 1 }, s.T_E_C,
                  points.data(), points.size(), tiles.data(), tiles.size());
    EXPECT_EQ(points[0].rgb[0], 99);
    EXPECT_EQ(points[0].rgb[2], 99);

    // Without an image the points are black.
    builder.build(map, s.camera, {}, s.T_E_C, points.data(), points.size(), tiles.data(), tiles.size());
    EXPECT_EQ(points[0].rgb[1], 0);

    EXPECT_THROW(builder.build(map, s.camera, {}, s.T_E_C, points.data(), points.size() - 1, tiles.data(),
                               tiles.size()),
                 MapProductsError);
    EXPECT_THROW(builder.build(map, s.camera, {}, s.T_E_C, points.data(), points.size(), tiles.data(), 1),
                 MapProductsError);

    PointSetBuilder strict(frame, grid(), 0.8);  // above the scene's 0.75
    EXPECT_EQ(strict.count(map, s.camera), 0u);
    EXPECT_EQ(strict.build(map, s.camera, {}, s.T_E_C, points.data(), points.size(), tiles.data(), tiles.size())
                  .points,
              0u);
}

// The point cloud of a point set equals the point cloud written straight
// from the depth map (the offline pipeline's path).
TEST(PointCloudWriter, WritesTheSameFilesFromAPointSet)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:32632", OST_MP_TEST_PROJ_DATA_DIR);
    const Scene                  s   = flatScene(conv);
    const fs::path               dir = fs::path(::testing::TempDir()) / "ost_map_products_point_set";
    fs::remove_all(dir);
    PointCloudWriterConfig config;
    config.projected_crs = "EPSG:32632";
    config.proj_data_dir = OST_MP_TEST_PROJ_DATA_DIR;
    config.grid          = grid();

    std::vector<std::uint8_t> img(s.depth.size() * 3);
    for (std::size_t i = 0; i < img.size(); ++i)
    {
        img[i] = static_cast<std::uint8_t>(i % 3 == 0 ? 10 : i % 3 == 1 ? 20 : 30);
    }
    const DepthMapView map { s.depth.data(), s.confidence.data() };

    config.directory = (dir / "depth").string();
    PointCloudWriter      from_depth(config, frame);
    const PointCloudStats a = from_depth.write(
        42, map, s.camera, { img.data(), s.camera.width, s.camera.height, static_cast<std::size_t>(3 * s.camera.width) },
        s.camera, s.T_E_C);

    PointSetBuilder          builder(frame, grid(), 0.0);
    std::vector<PointRecord> points(s.depth.size());
    std::vector<TileSpan>    tiles(16);
    const PointSetStats      st = builder.build(map, s.camera,
                                                { img.data(), static_cast<std::size_t>(3 * s.camera.width), 3 },
                                                s.T_E_C, points.data(), points.size(), tiles.data(), tiles.size());
    config.directory = (dir / "set").string();
    PointCloudWriter      from_set(config, frame);
    const PointCloudStats b = from_set.write(42, { points.data(), st.points, st.origin, tiles.data(), st.tiles });

    EXPECT_EQ(b.points, a.points);
    ASSERT_EQ(b.files.size(), a.files.size());
    for (std::size_t k = 0; k < a.files.size(); ++k)
    {
        EXPECT_EQ(fs::path(b.files[k]).filename(), fs::path(a.files[k]).filename());
        const test::LazFile fa = test::readLaz(a.files[k]);
        const test::LazFile fb = test::readLaz(b.files[k]);
        ASSERT_EQ(fb.points.size(), fa.points.size());
        for (std::size_t j = 0; j < fa.points.size(); ++j)
        {
            // Same pixel order; float storage moves a point by at most one millimetre step.
            EXPECT_NEAR(fb.points[j].x, fa.points[j].x, 0.0011);
            EXPECT_NEAR(fb.points[j].y, fa.points[j].y, 0.0011);
            EXPECT_NEAR(fb.points[j].z, fa.points[j].z, 0.0011);
            EXPECT_EQ(fb.points[j].intensity, fa.points[j].intensity);
            EXPECT_EQ(fb.points[j].source, 42);
            EXPECT_EQ(fb.points[j].r, fa.points[j].r);
            EXPECT_EQ(fb.points[j].b, fa.points[j].b);
        }
    }
}

}  // namespace
}  // namespace orthostream::map_products
