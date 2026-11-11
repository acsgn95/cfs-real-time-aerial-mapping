#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "orthostream/map_products/dsm.hpp"
#include "test_support.hpp"

namespace orthostream::map_products
{
namespace
{

using test::dorfberg;
using test::flatScene;
using test::Scene;

TEST(DsmTile, WeightedMeanAndVariance)
{
    DsmFusionConfig config;
    config.min_observations = 100;  // no rejection here
    DsmTile tile({ 3, -2 }, config.grid, 2000.0);
    const double h[] = { 2001.0, 2003.0, 2002.0 };
    const float  w[] = { 1.0f, 0.5f, 0.5f };
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(tile.add(10, 20, h[i], w[i], config), DsmTile::Update::added);
    }
    // Weighted mean (1*2001 + 0.5*2003 + 0.5*2002) / 2 = 2001.75.
    EXPECT_NEAR(*tile.height(10, 20), 2001.75, 1e-4);
    // Weighted variance: (1*0.5625 + 0.5*1.5625 + 0.5*0.0625) / 2 = 0.6875.
    EXPECT_NEAR(*tile.variance(10, 20), 0.6875, 1e-4);
    EXPECT_EQ(tile.observations(10, 20), 3);
    EXPECT_FALSE(tile.height(11, 20));
    EXPECT_EQ(tile.filledCells(), 1u);
    // Tile (3, -2) of 512 x 0.26 m: north-west corner.
    EXPECT_DOUBLE_EQ(tile.originEasting(), 3 * 133.12);
    EXPECT_DOUBLE_EQ(tile.originNorthing(), -1 * 133.12);
}

TEST(DsmTile, RejectsOutliersAfterEnoughObservations)
{
    DsmFusionConfig config;  // 3 observations, 3 sigma, sigma at least 0.25 m
    DsmTile         tile({ 0, 0 }, config.grid, 1500.0);
    EXPECT_EQ(tile.add(0, 0, 1500.0, 1.0f, config), DsmTile::Update::added);
    EXPECT_EQ(tile.add(0, 0, 1530.0, 1.0f, config), DsmTile::Update::added);  // too few to judge
    EXPECT_EQ(tile.add(0, 0, 1500.2, 1.0f, config), DsmTile::Update::added);
    // Mean 1510.07, sigma about 14.1 m: 1600 is more than 3 sigma away.
    EXPECT_EQ(tile.add(0, 0, 1600.0, 1.0f, config), DsmTile::Update::rejected);
    EXPECT_EQ(tile.add(0, 0, 1515.0, 1.0f, config), DsmTile::Update::added);
    EXPECT_EQ(tile.observations(0, 0), 4);
}

TEST(DsmFusion, FlatSurfaceAcrossTiles)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:2056", OST_MP_TEST_PROJ_DATA_DIR);
    const Scene                  s = flatScene(conv);
    DsmFusionConfig              config;
    // Pixels are about 4.2 m on the ground; 10 m cells receive several each.
    config.grid.cell_size  = 10.0;
    config.grid.tile_cells = 8;  // 80 m tiles: the 333 m x 250 m footprint spans many
    DsmFusion fusion(config, frame);

    const FusionStats stats = fusion.addDepthMap({ s.depth.data(), s.confidence.data() }, s.camera, s.T_E_C);
    EXPECT_EQ(stats.points, s.depth.size());
    EXPECT_EQ(stats.added, s.depth.size());
    EXPECT_EQ(stats.rejected, 0u);
    EXPECT_GE(stats.tiles.size(), 20u);

    double max_err = 0.0;
    for (const auto &[id, tile] : fusion.tiles())
    {
        for (int r = 0; r < tile->cells(); ++r)
        {
            for (int c = 0; c < tile->cells(); ++c)
            {
                if (const auto hgt = tile->height(c, r))
                {
                    max_err = std::max(max_err, std::abs(*hgt - 2000.0));
                    EXPECT_NEAR(tile->weight(c, r), 0.75f * tile->observations(c, r), 1e-5);
                }
            }
        }
    }
    EXPECT_LT(max_err, 0.002);
    // The point below the camera.
    const Eigen::Vector3d below = frame.fromEcef(conv.geodeticToEcef(dorfberg(2000.0)));
    ASSERT_TRUE(fusion.heightAt(below.x(), below.y()));
    EXPECT_NEAR(*fusion.heightAt(below.x(), below.y()), 2000.0, 0.002);
    EXPECT_FALSE(fusion.heightAt(below.x() + 1000.0, below.y()));
}

TEST(DsmFusion, SecondMapRejectsGrossErrors)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:2056", OST_MP_TEST_PROJ_DATA_DIR);
    Scene                        s = flatScene(conv);
    DsmFusionConfig              config;
    config.grid.cell_size  = 10.0;  // many pixels per cell
    config.grid.tile_cells = 64;
    DsmFusion fusion(config, frame);
    fusion.addDepthMap({ s.depth.data(), s.confidence.data() }, s.camera, s.T_E_C);

    // The same view again with every 10th pixel 30 m too close.
    for (std::size_t i = 0; i < s.depth.size(); i += 10)
    {
        s.depth[i] -= 30.0f;
    }
    const FusionStats stats = fusion.addDepthMap({ s.depth.data(), s.confidence.data() }, s.camera, s.T_E_C);
    EXPECT_GT(stats.rejected, s.depth.size() / 20);
    const Eigen::Vector3d below = frame.fromEcef(conv.geodeticToEcef(dorfberg(2000.0)));
    EXPECT_NEAR(*fusion.heightAt(below.x(), below.y()), 2000.0, 0.01);

    // Invalid pixels (depth 0) and pixels without confidence are skipped.
    std::vector<float> none(s.depth.size(), 0.0f);
    EXPECT_EQ(fusion.addDepthMap({ none.data(), s.confidence.data() }, s.camera, s.T_E_C).points, 0u);
    EXPECT_EQ(fusion.addDepthMap({ s.depth.data(), none.data() }, s.camera, s.T_E_C).points, 0u);
}

}  // namespace
}  // namespace orthostream::map_products
