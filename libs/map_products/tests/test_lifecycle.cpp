#include <filesystem>
#include <vector>

#include <gtest/gtest.h>

#include "orthostream/map_products/lifecycle.hpp"
#include "test_support.hpp"

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;
using test::flatScene;
using test::Scene;

// A short flight to the east with a small camera (83 m x 62 m footprint)
// over 64 m tiles.
struct Flight
{
    geo::GeodeticConverter     conv { OST_MP_TEST_PROJ_DATA_DIR };
    ProductFrame               frame { "EPSG:32632", OST_MP_TEST_PROJ_DATA_DIR };
    fs::path                   dir;
    DsmFusionConfig            config;
    std::unique_ptr<DsmFusion> dsm;
    std::unique_ptr<OrthoRenderer> ortho;
    std::unique_ptr<DsmWriter>     dsm_writer;
    std::unique_ptr<OrthoWriter>   ortho_writer;
    std::unique_ptr<TileManager>   manager;
    std::vector<std::uint8_t>      image = std::vector<std::uint8_t>(3 * 40 * 30, 90);

    explicit Flight(const std::string &name, std::size_t budget)
    {
        dir = fs::path(::testing::TempDir()) / ("ost_map_products_" + name);
        fs::remove_all(dir);
        config.grid.cell_size  = 4.0;
        config.grid.tile_cells = 16;
        dsm                    = std::make_unique<DsmFusion>(config, frame);
        ortho                  = std::make_unique<OrthoRenderer>(OrthoConfig { 2.0 }, *dsm, frame);
        dsm_writer = std::make_unique<DsmWriter>(DsmWriterConfig { (dir / "products").string(), "EPSG:32632", "",
                                                                   OST_MP_TEST_PROJ_DATA_DIR, {} },
                                                 frame);
        ortho_writer = std::make_unique<OrthoWriter>(
            OrthoWriterConfig { (dir / "products").string(), "EPSG:32632", OST_MP_TEST_PROJ_DATA_DIR });
        TileLifecycleConfig lc;
        lc.max_active_tiles  = budget;
        lc.finalise_distance = 150.0;
        lc.state_directory   = (dir / "state").string();
        manager              = std::make_unique<TileManager>(lc, *dsm, *ortho, *dsm_writer, *ortho_writer);
    }

    Scene scene(double east) { return flatScene(conv, 40, 30, 120.0, east, 0.0); }

    // One keyframe: fuse, render, update the lifecycle.
    std::vector<TileId> keyframe(const Scene &s)
    {
        const FusionStats fused = dsm->addDepthMap({ s.depth.data(), s.confidence.data() }, s.camera, s.T_E_C);
        const RenderStats drawn =
            ortho->render({ image.data(), 40, 30, 120 }, s.camera, s.T_E_C, fused.extent);
        manager->markChanged(fused.tiles);
        manager->markChanged(drawn.tiles);
        return manager->update(frame.fromEcef(s.T_E_C.translation()).head<2>());
    }
};

TEST(TileManager, KeepsTheBudgetAndWritesFinalisedTiles)
{
    Flight              f("lifecycle", 6);
    std::vector<TileId> all_finalised;
    for (int k = 0; k < 8; ++k)
    {
        const std::vector<TileId> done = f.keyframe(f.scene(60.0 * k));
        all_finalised.insert(all_finalised.end(), done.begin(), done.end());
        EXPECT_LE(f.manager->activeTiles(), 6u) << "keyframe " << k;
    }
    ASSERT_FALSE(all_finalised.empty());
    std::size_t ortho_files = 0;
    for (const TileId &id : all_finalised)
    {
        if (f.manager->state(id) == TileState::written)
        {
            EXPECT_TRUE(fs::exists(f.dir / "products" / "dsm" / DsmWriter::tileFileName(id)));
            // Edge tiles may have DSM cells but no orthomosaic pixels: a pixel
            // needs heights at all four surrounding cell corners.
            ortho_files += fs::exists(f.dir / "products" / "ortho" / OrthoWriter::tileFileName(id)) ? 1 : 0;
        }
    }
    EXPECT_GT(ortho_files, 0u);
    EXPECT_TRUE(fs::exists(f.dir / "products" / "dsm.vrt"));

    const std::size_t remaining = f.manager->activeTiles();
    EXPECT_EQ(f.manager->finaliseAll(), remaining);
    EXPECT_EQ(f.manager->activeTiles(), 0u);
    EXPECT_TRUE(f.dsm->tiles().empty());
    EXPECT_TRUE(f.ortho->tiles().empty());
}

TEST(TileManager, ReloadsAWrittenTileWhenTheVehicleReturns)
{
    Flight      f("lifecycle_return", 4);
    const Scene start = f.scene(0.0);
    f.keyframe(start);
    const Eigen::Vector3d below = f.frame.fromEcef(f.conv.geodeticToEcef(test::dorfberg(2000.0)));
    const TileId          id { static_cast<std::int64_t>(std::floor(below.x() / 64.0)),
                      static_cast<std::int64_t>(std::floor(below.y() / 64.0)) };
    const DsmTile        *tile = f.dsm->tile(id);
    ASSERT_NE(tile, nullptr);
    const int         col = static_cast<int>((below.x() - tile->originEasting()) / 4.0);
    const int         row = static_cast<int>((tile->originNorthing() - below.y()) / 4.0);
    const std::uint16_t first_pass = tile->observations(col, row);
    ASSERT_GT(first_pass, 0);

    for (int k = 1; k <= 6; ++k)  // fly away until the first tiles are written
    {
        f.keyframe(f.scene(80.0 * k));
    }
    ASSERT_EQ(f.manager->state(id), TileState::written);
    ASSERT_EQ(f.dsm->tile(id), nullptr);

    f.keyframe(start);  // back over the start
    ASSERT_EQ(f.manager->state(id), TileState::active);
    EXPECT_EQ(f.dsm->tile(id)->observations(col, row), 2 * first_pass);
    EXPECT_LT(f.ortho->tile(id)->angle(col * 2, row * 2), OrthoTile::kEmpty);  // ortho state kept
}

TEST(TileManager, PublishesOnlyChangedTiles)
{
    Flight f("lifecycle_publish", 100);
    f.keyframe(f.scene(0.0));
    const std::size_t first = f.manager->publish();
    EXPECT_GT(first, 0u);
    EXPECT_TRUE(fs::exists(f.dir / "products" / "dsm.vrt"));
    EXPECT_TRUE(fs::exists(f.dir / "products" / "ortho.vrt"));
    EXPECT_EQ(f.manager->publish(), 0u);  // nothing changed since
    EXPECT_EQ(f.manager->activeTiles(), first);  // published tiles stay active
}

TEST(TileManager, RejectsAnInvalidConfiguration)
{
    Flight              f("lifecycle_bad", 4);
    TileLifecycleConfig lc;
    EXPECT_THROW(TileManager(lc, *f.dsm, *f.ortho, *f.dsm_writer, *f.ortho_writer), MapProductsError);  // no directory
}

}  // namespace
}  // namespace orthostream::map_products
