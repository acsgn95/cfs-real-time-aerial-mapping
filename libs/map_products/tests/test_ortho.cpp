#include <cmath>
#include <filesystem>
#include <vector>

#include <gdal.h>
#include <gtest/gtest.h>

#include "orthostream/map_products/ortho.hpp"
#include "orthostream/map_products/writer.hpp"
#include "test_support.hpp"

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;
using test::depthToHeight;
using test::flatScene;
using test::kPi;
using test::Scene;

// Smooth colour pattern on the ground (easting, northing in metres).
std::array<double, 3> groundColour(double e, double n)
{
    return { 128.0 + 100.0 * std::sin(2 * kPi * e / 40.0), 128.0 + 100.0 * std::cos(2 * kPi * n / 30.0),
             128.0 + 60.0 * std::sin(2 * kPi * (e + n) / 50.0) };
}

// Keyframe image of the flat surface: camera with distortion; each pixel
// shows the ground colour of the point it sees.
std::vector<std::uint8_t> renderKeyframe(const geo::GeodeticConverter &conv, const ProductFrame &frame,
                                         const geo::CameraModel &cam, const Eigen::Isometry3d &T_E_C)
{
    std::vector<std::uint8_t> img(static_cast<std::size_t>(3 * cam.width * cam.height));
    for (int v = 0; v < cam.height; ++v)
    {
        for (int u = 0; u < cam.width; ++u)
        {
            const Eigen::Vector2d n = *geo::undistortPixel(cam, Eigen::Vector2d(u, v));
            const Eigen::Vector3d ray(n.x(), n.y(), 1.0);
            const double          d = depthToHeight(conv, T_E_C, ray, 2000.0);
            const Eigen::Vector3d p = frame.fromEcef(T_E_C * (d * ray));
            const auto            c = groundColour(p.x(), p.y());
            for (int ch = 0; ch < 3; ++ch)
            {
                img[static_cast<std::size_t>(3 * (v * cam.width + u) + ch)] =
                    static_cast<std::uint8_t>(std::lround(std::clamp(c[static_cast<std::size_t>(ch)], 0.0, 255.0)));
            }
        }
    }
    return img;
}

struct Fixture
{
    geo::GeodeticConverter conv { OST_MP_TEST_PROJ_DATA_DIR };
    ProductFrame           frame { "EPSG:2056", OST_MP_TEST_PROJ_DATA_DIR };
    DsmFusionConfig        config;
    std::unique_ptr<DsmFusion> dsm;

    Fixture()
    {
        config.grid.cell_size  = 2.0;
        config.grid.tile_cells = 32;  // 64 m tiles
        dsm                    = std::make_unique<DsmFusion>(config, frame);
    }

    FusionStats fuse(const Scene &s) { return dsm->addDepthMap({ s.depth.data(), s.confidence.data() }, s.camera, s.T_E_C); }
};

geo::CameraModel distortedCamera()
{
    geo::CameraModel c;
    c.width  = 160;
    c.height = 120;
    c.fx = c.fy = 120.0;
    c.cx        = 80.3;
    c.cy        = 59.1;
    c.k1        = -0.08;
    c.k2        = 0.02;
    c.p1        = 0.001;
    return c;
}

TEST(OrthoRenderer, ColoursMatchTheGround)
{
    Fixture           f;
    const Scene       s     = flatScene(f.conv, 120, 90, 90.0);  // DSM source, pinhole
    const FusionStats fused = f.fuse(s);

    const geo::CameraModel          cam = distortedCamera();
    const std::vector<std::uint8_t> img = renderKeyframe(f.conv, f.frame, cam, s.T_E_C);
    OrthoRenderer                   renderer({ 0.5 }, *f.dsm, f.frame);
    EXPECT_EQ(renderer.tilePixels(), 128);
    const RenderStats stats =
        renderer.render({ img.data(), cam.width, cam.height, static_cast<std::size_t>(3 * cam.width) }, cam, s.T_E_C,
                        fused.extent);
    EXPECT_GT(stats.pixels, 100000u);  // most of a 333 m x 250 m footprint at 0.5 m
    EXPECT_FALSE(stats.tiles.empty());

    double sum_err = 0.0, max_err = 0.0;
    std::size_t n = 0;
    for (const auto &[id, tile] : renderer.tiles())
    {
        for (int r = 0; r < tile->pixels(); ++r)
        {
            for (int c = 0; c < tile->pixels(); ++c)
            {
                if (tile->angle(c, r) == OrthoTile::kEmpty)
                {
                    continue;
                }
                const double e   = tile->originEasting() + (c + 0.5) * tile->pixelSize();
                const double nn  = tile->originNorthing() - (r + 0.5) * tile->pixelSize();
                const auto   ref = groundColour(e, nn);
                for (int ch = 0; ch < 3; ++ch)
                {
                    const double err = std::abs(tile->rgb()[3 * tile->index(c, r) + static_cast<std::size_t>(ch)]
                                                - ref[static_cast<std::size_t>(ch)]);
                    sum_err += err;
                    max_err = std::max(max_err, err);
                    ++n;
                }
                EXPECT_LT(tile->angle(c, r), 4100);  // image corners: atan(100 / 120) = 39.8 degrees
            }
        }
    }
    ASSERT_GT(n, 0u);
    RecordProperty("mean_abs_colour_error", std::to_string(sum_err / static_cast<double>(n)));
    RecordProperty("max_abs_colour_error", std::to_string(max_err));
    EXPECT_LT(sum_err / static_cast<double>(n), 2.0);
    EXPECT_LT(max_err, 12.0);
}

TEST(OrthoRenderer, NadirMostSourceWinsInAnyOrder)
{
    Fixture     f;
    const Scene a = flatScene(f.conv, 80, 60, 60.0);
    const Scene b = flatScene(f.conv, 80, 60, 60.0, 120.0, 0.0);  // 120 m east
    f.fuse(a);
    const FusionStats fb = f.fuse(b);
    Eigen::AlignedBox2d extent = f.fuse(a).extent;
    extent.extend(fb.extent);

    const std::vector<std::uint8_t> img_a = renderKeyframe(f.conv, f.frame, a.camera, a.T_E_C);
    const std::vector<std::uint8_t> img_b(static_cast<std::size_t>(3 * 80 * 60), 7);  // uniform
    const RgbImageView              view_a { img_a.data(), 80, 60, 240 }, view_b { img_b.data(), 80, 60, 240 };

    const Eigen::Vector3d below_a = f.frame.fromEcef(f.conv.geodeticToEcef(test::dorfberg(2000.0)));
    const double          east_b  = below_a.x() + 150.0;  // seen by b only
    for (const bool a_first : { true, false })
    {
        OrthoRenderer r({ 1.0 }, *f.dsm, f.frame);
        if (a_first)
        {
            r.render(view_a, a.camera, a.T_E_C, extent);
            r.render(view_b, b.camera, b.T_E_C, extent);
        }
        else
        {
            r.render(view_b, b.camera, b.T_E_C, extent);
            r.render(view_a, a.camera, a.T_E_C, extent);
        }
        auto pixel = [&](double e, double n) -> std::pair<const OrthoTile *, std::size_t>
        {
            const double T  = 64.0;
            const TileId id { static_cast<std::int64_t>(std::floor(e / T)), static_cast<std::int64_t>(std::floor(n / T)) };
            const OrthoTile *t = r.tile(id);
            if (t == nullptr)
            {
                return { nullptr, 0 };
            }
            const int c = static_cast<int>((e - t->originEasting()) / t->pixelSize());
            const int w = static_cast<int>((t->originNorthing() - n) / t->pixelSize());
            return { t, t->index(c, w) };
        };
        const auto [ta, ia] = pixel(below_a.x(), below_a.y());
        ASSERT_NE(ta, nullptr);
        EXPECT_NE(ta->rgb()[3 * ia], 7) << "a_first=" << a_first;  // from a, not b
        EXPECT_LT(ta->angle(static_cast<int>(ia % 64), static_cast<int>(ia / 64)), 100);
        const auto [tb, ib] = pixel(east_b, below_a.y());
        ASSERT_NE(tb, nullptr);
        EXPECT_EQ(tb->rgb()[3 * ib], 7) << "a_first=" << a_first;
    }
}

TEST(OrthoRenderer, SmoothedSurfaceIgnoresASpike)
{
    // A 20 m spike of a few DSM cells in flat ground, seen at about 24
    // degrees from nadir: without smoothing the orthomosaic around it takes
    // colours from image positions about 9 m away.
    Fixture f;
    Scene   s = flatScene(f.conv, 120, 90, 90.0);
    const int u0 = 99, v0 = 44;
    const Eigen::Vector3d ray((u0 + 0.5 - s.camera.cx) / 90.0, (v0 + 0.5 - s.camera.cy) / 90.0, 1.0);
    const double          d0 = s.depth[static_cast<std::size_t>(v0 * 120 + u0)];
    const Eigen::Vector3d spike = f.frame.fromEcef(s.T_E_C * (d0 * ray));
    for (int v = v0; v < v0 + 2; ++v)
    {
        for (int u = u0; u < u0 + 2; ++u)
        {
            float &d = s.depth[static_cast<std::size_t>(v * 120 + u)];
            d -= static_cast<float>(20.0 * d / 250.0);  // the point 20 m higher on the same ray
        }
    }
    const FusionStats               fused = f.fuse(s);
    const geo::CameraModel          cam   = distortedCamera();
    const std::vector<std::uint8_t> img   = renderKeyframe(f.conv, f.frame, cam, s.T_E_C);
    auto max_error_near_spike = [&](double smoothing)
    {
        OrthoConfig config;
        config.pixel_size        = 0.5;
        config.surface_smoothing = smoothing;
        OrthoRenderer r(config, *f.dsm, f.frame);
        r.render({ img.data(), cam.width, cam.height, static_cast<std::size_t>(3 * cam.width) }, cam, s.T_E_C,
                 fused.extent);
        double max_err = 0.0;
        for (const auto &[id, tile] : r.tiles())
        {
            for (int row = 0; row < tile->pixels(); ++row)
            {
                for (int col = 0; col < tile->pixels(); ++col)
                {
                    const double e = tile->originEasting() + (col + 0.5) * tile->pixelSize();
                    const double n = tile->originNorthing() - (row + 0.5) * tile->pixelSize();
                    if (tile->angle(col, row) == OrthoTile::kEmpty || std::hypot(e - spike.x(), n - spike.y()) > 15.0)
                    {
                        continue;
                    }
                    const auto ref = groundColour(e, n);
                    for (int ch = 0; ch < 3; ++ch)
                    {
                        max_err = std::max(max_err, std::abs(tile->rgb()[3 * tile->index(col, row) + static_cast<std::size_t>(ch)]
                                                             - ref[static_cast<std::size_t>(ch)]));
                    }
                }
            }
        }
        return max_err;
    };
    const double raw      = max_error_near_spike(0.0);
    const double smoothed = max_error_near_spike(8.0);  // radius of 4 cells
    RecordProperty("max_error_raw", std::to_string(raw));
    RecordProperty("max_error_smoothed", std::to_string(smoothed));
    EXPECT_GT(raw, 15.0);           // the spike shifts the colours
    EXPECT_LT(smoothed, raw / 4.0);  // the smoothed surface ignores it
}

TEST(OrthoRenderer, KeepsTheSourceUnlessClearlyCloserToNadir)
{
    Fixture     f;
    const Scene a = flatScene(f.conv, 80, 60, 60.0);
    const Scene b = flatScene(f.conv, 80, 60, 60.0, 3.0, 0.0);  // 3 m east: angles differ by under 1 degree
    const FusionStats fa = f.fuse(a);
    // Inner part, seen by both cameras: only replaced pixels can count.
    Eigen::AlignedBox2d inner = fa.extent;
    inner.min().array() += 30.0;
    inner.max().array() -= 30.0;
    const std::vector<std::uint8_t> img_a(static_cast<std::size_t>(3 * 80 * 60), 50), img_b(img_a.size(), 200);
    for (const double margin : { 2.0, 0.0 })
    {
        OrthoConfig config;
        config.pixel_size        = 1.0;
        config.switch_margin_deg = margin;
        OrthoRenderer r(config, *f.dsm, f.frame);
        r.render({ img_a.data(), 80, 60, 240 }, a.camera, a.T_E_C, inner);
        const RenderStats second = r.render({ img_b.data(), 80, 60, 240 }, b.camera, b.T_E_C, inner);
        if (margin > 0.0)
        {
            EXPECT_EQ(second.pixels, 0u);
        }
        else
        {
            EXPECT_GT(second.pixels, 1000u);  // the east half switches to b
        }
    }
}

TEST(OrthoRenderer, NothingWithoutDsm)
{
    Fixture                         f;
    const Scene                     s = flatScene(f.conv);
    const std::vector<std::uint8_t> img(static_cast<std::size_t>(3 * 80 * 60), 50);
    OrthoRenderer                   r({ 1.0 }, *f.dsm, f.frame);
    Eigen::AlignedBox2d             extent(Eigen::Vector2d(0, 0), Eigen::Vector2d(100, 100));
    EXPECT_EQ(r.render({ img.data(), 80, 60, 240 }, s.camera, s.T_E_C, extent).pixels, 0u);
    EXPECT_TRUE(r.tiles().empty());
    EXPECT_THROW(r.render({ img.data(), 70, 60, 210 }, s.camera, s.T_E_C, extent), MapProductsError);
}

TEST(OrthoWriter, RgbaTileReadsBack)
{
    OrthoTile           tile({ 10, 20 }, 64.0, 32);
    const std::uint8_t  red[3] = { 200, 10, 20 };
    tile.set(3, 4, red, 120);
    const fs::path dir = fs::path(::testing::TempDir()) / "ost_map_products_ortho";
    fs::remove_all(dir);
    OrthoWriter       writer({ dir.string(), "EPSG:2056", OST_MP_TEST_PROJ_DATA_DIR });
    const std::string path = writer.write(tile);

    GDALDatasetH ds = GDALOpen(path.c_str(), GA_ReadOnly);
    ASSERT_NE(ds, nullptr);
    EXPECT_EQ(GDALGetRasterCount(ds), 4);
    double gt[6];
    GDALGetGeoTransform(ds, gt);
    EXPECT_DOUBLE_EQ(gt[0], 640.0);
    EXPECT_DOUBLE_EQ(gt[3], 21 * 64.0);
    EXPECT_DOUBLE_EQ(gt[1], 2.0);
    EXPECT_EQ(GDALGetRasterColorInterpretation(GDALGetRasterBand(ds, 4)), GCI_AlphaBand);
    std::uint8_t px[4];
    for (int b = 0; b < 4; ++b)
    {
        ASSERT_EQ(GDALRasterIO(GDALGetRasterBand(ds, b + 1), GF_Read, 3, 4, 1, 1, &px[b], 1, 1, GDT_Byte, 0, 0),
                  CE_None);
    }
    EXPECT_EQ(px[0], 200);
    EXPECT_EQ(px[1], 10);
    EXPECT_EQ(px[2], 20);
    EXPECT_EQ(px[3], 255);
    std::uint8_t alpha_empty = 1;
    ASSERT_EQ(GDALRasterIO(GDALGetRasterBand(ds, 4), GF_Read, 0, 0, 1, 1, &alpha_empty, 1, 1, GDT_Byte, 0, 0), CE_None);
    EXPECT_EQ(alpha_empty, 0);
    GDALClose(ds);
    EXPECT_TRUE(fs::exists(writer.writeIndex()));
}

}  // namespace
}  // namespace orthostream::map_products
