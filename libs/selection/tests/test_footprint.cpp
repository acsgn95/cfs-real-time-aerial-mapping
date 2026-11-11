#include <cmath>

#include <gtest/gtest.h>

#include "orthostream/selection/footprint.hpp"
#include "test_support.hpp"

namespace orthostream::selection
{
namespace
{

using test::camera;
using test::flat;
using test::nadir;

constexpr double kPi = 3.14159265358979323846;

TEST(Footprint, NadirOverFlatTerrain)
{
    const auto f = computeFootprint(camera(), nadir({ 10.0, 20.0, 150.0 }), flat(50.0));
    ASSERT_TRUE(f);
    EXPECT_NEAR(f->plane_height, 50.0, 1e-9);
    EXPECT_NEAR(f->width_m, 200.0, 1e-9);  // 2 H with H = 100 m
    EXPECT_NEAR(f->height_m, 100.0, 1e-9);
    EXPECT_NEAR(footprintArea(*f), 20000.0, 1e-6);
    EXPECT_NEAR(f->height_spread, 0.0, 1e-12);
    EXPECT_TRUE(f->centre.isApprox(Eigen::Vector3d(10.0, 20.0, 50.0), 1e-12));
    // Top-left image corner: west and north of the centre.
    EXPECT_NEAR(f->corners[0].x(), 10.0 - 100.0, 1e-9);
    EXPECT_NEAR(f->corners[0].y(), 20.0 + 50.0, 1e-9);
}

TEST(Footprint, PlaneMovesToTheTerrain)
{
    // The nominal height above ground is 100 m wrong; terrain slopes east.
    FootprintConfig config;
    config.nominal_height_agl = 200.0;
    config.iterations         = 3;
    const TerrainHeight slope = [](double x, double) { return std::optional<double>(0.1 * x); };
    const auto          f     = computeFootprint(camera(), nadir({ 0.0, 0.0, 100.0 }), slope, config);
    ASSERT_TRUE(f);
    // Symmetric footprint: the mean of the five heights is the height below
    // the camera.
    EXPECT_NEAR(f->plane_height, 0.0, 1e-9);
    EXPECT_NEAR(f->corners[1].z(), 0.1 * f->corners[1].x(), 1e-12);
    EXPECT_GT(f->height_spread, 5.0);
}

TEST(Footprint, RejectsMissingTerrainAndUpwardRays)
{
    const TerrainHeight none = [](double, double) { return std::optional<double>(); };
    EXPECT_FALSE(computeFootprint(camera(), nadir({ 0.0, 0.0, 100.0 }), none));
    // Tilted by 70°: the upper corners look above the horizon.
    EXPECT_FALSE(computeFootprint(camera(), nadir({ 0.0, 0.0, 100.0 }, 0.0, 70.0 * kPi / 180.0), flat(0.0)));
    // Terrain above the camera.
    EXPECT_FALSE(computeFootprint(camera(), nadir({ 0.0, 0.0, 100.0 }), flat(150.0)));
}

TEST(Footprint, OverlapOfShiftedAndRotatedFootprints)
{
    const auto a = computeFootprint(camera(), nadir({ 0.0, 0.0, 100.0 }), flat(0.0));
    const auto b = computeFootprint(camera(), nadir({ 60.0, 0.0, 100.0 }), flat(0.0));
    const auto c = computeFootprint(camera(), nadir({ 500.0, 0.0, 100.0 }), flat(0.0));
    const auto d = computeFootprint(camera(), nadir({ 0.0, 0.0, 100.0 }, kPi / 2.0), flat(0.0));
    ASSERT_TRUE(a && b && c && d);
    EXPECT_NEAR(overlapFraction(*a, *a), 1.0, 1e-12);
    EXPECT_NEAR(overlapFraction(*a, *b), 0.7, 1e-12);  // 60 m of a 200 m width
    EXPECT_NEAR(overlapFraction(*a, *c), 0.0, 1e-12);
    // 200 x 100 and 100 x 200 centred on each other: 100 x 100 in common.
    EXPECT_NEAR(overlapFraction(*a, *d), 0.5, 1e-12);
}

TEST(Footprint, Tilt)
{
    EXPECT_NEAR(tiltFromNadir(nadir({ 0, 0, 100 })), 0.0, 1e-12);
    EXPECT_NEAR(tiltFromNadir(nadir({ 0, 0, 100 }, 1.0, 0.3)), 0.3, 1e-12);
}

}  // namespace
}  // namespace orthostream::selection
