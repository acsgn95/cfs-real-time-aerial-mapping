#include <random>

#include <gtest/gtest.h>

#include "orthostream/map_products/product_frame.hpp"
#include "test_support.hpp"

namespace orthostream::map_products
{
namespace
{

using test::dorfberg;

TEST(ProductFrame, RoundTripAndHeights)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    for (const char *crs : { "EPSG:2056", "EPSG:32632" })
    {
        const ProductFrame    frame(crs, OST_MP_TEST_PROJ_DATA_DIR);
        const Eigen::Vector3d p_E = conv.geodeticToEcef(dorfberg(1987.5));
        const Eigen::Vector3d p_P = frame.fromEcef(p_E);
        EXPECT_NEAR(p_P.z(), 1987.5, 1e-6) << crs;  // ellipsoidal height
        EXPECT_LT((frame.toEcef(p_P) - p_E).norm(), 1e-6) << crs;
    }
}

TEST(ProductFrame, LV95CoordinatesOfTheSite)
{
    // The site lies near E 2 782 700, N 1 188 300 in LV95 (reference DSM extent).
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:2056", OST_MP_TEST_PROJ_DATA_DIR);
    const Eigen::Vector3d        p = frame.fromEcef(conv.geodeticToEcef(dorfberg(2000.0)));
    EXPECT_NEAR(p.x(), 2782700.0, 2000.0);
    EXPECT_NEAR(p.y(), 1188300.0, 2000.0);
}

TEST(InterpolatedTransform, MatchesProjOverABatchRegion)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:2056", OST_MP_TEST_PROJ_DATA_DIR);
    // A region of a keyframe batch: 500 m wide, 300 m deep below 2250 m.
    const Eigen::Vector3d    c = conv.geodeticToEcef(dorfberg(2000.0));
    Eigen::AlignedBox3d      box(c - Eigen::Vector3d::Constant(250.0), c + Eigen::Vector3d::Constant(250.0));
    const InterpolatedTransform fast([&](const Eigen::Vector3d &p) { return frame.fromEcef(p); }, box);

    std::mt19937                           rng(7);
    std::uniform_real_distribution<double> inside(-250.0, 250.0), outside(-350.0, 350.0);
    double                                 max_in = 0.0, max_out = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
        const Eigen::Vector3d p(c.x() + inside(rng), c.y() + inside(rng), c.z() + inside(rng));
        max_in = std::max(max_in, (fast(p) - frame.fromEcef(p)).norm());
        const Eigen::Vector3d q(c.x() + outside(rng), c.y() + outside(rng), c.z() + outside(rng));
        max_out = std::max(max_out, (fast(q) - frame.fromEcef(q)).norm());
    }
    RecordProperty("max_error_inside_m", std::to_string(max_in));
    EXPECT_LT(max_in, 0.0005);   // half a millimetre
    EXPECT_LT(max_out, 0.005);   // extrapolated up to 100 m beyond the box
}

TEST(InterpolatedTransform, RejectsAnInvalidGrid)
{
    const auto          identity = [](const Eigen::Vector3d &p) { return p; };
    Eigen::AlignedBox3d empty;
    EXPECT_THROW(InterpolatedTransform(identity, empty), MapProductsError);
    Eigen::AlignedBox3d flat(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(10, 10, 0));
    const InterpolatedTransform t(identity, flat, 2);  // a flat axis is widened
    EXPECT_TRUE(t(Eigen::Vector3d(3, 4, 0.2)).isApprox(Eigen::Vector3d(3, 4, 0.2)));
}

}  // namespace
}  // namespace orthostream::map_products
