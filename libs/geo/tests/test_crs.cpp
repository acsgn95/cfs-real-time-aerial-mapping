#include <cmath>

#include <gtest/gtest.h>

#include "orthostream/geo/crs.hpp"

namespace orthostream::geo
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Dorfberg ground control point 1 as surveyed: ETRS89 ECEF and LV95.
const Eigen::Vector3d kGcp1Ecef(4309141.730, 747595.197, 4629311.867);
constexpr double      kGcp1Easting  = 2783401.345;
constexpr double      kGcp1Northing = 1188187.017;

TEST(Crs, GeodeticRoundTrip)
{
    const GeodeticConverter conv(OST_GEO_TEST_PROJ_DATA_DIR);
    const Geodetic          g = conv.ecefToGeodetic(kGcp1Ecef);
    // Davos: about 46.82 degrees north, 9.84 degrees east.
    EXPECT_NEAR(g.latitude_rad * 180.0 / kPi, 46.82, 0.01);
    EXPECT_NEAR(g.longitude_rad * 180.0 / kPi, 9.84, 0.01);
    EXPECT_LT((conv.geodeticToEcef(g) - kGcp1Ecef).norm(), 1e-6);
}

TEST(Crs, GeodeticOfAKnownPoint)
{
    const GeodeticConverter conv(OST_GEO_TEST_PROJ_DATA_DIR);
    // Point on the equator at the prime meridian, 100 m above the ellipsoid.
    const Geodetic g = conv.ecefToGeodetic(Eigen::Vector3d(6378137.0 + 100.0, 0.0, 0.0));
    EXPECT_NEAR(g.latitude_rad, 0.0, 1e-12);
    EXPECT_NEAR(g.longitude_rad, 0.0, 1e-12);
    EXPECT_NEAR(g.height_m, 100.0, 1e-6);
}

// LV95 is CHTRS95 / ETRS89 shifted by a translation and projected; PROJ must
// reproduce the surveyed LV95 coordinates of the ground control point.
TEST(Crs, EtrsEcefToLv95)
{
    const CrsTransformer  to_lv95("EPSG:4936", "EPSG:2056", OST_GEO_TEST_PROJ_DATA_DIR);
    const Eigen::Vector3d en = to_lv95.forward(kGcp1Ecef);
    EXPECT_NEAR(en.x(), kGcp1Easting, 0.02);
    EXPECT_NEAR(en.y(), kGcp1Northing, 0.02);
    // EPSG:2056 is two-dimensional: the height passes through the datum
    // shift as an ellipsoidal height and returns with about 1 mm error.
    EXPECT_LT((to_lv95.inverse(en) - kGcp1Ecef).norm(), 0.005);
}

TEST(Crs, UnknownCrsThrows)
{
    EXPECT_THROW(CrsTransformer("EPSG:999999", "EPSG:4979", OST_GEO_TEST_PROJ_DATA_DIR), GeoError);
}

TEST(Crs, MissingDataDirectoryThrows)
{
    EXPECT_THROW(CrsTransformer("EPSG:4978", "EPSG:4979", "/nonexistent"), GeoError);
}

}  // namespace
}  // namespace orthostream::geo
