#include <cmath>
#include <string>

#include <gtest/gtest.h>
#include <proj.h>

#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/frames.hpp"

namespace orthostream::geo
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(Frames, EnuRotationIsProperRotation)
{
    for (double lat : { -1.2, -0.3, 0.0, 0.8171, 1.5 })
    {
        for (double lon : { -3.0, -0.5, 0.0, 0.1717, 2.9 })
        {
            const Eigen::Matrix3d R = rotationEcefFromEnu(lat, lon);
            EXPECT_LT((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 1e-15);
            EXPECT_NEAR(R.determinant(), 1.0, 1e-15);
        }
    }
}

TEST(Frames, EnuAxesAtEquatorAndPrimeMeridian)
{
    const Eigen::Matrix3d R = rotationEcefFromEnu(0.0, 0.0);
    EXPECT_TRUE(R.col(0).isApprox(Eigen::Vector3d::UnitY()));  // east
    EXPECT_TRUE(R.col(1).isApprox(Eigen::Vector3d::UnitZ()));  // north
    EXPECT_TRUE(R.col(2).isApprox(Eigen::Vector3d::UnitX()));  // up
}

// ADR-004 section 7: the ECEF-to-ENU rotation is implemented directly and
// tested against PROJ's topocentric conversion.
TEST(Frames, LocalFrameMatchesProjTopocentric)
{
    const GeodeticConverter geodetic(OST_GEO_TEST_PROJ_DATA_DIR);
    const Eigen::Vector3d   origin(4309141.730, 747595.197, 4629311.867);  // Dorfberg GCP 1
    const Geodetic          g = geodetic.ecefToGeodetic(origin);
    const LocalFrame        frame(origin, g.latitude_rad, g.longitude_rad);

    PJ_CONTEXT       *ctx  = proj_context_create();
    const std::string def  = "+proj=topocentric +ellps=WGS84 +X_0=" + std::to_string(origin.x())
                          + " +Y_0=" + std::to_string(origin.y()) + " +Z_0=" + std::to_string(origin.z());
    PJ               *topo = proj_create(ctx, def.c_str());
    ASSERT_NE(topo, nullptr);

    for (const Eigen::Vector3d offset : { Eigen::Vector3d(250.0, -120.0, 40.0),
                                          Eigen::Vector3d(-1500.0, 800.0, 300.0),
                                          Eigen::Vector3d(3000.0, 2500.0, -500.0) })
    {
        const Eigen::Vector3d p_E = origin + offset;
        const PJ_COORD        enu = proj_trans(topo, PJ_FWD, proj_coord(p_E.x(), p_E.y(), p_E.z(), 0.0));
        const Eigen::Vector3d p_L = frame.toLocal(p_E);
        EXPECT_NEAR(p_L.x(), enu.enu.e, 1e-6);
        EXPECT_NEAR(p_L.y(), enu.enu.n, 1e-6);
        EXPECT_NEAR(p_L.z(), enu.enu.u, 1e-6);
        EXPECT_LT((frame.toEcef(p_L) - p_E).norm(), 1e-6);
    }
    proj_destroy(topo);
    proj_context_destroy(ctx);
}

TEST(Frames, PoseInLocalFrame)
{
    const Eigen::Vector3d origin(4309141.730, 747595.197, 4629311.867);
    const LocalFrame      frame(origin, 46.82 * kPi / 180.0, 9.84 * kPi / 180.0);

    Eigen::Isometry3d T_E_C = Eigen::Isometry3d::Identity();
    T_E_C.linear()          = Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    T_E_C.translation()     = origin + Eigen::Vector3d(10.0, 20.0, 30.0);

    const Eigen::Isometry3d T_L_C = frame.poseInLocal(T_E_C);
    const Eigen::Vector3d   p_C(1.0, -2.0, 50.0);
    EXPECT_LT((frame.toEcef(T_L_C * p_C) - T_E_C * p_C).norm(), 1e-8);
}

TEST(Frames, CanonicalQuaternionHasNonNegativeW)
{
    const Eigen::Quaterniond q(-0.5, 0.5, -0.5, 0.5);
    const Eigen::Quaterniond c = canonicalQuaternion(Eigen::Quaterniond(2.0 * q.coeffs()));
    EXPECT_GE(c.w(), 0.0);
    EXPECT_NEAR(c.norm(), 1.0, 1e-15);
    EXPECT_TRUE(c.toRotationMatrix().isApprox(q.normalized().toRotationMatrix()));
}

}  // namespace
}  // namespace orthostream::geo
