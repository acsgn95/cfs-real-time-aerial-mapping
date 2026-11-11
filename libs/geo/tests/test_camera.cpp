#include <gtest/gtest.h>

#include "orthostream/geo/camera.hpp"

namespace orthostream::geo
{
namespace
{

// Dorfberg Flight 01 adjusted calibration (Sony RX1R II, 7952 x 5304).
MetashapeCalibration dorfbergCalibration()
{
    MetashapeCalibration c;
    c.width  = 7952;
    c.height = 5304;
    c.f      = 7466.6708124128609;
    c.cx     = -15.3217577116648;
    c.cy     = -13.668940851948072;
    c.k1     = -0.011379646133417727;
    c.k2     = 0.05104158467607952;
    c.k3     = -0.093185947648228865;
    c.p1     = -4.8335886441893383e-05;
    c.p2     = -1.424956079674974e-05;
    return c;
}

TEST(Camera, MetashapeConversion)
{
    const MetashapeCalibration m = dorfbergCalibration();
    const CameraModel          c = cameraFromMetashape(m);
    EXPECT_EQ(c.width, 7952);
    EXPECT_EQ(c.height, 5304);
    EXPECT_DOUBLE_EQ(c.fx, m.f);
    EXPECT_DOUBLE_EQ(c.fy, m.f);
    EXPECT_DOUBLE_EQ(c.cx, 3976.0 + m.cx - 0.5);
    EXPECT_DOUBLE_EQ(c.cy, 2652.0 + m.cy - 0.5);
    EXPECT_DOUBLE_EQ(c.p1, m.p2);
    EXPECT_DOUBLE_EQ(c.p2, m.p1);
}

TEST(Camera, ColmapParametersShiftThePrincipalPoint)
{
    const CameraModel c = cameraFromMetashape(dorfbergCalibration());
    const auto        p = colmapFullOpenCvParams(c);
    EXPECT_DOUBLE_EQ(p[2], c.cx + 0.5);
    EXPECT_DOUBLE_EQ(p[3], c.cy + 0.5);
    EXPECT_DOUBLE_EQ(p[6], c.p1);
    EXPECT_DOUBLE_EQ(p[7], c.p2);
    EXPECT_DOUBLE_EQ(p[8], c.k3);
}

TEST(Camera, PointsBehindTheCameraDoNotProject)
{
    const CameraModel c = cameraFromMetashape(dorfbergCalibration());
    EXPECT_FALSE(projectPoint(c, Eigen::Vector3d(0.0, 0.0, 0.0)).has_value());
    EXPECT_FALSE(projectPoint(c, Eigen::Vector3d(1.0, 1.0, -5.0)).has_value());
    const auto centre = projectPoint(c, Eigen::Vector3d(0.0, 0.0, 10.0));
    ASSERT_TRUE(centre.has_value());
    EXPECT_DOUBLE_EQ(centre->x(), c.cx);
    EXPECT_DOUBLE_EQ(centre->y(), c.cy);
}

// Undistortion inverts the projection over the whole image, corners
// included.
TEST(Camera, UndistortInvertsProjectionOverTheImage)
{
    const CameraModel c = cameraFromMetashape(dorfbergCalibration());
    for (int v = 0; v <= c.height; v += c.height / 8)
    {
        for (int u = 0; u <= c.width; u += c.width / 8)
        {
            const Eigen::Vector2d pixel(u - 0.5, v - 0.5);
            const auto            xy = undistortPixel(c, pixel);
            ASSERT_TRUE(xy.has_value()) << u << ", " << v;
            const auto back = projectPoint(c, Eigen::Vector3d(xy->x(), xy->y(), 1.0));
            ASSERT_TRUE(back.has_value());
            EXPECT_LT((*back - pixel).norm(), 1e-8) << u << ", " << v;
        }
    }
}

TEST(Camera, UndistortRejectsInvalidCamera)
{
    CameraModel c;
    EXPECT_FALSE(undistortPixel(c, Eigen::Vector2d(1.0, 1.0)).has_value());
}

}  // namespace
}  // namespace orthostream::geo
