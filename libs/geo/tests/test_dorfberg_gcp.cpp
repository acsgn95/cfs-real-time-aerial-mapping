// Ground control point check of ADR-008 section 2 on Dorfberg Flight 01:
// five check points, each marked in five images.
//
// 1. The converted OpenCV model projects exactly like the Metashape model.
// 2. Markers triangulated from their observations reproject with the
//    per-marker error of the Metashape report.
// 3. Triangulated minus surveyed positions match the report's check point
//    errors.
// 4. Projecting the surveyed positions directly is reported (about 2 px RMS:
//    the Metashape model has 9.6 cm RMS error at these check points).

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "dorfberg_fixture.hpp"
#include "orthostream/geo/camera.hpp"
#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/metashape.hpp"
#include "orthostream/geo/triangulation.hpp"

namespace orthostream::geo
{
namespace
{

// Metashape's frame camera projection, written independently of the library
// from the Metashape documentation (Appendix C, "Camera models"); k4, b1 and
// b2 are zero for this project.
Eigen::Vector2d metashapeProject(const MetashapeCalibration &c, const Eigen::Vector3d &p_C)
{
    const double x  = p_C.x() / p_C.z();
    const double y  = p_C.y() / p_C.z();
    const double r2 = x * x + y * y;
    const double rd = 1.0 + c.k1 * r2 + c.k2 * r2 * r2 + c.k3 * r2 * r2 * r2;
    const double xd = x * rd + c.p1 * (r2 + 2.0 * x * x) + 2.0 * c.p2 * x * y;
    const double yd = y * rd + c.p2 * (r2 + 2.0 * y * y) + 2.0 * c.p1 * x * y;
    return { 0.5 * c.width + c.cx + xd * c.f, 0.5 * c.height + c.cy + yd * c.f };
}

class DorfbergGcp : public ::testing::Test
{
protected:
    void SetUp() override
    {
        fx_     = test::loadDorfbergFixture(OST_GEO_TEST_DATA_DIR "/dorfberg_gcp.txt");
        camera_ = cameraFromMetashape(fx_.calibration);
        ASSERT_EQ(fx_.gcp_ecef.size(), 5u);
        ASSERT_EQ(fx_.markers.size(), 25u);
        for (const auto &[label, cam] : fx_.cameras)
        {
            poses_[label] = cameraPoseEcef(fx_.chunk, cam.rotation, cam.translation);
        }
    }

    // OpenCV pixel of a Metashape pixel: the top-left pixel centre moves from
    // (0.5, 0.5) to (0, 0).
    static Eigen::Vector2d toOpenCvPixel(const Eigen::Vector2d &metashape_pixel)
    {
        return metashape_pixel - Eigen::Vector2d(0.5, 0.5);
    }

    test::DorfbergFixture                    fx_;
    CameraModel                              camera_;
    std::map<std::string, Eigen::Isometry3d> poses_;
};

TEST_F(DorfbergGcp, ConvertedModelProjectsLikeMetashape)
{
    for (const auto &m : fx_.markers)
    {
        const Eigen::Vector3d p_C = poses_.at(m.camera).inverse(Eigen::Isometry) * fx_.gcp_ecef.at(m.gcp);
        const auto            uv  = projectPoint(camera_, p_C);
        ASSERT_TRUE(uv.has_value());
        const Eigen::Vector2d expected = toOpenCvPixel(metashapeProject(fx_.calibration, p_C));
        EXPECT_LT((*uv - expected).norm(), 1e-6) << "GCP " << m.gcp << " in " << m.camera;
    }
}

TEST_F(DorfbergGcp, TriangulatedMarkersReproduceTheMetashapeReport)
{
    const GeodeticConverter geodetic(OST_GEO_TEST_PROJ_DATA_DIR);
    // Surveyed and estimated points are compared in LV95 (EPSG:2056), as in
    // the report. Heights are compared as ellipsoidal height differences,
    // where the geoid cancels.
    const CrsTransformer to_lv95("EPSG:4936", "EPSG:2056", OST_GEO_TEST_PROJ_DATA_DIR);

    for (const auto &[label, report] : fx_.report)
    {
        std::vector<Ray>                                                  rays;
        std::vector<std::pair<const Eigen::Isometry3d *, Eigen::Vector2d>> observations;
        for (const auto &m : fx_.markers)
        {
            if (m.gcp != label)
            {
                continue;
            }
            const Eigen::Isometry3d &T_E_C = poses_.at(m.camera);
            const Eigen::Vector2d    pixel = toOpenCvPixel(m.metashape_pixel);
            const auto               xy    = undistortPixel(camera_, pixel);
            ASSERT_TRUE(xy.has_value());
            rays.push_back({ T_E_C.translation(), T_E_C.linear() * Eigen::Vector3d(xy->x(), xy->y(), 1.0) });
            observations.emplace_back(&T_E_C, pixel);
        }
        ASSERT_EQ(rays.size(), 5u) << "GCP " << label;

        const auto estimated = triangulateRays(rays);
        ASSERT_TRUE(estimated.has_value());

        double sum_sq = 0.0;
        for (const auto &[T_E_C, pixel] : observations)
        {
            const auto uv = projectPoint(camera_, T_E_C->inverse(Eigen::Isometry) * *estimated);
            ASSERT_TRUE(uv.has_value());
            sum_sq += (*uv - pixel).squaredNorm();
        }
        const double rms_px = std::sqrt(sum_sq / static_cast<double>(observations.size()));
        EXPECT_NEAR(rms_px, report.image_error_px, 0.01) << "GCP " << label;

        const Eigen::Vector3d est_lv95 = to_lv95.forward(*estimated);
        const Eigen::Vector3d gcp_lv95 = to_lv95.forward(fx_.gcp_ecef.at(label));
        const double          dh_cm    = 100.0
                              * (geodetic.ecefToGeodetic(*estimated).height_m
                                 - geodetic.ecefToGeodetic(fx_.gcp_ecef.at(label)).height_m);
        EXPECT_NEAR(100.0 * (est_lv95.x() - gcp_lv95.x()), report.easting_cm, 1.5) << "GCP " << label;
        EXPECT_NEAR(100.0 * (est_lv95.y() - gcp_lv95.y()), report.northing_cm, 1.5) << "GCP " << label;
        EXPECT_NEAR(dh_cm, report.altitude_cm, 1.5) << "GCP " << label;

        RecordProperty("gcp" + label + "_reprojection_rms_px", std::to_string(rms_px));
        RecordProperty("gcp" + label + "_dE_cm", std::to_string(100.0 * (est_lv95.x() - gcp_lv95.x())));
        RecordProperty("gcp" + label + "_dN_cm", std::to_string(100.0 * (est_lv95.y() - gcp_lv95.y())));
        RecordProperty("gcp" + label + "_dh_cm", std::to_string(dh_cm));
    }
}

TEST_F(DorfbergGcp, SurveyedPointsProjectNearTheirMarkers)
{
    double sum_sq = 0.0;
    for (const auto &m : fx_.markers)
    {
        const Eigen::Vector3d p_C = poses_.at(m.camera).inverse(Eigen::Isometry) * fx_.gcp_ecef.at(m.gcp);
        const auto            uv  = projectPoint(camera_, p_C);
        ASSERT_TRUE(uv.has_value());
        sum_sq += (*uv - toOpenCvPixel(m.metashape_pixel)).squaredNorm();
    }
    const double rms_px = std::sqrt(sum_sq / static_cast<double>(fx_.markers.size()));
    RecordProperty("surveyed_projection_rms_px", std::to_string(rms_px));
    // Not an accuracy threshold (see the file comment): a sanity bound that
    // catches frame errors, which cause errors of hundreds of pixels.
    EXPECT_LT(rms_px, 5.0);
}

}  // namespace
}  // namespace orthostream::geo
