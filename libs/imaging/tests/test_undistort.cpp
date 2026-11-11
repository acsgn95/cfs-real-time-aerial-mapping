#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "orthostream/geo/camera.hpp"
#include "orthostream/imaging/undistort.hpp"
#include "test_images.hpp"

namespace orthostream::imaging
{
namespace
{

// A camera with strong distortion (tens of pixels at the corners), so that
// geometric errors of the undistortion are clearly measurable.
geo::CameraModel distortedCamera()
{
    geo::CameraModel c;
    c.width  = 800;
    c.height = 600;
    c.fx     = 700.0;
    c.fy     = 702.0;
    c.cx     = 402.3;
    c.cy     = 297.8;
    c.k1     = -0.20;
    c.k2     = 0.05;
    c.k3     = 0.0;
    c.p1     = 0.001;
    c.p2     = -0.0005;
    return c;
}

constexpr double kSigma = 1.5;  // blob size in output pixels

// Grid of blob centres in output pixel coordinates, away from the border.
std::vector<Eigen::Vector2d> blobCentres(const geo::CameraModel &out)
{
    std::vector<Eigen::Vector2d> centres;
    for (double v = 0.15; v < 0.9; v += 0.14)
    {
        for (double u = 0.12; u < 0.9; u += 0.12)
        {
            centres.emplace_back(u * out.width + 0.37, v * out.height - 0.21);
        }
    }
    return centres;
}

// Renders the distorted input image of a scene whose undistorted image (in
// the output camera) shows Gaussian blobs at the given centres.
std::vector<std::uint8_t> renderDistorted(const geo::CameraModel             &input,
                                          const geo::CameraModel             &output,
                                          const std::vector<Eigen::Vector2d> &centres)
{
    std::vector<std::uint8_t> img(static_cast<std::size_t>(input.width) * input.height);
    for (int v = 0; v < input.height; ++v)
    {
        for (int u = 0; u < input.width; ++u)
        {
            const auto xy = geo::undistortPixel(input, Eigen::Vector2d(u, v));
            double     value = 0.0;
            if (xy)
            {
                const Eigen::Vector2d p(output.fx * xy->x() + output.cx, output.fy * xy->y() + output.cy);
                for (const auto &c : centres)
                {
                    value += std::exp(-(p - c).squaredNorm() / (2.0 * kSigma * kSigma));
                }
            }
            img[static_cast<std::size_t>(v) * input.width + u] =
                static_cast<std::uint8_t>(std::lround(std::min(1.0, value) * 250.0));
        }
    }
    return img;
}

// Intensity-weighted centroid in a window around a point.
Eigen::Vector2d centroid(const std::vector<std::uint8_t> &img, int width, const Eigen::Vector2d &around)
{
    const int cu = static_cast<int>(std::lround(around.x()));
    const int cv = static_cast<int>(std::lround(around.y()));
    double    sw = 0.0, su = 0.0, sv = 0.0;
    for (int v = cv - 6; v <= cv + 6; ++v)
    {
        for (int u = cu - 6; u <= cu + 6; ++u)
        {
            const double w = img[static_cast<std::size_t>(v) * width + u];
            sw += w;
            su += w * u;
            sv += w * v;
        }
    }
    return { su / sw, sv / sw };
}

void expectBlobsInPlace(int output_width, int output_height)
{
    const geo::CameraModel input = distortedCamera();
    Undistorter            undistorter(input, output_width, output_height, 1);
    const auto            &out_cam = undistorter.outputCamera();
    const auto             centres = blobCentres(out_cam);

    auto                      src = renderDistorted(input, out_cam, centres);
    std::vector<std::uint8_t> dst(static_cast<std::size_t>(output_width) * output_height);
    undistorter.apply(test::view(src, input.width, input.height, 1), test::view(dst, output_width, output_height, 1));

    double max_err = 0.0;
    for (const auto &c : centres)
    {
        max_err = std::max(max_err, (centroid(dst, output_width, c) - c).norm());
    }
    EXPECT_LT(max_err, 0.15) << "output " << output_width << " x " << output_height;
}

TEST(Undistort, BlobsLandWhereTheOutputCameraProjectsThem)
{
    expectBlobsInPlace(800, 600);  // remap only
}

TEST(Undistort, BlobsLandInPlaceAfterReduction)
{
    // Area reduction to half size, then remap. The blobs are rendered in the
    // output camera, so they are 3 px wide in the 2x larger input.
    expectBlobsInPlace(400, 300);
}

TEST(Undistort, OutputCameraIsTheScaledCameraWithoutDistortion)
{
    const geo::CameraModel input = distortedCamera();
    Undistorter            undistorter(input, 400, 300, 3);
    const auto            &out      = undistorter.outputCamera();
    const auto             expected = geo::pinholeCamera(geo::scaledCamera(input, 400, 300));
    EXPECT_EQ(out.width, 400);
    EXPECT_EQ(out.height, 300);
    EXPECT_DOUBLE_EQ(out.fx, expected.fx);
    EXPECT_DOUBLE_EQ(out.cx, expected.cx);
    EXPECT_DOUBLE_EQ(out.cy, expected.cy);
    EXPECT_EQ(out.k1, 0.0);
    EXPECT_EQ(out.p2, 0.0);
}

TEST(Undistort, HeightForWidthKeepsTheAspectRatio)
{
    geo::CameraModel dorfberg;
    dorfberg.width  = 7952;
    dorfberg.height = 5304;
    EXPECT_EQ(heightForWidth(dorfberg, 2048), 1366);
}

TEST(Undistort, RejectsMismatchedImages)
{
    Undistorter               undistorter(distortedCamera(), 400, 300, 3);
    std::vector<std::uint8_t> in(800 * 500 * 3), out(400 * 300 * 3), grey(400 * 300);
    // Wrong aspect ratio of the input.
    EXPECT_THROW(undistorter.apply(test::view(in, 800, 500, 3), test::view(out, 400, 300, 3)), ImagingError);
    // Wrong channel count of the output.
    std::vector<std::uint8_t> in_ok(800 * 600 * 3);
    EXPECT_THROW(undistorter.apply(test::view(in_ok, 800, 600, 3), test::view(grey, 400, 300, 1)), ImagingError);
    EXPECT_THROW(Undistorter(distortedCamera(), 0, 300, 3), ImagingError);
}

}  // namespace
}  // namespace orthostream::imaging
