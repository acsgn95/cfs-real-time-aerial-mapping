#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "orthostream/geo/camera.hpp"
#include "orthostream/imaging/undistort.hpp"
#include "test_images.hpp"

using orthostream::imaging::test::blobCentres;
using orthostream::imaging::test::centroid;
using orthostream::imaging::test::distortedCamera;
using orthostream::imaging::test::renderDistorted;

namespace orthostream::imaging
{
namespace
{

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
