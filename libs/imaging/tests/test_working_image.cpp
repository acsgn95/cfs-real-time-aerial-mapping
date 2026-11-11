#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "orthostream/imaging/working_image.hpp"
#include "test_images.hpp"

namespace orthostream::imaging
{
namespace
{

using test::blobCentres;
using test::centroid;
using test::distortedCamera;
using test::renderDistorted;

TEST(WorkingImage, ChoosesTheDctScaleOnce)
{
    WorkingImageConverter converter(distortedCamera(), 400, 1);
    EXPECT_EQ(converter.dctScale().num, 1);
    EXPECT_EQ(converter.dctScale().denom, 2);
    EXPECT_EQ(converter.workingCamera().width, 400);
    EXPECT_EQ(converter.workingCamera().height, 300);
    EXPECT_EQ(converter.workingCamera().k1, 0.0);
}

// The whole chain through a real JPEG: blobs land where the working camera
// projects them, despite JPEG compression.
TEST(WorkingImage, JpegToUndistortedWorkingImage)
{
    const geo::CameraModel camera = distortedCamera();
    WorkingImageConverter  converter(camera, 400, 1);
    const auto            &work    = converter.workingCamera();
    const auto             centres = blobCentres(work);
    const auto             jpeg    = test::encodeJpeg(renderDistorted(camera, work, centres), camera.width, camera.height, 1);

    std::vector<std::uint8_t> out(static_cast<std::size_t>(work.width) * work.height);
    converter.convert(jpeg.data(), jpeg.size(), test::view(out, work.width, work.height, 1));

    double max_err = 0.0;
    for (const auto &c : centres)
    {
        max_err = std::max(max_err, (centroid(out, work.width, c) - c).norm());
    }
    EXPECT_LT(max_err, 0.2);
}

TEST(WorkingImage, RejectsAJpegOfAnotherSize)
{
    WorkingImageConverter     converter(distortedCamera(), 400, 3);
    const auto                jpeg = test::encodeJpeg(test::smoothRgb(640, 480), 640, 480, 3);
    std::vector<std::uint8_t> out(400 * 300 * 3);
    EXPECT_THROW(converter.convert(jpeg.data(), jpeg.size(), test::view(out, 400, 300, 3)), ImagingError);
}

TEST(WorkingImage, RejectsAWorkingWidthAboveTheInput)
{
    EXPECT_THROW(WorkingImageConverter(distortedCamera(), 1600, 3), ImagingError);
}

}  // namespace
}  // namespace orthostream::imaging
