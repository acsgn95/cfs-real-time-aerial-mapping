#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "imaging_c_api.h"
#include "test_images.hpp"

extern "C" int ost_imaging_c_api_usage(const uint8_t *jpeg, size_t size);

namespace
{

TEST(ImagingCApi, UsageFromC)
{
    using namespace orthostream::imaging;
    const auto jpeg = test::encodeJpeg(test::smoothRgb(320, 240), 320, 240, 3);
    EXPECT_EQ(ost_imaging_c_api_usage(jpeg.data(), jpeg.size()), 0);
}

TEST(ImagingCApi, RejectsNullArguments)
{
    EXPECT_EQ(OST_Imaging_DecoderCreate(nullptr), OST_IMAGING_ERR_INVALID_ARGUMENT);
    OST_Imaging_DctScale_t scale;
    EXPECT_EQ(OST_Imaging_ChooseDctScale(0, 10, 1, 1, &scale), OST_IMAGING_ERR_INVALID_ARGUMENT);
    OST_Imaging_Undistorter_t *u = nullptr;
    OST_Geo_Camera_t           camera {};
    EXPECT_EQ(OST_Imaging_UndistorterCreate(&u, &camera, 100, 0, 3), OST_IMAGING_ERR_INVALID_ARGUMENT);
    EXPECT_EQ(u, nullptr);
}

TEST(ImagingCApi, WorkingConverter)
{
    using namespace orthostream::imaging;
    const auto       jpeg = test::encodeJpeg(test::smoothRgb(320, 240), 320, 240, 3);
    OST_Geo_Camera_t camera {};
    camera.width  = 320;
    camera.height = 240;
    camera.fx     = 300.0;
    camera.fy     = 300.0;
    camera.cx     = 159.5;
    camera.cy     = 119.5;
    camera.k1     = -0.05;

    OST_Imaging_WorkingConverter_t *conv = nullptr;
    ASSERT_EQ(OST_Imaging_WorkingConverterCreate(&conv, &camera, 160, 3), OST_IMAGING_SUCCESS);
    OST_Geo_Camera_t work {};
    ASSERT_EQ(OST_Imaging_WorkingConverterCamera(conv, &work), OST_IMAGING_SUCCESS);
    EXPECT_EQ(work.width, 160);
    EXPECT_EQ(work.height, 120);

    std::vector<std::uint8_t> pixels(160 * 120 * 3);
    OST_Imaging_Image_t       out { pixels.data(), 160, 120, 3, 160 * 3 };
    EXPECT_EQ(OST_Imaging_ConvertJpeg(conv, jpeg.data(), jpeg.size(), &out), OST_IMAGING_SUCCESS);
    const std::uint8_t garbage[16] = { 0 };
    EXPECT_EQ(OST_Imaging_ConvertJpeg(conv, garbage, sizeof(garbage), &out), OST_IMAGING_ERR_DECODE);
    EXPECT_GT(std::string(OST_Imaging_WorkingConverterLastError(conv)).size(), 0u);
    OST_Imaging_WorkingConverterDestroy(conv);
}

}  // namespace
