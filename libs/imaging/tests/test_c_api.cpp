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

}  // namespace
