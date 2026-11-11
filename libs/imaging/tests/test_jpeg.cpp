#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include <gtest/gtest.h>
#include <turbojpeg.h>

#include "orthostream/imaging/jpeg.hpp"
#include "test_images.hpp"

namespace orthostream::imaging
{
namespace
{

TEST(Jpeg, ChoosesTheSmallestSufficientScaleForDorfberg)
{
    // 42 MP Dorfberg images to a 2048 x 1366 working resolution: 1/4 gives
    // 1988 x 1326, too small, so 3/8 is the smallest sufficient reduction.
    const DctScale s = chooseDctScale({ 7952, 5304 }, 2048, 1366);
    EXPECT_EQ(s.num, 3);
    EXPECT_EQ(s.denom, 8);
    EXPECT_EQ(s.width, 2982);
    EXPECT_EQ(s.height, 1989);
}

TEST(Jpeg, KeepsFullSizeWhenNoReductionIsLargeEnough)
{
    const DctScale s = chooseDctScale({ 640, 480 }, 640, 480);
    EXPECT_EQ(s.num, 1);
    EXPECT_EQ(s.denom, 1);
    EXPECT_EQ(s.width, 640);
    EXPECT_EQ(s.height, 480);
}

TEST(Jpeg, ReadsTheHeader)
{
    const auto  jpeg = test::encodeJpeg(test::smoothRgb(320, 240), 320, 240, 3);
    JpegDecoder decoder;
    const auto  info = decoder.readHeader(jpeg.data(), jpeg.size());
    EXPECT_EQ(info.width, 320);
    EXPECT_EQ(info.height, 240);
}

// A DCT-scaled decode matches the full decode reduced by area averaging.
TEST(Jpeg, ScaledDecodeMatchesAreaReduction)
{
    constexpr int W = 320, H = 240;
    const auto    jpeg = test::encodeJpeg(test::smoothRgb(W, H), W, H, 3);
    JpegDecoder   decoder;

    std::vector<std::uint8_t> full(static_cast<std::size_t>(W) * H * 3);
    decoder.decode(jpeg.data(), jpeg.size(), { 1, 1, W, H }, test::view(full, W, H, 3));

    const DctScale            half { 1, 2, W / 2, H / 2 };
    std::vector<std::uint8_t> scaled(static_cast<std::size_t>(half.width) * half.height * 3);
    decoder.decode(jpeg.data(), jpeg.size(), half, test::view(scaled, half.width, half.height, 3));

    double sum_abs = 0.0;
    for (int y = 0; y < half.height; ++y)
    {
        for (int x = 0; x < half.width; ++x)
        {
            for (int c = 0; c < 3; ++c)
            {
                auto px = [&](int xx, int yy) { return full[(static_cast<std::size_t>(yy) * W + xx) * 3 + c]; };
                const double area = (px(2 * x, 2 * y) + px(2 * x + 1, 2 * y) + px(2 * x, 2 * y + 1)
                                     + px(2 * x + 1, 2 * y + 1))
                                  / 4.0;
                sum_abs += std::abs(area - scaled[(static_cast<std::size_t>(y) * half.width + x) * 3 + c]);
            }
        }
    }
    EXPECT_LT(sum_abs / (half.width * half.height * 3.0), 2.0);  // grey levels
}

TEST(Jpeg, DecodesToGrey)
{
    const auto                jpeg = test::encodeJpeg(test::smoothRgb(160, 120), 160, 120, 3);
    JpegDecoder               decoder;
    std::vector<std::uint8_t> grey(160 * 120);
    EXPECT_NO_THROW(decoder.decode(jpeg.data(), jpeg.size(), { 1, 1, 160, 120 }, test::view(grey, 160, 120, 1)));
}

TEST(Jpeg, RejectsAMismatchedOutputSize)
{
    const auto                jpeg = test::encodeJpeg(test::smoothRgb(160, 120), 160, 120, 3);
    JpegDecoder               decoder;
    std::vector<std::uint8_t> out(100 * 100 * 3);
    EXPECT_THROW(decoder.decode(jpeg.data(), jpeg.size(), { 1, 1, 100, 100 }, test::view(out, 100, 100, 3)),
                 ImagingError);
}

TEST(Jpeg, RejectsDataThatIsNotAJpeg)
{
    const std::vector<std::uint8_t> garbage(1024, 0x5a);
    JpegDecoder                     decoder;
    EXPECT_THROW(decoder.readHeader(garbage.data(), garbage.size()), ImagingError);
    EXPECT_THROW(decoder.readHeader(nullptr, 0), ImagingError);
}

}  // namespace
}  // namespace orthostream::imaging
