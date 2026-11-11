// Synthetic images for the imaging tests.

#ifndef ORTHOSTREAM_IMAGING_TESTS_TEST_IMAGES_HPP
#define ORTHOSTREAM_IMAGING_TESTS_TEST_IMAGES_HPP

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <turbojpeg.h>

#include "orthostream/imaging/image.hpp"

namespace orthostream::imaging::test
{

inline ImageView view(std::vector<std::uint8_t> &buffer, int width, int height, int channels)
{
    ImageView v;
    v.data     = buffer.data();
    v.width    = width;
    v.height   = height;
    v.channels = channels;
    v.stride   = static_cast<std::size_t>(width) * static_cast<std::size_t>(channels);
    return v;
}

/// Smooth RGB pattern without sharp edges, so JPEG artefacts stay small.
inline std::vector<std::uint8_t> smoothRgb(int width, int height)
{
    std::vector<std::uint8_t> rgb(static_cast<std::size_t>(width) * height * 3);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const double fx = static_cast<double>(x) / width;
            const double fy = static_cast<double>(y) / height;
            std::uint8_t *p = &rgb[(static_cast<std::size_t>(y) * width + x) * 3];
            p[0]            = static_cast<std::uint8_t>(127.5 + 100.0 * std::sin(6.0 * fx));
            p[1]            = static_cast<std::uint8_t>(127.5 + 100.0 * std::cos(5.0 * fy));
            p[2]            = static_cast<std::uint8_t>(127.5 + 100.0 * std::sin(4.0 * (fx + fy)));
        }
    }
    return rgb;
}

/// Encodes an image as a JPEG with quality 95 and 4:2:0 chroma subsampling,
/// like a typical survey camera.
inline std::vector<std::uint8_t> encodeJpeg(const std::vector<std::uint8_t> &pixels,
                                            int                              width,
                                            int                              height,
                                            int                              channels)
{
    tjhandle handle = tj3Init(TJINIT_COMPRESS);
    if (handle == nullptr)
    {
        throw std::runtime_error("tj3Init failed");
    }
    tj3Set(handle, TJPARAM_QUALITY, 95);
    tj3Set(handle, TJPARAM_SUBSAMP, channels == 3 ? TJSAMP_420 : TJSAMP_GRAY);
    unsigned char *jpeg = nullptr;
    std::size_t    size = 0;
    const int      rc   = tj3Compress8(
        handle, pixels.data(), width, 0, height, channels == 3 ? TJPF_RGB : TJPF_GRAY, &jpeg, &size);
    std::vector<std::uint8_t> out;
    if (rc == 0)
    {
        out.assign(jpeg, jpeg + size);
    }
    tj3Free(jpeg);
    tj3Destroy(handle);
    if (rc != 0)
    {
        throw std::runtime_error("JPEG encoding failed");
    }
    return out;
}

}  // namespace orthostream::imaging::test

#endif  // ORTHOSTREAM_IMAGING_TESTS_TEST_IMAGES_HPP
