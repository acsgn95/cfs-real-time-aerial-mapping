// Synthetic images for the imaging tests.

#ifndef ORTHOSTREAM_IMAGING_TESTS_TEST_IMAGES_HPP
#define ORTHOSTREAM_IMAGING_TESTS_TEST_IMAGES_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <turbojpeg.h>

#include "orthostream/geo/camera.hpp"
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

// A camera with strong distortion (tens of pixels at the corners), so that
// geometric errors of the undistortion are clearly measurable.
inline orthostream::geo::CameraModel distortedCamera()
{
    orthostream::geo::CameraModel c;
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

inline constexpr double kSigma = 1.5;  // blob size in output pixels

// Grid of blob centres in output pixel coordinates, away from the border.
inline std::vector<Eigen::Vector2d> blobCentres(const orthostream::geo::CameraModel &out)
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
inline std::vector<std::uint8_t> renderDistorted(const orthostream::geo::CameraModel &input,
                                                 const orthostream::geo::CameraModel &output,
                                                 const std::vector<Eigen::Vector2d>  &centres)
{
    std::vector<std::uint8_t> img(static_cast<std::size_t>(input.width) * input.height);
    for (int v = 0; v < input.height; ++v)
    {
        for (int u = 0; u < input.width; ++u)
        {
            const auto xy = orthostream::geo::undistortPixel(input, Eigen::Vector2d(u, v));
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
inline Eigen::Vector2d centroid(const std::vector<std::uint8_t> &img, int width, const Eigen::Vector2d &around)
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

}  // namespace orthostream::imaging::test

#endif  // ORTHOSTREAM_IMAGING_TESTS_TEST_IMAGES_HPP
