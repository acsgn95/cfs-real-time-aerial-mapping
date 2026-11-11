// Undistortion and resampling to a working resolution (ADR-005 section 1,
// ADR-012 section 4).

#ifndef ORTHOSTREAM_IMAGING_UNDISTORT_HPP
#define ORTHOSTREAM_IMAGING_UNDISTORT_HPP

#include <memory>

#include "orthostream/geo/camera.hpp"
#include "orthostream/imaging/image.hpp"

namespace orthostream::imaging
{

/// Output height for a given output width that keeps the aspect ratio of
/// the camera's image.
int heightForWidth(const geo::CameraModel &camera, int width);

/// Produces undistorted images of one camera at a fixed output size.
///
/// The input is the camera's image at any resolution with the same aspect
/// ratio, for example a DCT-scaled decode. It is first reduced to the output
/// size with area averaging, which avoids aliasing, and then remapped once
/// to remove the distortion. The output image follows outputCamera(): the
/// input camera scaled to the output size, without distortion. Output pixels
/// that see outside the input image are set to 0.
///
/// All buffers are allocated on construction; apply() does not allocate. Not
/// thread-safe: each task uses its own instance.
class Undistorter
{
public:
    /// camera: the full-resolution camera model; channels: 1 or 3.
    Undistorter(const geo::CameraModel &camera, int output_width, int output_height, int channels);
    ~Undistorter();

    Undistorter(const Undistorter &)            = delete;
    Undistorter &operator=(const Undistorter &) = delete;

    const geo::CameraModel &outputCamera() const;

    /// input: the camera's image at any size with the camera's aspect ratio
    /// (within one pixel); out: output size and the configured channels.
    void apply(const ImageView &input, const ImageView &out);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orthostream::imaging

#endif  // ORTHOSTREAM_IMAGING_UNDISTORT_HPP
