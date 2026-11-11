// From a camera's JPEG to an undistorted image at the dense working
// resolution in one call (ADR-001 section 1d, ADR-005 section 1).

#ifndef ORTHOSTREAM_IMAGING_WORKING_IMAGE_HPP
#define ORTHOSTREAM_IMAGING_WORKING_IMAGE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

#include "orthostream/geo/camera.hpp"
#include "orthostream/imaging/image.hpp"
#include "orthostream/imaging/jpeg.hpp"

namespace orthostream::imaging
{

/// Decodes the JPEGs of one camera at the smallest sufficient DCT scale and
/// undistorts them to a fixed working width (the height keeps the aspect
/// ratio). The DCT scale and all buffers are fixed on construction, so
/// convert() does not allocate. Not thread-safe: each task uses its own.
class WorkingImageConverter
{
public:
    WorkingImageConverter(const geo::CameraModel &camera, int working_width, int channels);
    ~WorkingImageConverter();

    WorkingImageConverter(const WorkingImageConverter &)            = delete;
    WorkingImageConverter &operator=(const WorkingImageConverter &) = delete;

    /// Camera model of the working images (no distortion).
    const geo::CameraModel &workingCamera() const;

    /// DCT scale used for decoding.
    const DctScale &dctScale() const;

    /// The JPEG must have the camera's image size; out must have the working
    /// size and the configured channels. Throws ImagingError otherwise or on
    /// corrupt data.
    void convert(const std::uint8_t *jpeg, std::size_t size, const ImageView &out);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orthostream::imaging

#endif  // ORTHOSTREAM_IMAGING_WORKING_IMAGE_HPP
