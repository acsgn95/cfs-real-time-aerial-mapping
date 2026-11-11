// Image buffers of the imaging library (ADR-011: the caller owns them).

#ifndef ORTHOSTREAM_IMAGING_IMAGE_HPP
#define ORTHOSTREAM_IMAGING_IMAGE_HPP

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace orthostream::imaging
{

class ImagingError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

/// View of an 8-bit image owned by the caller. Pixels are interleaved (RGB or
/// grey), rows are stride bytes apart. Pixel coordinates follow the OpenCV
/// convention: the centre of the top-left pixel is (0, 0).
struct ImageView
{
    std::uint8_t *data     = nullptr;
    int           width    = 0;
    int           height   = 0;
    int           channels = 0;  ///< 1 (grey) or 3 (RGB)
    std::size_t   stride   = 0;  ///< bytes per row, at least width * channels

    bool valid() const
    {
        return data != nullptr && width > 0 && height > 0 && (channels == 1 || channels == 3)
               && stride >= static_cast<std::size_t>(width) * static_cast<std::size_t>(channels);
    }
};

}  // namespace orthostream::imaging

#endif  // ORTHOSTREAM_IMAGING_IMAGE_HPP
