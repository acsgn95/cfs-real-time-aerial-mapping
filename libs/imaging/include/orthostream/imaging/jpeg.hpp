// JPEG decoding with DCT scaling (ADR-001 section 1d, ADR-012 section 4).
//
// libjpeg-turbo can decode a JPEG directly at a reduced scale (for example
// 3/8 or 1/4) by evaluating fewer DCT coefficients. This is several times
// faster than decoding at full size and resampling afterwards, and it is the
// first step from the 42 MP input to the dense working resolution.

#ifndef ORTHOSTREAM_IMAGING_JPEG_HPP
#define ORTHOSTREAM_IMAGING_JPEG_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

#include "orthostream/imaging/image.hpp"

namespace orthostream::imaging
{

struct JpegInfo
{
    int width  = 0;
    int height = 0;
};

/// A DCT scaling factor num / denom and the image size it produces.
struct DctScale
{
    int num    = 1;
    int denom  = 1;
    int width  = 0;
    int height = 0;
};

/// The smallest DCT scaling factor (at most 1) whose output is at least
/// min_width x min_height. Returns scale 1 if no reduction is large enough.
DctScale chooseDctScale(const JpegInfo &info, int min_width, int min_height);

/// JPEG decoder. Not thread-safe: each task uses its own instance. Input
/// is untrusted: corrupt data raises ImagingError, and progressive JPEGs
/// with an excessive number of scans are rejected.
class JpegDecoder
{
public:
    JpegDecoder();
    ~JpegDecoder();

    JpegDecoder(const JpegDecoder &)            = delete;
    JpegDecoder &operator=(const JpegDecoder &) = delete;

    JpegInfo readHeader(const std::uint8_t *jpeg, std::size_t size);

    /// Decodes at the given scale into out, which must have exactly the
    /// scaled size; out.channels selects RGB (3) or grey (1) output.
    void decode(const std::uint8_t *jpeg, std::size_t size, const DctScale &scale, const ImageView &out);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orthostream::imaging

#endif  // ORTHOSTREAM_IMAGING_JPEG_HPP
