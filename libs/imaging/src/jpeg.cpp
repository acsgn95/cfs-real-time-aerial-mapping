#include "orthostream/imaging/jpeg.hpp"

#include <string>

#include <turbojpeg.h>

namespace orthostream::imaging
{

namespace
{

// Upper limit of scans in a progressive JPEG; protects against crafted files
// that would take very long to decode (libjpeg-turbo's recommendation).
constexpr int kScanLimit = 500;

int scaled(int size, const tjscalingfactor &f)
{
    return TJSCALED(size, f);
}

}  // namespace

DctScale chooseDctScale(const JpegInfo &info, int min_width, int min_height)
{
    DctScale best { 1, 1, info.width, info.height };
    int      count   = 0;
    const tjscalingfactor *factors = tj3GetScalingFactors(&count);
    if (factors == nullptr)
    {
        return best;
    }
    for (int i = 0; i < count; ++i)
    {
        const tjscalingfactor &f = factors[i];
        if (f.num > f.denom)
        {
            continue;  // only reductions
        }
        const int w = scaled(info.width, f);
        const int h = scaled(info.height, f);
        if (w >= min_width && h >= min_height && w < best.width)
        {
            best = { f.num, f.denom, w, h };
        }
    }
    return best;
}

struct JpegDecoder::Impl
{
    tjhandle handle = nullptr;

    ~Impl()
    {
        if (handle != nullptr)
        {
            tj3Destroy(handle);
        }
    }

    [[noreturn]] void fail(const std::string &what) const
    {
        throw ImagingError(what + ": " + tj3GetErrorStr(handle));
    }

    // TurboJPEG reports recoverable problems as warnings; only fatal errors
    // fail the call.
    void check(int result, const char *what) const
    {
        if (result != 0 && tj3GetErrorCode(handle) == TJERR_FATAL)
        {
            fail(what);
        }
    }
};

JpegDecoder::JpegDecoder() : impl_(std::make_unique<Impl>())
{
    impl_->handle = tj3Init(TJINIT_DECOMPRESS);
    if (impl_->handle == nullptr)
    {
        throw ImagingError(std::string("cannot create a JPEG decoder: ") + tj3GetErrorStr(nullptr));
    }
    impl_->check(tj3Set(impl_->handle, TJPARAM_SCANLIMIT, kScanLimit), "cannot set the scan limit");
}

JpegDecoder::~JpegDecoder() = default;

JpegInfo JpegDecoder::readHeader(const std::uint8_t *jpeg, std::size_t size)
{
    if (jpeg == nullptr || size == 0)
    {
        throw ImagingError("empty JPEG data");
    }
    impl_->check(tj3DecompressHeader(impl_->handle, jpeg, size), "invalid JPEG header");
    JpegInfo info;
    info.width  = tj3Get(impl_->handle, TJPARAM_JPEGWIDTH);
    info.height = tj3Get(impl_->handle, TJPARAM_JPEGHEIGHT);
    if (info.width <= 0 || info.height <= 0)
    {
        throw ImagingError("invalid JPEG image size");
    }
    return info;
}

void JpegDecoder::decode(const std::uint8_t *jpeg, std::size_t size, const DctScale &scale, const ImageView &out)
{
    if (!out.valid())
    {
        throw ImagingError("invalid output image");
    }
    const JpegInfo        info = readHeader(jpeg, size);
    const tjscalingfactor factor { scale.num, scale.denom };
    if (scaled(info.width, factor) != out.width || scaled(info.height, factor) != out.height)
    {
        throw ImagingError("output image size does not match the scaled JPEG size");
    }
    impl_->check(tj3SetScalingFactor(impl_->handle, factor), "unsupported scaling factor");
    impl_->check(tj3Decompress8(impl_->handle,
                                jpeg,
                                size,
                                out.data,
                                static_cast<int>(out.stride),
                                out.channels == 3 ? TJPF_RGB : TJPF_GRAY),
                 "JPEG decoding failed");
}

}  // namespace orthostream::imaging
