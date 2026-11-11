#include "orthostream/imaging/working_image.hpp"

#include <vector>

#include "orthostream/imaging/undistort.hpp"

namespace orthostream::imaging
{

struct WorkingImageConverter::Impl
{
    Impl(const geo::CameraModel &camera, int working_width, int channels) :
        input_width(camera.width),
        input_height(camera.height),
        scale(chooseDctScale({ camera.width, camera.height },
                             working_width,
                             heightForWidth(camera, working_width))),
        undistorter(camera, working_width, heightForWidth(camera, working_width), channels)
    {
        decoded.resize(static_cast<std::size_t>(scale.width) * static_cast<std::size_t>(scale.height)
                       * static_cast<std::size_t>(channels));
        decoded_view.data     = decoded.data();
        decoded_view.width    = scale.width;
        decoded_view.height   = scale.height;
        decoded_view.channels = channels;
        decoded_view.stride   = static_cast<std::size_t>(scale.width) * static_cast<std::size_t>(channels);
    }

    int                       input_width;
    int                       input_height;
    DctScale                  scale;
    JpegDecoder               decoder;
    Undistorter               undistorter;
    std::vector<std::uint8_t> decoded;
    ImageView                 decoded_view;
};

WorkingImageConverter::WorkingImageConverter(const geo::CameraModel &camera, int working_width, int channels)
{
    if (camera.width <= 0 || camera.height <= 0 || working_width <= 0 || working_width > camera.width)
    {
        throw ImagingError("invalid camera size or working width");
    }
    impl_ = std::make_unique<Impl>(camera, working_width, channels);
}

WorkingImageConverter::~WorkingImageConverter() = default;

const geo::CameraModel &WorkingImageConverter::workingCamera() const
{
    return impl_->undistorter.outputCamera();
}

const DctScale &WorkingImageConverter::dctScale() const
{
    return impl_->scale;
}

void WorkingImageConverter::convert(const std::uint8_t *jpeg, std::size_t size, const ImageView &out)
{
    const JpegInfo info = impl_->decoder.readHeader(jpeg, size);
    if (info.width != impl_->input_width || info.height != impl_->input_height)
    {
        throw ImagingError("JPEG size does not match the camera");
    }
    impl_->decoder.decode(jpeg, size, impl_->scale, impl_->decoded_view);
    impl_->undistorter.apply(impl_->decoded_view, out);
}

}  // namespace orthostream::imaging
