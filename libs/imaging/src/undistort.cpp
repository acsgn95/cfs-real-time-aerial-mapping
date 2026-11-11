#include "orthostream/imaging/undistort.hpp"

#include <cmath>
#include <cstdlib>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace orthostream::imaging
{

int heightForWidth(const geo::CameraModel &camera, int width)
{
    return static_cast<int>(std::lround(static_cast<double>(width) * camera.height / camera.width));
}

struct Undistorter::Impl
{
    geo::CameraModel input_camera;   // full resolution, with distortion
    geo::CameraModel output_camera;  // output size, without distortion
    int              channels = 0;
    cv::Mat          map_xy;         // fixed-point maps of cv::remap
    cv::Mat          map_frac;
    cv::Mat          reduced;        // input reduced to the output size
};

namespace
{

cv::Mat wrap(const ImageView &image)
{
    return cv::Mat(image.height, image.width, CV_8UC(image.channels), image.data, image.stride);
}

}  // namespace

Undistorter::Undistorter(const geo::CameraModel &camera, int output_width, int output_height, int channels) :
    impl_(std::make_unique<Impl>())
{
    if (camera.width <= 0 || camera.height <= 0 || !(camera.fx > 0.0) || !(camera.fy > 0.0))
    {
        throw ImagingError("invalid camera model");
    }
    if (output_width <= 0 || output_height <= 0 || (channels != 1 && channels != 3))
    {
        throw ImagingError("invalid output size or channel count");
    }

    impl_->input_camera = camera;
    impl_->channels     = channels;

    // The distorted image at the output size, and the same camera without
    // distortion for the output.
    const geo::CameraModel scaled = geo::scaledCamera(camera, output_width, output_height);
    impl_->output_camera          = geo::pinholeCamera(scaled);

    const cv::Matx33d K(scaled.fx, 0.0, scaled.cx, 0.0, scaled.fy, scaled.cy, 0.0, 0.0, 1.0);
    const cv::Vec<double, 5> distortion(scaled.k1, scaled.k2, scaled.p1, scaled.p2, scaled.k3);
    cv::initUndistortRectifyMap(K,
                                distortion,
                                cv::noArray(),
                                K,
                                cv::Size(output_width, output_height),
                                CV_16SC2,
                                impl_->map_xy,
                                impl_->map_frac);
    impl_->reduced.create(output_height, output_width, CV_8UC(channels));
}

Undistorter::~Undistorter() = default;

const geo::CameraModel &Undistorter::outputCamera() const
{
    return impl_->output_camera;
}

void Undistorter::apply(const ImageView &input, const ImageView &out)
{
    const geo::CameraModel &cam = impl_->output_camera;
    if (!input.valid() || input.channels != impl_->channels)
    {
        throw ImagingError("invalid input image");
    }
    if (!out.valid() || out.channels != impl_->channels || out.width != cam.width || out.height != cam.height)
    {
        throw ImagingError("output image does not match the configured size and channels");
    }
    // The input must show the full camera image: same aspect ratio within
    // one pixel.
    if (std::abs(heightForWidth(impl_->input_camera, input.width) - input.height) > 1)
    {
        throw ImagingError("input image does not have the camera's aspect ratio");
    }

    const cv::Mat in = wrap(input);
    cv::Mat       source;
    if (input.width == cam.width && input.height == cam.height)
    {
        source = in;
    }
    else
    {
        // Area averaging keeps pixel edges aligned (as geo::scaledCamera
        // assumes) and avoids aliasing when reducing.
        cv::resize(in, impl_->reduced, impl_->reduced.size(), 0.0, 0.0, cv::INTER_AREA);
        source = impl_->reduced;
    }

    cv::Mat dst = wrap(out);
    cv::remap(source, dst, impl_->map_xy, impl_->map_frac, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    if (dst.data != out.data)
    {
        // OpenCV reallocates only if the size or type differ, which the checks
        // above exclude.
        throw ImagingError("internal error: output buffer was replaced");
    }
}

}  // namespace orthostream::imaging
