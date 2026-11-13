// C interface of the imaging library. No C++ exception crosses this
// boundary.

#include "imaging_c_api.h"

#include <new>
#include <optional>
#include <string>

#include "orthostream/imaging/jpeg.hpp"
#include "orthostream/imaging/undistort.hpp"
#include "orthostream/imaging/working_image.hpp"

struct OST_Imaging_Decoder
{
    orthostream::imaging::JpegDecoder decoder;
    std::string                       last_error;
};

struct OST_Imaging_Undistorter
{
    std::optional<orthostream::imaging::Undistorter> undistorter;
    std::string                                      last_error;
};

struct OST_Imaging_WorkingConverter
{
    std::optional<orthostream::imaging::WorkingImageConverter> converter;
    std::string                                                last_error;
};

namespace
{

using orthostream::imaging::ImageView;

void fromModel(const orthostream::geo::CameraModel &m, OST_Geo_Camera_t *camera)
{
    camera->width  = m.width;
    camera->height = m.height;
    camera->fx     = m.fx;
    camera->fy     = m.fy;
    camera->cx     = m.cx;
    camera->cy     = m.cy;
    camera->k1     = m.k1;
    camera->k2     = m.k2;
    camera->k3     = m.k3;
    camera->p1     = m.p1;
    camera->p2     = m.p2;
}

ImageView toView(const OST_Imaging_Image_t &image)
{
    ImageView view;
    view.data     = image.data;
    view.width    = image.width;
    view.height   = image.height;
    view.channels = image.channels;
    view.stride   = image.stride;
    return view;
}

orthostream::geo::CameraModel toModel(const OST_Geo_Camera_t &c)
{
    orthostream::geo::CameraModel m;
    m.width  = c.width;
    m.height = c.height;
    m.fx     = c.fx;
    m.fy     = c.fy;
    m.cx     = c.cx;
    m.cy     = c.cy;
    m.k1     = c.k1;
    m.k2     = c.k2;
    m.k3     = c.k3;
    m.p1     = c.p1;
    m.p2     = c.p2;
    return m;
}

// Runs f, converting exceptions to status codes and recording the message.
template <typename F>
OST_Status_t guarded(std::string *last_error, F &&f)
{
    try
    {
        if (last_error != nullptr)
        {
            last_error->clear();
        }
        return f();
    }
    catch (const orthostream::imaging::ImagingError &e)
    {
        if (last_error != nullptr)
        {
            *last_error = e.what();
        }
        return OST_ERR_FORMAT;
    }
    catch (const std::bad_alloc &)
    {
        return OST_ERR_NO_MEMORY;
    }
    catch (const std::exception &e)
    {
        if (last_error != nullptr)
        {
            *last_error = e.what();
        }
        return OST_ERR_INTERNAL;
    }
    catch (...)
    {
        return OST_ERR_INTERNAL;
    }
}

}  // namespace

extern "C" {

OST_Status_t OST_Imaging_DecoderCreate(OST_Imaging_Decoder_t **decoder)
{
    if (decoder == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *decoder = nullptr;
    return guarded(nullptr,
                   [&]
                   {
                       *decoder = new OST_Imaging_Decoder_t();
                       return OST_SUCCESS;
                   });
}

void OST_Imaging_DecoderDestroy(OST_Imaging_Decoder_t *decoder)
{
    delete decoder;
}

const char *OST_Imaging_DecoderLastError(const OST_Imaging_Decoder_t *decoder)
{
    return decoder != nullptr ? decoder->last_error.c_str() : "";
}

OST_Status_t OST_Imaging_JpegSize(OST_Imaging_Decoder_t *decoder,
                                          const uint8_t         *jpeg,
                                          size_t                 size,
                                          int32_t               *width,
                                          int32_t               *height)
{
    if (decoder == nullptr || jpeg == nullptr || size == 0 || width == nullptr || height == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(&decoder->last_error,
                   [&]
                   {
                       const auto info = decoder->decoder.readHeader(jpeg, size);
                       *width          = info.width;
                       *height         = info.height;
                       return OST_SUCCESS;
                   });
}

OST_Status_t OST_Imaging_ChooseDctScale(int32_t                 width,
                                                int32_t                 height,
                                                int32_t                 min_width,
                                                int32_t                 min_height,
                                                OST_Imaging_DctScale_t *scale)
{
    if (width <= 0 || height <= 0 || min_width < 0 || min_height < 0 || scale == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    const auto s  = orthostream::imaging::chooseDctScale({ width, height }, min_width, min_height);
    scale->num    = s.num;
    scale->denom  = s.denom;
    scale->width  = s.width;
    scale->height = s.height;
    return OST_SUCCESS;
}

OST_Status_t OST_Imaging_DecodeJpeg(OST_Imaging_Decoder_t        *decoder,
                                            const uint8_t                *jpeg,
                                            size_t                        size,
                                            const OST_Imaging_DctScale_t *scale,
                                            const OST_Imaging_Image_t    *out)
{
    if (decoder == nullptr || jpeg == nullptr || size == 0 || scale == nullptr || out == nullptr
        || !toView(*out).valid())
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(&decoder->last_error,
                   [&]
                   {
                       decoder->decoder.decode(
                           jpeg, size, { scale->num, scale->denom, scale->width, scale->height }, toView(*out));
                       return OST_SUCCESS;
                   });
}

OST_Status_t OST_Imaging_UndistorterCreate(OST_Imaging_Undistorter_t **undistorter,
                                                   const OST_Geo_Camera_t     *camera,
                                                   int32_t                     output_width,
                                                   int32_t                     output_height,
                                                   int32_t                     channels)
{
    if (undistorter == nullptr || camera == nullptr || camera->width <= 0 || camera->height <= 0
        || output_width <= 0 || output_height < 0 || (channels != 1 && channels != 3))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *undistorter = nullptr;
    OST_Imaging_Undistorter_t *created = new (std::nothrow) OST_Imaging_Undistorter_t();
    if (created == nullptr)
    {
        return OST_ERR_NO_MEMORY;
    }
    const auto model  = toModel(*camera);
    const int  height = output_height > 0 ? output_height : orthostream::imaging::heightForWidth(model, output_width);
    const OST_Status_t status = guarded(&created->last_error,
                                                [&]
                                                {
                                                    created->undistorter.emplace(model, output_width, height, channels);
                                                    return OST_SUCCESS;
                                                });
    if (status != OST_SUCCESS)
    {
        delete created;
        return status == OST_ERR_FORMAT ? OST_ERR_INVALID_ARGUMENT : status;
    }
    *undistorter = created;
    return OST_SUCCESS;
}

void OST_Imaging_UndistorterDestroy(OST_Imaging_Undistorter_t *undistorter)
{
    delete undistorter;
}

const char *OST_Imaging_UndistorterLastError(const OST_Imaging_Undistorter_t *undistorter)
{
    return undistorter != nullptr ? undistorter->last_error.c_str() : "";
}

OST_Status_t OST_Imaging_UndistorterOutputCamera(const OST_Imaging_Undistorter_t *undistorter,
                                                         OST_Geo_Camera_t                *camera)
{
    if (undistorter == nullptr || camera == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    fromModel(undistorter->undistorter->outputCamera(), camera);
    return OST_SUCCESS;
}

OST_Status_t OST_Imaging_Undistort(OST_Imaging_Undistorter_t *undistorter,
                                           const OST_Imaging_Image_t *input,
                                           const OST_Imaging_Image_t *out)
{
    if (undistorter == nullptr || input == nullptr || out == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    const OST_Status_t status =
        guarded(&undistorter->last_error,
                [&]
                {
                    undistorter->undistorter->apply(toView(*input), toView(*out));
                    return OST_SUCCESS;
                });
    // Undistortion fails only on mismatched buffers, which is a caller error.
    return status == OST_ERR_FORMAT ? OST_ERR_INVALID_ARGUMENT : status;
}

OST_Status_t OST_Imaging_WorkingConverterCreate(OST_Imaging_WorkingConverter_t **converter,
                                                        const OST_Geo_Camera_t          *camera,
                                                        int32_t                          working_width,
                                                        int32_t                          channels)
{
    if (converter == nullptr || camera == nullptr || camera->width <= 0 || camera->height <= 0
        || working_width <= 0 || working_width > camera->width || (channels != 1 && channels != 3))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *converter = nullptr;
    OST_Imaging_WorkingConverter_t *created = new (std::nothrow) OST_Imaging_WorkingConverter_t();
    if (created == nullptr)
    {
        return OST_ERR_NO_MEMORY;
    }
    const OST_Status_t status = guarded(&created->last_error,
                                                [&]
                                                {
                                                    created->converter.emplace(
                                                        toModel(*camera), working_width, channels);
                                                    return OST_SUCCESS;
                                                });
    if (status != OST_SUCCESS)
    {
        delete created;
        return status == OST_ERR_FORMAT ? OST_ERR_INVALID_ARGUMENT : status;
    }
    *converter = created;
    return OST_SUCCESS;
}

void OST_Imaging_WorkingConverterDestroy(OST_Imaging_WorkingConverter_t *converter)
{
    delete converter;
}

const char *OST_Imaging_WorkingConverterLastError(const OST_Imaging_WorkingConverter_t *converter)
{
    return converter != nullptr ? converter->last_error.c_str() : "";
}

OST_Status_t OST_Imaging_WorkingConverterCamera(const OST_Imaging_WorkingConverter_t *converter,
                                                        OST_Geo_Camera_t                     *camera)
{
    if (converter == nullptr || camera == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    fromModel(converter->converter->workingCamera(), camera);
    return OST_SUCCESS;
}

OST_Status_t OST_Imaging_ConvertJpeg(OST_Imaging_WorkingConverter_t *converter,
                                             const uint8_t                  *jpeg,
                                             size_t                          size,
                                             const OST_Imaging_Image_t      *out)
{
    if (converter == nullptr || jpeg == nullptr || size == 0 || out == nullptr || !toView(*out).valid())
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(&converter->last_error,
                   [&]
                   {
                       converter->converter->convert(jpeg, size, toView(*out));
                       return OST_SUCCESS;
                   });
}

}  // extern "C"
