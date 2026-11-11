/*
 * C interface of the OrthoStream imaging library (ADR-011, ADR-012).
 *
 * JPEG decoding with DCT scaling, and undistortion and resampling to a
 * working resolution. Images are 8-bit, interleaved RGB or grey, owned by
 * the caller. Pixel coordinates and camera models follow geo_c_api.h.
 *
 * Handles are not thread-safe: each task uses its own. No function throws
 * or keeps threads running after it returns. Undistortion allocates its
 * buffers when the handle is created, not per image.
 */

#ifndef ORTHOSTREAM_IMAGING_C_API_H
#define ORTHOSTREAM_IMAGING_C_API_H

#include <stddef.h>
#include <stdint.h>

#include "geo_c_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    OST_IMAGING_SUCCESS              = 0,
    OST_IMAGING_ERR_INVALID_ARGUMENT = -1,
    OST_IMAGING_ERR_DECODE           = -2, /* invalid or unsupported JPEG data */
    OST_IMAGING_ERR_OUT_OF_MEMORY    = -3,
    OST_IMAGING_ERR_INTERNAL         = -4
} OST_Imaging_Status_t;

typedef struct
{
    uint8_t *data;
    int32_t  width;
    int32_t  height;
    int32_t  channels; /* 1 (grey) or 3 (RGB) */
    size_t   stride;   /* bytes per row */
} OST_Imaging_Image_t;

/* A DCT scaling factor num / denom and the image size it produces. */
typedef struct
{
    int32_t num;
    int32_t denom;
    int32_t width;
    int32_t height;
} OST_Imaging_DctScale_t;

typedef struct OST_Imaging_Decoder     OST_Imaging_Decoder_t;
typedef struct OST_Imaging_Undistorter OST_Imaging_Undistorter_t;
typedef struct OST_Imaging_WorkingConverter OST_Imaging_WorkingConverter_t;

const char *OST_Imaging_StatusText(OST_Imaging_Status_t status);

/* JPEG decoding. LastError gives the message of the last failed call on the
 * handle; it is empty after a successful call. */
OST_Imaging_Status_t OST_Imaging_DecoderCreate(OST_Imaging_Decoder_t **decoder);
void                 OST_Imaging_DecoderDestroy(OST_Imaging_Decoder_t *decoder);
const char          *OST_Imaging_DecoderLastError(const OST_Imaging_Decoder_t *decoder);

OST_Imaging_Status_t OST_Imaging_JpegSize(OST_Imaging_Decoder_t *decoder,
                                          const uint8_t         *jpeg,
                                          size_t                 size,
                                          int32_t               *width,
                                          int32_t               *height);

/* Smallest DCT reduction whose output is at least min_width x min_height. */
OST_Imaging_Status_t OST_Imaging_ChooseDctScale(int32_t                 width,
                                                int32_t                 height,
                                                int32_t                 min_width,
                                                int32_t                 min_height,
                                                OST_Imaging_DctScale_t *scale);

/* out must have exactly the scaled size; out->channels selects RGB or grey. */
OST_Imaging_Status_t OST_Imaging_DecodeJpeg(OST_Imaging_Decoder_t        *decoder,
                                            const uint8_t                *jpeg,
                                            size_t                        size,
                                            const OST_Imaging_DctScale_t *scale,
                                            const OST_Imaging_Image_t    *out);

/* Undistortion to a fixed output size. output_height may be 0 to keep the
 * camera's aspect ratio. */
OST_Imaging_Status_t OST_Imaging_UndistorterCreate(OST_Imaging_Undistorter_t **undistorter,
                                                   const OST_Geo_Camera_t     *camera,
                                                   int32_t                     output_width,
                                                   int32_t                     output_height,
                                                   int32_t                     channels);
void                 OST_Imaging_UndistorterDestroy(OST_Imaging_Undistorter_t *undistorter);
const char          *OST_Imaging_UndistorterLastError(const OST_Imaging_Undistorter_t *undistorter);

/* Camera model of the output images: scaled input camera without distortion. */
OST_Imaging_Status_t OST_Imaging_UndistorterOutputCamera(const OST_Imaging_Undistorter_t *undistorter,
                                                         OST_Geo_Camera_t                *camera);

OST_Imaging_Status_t OST_Imaging_Undistort(OST_Imaging_Undistorter_t *undistorter,
                                           const OST_Imaging_Image_t *input,
                                           const OST_Imaging_Image_t *out);

/* From a camera's JPEG to an undistorted working image in one call: decodes
 * at the smallest sufficient DCT scale, reduces and undistorts. The output
 * height follows the camera's aspect ratio. */
OST_Imaging_Status_t OST_Imaging_WorkingConverterCreate(OST_Imaging_WorkingConverter_t **converter,
                                                        const OST_Geo_Camera_t          *camera,
                                                        int32_t                          working_width,
                                                        int32_t                          channels);
void                 OST_Imaging_WorkingConverterDestroy(OST_Imaging_WorkingConverter_t *converter);
const char          *OST_Imaging_WorkingConverterLastError(const OST_Imaging_WorkingConverter_t *converter);

/* Camera model of the working images (no distortion). */
OST_Imaging_Status_t OST_Imaging_WorkingConverterCamera(const OST_Imaging_WorkingConverter_t *converter,
                                                        OST_Geo_Camera_t                     *camera);

OST_Imaging_Status_t OST_Imaging_ConvertJpeg(OST_Imaging_WorkingConverter_t *converter,
                                             const uint8_t                  *jpeg,
                                             size_t                          size,
                                             const OST_Imaging_Image_t      *out);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_IMAGING_C_API_H */
