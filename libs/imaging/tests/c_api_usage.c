/*
 * Uses the C interface from C99, as a cFS application does: decode a JPEG
 * at a DCT scale, then undistort it to a working resolution. Returns 0 on
 * success or the number of the failed check.
 */

#include <stdlib.h>
#include <string.h>

#include "imaging_c_api.h"

int ost_imaging_c_api_usage(const uint8_t *jpeg, size_t size)
{
    OST_Imaging_Decoder_t     *dec = NULL;
    OST_Imaging_Undistorter_t *und = NULL;
    OST_Imaging_DctScale_t     scale;
    OST_Imaging_Image_t        decoded;
    OST_Imaging_Image_t        working;
    OST_Geo_Camera_t           camera;
    OST_Geo_Camera_t           out_camera;
    int32_t                    w = 0, h = 0;
    int                        result = 0;

    memset(&decoded, 0, sizeof(decoded));
    memset(&working, 0, sizeof(working));

    if (OST_Imaging_DecoderCreate(&dec) != OST_IMAGING_SUCCESS)
    {
        return 1;
    }
    if (OST_Imaging_JpegSize(dec, jpeg, size, &w, &h) != OST_IMAGING_SUCCESS || w != 320 || h != 240)
    {
        result = 2;
        goto done;
    }
    if (OST_Imaging_ChooseDctScale(w, h, 100, 75, &scale) != OST_IMAGING_SUCCESS || scale.width != 120)
    {
        result = 3; /* 3/8 of 320 */
        goto done;
    }

    decoded.width    = scale.width;
    decoded.height   = scale.height;
    decoded.channels = 3;
    decoded.stride   = (size_t)scale.width * 3u;
    decoded.data     = malloc(decoded.stride * (size_t)decoded.height);
    if (decoded.data == NULL || OST_Imaging_DecodeJpeg(dec, jpeg, size, &scale, &decoded) != OST_IMAGING_SUCCESS)
    {
        result = 4;
        goto done;
    }

    memset(&camera, 0, sizeof(camera));
    camera.width  = w;
    camera.height = h;
    camera.fx     = 300.0;
    camera.fy     = 300.0;
    camera.cx     = 159.5;
    camera.cy     = 119.5;
    camera.k1     = -0.05;
    if (OST_Imaging_UndistorterCreate(&und, &camera, 100, 0, 3) != OST_IMAGING_SUCCESS)
    {
        result = 5;
        goto done;
    }
    if (OST_Imaging_UndistorterOutputCamera(und, &out_camera) != OST_IMAGING_SUCCESS || out_camera.height != 75
        || out_camera.k1 != 0.0)
    {
        result = 6;
        goto done;
    }

    working.width    = out_camera.width;
    working.height   = out_camera.height;
    working.channels = 3;
    working.stride   = (size_t)working.width * 3u;
    working.data     = malloc(working.stride * (size_t)working.height);
    if (working.data == NULL || OST_Imaging_Undistort(und, &decoded, &working) != OST_IMAGING_SUCCESS)
    {
        result = 7;
        goto done;
    }
    /* Mismatched output: rejected as a caller error. */
    working.width = 50;
    if (OST_Imaging_Undistort(und, &decoded, &working) != OST_IMAGING_ERR_INVALID_ARGUMENT
        || strlen(OST_Imaging_UndistorterLastError(und)) == 0)
    {
        result = 8;
        goto done;
    }
    if (OST_Imaging_JpegSize(dec, (const uint8_t *)"not a jpeg", 10, &w, &h) != OST_IMAGING_ERR_DECODE)
    {
        result = 9;
        goto done;
    }

done:
    free(decoded.data);
    free(working.data);
    OST_Imaging_UndistorterDestroy(und);
    OST_Imaging_DecoderDestroy(dec);
    return result;
}
