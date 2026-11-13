/*
 * Uses the C interface from C99, as a cFS application does. Compiled as C,
 * so the header is checked for C compatibility. Returns 0 on success or the
 * number of the failed check.
 */

#include <math.h>
#include <string.h>

#include "geo_c_api.h"

int ost_geo_c_api_usage(const char *proj_data_dir)
{
    OST_Geo_Context_t             *ctx = NULL;
    OST_Geo_MetashapeCalibration_t ms;
    OST_Geo_Camera_t               cam;
    double                         ecef[3] = { 4309141.730, 747595.197, 4629311.867 };
    double                         back[3];
    double                         lat, lon, h;
    double                         rot[9];
    double                         point[3] = { 0.5, -0.25, 10.0 };
    double                         pixel[2];
    double                         normalized[2];

    if (OST_Geo_Create(&ctx, proj_data_dir) != OST_SUCCESS || ctx == NULL)
    {
        return 1;
    }
    if (OST_Geo_EcefToGeodetic(ctx, ecef, &lat, &lon, &h) != OST_SUCCESS)
    {
        return 2;
    }
    if (OST_Geo_GeodeticToEcef(ctx, lat, lon, h, back) != OST_SUCCESS || fabs(back[0] - ecef[0]) > 1e-6)
    {
        return 3;
    }
    if (OST_Geo_RotationEcefFromEnu(lat, lon, rot) != OST_SUCCESS)
    {
        return 4;
    }
    if (OST_Geo_EcefToGeodetic(ctx, NULL, &lat, &lon, &h) != OST_ERR_INVALID_ARGUMENT)
    {
        return 5;
    }

    memset(&ms, 0, sizeof(ms));
    ms.width  = 7952;
    ms.height = 5304;
    ms.f      = 7466.67;
    if (OST_Geo_CameraFromMetashape(&ms, &cam) != OST_SUCCESS || cam.cx != 3975.5)
    {
        return 6;
    }
    if (OST_Geo_ProjectPoint(&cam, point, pixel) != OST_SUCCESS)
    {
        return 7;
    }
    if (OST_Geo_UndistortPixel(&cam, pixel, normalized) != OST_SUCCESS
        || fabs(normalized[0] - point[0] / point[2]) > 1e-12)
    {
        return 8;
    }
    point[2] = -1.0;
    if (OST_Geo_ProjectPoint(&cam, point, pixel) != OST_ERR_NUMERIC)
    {
        return 9;
    }
    if (strlen(OST_Geo_LastError(ctx)) != 0)
    {
        return 10;
    }

    OST_Geo_Destroy(ctx);
    return 0;
}
