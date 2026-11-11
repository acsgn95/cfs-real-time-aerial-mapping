/*
 * C interface of the OrthoStream geo library (ADR-011).
 *
 * Conventions (ADR-004, ADR-008):
 * - Lengths in metres, angles in radians, geodetic coordinates on WGS 84.
 * - ECEF is WGS 84 Earth-centred, Earth-fixed.
 * - Matrices are row-major 3x3 arrays.
 * - Camera model: OpenCV pinhole with k1, k2, k3, p1, p2; pixel origin at the
 *   centre of the top-left pixel; camera axes x right, y down, z forward.
 *
 * Functions that need PROJ take a context created by OST_Geo_Create. A
 * context is not thread-safe: each task uses its own. No function throws or
 * keeps threads running after it returns.
 */

#ifndef ORTHOSTREAM_GEO_C_API_H
#define ORTHOSTREAM_GEO_C_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    OST_GEO_SUCCESS                = 0,
    OST_GEO_ERR_INVALID_ARGUMENT   = -1,
    OST_GEO_ERR_PROJ               = -2, /* PROJ could not create or apply a transformation */
    OST_GEO_ERR_BEHIND_CAMERA      = -3, /* point at or behind the camera */
    OST_GEO_ERR_NO_CONVERGENCE     = -4, /* iterative inversion did not converge */
    OST_GEO_ERR_OUT_OF_MEMORY      = -5,
    OST_GEO_ERR_INTERNAL           = -6
} OST_Geo_Status_t;

typedef struct
{
    int32_t width;
    int32_t height;
    double  fx;
    double  fy;
    double  cx;
    double  cy;
    double  k1;
    double  k2;
    double  k3;
    double  p1;
    double  p2;
} OST_Geo_Camera_t;

/* Agisoft Metashape frame calibration: principal point as offset from the
 * image centre; centre of the top-left pixel at (0.5, 0.5). */
typedef struct
{
    int32_t width;
    int32_t height;
    double  f;
    double  cx;
    double  cy;
    double  k1;
    double  k2;
    double  k3;
    double  p1;
    double  p2;
} OST_Geo_MetashapeCalibration_t;

typedef struct OST_Geo_Context OST_Geo_Context_t;

/* Short description of a status code; never NULL. */
const char *OST_Geo_StatusText(OST_Geo_Status_t status);

/* Creates a context. proj_data_dir is the directory holding proj.db. */
OST_Geo_Status_t OST_Geo_Create(OST_Geo_Context_t **context, const char *proj_data_dir);
void             OST_Geo_Destroy(OST_Geo_Context_t *context);

/* Message of the last failed call on this context; empty if none. Valid
 * until the next call on the context. */
const char *OST_Geo_LastError(const OST_Geo_Context_t *context);

OST_Geo_Status_t OST_Geo_EcefToGeodetic(OST_Geo_Context_t *context,
                                        const double       ecef[3],
                                        double            *latitude_rad,
                                        double            *longitude_rad,
                                        double            *height_m);

OST_Geo_Status_t OST_Geo_GeodeticToEcef(OST_Geo_Context_t *context,
                                        double             latitude_rad,
                                        double             longitude_rad,
                                        double             height_m,
                                        double             ecef[3]);

/* R_E_L: rotation from local ENU at the given position to ECEF. */
OST_Geo_Status_t OST_Geo_RotationEcefFromEnu(double latitude_rad, double longitude_rad, double rotation[9]);

OST_Geo_Status_t OST_Geo_CameraFromMetashape(const OST_Geo_MetashapeCalibration_t *calibration,
                                             OST_Geo_Camera_t                     *camera);

/* Projects a point in the camera frame to pixel coordinates. */
OST_Geo_Status_t OST_Geo_ProjectPoint(const OST_Geo_Camera_t *camera, const double point_camera[3], double pixel[2]);

/* Normalised undistorted image coordinates (x / z, y / z) of a pixel. */
OST_Geo_Status_t OST_Geo_UndistortPixel(const OST_Geo_Camera_t *camera, const double pixel[2], double normalized[2]);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_GEO_C_API_H */
