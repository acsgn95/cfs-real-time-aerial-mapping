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
 * Status codes (ADR-020 section 5, ost_status.h):
 * - OST_ERR_INVALID_ARGUMENT: invalid argument, or PROJ cannot be set up
 *   from the given data directory (OST_Geo_Create);
 * - OST_ERR_NUMERIC: PROJ cannot apply a transformation, a point lies at or
 *   behind the camera (OST_Geo_ProjectPoint), or the undistortion does not
 *   converge (OST_Geo_UndistortPixel);
 * - OST_ERR_NO_MEMORY, OST_ERR_INTERNAL.
 * OST_Geo_LastError gives the message of the last failed call on a context.
 * Functions that need PROJ take a context created by OST_Geo_Create. A
 * context is not thread-safe: each task uses its own. No function throws or
 * keeps threads running after it returns.
 */

#ifndef ORTHOSTREAM_GEO_C_API_H
#define ORTHOSTREAM_GEO_C_API_H

#include <stdint.h>

#include "ost_status.h"

#ifdef __cplusplus
extern "C" {
#endif

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

/* Creates a context. proj_data_dir is the directory holding proj.db. */
OST_Status_t OST_Geo_Create(OST_Geo_Context_t **context, const char *proj_data_dir);
void             OST_Geo_Destroy(OST_Geo_Context_t *context);

/* Message of the last failed call on this context; empty if none. Valid
 * until the next call on the context. */
const char *OST_Geo_LastError(const OST_Geo_Context_t *context);

OST_Status_t OST_Geo_EcefToGeodetic(OST_Geo_Context_t *context,
                                        const double       ecef[3],
                                        double            *latitude_rad,
                                        double            *longitude_rad,
                                        double            *height_m);

OST_Status_t OST_Geo_GeodeticToEcef(OST_Geo_Context_t *context,
                                        double             latitude_rad,
                                        double             longitude_rad,
                                        double             height_m,
                                        double             ecef[3]);

/* R_E_L: rotation from local ENU at the given position to ECEF. */
OST_Status_t OST_Geo_RotationEcefFromEnu(double latitude_rad, double longitude_rad, double rotation[9]);

OST_Status_t OST_Geo_CameraFromMetashape(const OST_Geo_MetashapeCalibration_t *calibration,
                                             OST_Geo_Camera_t                     *camera);

/* Projects a point in the camera frame to pixel coordinates; OST_ERR_NUMERIC
 * for a point at or behind the camera. */
OST_Status_t OST_Geo_ProjectPoint(const OST_Geo_Camera_t *camera, const double point_camera[3], double pixel[2]);

/* Normalised undistorted image coordinates (x / z, y / z) of a pixel;
 * OST_ERR_NUMERIC if the iterative inversion does not converge. */
OST_Status_t OST_Geo_UndistortPixel(const OST_Geo_Camera_t *camera, const double pixel[2], double normalized[2]);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_GEO_C_API_H */
