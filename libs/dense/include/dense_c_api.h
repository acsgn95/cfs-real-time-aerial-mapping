/*
 * C interface of the OrthoStream dense reconstruction (ADR-005, ADR-011)
 * for DENSE_APP: the depth estimators of orthostream/dense with the terrain
 * prior of libs/terrain.
 *
 *   OST_Dense_Create    with the configuration table and the terrain files
 *   OST_Dense_Estimate  one keyframe batch: working frame L_k, depth range
 *                       over the terrain, SGBM or PatchMatch, terrain gate
 *                       (ADR-005 sections 1, 4 and 5a)
 *
 * Poses are camera centres in ECEF and q_E_C (Hamilton w, x, y, z), as in
 * the Software Bus messages (ADR-016). The batch frame L_k is the local
 * east-north-up frame at the keyframe's camera centre (ADR-004 section 4);
 * OST_Dense_Result_t gives its origin. The terrain window is read around
 * the first keyframe and read again when a keyframe comes within half the
 * radius of its edge.
 *
 * Output maps are caller-owned, camera width x height pixels, row-major:
 * depth (z in the reference camera, metres, 0 where invalid), confidence
 * in [0, 1], and optionally normals (unit vectors in the reference camera,
 * x, y, z interleaved; SGBM gives normals pointing towards the camera).
 *
 * Status codes (ost_status.h): OST_ERR_INVALID_ARGUMENT for invalid
 * arguments, configuration or batches (no sources, camera looking too far
 * from nadir); OST_ERR_IO if the elevation model or the PROJ data cannot be
 * opened (OST_Dense_Create); OST_ERR_UNSUPPORTED for PatchMatch in a build
 * without COLMAP; OST_ERR_GPU when the GPU has less free memory than the
 * configured minimum (PatchMatch is not started then); OST_ERR_NUMERIC if
 * the estimator fails on the batch; OST_ERR_INTERNAL for unexpected
 * errors. A keyframe without terrain is not an error: its depth range
 * follows the footprint plane height and the terrain gate is skipped
 * (terrain_available = 0). OST_Dense_LastError gives the text of the last
 * failed call or of the last terrain failure.
 *
 * Known limitation (ADR-005 open question): COLMAP ends the process on
 * CUDA errors. The free-memory check avoids the most likely one (no GPU
 * memory); other CUDA errors still end the process.
 *
 * A handle is used by one task at a time. No function throws. OpenCV runs
 * its own worker threads inside OST_Dense_Estimate (threads).
 */

#ifndef ORTHOSTREAM_DENSE_C_API_H
#define ORTHOSTREAM_DENSE_C_API_H

#include <stddef.h>
#include <stdint.h>

#include "ost_status.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OST_DENSE_MAX_SOURCES 16

typedef enum
{
    OST_DENSE_METHOD_SGBM       = 0, /* default (ADR-005 section 1) */
    OST_DENSE_METHOD_PATCHMATCH = 1  /* alternative (section 4) */
} OST_Dense_Method_t;

typedef struct OST_Dense OST_Dense_t;

typedef struct
{
    int32_t method;  /* OST_Dense_Method_t */
    int32_t threads; /* OpenCV worker threads; 0: OpenCV's default */

    /* SGBM (section 5) */
    double  sgbm_target_baseline_ratio;
    int32_t sgbm_block_size;
    int32_t sgbm_p1_factor;
    int32_t sgbm_p2_factor;
    int32_t sgbm_uniqueness_ratio;
    int32_t sgbm_disp12_max_diff;
    int32_t sgbm_speckle_window;
    int32_t sgbm_speckle_range;
    int32_t sgbm_max_disparities;

    /* PatchMatch (section 5) */
    int32_t pm_gpu_index;
    int32_t pm_iterations;
    int32_t pm_window_radius;
    int32_t pm_window_step;
    int32_t pm_samples;
    int32_t pm_filter_min_consistent;
    double  pm_filter_min_ncc;
    double  pm_filter_min_angle_deg;
    double  pm_min_free_gpu_mb; /* 0: no check */

    /* Depth range and terrain gate (section 5a) */
    double depth_margin_m;    /* added below and above the terrain heights */
    double terrain_gate_m;    /* 0: no gate */
    double gate_spacing_m;    /* height grid spacing */
    double gate_margin_m;     /* grid margin around the footprint */
    double terrain_radius_m;  /* terrain window held in memory */
} OST_Dense_Config_t;

typedef struct
{
    const char *dem_path;      /* raster readable by GDAL */
    const char *dem_crs;       /* CRS with heights, for example "EPSG:4326+3855" */
    const char *proj_data_dir; /* directory of proj.db */
    const char *grid_dir;      /* directory of the transformation grids; may be empty */
} OST_Dense_Terrain_t;

/* Pinhole camera of the undistorted working images. */
typedef struct
{
    int32_t width;
    int32_t height;
    double  fx;
    double  fy;
    double  cx;
    double  cy;
} OST_Dense_Camera_t;

/* One image of the batch: 8-bit grey, camera size, stride in bytes. */
typedef struct
{
    const uint8_t *grey;
    size_t         stride;
    double         position_ecef[3];
    double         q_ecef_from_camera[4];
} OST_Dense_View_t;

typedef struct
{
    float *depth;
    float *confidence;
    float *normals; /* may be NULL */
} OST_Dense_Output_t;

typedef struct
{
    double   origin_ecef[3];     /* L_k origin: the keyframe's camera centre */
    double   latitude_rad;       /* of the origin; L_k is east-north-up there */
    double   longitude_rad;
    double   height_m;           /* ellipsoidal */
    double   depth_min_m;        /* depth range searched */
    double   depth_max_m;
    uint32_t valid_pixels;       /* depth > 0 after the gate */
    uint32_t gated_pixels;       /* removed by the terrain gate */
    int32_t  source_index;       /* SGBM: the source used; -1: all (PatchMatch) */
    uint8_t  terrain_available;  /* 0: depth range from the plane height, no gate */
    uint8_t  gate_applied;
    uint8_t  spare[2];
    uint32_t terrain_loads;      /* windows read since Create */
    double   setup_s;            /* frame, terrain grid and depth range */
    double   estimate_s;         /* depth estimation */
    double   gate_s;
} OST_Dense_Result_t;

/* The defaults of ADR-005 (SGBM, 50 m depth margin, 30 m gate). */
void OST_Dense_DefaultConfig(OST_Dense_Config_t *config);

/* 1 if this build contains PatchMatch (COLMAP). */
int OST_Dense_PatchMatchAvailable(void);

OST_Status_t OST_Dense_Create(const OST_Dense_Config_t *config, const OST_Dense_Terrain_t *terrain,
                              OST_Dense_t **handle);
void         OST_Dense_Destroy(OST_Dense_t *handle);
const char  *OST_Dense_LastError(const OST_Dense_t *handle);

/* Depth of the reference view. plane_height_m is the ellipsoidal height of
 * the footprint plane under the keyframe (KeyframeBatch
 * FootprintPlaneHeight); it sets the depth range where the terrain prior
 * has no data. */
OST_Status_t OST_Dense_Estimate(OST_Dense_t *handle, const OST_Dense_Camera_t *camera,
                                const OST_Dense_View_t *reference, const OST_Dense_View_t *sources,
                                uint32_t source_count, double plane_height_m, const OST_Dense_Output_t *output,
                                OST_Dense_Result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_DENSE_C_API_H */
