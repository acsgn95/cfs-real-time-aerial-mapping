/*
 * C interface of the OrthoStream selection (ADR-006, ADR-011) for
 * SELECT_APP: orthostream/selection/selector.hpp with the terrain prior of
 * libs/terrain.
 *
 *   OST_Select_Create      with the configuration table, the terrain files
 *                          and the camera of the session (with distortion)
 *   OST_Select_AddImage    each image with a valid pose, in arrival order:
 *                          footprint, flight line, keyframe decision, and
 *                          for a keyframe its source views from the pool
 *   OST_Select_Evict       pool images to release after each image
 *   OST_Select_EvictOldest one more image when memory is short
 *   OST_Select_Reset       at a session change
 *
 * Poses are camera centres in ECEF and q_E_C (Hamilton w, x, y, z), as in
 * the Software Bus messages (ADR-016). The terrain window is read around
 * the first image and read again when the camera comes within half the
 * radius of its edge.
 *
 * Status codes (ost_status.h): OST_ERR_INVALID_ARGUMENT for invalid
 * arguments or configuration; OST_ERR_IO if the elevation model or the
 * PROJ data cannot be opened (OST_Select_Create); OST_ERR_INTERNAL for
 * unexpected errors. An image without terrain is not an error: its
 * decision has no footprint and terrain_available = 0.
 * OST_Select_LastError gives the text of the last failed call or of the
 * last terrain failure.
 *
 * A handle is used by one task at a time. No function throws or starts a
 * thread.
 */

#ifndef ORTHOSTREAM_SELECTION_C_API_H
#define ORTHOSTREAM_SELECTION_C_API_H

#include <stdint.h>

#include "geo_c_api.h"
#include "ost_status.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OST_SELECT_MAX_SOURCES 16

typedef struct OST_Select OST_Select_t;

typedef struct
{
    /* Footprint (section 1) */
    double  nominal_height_agl_m;
    int32_t footprint_iterations;
    int32_t max_sources; /* K (section 5), at most OST_SELECT_MAX_SOURCES */

    /* Keyframes (section 4) */
    double max_tilt_deg;
    double target_overlap;

    /* Source views (section 5) */
    double min_overlap;
    double min_baseline_ratio;
    double max_baseline_ratio;
    double max_view_angle_deg;
    double preferred_triangulation_angle_deg;

    /* Flight lines, pool (section 6) and terrain window */
    double   line_turn_deg;
    double   pool_distance_m;
    uint32_t max_pool_images;
    uint32_t spare;
    double   terrain_radius_m;
} OST_Select_Config_t;

typedef struct
{
    const char *dem_path;      /* raster readable by GDAL */
    const char *dem_crs;       /* CRS with heights, for example "EPSG:4326+3855" */
    const char *proj_data_dir; /* directory of proj.db */
    const char *grid_dir;      /* directory of the transformation grids; may be empty */
} OST_Select_Terrain_t;

typedef struct
{
    uint64_t id;
    double   overlap;
    double   baseline_ratio;
    double   triangulation_angle_deg;
    double   score;
    uint8_t  other_line;
    uint8_t  spare[7];
} OST_Select_Source_t;

typedef struct
{
    uint8_t  footprint;         /* 1: footprint computed; the image joined the pool */
    uint8_t  keyframe;
    uint8_t  terrain_available; /* 0: no terrain window covers the image */
    uint8_t  spare;
    int32_t  flight_line;
    double   tilt_deg;
    double   overlap_last;      /* with the last keyframe; -1 if none */
    double   plane_height_m;    /* ellipsoidal height of the footprint plane at its centre */
    double   height_spread_m;
    double   width_m;
    double   height_m;
    double   centre_ecef[3];    /* footprint centre */
    uint32_t source_count;
    uint32_t spare2;
    OST_Select_Source_t sources[OST_SELECT_MAX_SOURCES];
} OST_Select_Decision_t;

typedef struct
{
    uint32_t images;
    uint32_t footprints;
    uint32_t keyframes;
    uint32_t pool;
    uint32_t evicted;
    uint32_t flight_lines;
    uint32_t terrain_loads;
    uint8_t  terrain_loaded;
    uint8_t  spare[3];
} OST_Select_Stats_t;

/* The defaults of ADR-006. */
void OST_Select_DefaultConfig(OST_Select_Config_t *config);

OST_Status_t OST_Select_Create(const OST_Select_Config_t *config, const OST_Select_Terrain_t *terrain,
                               const OST_Geo_Camera_t *camera, OST_Select_t **handle);
void         OST_Select_Destroy(OST_Select_t *handle);
const char  *OST_Select_LastError(const OST_Select_t *handle);

OST_Status_t OST_Select_AddImage(OST_Select_t *handle, uint64_t id, const double position_ecef[3],
                                 const double q_ecef_from_camera[4], OST_Select_Decision_t *decision);

/* Returns up to capacity pool images to release; the others stay queued
 * for the next call. */
OST_Status_t OST_Select_Evict(OST_Select_t *handle, uint64_t *ids, uint32_t capacity, uint32_t *count);

/* found = 0 if the pool is empty. */
OST_Status_t OST_Select_EvictOldest(OST_Select_t *handle, uint64_t *id, int *found);

/* Removes an image from the pool without returning it; found = 0 if it is
 * not there. */
OST_Status_t OST_Select_Remove(OST_Select_t *handle, uint64_t id, int *found);

/* New session with this camera. The caller releases the data of all its
 * pool images itself; the pool is emptied without returning them. */
OST_Status_t OST_Select_Reset(OST_Select_t *handle, const OST_Geo_Camera_t *camera);

OST_Status_t OST_Select_GetStats(const OST_Select_t *handle, OST_Select_Stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_SELECTION_C_API_H */
