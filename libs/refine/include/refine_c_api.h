/*
 * C interface of the OrthoStream pose refinement (ADR-011, ADR-019): the
 * sliding window of orthostream/refine/sliding.hpp for REFINE_APP.
 *
 *   OST_Refine_Create        once, with the working camera of the session
 *   OST_Refine_AddImage      each image of a batch that the handle does not
 *                            have yet (OST_Refine_HasImage); features are
 *                            extracted here, the image data is not kept
 *   OST_Refine_AddBatch      the batch: keyframe first, then its sources
 *   OST_Refine_Poll          the batches that are ready, with their poses
 *
 * Poses are camera centres in ECEF and q_E_C (Hamilton w, x, y, z, camera
 * to ECEF), as in the Software Bus messages (ADR-016). Internally the
 * handle uses a local east-north-up frame at its first image. Images are
 * undistorted 8-bit grey working images of the given camera.
 *
 * Status codes (ost_status.h): OST_ERR_INVALID_ARGUMENT for invalid
 * arguments (unknown image, wrong image size, repeated identifier),
 * OST_ERR_UNSUPPORTED in a build without GTSAM (never: the library is only
 * built with it), OST_ERR_INTERNAL for unexpected errors. A batch whose
 * refinement fails is not an error: its result is marked not refined and
 * carries the telemetry poses (ADR-019 section 5). OST_Refine_LastError
 * gives the text of the last failed call.
 *
 * A handle is used by one task at a time. No function throws, and no
 * thread keeps running after a call returns.
 */

#ifndef ORTHOSTREAM_REFINE_C_API_H
#define ORTHOSTREAM_REFINE_C_API_H

#include <stddef.h>
#include <stdint.h>

#include "ost_status.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OST_REFINE_MAX_BATCH_IMAGES 16
#define OST_REFINE_REASON_LEN       96

typedef struct OST_Refine OST_Refine_t;

typedef struct
{
    /* Priors (ADR-019 sections 1 and 3). */
    uint8_t  precise_position;   /* 1: PPK/RTK positions, held by a tight prior */
    uint8_t  estimate_boresight; /* 1: shared mounting rotation */
    uint8_t  spare[2];
    double   position_sigma_m;         /* when the image reports none */
    double   precise_position_sigma_m;
    double   attitude_sigma_deg;       /* when the image reports none */
    double   boresight_sigma_deg;      /* until the boresight is held */
    uint32_t boresight_settle_batches; /* refined batches before it is held */
    double   held_boresight_sigma_deg;
    double   refined_attitude_sigma_deg; /* prior of images refined before */

    /* Features and the local solution. */
    int32_t  max_features;
    uint32_t max_window_images;
    uint32_t max_points;
    int32_t  max_iterations;

    /* Waiting and memory (sections 3 and 6). */
    double max_wait_s;
    double window_s;

    /* Checks (section 5). */
    uint32_t min_image_observations;
    double   max_reprojection_rms_px;
    double   max_correction_sigmas;
} OST_Refine_Config_t;

typedef struct
{
    int32_t width;
    int32_t height;
    double  fx, fy, cx, cy; /* pixels; no distortion */
} OST_Refine_Camera_t;

typedef struct
{
    double position_ecef[3];
    double q_ecef_from_camera[4]; /* w x y z */
} OST_Refine_Pose_t;

typedef struct
{
    uint64_t          id;             /* unique per session, for example the image sequence number */
    int64_t           capture_utc_ns;
    const uint8_t    *grey;
    int32_t           width;
    int32_t           height;
    size_t            stride;
    OST_Refine_Pose_t pose;               /* telemetry pose */
    double            attitude_sigma_deg; /* reported accuracy; 0: the configured one */
    double            position_sigma_m;   /* reported accuracy; 0: the configured one */
} OST_Refine_Image_t;

typedef struct
{
    uint64_t          batch_id;
    uint8_t           refined; /* 1: refined; 0: refine_failed, telemetry poses */
    uint8_t           covered; /* 1: the next flight line covered it; 0: waiting limit */
    uint8_t           boresight_held;
    uint8_t           spare;
    uint32_t          count;   /* images, in the order given to OST_Refine_AddBatch */
    OST_Refine_Pose_t poses[OST_REFINE_MAX_BATCH_IMAGES];
    double            waited_s;      /* data time from the keyframe capture */
    double            rms_px;
    double            boresight_deg;
    double            matching_s;
    double            adjustment_s;
    uint32_t          window_images;
    uint32_t          points;
    uint32_t          iterations;
    char              reason[OST_REFINE_REASON_LEN]; /* empty when refined */
} OST_Refine_Result_t;

typedef struct
{
    uint32_t images;          /* in the window */
    uint32_t pending_batches;
    uint32_t cached_pairs;
    uint32_t refined_batches;
    uint32_t failed_batches;
    uint32_t queued_results;  /* solved, not yet returned by OST_Refine_Poll */
    double   features_s;      /* total feature extraction time */
} OST_Refine_Stats_t;

/* The defaults of ADR-019 and the sliding-window benchmark. */
void OST_Refine_DefaultConfig(OST_Refine_Config_t *config);

OST_Status_t OST_Refine_Create(const OST_Refine_Config_t *config, const OST_Refine_Camera_t *camera,
                               OST_Refine_t **handle);
void         OST_Refine_Destroy(OST_Refine_t *handle);
const char  *OST_Refine_LastError(const OST_Refine_t *handle);

OST_Status_t OST_Refine_HasImage(const OST_Refine_t *handle, uint64_t id, int *has);
OST_Status_t OST_Refine_AddImage(OST_Refine_t *handle, const OST_Refine_Image_t *image);

/* ids[0] is the keyframe; count <= OST_REFINE_MAX_BATCH_IMAGES; all images
 * added before. plane_height_m: ellipsoidal height of the footprint plane. */
OST_Status_t OST_Refine_AddBatch(OST_Refine_t *handle, uint64_t batch_id, const uint64_t *ids, uint32_t count,
                                 double plane_height_m);

/* Solves the batches ready at data time now_utc_ns (all of them with flush)
 * and returns up to capacity results, oldest first; further results stay
 * queued for the next call. */
OST_Status_t OST_Refine_Poll(OST_Refine_t *handle, int64_t now_utc_ns, int flush, OST_Refine_Result_t *results,
                             uint32_t capacity, uint32_t *count);

OST_Status_t OST_Refine_GetStats(const OST_Refine_t *handle, OST_Refine_Stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_REFINE_C_API_H */
