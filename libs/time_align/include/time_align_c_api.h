/*
 * C interface of the OrthoStream time alignment (ADR-011, ADR-014 section 4)
 * for ALIGN_APP: orthostream/time_align/aligner.hpp.
 *
 *   OST_Align_Create       once, with the limits of the configuration table
 *   OST_Align_SetMounting  from the session calibration (q_B_C, t_B_C)
 *   OST_Align_AddSample    each telemetry sample, in arrival order
 *   OST_Align_AddImage     each image, with its arrival time
 *   OST_Align_Poll         the images whose outcome is known, in the order
 *                          they were added
 *   OST_Align_Clear        at a session change
 *
 * Samples carry the body pose as reported by the telemetry: geodetic
 * position (WGS 84, degrees, ellipsoidal height) and q_N_B (body to local
 * north-east-down; Hamilton w, x, y, z). Poses come out as the camera centre
 * in ECEF and q_E_C (camera to ECEF, w >= 0), as in the Software Bus
 * messages (ADR-016 section 2). Sample and capture times are data time (UTC
 * ns); arrival and poll times are in one clock that the caller chooses.
 *
 * Status codes (ost_status.h): OST_ERR_INVALID_ARGUMENT for invalid
 * arguments, OST_ERR_INTERNAL for unexpected errors. A rejected sample or
 * image is not an error: it is a verdict or a result. OST_Align_LastError
 * gives the text of the last failed call.
 *
 * A handle is used by one task at a time. No function throws or starts a
 * thread.
 */

#ifndef ORTHOSTREAM_TIME_ALIGN_C_API_H
#define ORTHOSTREAM_TIME_ALIGN_C_API_H

#include <stdint.h>

#include "ost_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct OST_Align OST_Align_t;

typedef struct
{
    int64_t  max_gap_ns;  /* gap between the two samples around an image */
    int64_t  max_wait_ns; /* waiting time for the next sample */
    int64_t  buffer_ns;   /* telemetry kept, back from the newest sample */
    uint32_t max_pending; /* images waiting at the same time */
    uint32_t spare;
} OST_Align_Config_t;

/* One standard deviation; fix: OST/PositionFix. */
typedef struct
{
    float   position_std_north_m;
    float   position_std_east_m;
    float   position_std_down_m;
    float   attitude_std_roll_deg;
    float   attitude_std_pitch_deg;
    float   attitude_std_yaw_deg;
    uint8_t fix;
    uint8_t spare[3];
} OST_Align_Accuracy_t;

typedef struct
{
    int64_t              time_ns;
    double               latitude_deg;
    double               longitude_deg;
    double               height_m;
    double               q_ned_from_body[4]; /* w x y z */
    OST_Align_Accuracy_t accuracy;
} OST_Align_Sample_t;

/* Verdicts of OST_Align_AddSample. */
#define OST_ALIGN_SAMPLE_ACCEPTED     0
#define OST_ALIGN_SAMPLE_OUT_OF_ORDER 1
#define OST_ALIGN_SAMPLE_DUPLICATE    2
#define OST_ALIGN_SAMPLE_INVALID      3 /* not finite, not a unit quaternion, or no position fix */

/* Results of an image; the values of the EDS enumeration ALIGN_APP/AlignResult. */
#define OST_ALIGN_RESULT_VALID            0
#define OST_ALIGN_RESULT_NO_SAMPLE_BEFORE 1
#define OST_ALIGN_RESULT_GAP_TOO_LARGE    2
#define OST_ALIGN_RESULT_WAIT_EXPIRED     3
#define OST_ALIGN_RESULT_NO_CALIBRATION   4
#define OST_ALIGN_RESULT_QUEUE_FULL       5
#define OST_ALIGN_RESULT_CLEARED          6

typedef struct
{
    uint64_t             id;
    int64_t              capture_ns;
    uint8_t              result; /* OST_ALIGN_RESULT_* */
    uint8_t              spare[7];
    double               position_ecef[3];      /* camera centre; valid results only */
    double               q_ecef_from_camera[4]; /* w x y z */
    OST_Align_Accuracy_t accuracy;              /* the worse of the two samples */
    int64_t              before_ns;             /* sample times used; 0 if none */
    int64_t              after_ns;
    int64_t              waited_ns;             /* from arrival to the outcome */
} OST_Align_ImagePose_t;

typedef struct
{
    uint64_t samples_accepted;
    uint64_t samples_out_of_order;
    uint64_t samples_duplicate;
    uint64_t samples_invalid;
    uint32_t buffered_samples;
    uint32_t pending_images;
    int64_t  buffer_span_ns;
} OST_Align_Stats_t;

/* The defaults of ADR-014 section 4: 0.3 s gap, 1 s wait, 10 s buffer, 64 images. */
void OST_Align_DefaultConfig(OST_Align_Config_t *config);

OST_Status_t OST_Align_Create(const OST_Align_Config_t *config, OST_Align_t **handle);
void         OST_Align_Destroy(OST_Align_t *handle);
const char  *OST_Align_LastError(const OST_Align_t *handle);

/* q_body_from_camera: q_B_C (w x y z); camera_in_body: t_B_C in metres. */
OST_Status_t OST_Align_SetMounting(OST_Align_t *handle, const double q_body_from_camera[4],
                                   const double camera_in_body[3]);

/* verdict: OST_ALIGN_SAMPLE_*. */
OST_Status_t OST_Align_AddSample(OST_Align_t *handle, const OST_Align_Sample_t *sample, int *verdict);

OST_Status_t OST_Align_AddImage(OST_Align_t *handle, uint64_t id, int64_t capture_ns, int64_t arrival_ns);

/* Decides the images that can be decided at now_ns and returns up to
 * capacity of them, oldest first; the others stay for the next call. */
OST_Status_t OST_Align_Poll(OST_Align_t *handle, int64_t now_ns, OST_Align_ImagePose_t *poses, uint32_t capacity,
                            uint32_t *count);

/* Discards the samples and the mounting; waiting images come out of the next
 * poll with OST_ALIGN_RESULT_CLEARED. */
OST_Status_t OST_Align_Clear(OST_Align_t *handle);

OST_Status_t OST_Align_GetStats(const OST_Align_t *handle, OST_Align_Stats_t *stats);

/* Short text of a result, for events. */
const char *OST_Align_ResultName(int result);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_TIME_ALIGN_C_API_H */
