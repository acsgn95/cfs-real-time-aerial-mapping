/*
 * Uses the C interface from C99, as ALIGN_APP does: a level body flying
 * north, two telemetry samples and one image between them. Returns 0 on
 * success or the number of the failed check.
 */

#include <math.h>
#include <stddef.h>

#include "time_align_c_api.h"

int ost_time_align_c_api_usage(void)
{
    OST_Align_Config_t    config;
    OST_Align_t          *align = NULL;
    OST_Align_Sample_t    s;
    OST_Align_ImagePose_t pose;
    uint32_t              count   = 0;
    int                   verdict = -1;
    const double          q_b_c[4] = {1.0, 0.0, 0.0, 0.0};
    const double          t_b_c[3] = {0.0, 0.0, 0.0};
    const int64_t         t0       = 1763100000000000000LL;
    int                   result   = 0;

    OST_Align_DefaultConfig(&config);
    if (OST_Align_Create(&config, &align) != OST_SUCCESS)
    {
        return 1;
    }
    if (OST_Align_SetMounting(align, q_b_c, t_b_c) != OST_SUCCESS)
    {
        result = 2;
        goto done;
    }

    s.time_ns            = t0;
    s.latitude_deg       = 0.0;
    s.longitude_deg      = 0.0;
    s.height_m           = 100.0;
    s.q_ned_from_body[0] = 1.0;
    s.q_ned_from_body[1] = 0.0;
    s.q_ned_from_body[2] = 0.0;
    s.q_ned_from_body[3] = 0.0;
    s.accuracy.position_std_north_m   = 0.02f;
    s.accuracy.position_std_east_m    = 0.02f;
    s.accuracy.position_std_down_m    = 0.03f;
    s.accuracy.attitude_std_roll_deg  = 3.0f;
    s.accuracy.attitude_std_pitch_deg = 3.0f;
    s.accuracy.attitude_std_yaw_deg   = 3.0f;
    s.accuracy.fix                    = 6;
    if (OST_Align_AddSample(align, &s, &verdict) != OST_SUCCESS || verdict != OST_ALIGN_SAMPLE_ACCEPTED)
    {
        result = 3;
        goto done;
    }
    s.time_ns      = t0 + 100000000;
    s.latitude_deg = 1e-5; /* about 1.1 m north */
    if (OST_Align_AddSample(align, &s, &verdict) != OST_SUCCESS || verdict != OST_ALIGN_SAMPLE_ACCEPTED)
    {
        result = 4;
        goto done;
    }
    if (OST_Align_AddImage(align, 42u, t0 + 50000000, 0) != OST_SUCCESS)
    {
        result = 5;
        goto done;
    }
    if (OST_Align_Poll(align, 0, &pose, 1, &count) != OST_SUCCESS || count != 1u)
    {
        result = 6;
        goto done;
    }
    if (pose.id != 42u || pose.result != OST_ALIGN_RESULT_VALID)
    {
        result = 7;
        goto done;
    }
    /* Halfway north on the equator: ECEF z about 0.55 m. */
    if (fabs(pose.position_ecef[2] - 0.5528) > 1e-3 || fabs(pose.position_ecef[0] - 6378237.0) > 1e-3)
    {
        result = 8;
        goto done;
    }

done:
    OST_Align_Destroy(align);
    return result;
}
