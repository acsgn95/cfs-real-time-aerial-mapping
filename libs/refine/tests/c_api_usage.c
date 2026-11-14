/*
 * Uses the C interface from C99, as REFINE_APP does: configuration,
 * creation and the argument checks. Returns 0 on success or the number of
 * the failed check.
 */

#include <stddef.h>

#include "refine_c_api.h"

int ost_refine_c_api_usage(void)
{
    OST_Refine_Config_t c;
    OST_Refine_Camera_t cam = { 640, 480, 500.0, 500.0, 319.5, 239.5 };
    OST_Refine_t       *h   = NULL;
    OST_Refine_Stats_t  st;
    uint32_t            n = 0;
    int                 has = 1;

    OST_Refine_DefaultConfig(&c);
    if (c.max_window_images == 0 || c.max_points == 0)
    {
        return 1;
    }
    if (OST_Refine_Create(&c, NULL, &h) != OST_ERR_INVALID_ARGUMENT || h != NULL)
    {
        return 2;
    }
    if (OST_Refine_Create(&c, &cam, &h) != OST_SUCCESS)
    {
        return 3;
    }
    if (OST_Refine_HasImage(h, 7, &has) != OST_SUCCESS || has != 0)
    {
        return 4;
    }
    if (OST_Refine_Poll(h, 0, 1, NULL, 0, &n) != OST_SUCCESS || n != 0)
    {
        return 5;
    }
    if (OST_Refine_GetStats(h, &st) != OST_SUCCESS || st.images != 0)
    {
        return 6;
    }
    OST_Refine_Destroy(h);
    OST_Refine_Destroy(NULL);
    return 0;
}
