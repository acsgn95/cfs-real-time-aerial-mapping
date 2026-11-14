/*
 * The C interface compiles as C99 and its defaults are those of ADR-006.
 * Returns 0 on success or the number of the failed check.
 */

#include <stddef.h>

#include "selection_c_api.h"

int ost_selection_c_api_usage(void)
{
    OST_Select_Config_t  config;
    OST_Select_Terrain_t terrain = {NULL, "EPSG:4326+3855", NULL, NULL};
    OST_Geo_Camera_t     camera  = {0};
    OST_Select_t        *handle  = NULL;

    OST_Select_DefaultConfig(&config);
    if (config.max_sources != 5 || config.max_tilt_deg != 25.0 || config.min_overlap != 0.4)
    {
        return 1;
    }
    if (OST_Select_Create(&config, &terrain, &camera, &handle) != OST_ERR_INVALID_ARGUMENT || handle != NULL)
    {
        return 2;
    }
    return 0;
}
