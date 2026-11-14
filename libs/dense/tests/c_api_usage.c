/*
 * The C interface compiles as C99 and its defaults are those of ADR-005.
 * Returns 0 on success or the number of the failed check.
 */

#include <stddef.h>

#include "dense_c_api.h"

int ost_dense_c_api_usage(void)
{
    OST_Dense_Config_t  config;
    OST_Dense_Terrain_t terrain = {NULL, "EPSG:4326+3855", NULL, NULL};
    OST_Dense_t        *handle  = NULL;

    OST_Dense_DefaultConfig(&config);
    if (config.method != OST_DENSE_METHOD_SGBM || config.sgbm_block_size != 5 || config.terrain_gate_m != 30.0 ||
        config.depth_margin_m != 50.0 || config.pm_iterations != 3 || config.pm_window_radius != 3)
    {
        return 1;
    }
    if (OST_Dense_Create(&config, &terrain, &handle) != OST_ERR_INVALID_ARGUMENT || handle != NULL)
    {
        return 2;
    }
    if (OST_Dense_Estimate(NULL, NULL, NULL, NULL, 0, 0.0, NULL, NULL) != OST_ERR_INVALID_ARGUMENT)
    {
        return 3;
    }
    return 0;
}
