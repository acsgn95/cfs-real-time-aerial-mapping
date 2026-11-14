/*
 * The C interface compiles as C99 and its defaults are those of ADR-007.
 * Returns 0 on success or the number of the failed check.
 */

#include <stddef.h>
#include <string.h>

#include "map_products_c_api.h"

int ost_map_c_api_usage(void)
{
    OST_Map_GeorefConfig_t config;
    OST_Map_Georef_t      *handle = NULL;
    OST_Map_GeorefResult_t result;
    uint32_t               points = 0;

    OST_Map_GeorefDefaultConfig(&config);
    if (strcmp(config.projected_crs, "EPSG:2056") != 0 || strcmp(config.vertical_crs, "EPSG:5729") != 0 ||
        config.cell_size_m != 0.26 || config.tile_cells != 512 || config.min_confidence != 0.0)
    {
        return 1;
    }
    /* No PROJ data directory. */
    if (OST_Map_GeorefCreate(&config, &handle) != OST_ERR_INVALID_ARGUMENT || handle != NULL)
    {
        return 2;
    }
    if (OST_Map_GeorefCount(NULL, NULL, &points) != OST_ERR_INVALID_ARGUMENT ||
        OST_Map_GeorefConvert(NULL, NULL, NULL, NULL, NULL, 0, NULL, 0, &result) != OST_ERR_INVALID_ARGUMENT ||
        OST_Map_GeorefWriteLaz(NULL, 0, NULL, NULL) != OST_ERR_INVALID_ARGUMENT)
    {
        return 3;
    }
    if (sizeof(OST_Map_Point_t) != 20 || sizeof(OST_Map_TileSpan_t) != 16)
    {
        return 4;
    }
    OST_Map_GeorefDestroy(NULL);
    {
        OST_Map_ProductsConfig_t products;
        OST_Map_Products_t      *map = NULL;
        OST_Map_WriteResult_t    written;

        OST_Map_ProductsDefaultConfig(&products);
        if (products.cell_size_m != 0.26 || products.tile_cells != 512 || products.min_observations != 3 ||
            products.rejection_sigmas != 3.0 || products.min_sigma_m != 0.25 || products.ortho_pixel_m != 0.066 ||
            products.surface_smoothing_m != 1.0 || products.switch_margin_deg != 2.0 ||
            products.max_active_tiles != 64 || products.finalise_distance_m != 600.0 || products.defer_writes != 1)
        {
            return 5;
        }
        /* No output directory. */
        if (OST_Map_ProductsCreate(&products, &map) != OST_ERR_INVALID_ARGUMENT || map != NULL ||
            OST_Map_ProductsPublish(NULL, &written) != OST_ERR_INVALID_ARGUMENT)
        {
            return 6;
        }
        OST_Map_ProductsDestroy(NULL);
    }
    return 0;
}
