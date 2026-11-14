/*
 * C interface of the OrthoStream map products (ADR-007, ADR-011).
 *
 * Georeferencing, for GEOREF_APP (ADR-012 section 1):
 *
 *   OST_Map_GeorefCreate    product frame P, tile grid, point cloud output
 *   OST_Map_GeorefCount     number of points a depth map gives, to size the
 *                           point set
 *   OST_Map_GeorefConvert   depth map to a point set in P, sorted by tile
 *   OST_Map_GeorefWriteLaz  point cloud files of a point set, one LAZ file
 *                           per tile (ADR-011 section 6)
 *
 * Depth maps are those of DENSE_APP: z in the keyframe camera (metres, 0
 * where invalid) and a confidence in [0, 1], float32, camera width x height
 * pixels, row-major. The camera is the pinhole camera of the undistorted
 * working image; the keyframe pose is the camera centre in ECEF and q_E_C
 * (Hamilton w, x, y, z), as in the Software Bus messages (ADR-016).
 *
 * Point set (data store type POINT_SET, ADR-013): OST_Map_Point_t records,
 * 20 bytes each, native byte order. East, north and height are metres from
 * the set's origin (easting, northing and ellipsoidal height of the camera
 * centre in P, rounded to whole metres); the weight is the depth
 * confidence; the colour comes from the working image. Records are ordered
 * by tile (column, then row) and, within a tile, by pixel. The tile spans
 * give each tile's first record and count. Tile (column, row) covers
 * easting [column, column + 1) x T and northing [row, row + 1) x T, with T
 * the cell size times the cells per tile (ADR-007 section 1).
 *
 * Point cloud files: LAS 1.2, point format 2, LAZ, under
 * <output_dir>/points/points_<column>_<row>_kf<keyframe>.laz, heights in
 * the configured vertical datum. A file is written under a temporary name
 * and renamed when complete.
 *
 * Map products, for MAPPROD_APP (ADR-007):
 *
 *   OST_Map_ProductsCreate       DSM fusion, orthomosaic, tile lifecycle and
 *                                writers on the tile grid of P
 *   OST_Map_ProductsAddPoints    fuses a point set into the DSM tiles
 *   OST_Map_ProductsRender       renders a keyframe into the orthomosaic
 *                                over its footprint (ADR-007 section 3)
 *   OST_Map_ProductsUpdate       finalises tiles by distance and budget,
 *                                writing them (section 5)
 *   OST_Map_ProductsPublish      writes the changed active tiles and the
 *                                mosaic indexes (section 6)
 *   OST_Map_ProductsSnapshot     copies the changed active tiles (a few
 *                                milliseconds per tile), and
 *   OST_Map_ProductsWriteSnapshot  writes the copies: publication on a task
 *                                of its own, so that writing (most of a
 *                                second per orthomosaic tile) does not hold
 *                                up the map update
 *   OST_Map_ProductsFinaliseAll  writes and evicts every active tile
 *
 * Products go to <output_dir>/dsm, /ortho, dsm.vrt and ortho.vrt (ADR-007
 * section 7); state files of evicted tiles to <output_dir>/state.
 *
 * Status codes (ost_status.h): OST_ERR_INVALID_ARGUMENT for invalid
 * arguments or configuration, a point set or tile list that is too small,
 * or a pose that is not a unit quaternion; OST_ERR_IO if the PROJ data
 * cannot be used (OST_Map_GeorefCreate) or a point cloud file cannot be
 * written; OST_ERR_UNSUPPORTED for OST_Map_GeorefWriteLaz without an output
 * directory; OST_ERR_INTERNAL for unexpected errors. OST_Map_GeorefLastError
 * gives the text of the last failed call.
 *
 * A handle is used by one task at a time, except that one other task may
 * call OST_Map_ProductsWriteSnapshot (and OST_Map_ProductsPublishError)
 * while the owner works. No function throws.
 */

#ifndef ORTHOSTREAM_MAP_PRODUCTS_C_API_H
#define ORTHOSTREAM_MAP_PRODUCTS_C_API_H

#include <stddef.h>
#include <stdint.h>

#include "ost_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct OST_Map_Georef OST_Map_Georef_t;

typedef struct
{
    const char *projected_crs; /* CRS of P, for example "EPSG:2056" */
    const char *vertical_crs;  /* of the point cloud files, "EPSG:<code>"; empty: ellipsoidal heights */
    const char *proj_data_dir; /* directory of proj.db */
    const char *grid_dir;      /* directory of the PROJ grids; may be empty */
    const char *output_dir;    /* point cloud files go to <output_dir>/points; empty: no files */
    double      cell_size_m;   /* DSM cell size (ADR-007 section 1) */
    int32_t     tile_cells;    /* cells per tile side */
    double      min_confidence; /* pixels below it give no point */
} OST_Map_GeorefConfig_t;

/* Pinhole camera of the undistorted working images. */
typedef struct
{
    int32_t width;
    int32_t height;
    double  fx;
    double  fy;
    double  cx;
    double  cy;
} OST_Map_Camera_t;

typedef struct
{
    OST_Map_Camera_t camera;
    const float     *depth;
    const float     *confidence;
} OST_Map_DepthMap_t;

/* Working image of the keyframe: camera size, 1 (grey) or 3 (RGB) bytes
 * per pixel. */
typedef struct
{
    const uint8_t *data;
    size_t         stride; /* bytes per row */
    int32_t        channels;
} OST_Map_Image_t;

typedef struct
{
    double position_ecef[3];      /* camera centre, m */
    double q_ecef_from_camera[4]; /* w, x, y, z */
} OST_Map_Pose_t;

typedef struct
{
    float   east;   /* m from the origin */
    float   north;
    float   height; /* ellipsoidal, m from the origin height */
    float   weight; /* depth confidence */
    uint8_t rgb[3];
    uint8_t spare;
} OST_Map_Point_t;

typedef struct
{
    int32_t  column;
    int32_t  row;
    uint32_t first; /* index of its first record */
    uint32_t count;
} OST_Map_TileSpan_t;

typedef struct
{
    const OST_Map_Point_t    *points;
    uint32_t                  count;
    uint32_t                  tile_count;
    const OST_Map_TileSpan_t *tiles;
    double                    origin[3]; /* easting, northing, ellipsoidal height */
} OST_Map_PointSet_t;

typedef struct
{
    uint32_t points;
    uint32_t tile_count;
    double   origin[3];
    double   height_min; /* ellipsoidal, of the points */
    double   height_max;
    double   convert_s;  /* depth to P with the colours, ordering by tile */
} OST_Map_GeorefResult_t;

typedef struct
{
    uint32_t files;
    uint32_t points;
    double   write_s;
} OST_Map_LazResult_t;

/* Defaults: EPSG:2056 with LHN95 heights (EPSG:5729), 0.26 m cells, 512
 * cells per tile, minimum confidence 0; paths empty. */
void OST_Map_GeorefDefaultConfig(OST_Map_GeorefConfig_t *config);

OST_Status_t OST_Map_GeorefCreate(const OST_Map_GeorefConfig_t *config, OST_Map_Georef_t **handle);
void         OST_Map_GeorefDestroy(OST_Map_Georef_t *handle);

/* Text of the last failed call; "" if none. */
const char *OST_Map_GeorefLastError(const OST_Map_Georef_t *handle);

OST_Status_t OST_Map_GeorefCount(OST_Map_Georef_t *handle, const OST_Map_DepthMap_t *map, uint32_t *points);

/* image may be NULL (black points). points: capacity records; tiles:
 * max_tiles spans. */
OST_Status_t OST_Map_GeorefConvert(OST_Map_Georef_t *handle, const OST_Map_DepthMap_t *map,
                                   const OST_Map_Image_t *image, const OST_Map_Pose_t *pose, OST_Map_Point_t *points,
                                   uint32_t capacity, OST_Map_TileSpan_t *tiles, uint32_t max_tiles,
                                   OST_Map_GeorefResult_t *result);

OST_Status_t OST_Map_GeorefWriteLaz(OST_Map_Georef_t *handle, uint32_t keyframe, const OST_Map_PointSet_t *set,
                                    OST_Map_LazResult_t *result);

/* --------------------------------------------------------- map products */

typedef struct OST_Map_Products OST_Map_Products_t;

typedef struct
{
    const char *projected_crs; /* CRS of P */
    const char *vertical_crs;  /* of the written DSM; empty: ellipsoidal heights */
    const char *proj_data_dir;
    const char *grid_dir;      /* may be empty */
    const char *output_dir;    /* required */

    /* Tile grid and DSM fusion (ADR-007 sections 1 and 2) */
    double  cell_size_m;
    int32_t tile_cells;
    int32_t min_observations;
    double  rejection_sigmas;
    double  min_sigma_m;
    double  min_confidence;

    /* Orthomosaic (section 3) */
    double ortho_pixel_m;
    double surface_smoothing_m;
    double switch_margin_deg;
    double render_margin_m; /* around the keyframe footprint */

    /* Lifecycle (section 5) */
    int32_t max_active_tiles;
    double  finalise_distance_m;
    /* 1: OST_Map_ProductsUpdate evicts finalised tiles and saves their
     * state, and their products are written with the next snapshot or
     * publication; 0: it writes them itself. */
    int32_t defer_writes;
} OST_Map_ProductsConfig_t;

/* Camera model of the full-resolution keyframe (session calibration):
 * OpenCV pinhole with radial and tangential distortion. */
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
} OST_Map_FullCamera_t;

/* Keyframe in 8-bit RGB at any scale of the full camera. */
typedef struct
{
    const uint8_t *data;
    int32_t        width;
    int32_t        height;
    size_t         stride;
} OST_Map_RgbImage_t;

typedef struct
{
    uint32_t points;       /* records fused (weight above the minimum) */
    uint32_t added;
    uint32_t rejected;     /* outliers (k sigma) */
    uint32_t tile_count;   /* tiles touched; the first max_tiles are listed */
    double   extent[4];    /* easting min, northing min, easting max, northing max */
    double   height_median; /* ellipsoidal, of the fused points */
    double   fuse_s;
} OST_Map_FusionResult_t;

typedef struct
{
    uint32_t pixels;     /* orthomosaic pixels that took this keyframe */
    uint32_t tile_count; /* tiles changed */
    double   render_s;
} OST_Map_RenderResult_t;

typedef struct
{
    uint32_t tiles;   /* tiles written (finalised, or published) */
    uint32_t active_tiles;
    double   write_s;
} OST_Map_WriteResult_t;

typedef struct
{
    uint32_t active_tiles;
    uint32_t changed_tiles; /* active tiles changed since the last publication */
    uint32_t deferred_tiles; /* finalised, products waiting for the next snapshot */
    uint64_t memory_bytes;  /* tile data in memory, deferred tiles included */
} OST_Map_ProductsStats_t;

/* Defaults of ADR-007: EPSG:2056 with LHN95 heights, 0.26 m cells, 512
 * cells per tile, 3 observations, 3 sigma, 0.25 m, 0.066 m orthomosaic
 * pixels, 1 m surface smoothing, 2 degrees, 30 m render margin, 64 active
 * tiles, 600 m, deferred writes; paths empty. */
void OST_Map_ProductsDefaultConfig(OST_Map_ProductsConfig_t *config);

OST_Status_t OST_Map_ProductsCreate(const OST_Map_ProductsConfig_t *config, OST_Map_Products_t **handle);
void         OST_Map_ProductsDestroy(OST_Map_Products_t *handle);
const char  *OST_Map_ProductsLastError(const OST_Map_Products_t *handle);

/* Fuses a point set (layout of OST_Map_GeorefConvert, same tile grid).
 * tiles: max_tiles entries for the touched tiles, may be NULL. */
OST_Status_t OST_Map_ProductsAddPoints(OST_Map_Products_t *handle, const OST_Map_PointSet_t *set,
                                       OST_Map_TileSpan_t *tiles, uint32_t max_tiles, OST_Map_FusionResult_t *result);

/* Renders a keyframe: ground heights from the DSM, over its footprint on
 * the ellipsoidal height ground_height (plus the render margin), limited to
 * extent (easting min, northing min, easting max, northing max) when
 * extent is not NULL. tiles as for OST_Map_ProductsAddPoints. */
OST_Status_t OST_Map_ProductsRender(OST_Map_Products_t *handle, const OST_Map_RgbImage_t *image,
                                    const OST_Map_FullCamera_t *camera, const OST_Map_Pose_t *pose,
                                    double ground_height, const double *extent, OST_Map_TileSpan_t *tiles,
                                    uint32_t max_tiles, OST_Map_RenderResult_t *result);

/* Finalises tiles far from the vehicle (camera centre, ECEF) or above the
 * budget; result->tiles: tiles finalised. */
OST_Status_t OST_Map_ProductsUpdate(OST_Map_Products_t *handle, const double vehicle_ecef[3],
                                    OST_Map_WriteResult_t *result);

OST_Status_t OST_Map_ProductsPublish(OST_Map_Products_t *handle, OST_Map_WriteResult_t *result);

typedef struct OST_Map_Snapshot OST_Map_Snapshot_t;

/* Copies the changed active tiles and clears the changed set (owner task).
 * *snapshot is NULL when no tile changed; tiles: number of tiles copied. */
OST_Status_t OST_Map_ProductsSnapshot(OST_Map_Products_t *handle, OST_Map_Snapshot_t **snapshot, uint32_t *tiles);

/* Writes a snapshot and the mosaic indexes and frees the snapshot (any one
 * task). A tile finalised meanwhile in the same or a newer version is not
 * written again. result->active_tiles is 0. */
OST_Status_t OST_Map_ProductsWriteSnapshot(OST_Map_Products_t *handle, OST_Map_Snapshot_t *snapshot,
                                           OST_Map_WriteResult_t *result);

/* Text of the last failed OST_Map_ProductsWriteSnapshot. */
const char *OST_Map_ProductsPublishError(const OST_Map_Products_t *handle);
OST_Status_t OST_Map_ProductsFinaliseAll(OST_Map_Products_t *handle, OST_Map_WriteResult_t *result);
OST_Status_t OST_Map_ProductsGetStats(const OST_Map_Products_t *handle, OST_Map_ProductsStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_MAP_PRODUCTS_C_API_H */
