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
 * Status codes (ost_status.h): OST_ERR_INVALID_ARGUMENT for invalid
 * arguments or configuration, a point set or tile list that is too small,
 * or a pose that is not a unit quaternion; OST_ERR_IO if the PROJ data
 * cannot be used (OST_Map_GeorefCreate) or a point cloud file cannot be
 * written; OST_ERR_UNSUPPORTED for OST_Map_GeorefWriteLaz without an output
 * directory; OST_ERR_INTERNAL for unexpected errors. OST_Map_GeorefLastError
 * gives the text of the last failed call.
 *
 * A handle is used by one task at a time. No function throws.
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

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_MAP_PRODUCTS_C_API_H */
