# GEOREF_APP

Georeferencing of the depth sets (ADR-007, ADR-012). It takes the depth
sets of `DEPTH_READY` (from `DENSE_APP`), converts each to a point set in
the product frame `P` with `libs/map_products` (`OST_Map_Georef*`),
publishes it on `POINTS_READY` for `MAPPROD_APP` and writes the point cloud
(LAZ, one file per keyframe and tile, ADR-011 section 6).

## Messages

| Direction | Message | Use |
| --- | --- | --- |
| in | `DENSE_APP` `DEPTH_READY_TLM` | depth set, keyframe (working image, JPEG, pose), working camera |
| out | `POINTS_READY_TLM` | point set reference, origin, tile spans, keyframe (JPEG and pose), tile grid and CRS, times |
| out | `HK_TLM` | counters, last and largest times, point cloud counters, latency |

Commands: no-op (0), reset counters (1), clear latched faults (2).

## Point set

A data store object of type `POINT_SET` (ADR-013): records of 20 bytes,
native byte order (`OST_Map_Point_t` in `map_products_c_api.h`):

| Bytes | Field |
| --- | --- |
| 0–11 | float32 east, north, height: metres from the origin of the set; the height is ellipsoidal |
| 12–15 | float32 weight: the depth confidence (ADR-007 section 2) |
| 16–19 | uint8 red, green, blue from the working image, spare |

The origin (`OriginEasting`, `OriginNorthing`, `OriginHeight`) is the
keyframe camera centre in `P`, rounded to whole metres. Relative single
precision coordinates hold a point to 0.06 mm at 1 km. Records are ordered
by tile (column, then row) and within a tile by pixel; `Tiles` gives the
first record and the count of each tile. Tile (column, row) covers easting
`[column, column + 1) · T` and northing `[row, row + 1) · T`, with
`T = CellSizeM · TileCells` (133.12 m by default, ADR-007 section 1).

At Dorfberg (2048 × 1366 working images) a keyframe gives about 1.8 million
points (35 MB) in 8 to 10 tiles.

## A job

1. The depth set and the working image are read under their own counts.
2. The points are counted (depth and confidence above 0, confidence at
   least `MinConfidence`) and a point set of that size is allocated. When
   the quota is full the job waits up to `QuotaWaitMs` for `MAPPROD_APP`
   (ADR-015 section 3); after that the keyframe is dropped
   (`DATASTORE_QUOTA`, `POINTS_FAILED`).
3. `OST_Map_GeorefConvert`: pixel to the keyframe camera, to ECEF with the
   keyframe pose, to `P` (the exact PROJ transformation on a 9 × 9 × 9 grid
   over the extent of the map, interpolated, as in DSM fusion; ADR-007
   section 8), colour from the working image, ordering by tile.
4. The counts of the depth set and of the working image are released.
   `POINTS_READY` hands on the point set count and the JPEG count
   (ADR-013 section 3); its `Keyframe.WorkImage` is 0.
5. The point cloud files are written after the publication, from the point
   set under the application's own count, so they are not on the path of
   the map update (ADR-001 R2). Heights are converted to `VerticalCrs`
   (LHN95 by default). A file that cannot be written raises `TILE_WRITE`;
   the point set is published anyway.

A keyframe without valid depth is published with no point set (reference 0,
no tiles), so that `MAPPROD_APP` still receives its JPEG count.

GEOREF_APP does not drop data otherwise. A depth set that finds the worker
queue full (16), no table or no product frame is dropped with an event and
`POINTS_FAILED`.

## Tasks

| Task | Work |
| --- | --- |
| main | commands, housekeeping; depth sets go into the queue |
| `GEOREF_WORK` | one keyframe at a time (priority level 3 of ADR-015) |

## Configuration

Table `GEOREF_APP.ConfigTable` (`fsw/tables/georef_app_tbl.c`):

| Field | Default |
| --- | --- |
| `ProjectedCrs`, `VerticalCrs` | `EPSG:2056` (LV95), `EPSG:5729` (LHN95, for the files) |
| `ProjDataDir`, `GridDir` | `/cf/geodata/proj`, `/cf/geodata/grids` |
| `OutputDir`, `WriteLaz` | `/cf/products` (files in `points/`), 1 |
| `CellSizeM`, `TileCells` | 0.26 m, 512: the tile grid of `MAPPROD_APP` |
| `MinConfidence` | 0 |
| `QuotaWaitMs` | 5000 ms |

The tile grid and the CRS must equal those of `MAPPROD_APP`; `POINTS_READY`
carries them, so that the consumer can check.

The module links `libs/map_products` statically (PROJ through `libs/geo`,
LASzip).

## Events and faults

See [`docs/interfaces/events.md`](../../docs/interfaces/events.md) and
[`docs/interfaces/faults.md`](../../docs/interfaces/faults.md).
`tools/ground/georef_report.py` summarises a capture of `POINTS_READY` and
the housekeeping, and checks the LAS headers of the point cloud files.
