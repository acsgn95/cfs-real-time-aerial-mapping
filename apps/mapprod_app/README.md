# MAPPROD_APP

DSM and orthomosaic tiles (ADR-007, ADR-012). It takes the point sets of
`POINTS_READY` (from `GEOREF_APP`), fuses them into DSM tiles, renders the
keyframe's JPEG into orthomosaic tiles, finalises tiles far from the vehicle
and writes changed tiles periodically, with `libs/map_products`
(`OST_Map_Products*`) and `libs/imaging` (JPEG decoding). It is the last
consumer of the data path.

## Messages

| Direction | Message | Use |
| --- | --- | --- |
| in | `GEOREF_APP` `POINTS_READY_TLM` | point set, tile spans, keyframe (JPEG and pose), tile grid and CRS |
| in | `INGEST_APP` `SESSION_CALIBRATION_TLM` | full camera model of the keyframe JPEGs |
| in | `WAKEUP` (1 Hz, `SCH_LAB`) | publication period |
| out | `TILE_UPDATED_TLM` | one per keyframe: the map update in memory; tiles changed, points fused and rejected, orthomosaic pixels, times, latency from the capture (ADR-001 R2) |
| out | `TILE_PUBLISHED_TLM` | one per publication: tiles written, write time, publication delay |
| out | `HK_TLM` | counters, tiles in memory, memory, last and largest times |

Commands: no-op (0), reset counters (1), clear latched faults (2), finalise
all (3): writes and evicts every active tile at the end of a flight.
`tools/ground/send_command.py MAPPROD_APP_CMD_FINALISE_ALL` sends it without
COSMOS.

## A keyframe

1. **Fusion** (ADR-007 section 2): the point set is checked against the
   table's tile grid and CRS, then fused into the DSM tiles
   (`OST_Map_ProductsAddPoints`). Within a tile the points keep the pixel
   order, so each cell receives its observations in the order of the
   offline pipeline and the k-sigma test gives the same result (apart from
   points within float rounding, hundredths of a millimetre, of a cell
   edge). The point set's counts are released.
2. **Orthomosaic** (section 3): the JPEG is decoded at the smallest DCT
   scale at least `OrthoWidth` wide (default half the camera width: 1/2 at
   Dorfberg, 3976 × 2652) and rendered over the keyframe's footprint at the
   median height of its points, widened by `RenderMarginM` and limited to
   the extent of its points, on the DSM it has just updated. Needs the
   session calibration; without it, or for a keyframe without points, the
   keyframe is only in the DSM. The JPEG count is released: the end of the
   data path.
3. **Lifecycle** (section 5): tiles farther than `FinaliseDistanceM` from
   the keyframe's camera centre, or above `MaxActiveTiles`, are finalised:
   written, their state saved under `OutputDir/state`, evicted. A later
   keyframe over a finalised tile reloads it.
4. `TILE_UPDATED`: the map is updated in memory.

## Publication

Every `PublishPeriodS` the worker copies the changed active tiles
(`OST_Map_ProductsSnapshot`, about 3 ms per tile) and hands the copy to the
publisher task, which writes it (`OST_Map_ProductsWriteSnapshot`) and sends
`TILE_PUBLISHED`. Writing an orthomosaic tile (2017 × 2017 RGBA, deflate)
takes about 0.75 s, a DSM tile 0.09 s; on the worker, publication held up
the map update by up to 20 s (first run, 2026-11-14). While the publisher
still writes, the changes wait for the next period. Each tile carries a
version; a copy is not written over files of a tile that was finalised in
the same or a newer version meanwhile.

`TILE_PUBLISHED.PendingS` is the publication delay: from the first map
update included to the end of writing (ADR-007 section 6).

Products (ADR-007 section 7) under `OutputDir`: `dsm/`, `ortho/`, `dsm.vrt`,
`ortho.vrt`; heights in `VerticalCrs` (LHN95 by default).

## Stopping

The point sets still queued are released and reported. With
`FinaliseOnStop` the worker finalises all tiles when the application is
stopped through cFE. A process ended by a signal does not run this: the
products are then those of the last publication, at most `PublishPeriodS`
plus one publication old; the state files are missing for tiles that were
not finalised.

## Tasks

| Task | Work |
| --- | --- |
| main | commands, housekeeping, the 1 Hz wake-up; point sets, the calibration and publication requests go into the queue in arrival order |
| `MAPPROD_WORK` | owns the tiles: fusion, orthomosaic, lifecycle, tile copies (priority level 3 of ADR-015) |
| `MAPPROD_PUB` | writes the tile copies (below the worker) |

## Configuration

Table `MAPPROD_APP.ConfigTable` (`fsw/tables/mapprod_app_tbl.c`):

| Field | Default |
| --- | --- |
| `ProjectedCrs`, `VerticalCrs` | `EPSG:2056` (LV95), `EPSG:5729` (LHN95, for the written DSM) |
| `ProjDataDir`, `GridDir`, `OutputDir` | `/cf/geodata/proj`, `/cf/geodata/grids`, `/cf/products` |
| `CellSizeM`, `TileCells` | 0.26 m, 512 (as `GEOREF_APP`) |
| `MinObservations`, `RejectionSigmas`, `MinSigmaM`, `MinConfidence` | 3, 3, 0.25 m, 0 |
| `OrthoPixelM`, `SurfaceSmoothingM`, `SwitchMarginDeg`, `RenderMarginM`, `OrthoWidth` | 0.066 m, 1 m, 2°, 30 m, 0 (half the camera width) |
| `MaxActiveTiles`, `FinaliseDistanceM` | 64, 600 m |
| `PublishPeriodS`, `RenderOrtho`, `FinaliseOnStop` | 5 s, 1, 1 |

The module links `libs/map_products` and `libs/imaging` statically.

## Events and faults

See [`docs/interfaces/events.md`](../../docs/interfaces/events.md) and
[`docs/interfaces/faults.md`](../../docs/interfaces/faults.md).
`tools/ground/mapprod_report.py` summarises a capture of `TILE_UPDATED`,
`TILE_PUBLISHED` and the housekeeping; the written DSM is evaluated with
`tools/offline_pipeline --evaluate-only`.
