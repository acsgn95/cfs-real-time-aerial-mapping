# ADR-007: Depth Fusion and Incremental Map Products

* Status: Accepted
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D07
* Evidence: thesis proposal (FB_YL-01) sections 2, 4.2 and 4.3; ADR-001
  (R2, R4, R5, product resolutions); ADR-004 (product frame `P`, working
  frames `L_k`); ADR-005 (depth, normal and confidence maps; no
  scene-wide fusion); ADR-006 (footprints, image pool)

## Context

The proposal requires a dense point cloud, a DSM and an orthomosaic. It asks
that new images update only the affected DSM cells and orthomosaic regions,
following the incremental structure of AerialMapper. The point cloud accuracy
is one of the evaluation metrics.

ADR-001 requires map tiles to be updated within the latency target (R2) and
memory to stay bounded while the area is unlimited (R4, R5). ADR-005 produces
one depth map with a confidence measure per keyframe and leaves fusion to
this record.

## Decision

### 1. Tile layout

* Map products are tiled on a regular grid in the product frame `P`
  (ADR-004), aligned to integer multiples of the tile size.
* A tile holds 512 × 512 DSM cells. At the nominal point (DSM about 26 cm),
  a tile covers about 133 m × 133 m.
* The orthomosaic tile covers the same ground area at the orthomosaic
  resolution (about 2000 × 2000 pixels at 6.6 cm).
* DSM and orthomosaic share the tile identifier, so updates and writes are
  done per tile.

### 2. DSM fusion: confidence-weighted mean

* Each depth map pixel is converted to a 3D point in `L_k`, transformed to
  ECEF and then to `P` (ADR-004), and assigned to the DSM cell that contains
  it.
* Each cell keeps four values: the sum of weights, the weighted sum of
  heights, the weighted sum of squared heights, and the number of
  observations. The weight is the depth confidence from ADR-005.
* The cell height is the weighted mean. The weighted variance is kept as a
  quality layer.
* Once a cell has at least a configured number of observations, a new
  observation that deviates more than `k · σ` from the current mean is
  rejected.
* Heights are kept as ellipsoidal heights and converted to the configured
  vertical datum when written (ADR-004).
* Cells without observations stay empty (no data). Small holes may be
  filled when a tile is finalised; this is optional and off by default.
* Memory: about 4 MB per tile for the cell values.

### 3. Orthomosaic: nadir-most source, no blending

* The orthomosaic is rendered from keyframes. The full-resolution JPEG is
  decoded at the scale closest to the orthomosaic resolution (DCT scaling),
  for example 1/2 scale (about 10 MP) at the nominal point.
* For each orthomosaic pixel in the tiles a keyframe covers, the ground point
  is taken from the current DSM and projected into the keyframe.
* The pixel takes its colour from the keyframe that views that ground point
  closest to nadir. Each pixel keeps the viewing angle of its source. A
  later keyframe overwrites a pixel only if it views the point closer to
  nadir.
* There is no blending between images.
* Rendering happens as soon as the keyframe's depth map has updated the DSM,
  so that R2 holds.
* Known limitations, reported in the evaluation:
  * pixels are not re-rendered when the DSM is refined later;
  * there is no occlusion test as in a true orthophoto. Nadir-most selection
    reduces the effect.

### 4. Dense point cloud

* Depth map points that pass the geometric consistency check (ADR-005) are
  appended immediately to per-tile point files (LAZ) in `P`. They are not
  kept in memory.
* Each point carries its colour from the keyframe, its confidence and the
  keyframe identifier.
* Thinning and removal of duplicate points are done offline for the
  evaluation.

### 5. Tile lifecycle

| State | Meaning |
| --- | --- |
| Active | in memory, receives updates |
| Finalised | no image pool entry covers the tile any more, or the vehicle is farther than a configured distance; no further updates expected |
| Written | final DSM and orthomosaic tiles are on disk |
| Evicted | released from memory |

* The number of active tiles is limited by a configured budget. When the
  budget is exceeded, the tile farthest from the vehicle is finalised first.
  This keeps memory bounded (ADR-001 R4).
* If the vehicle returns over a written tile, the tile is loaded from disk
  and becomes active again.

### 6. Publication during flight

* Changed active tiles are written to disk periodically (default every 5 s),
  so that an operator can view the map during the flight.
* Map update latency (ADR-001 R2) is measured to the in-memory tile update.
  The publication delay is reported separately.

### 7. File formats

| Product | Format |
| --- | --- |
| DSM tile | GeoTIFF, float32, no-data value, one band for height and one for weighted variance |
| Orthomosaic tile | GeoTIFF, RGB with a mask, compressed |
| Point cloud tile | LAZ |
| Mosaic index | GDAL VRT over all DSM tiles and over all orthomosaic tiles, updated at publication |

## Rationale

* A confidence-weighted mean needs constant memory per cell and is updated
  in constant time, which suits incremental fusion. Outlier rejection keeps it
  robust once a cell has a stable estimate.
* Nadir-most selection without blending is simple, keeps pixels sharp, and
  minimises relief displacement and occlusion artefacts without a true
  orthophoto pipeline.
* Writing points straight to disk keeps the point cloud out of the memory
  budget while still providing the product the proposal evaluates.
* Shared DSM and orthomosaic tiles with an explicit lifecycle make unlimited
  area with bounded memory concrete.
* VRT indexes let standard GIS tools open the growing map as one layer.

## Alternatives Considered

* **Median per cell.** Rejected: robust, but it needs every observation per
  cell to be stored.
* **Maximum height per cell.** Rejected: biased towards noise and outliers.
* **Blending (feathering) between images.** Rejected: more computation and
  blurred details. Seam lines are accepted.
* **True orthophoto with occlusion test.** Rejected for now: high cost.
  Nadir-most selection is used instead.
* **Rendering the orthomosaic only when a tile is finalised.** Rejected:
  violates the latency target (R2).
* **Keeping the point cloud in memory per tile.** Rejected: tens of MB per
  tile would consume the memory budget.
* **One growing GeoTIFF for the whole map.** Rejected: incompatible with
  unlimited area and bounded memory.

## Affected Areas

* [x] algorithm libraries
* [x] cFS application structure
* [x] large data transport
* [x] configuration/tables
* [x] observability and performance measurement
* [x] evaluation and experiments
* [x] documentation

## Impact

* **D11:** DSM and orthomosaic share tiles, which favours one Map Products
  application owning the tile store. Point cloud writing may sit in the same
  application or in the Data Recorder.
* **D12:** depth maps and decoded keyframes are passed by reference to the
  map product stage.
* **D14:** map product updates run after each keyframe's depth map.
  Publication runs on its own period.
* **D15:** tile size, budgets, distances, publication period, outlier
  threshold and hole filling are configuration values.
* **D16:** per-tile update time, publication delay, active tile count and
  rejected observations are reported.
* **D17:** the evaluation reads the written tiles and point files. It reports
  the orthomosaic limitations of section 3.

## Open Questions

None.
