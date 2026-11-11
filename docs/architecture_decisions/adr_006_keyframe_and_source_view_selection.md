# ADR-006: Keyframe and Source View Selection

* Status: Accepted
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D06
* Revised: 2026-11-11 (dense working resolution given as image width; 2048 × 1366 is 2.8 MP)
* Revised: 2026-11-11 (terrain prior: Copernicus DEM GLO-30 instead of SRTM)
* Evidence: ADR-001 (keyframe overlap assumption, tilt envelope, R4);
  ADR-004 (ellipsoidal heights, working frames); ADR-005 (causal processing,
  K = 5); Copernicus DEM GLO-30 on AWS Open Data (bucket readme: tiles,
  surface model); comparison with the Dorfberg reference DSM at the 753
  footprint centres (`tools/dense_benchmark --terrain-dem`): median
  +0.37 m, NMAD 1.64 m, 95th percentile of |e| 5.97 m; −50 m without the
  geoid conversion

## Context

ADR-001 defines real time on keyframes and assumes a 70 % keyframe overlap.
ADR-005 needs K source views per keyframe, chosen causally among images
already received. Both selections need the ground footprint of each image.

The footprint depends on the terrain height under the image. The nominal
dataset is an alpine slope (camera ellipsoidal heights 1856–2585 m), so a
single flat ground plane for the whole flight is not usable.

## Decision

### 1. Image footprint

For every input image with a valid pose:

1. Five pixels are unprojected through the camera model, including lens
   distortion: the four image corners and the principal point. Their
   directions are rotated into the local ENU frame with the camera pose.
2. Each ray is intersected with a horizontal plane at height `h`. A ray that
   does not point downwards rejects the footprint.
3. The plane starts at the camera height minus the nominal height above
   ground of the mission (configuration).
4. The plane is refined by a fixed-point iteration (default 3 iterations):
   the DEM is sampled at the five ground points, and the plane moves to their
   mean height.
5. If a ground point has no DEM height, the footprint is rejected.

Result: the four corner points (geodetic and ENU) with their DEM heights, the
centre point, the plane height, the spread `σ_h` of the five DEM heights, and
the footprint's width and height in metres. `σ_h` indicates rough or steep
terrain, where the footprint is less reliable.

### 2. Terrain prior: Copernicus DEM GLO-30

* The DEM for footprints is the Copernicus DEM GLO-30 (1 arc-second, about
  30 m), a surface model including vegetation and buildings. It is a
  pre-mission asset: `tools/dataset/fetch_dem.py` downloads the 1° × 1°
  tiles of the mission area from the public AWS Open Data bucket
  (`copernicus-dem-30m`, no account needed), records their SHA-256 sums in a
  manifest and writes a VRT mosaic. The path of the mosaic and its CRS are
  set in configuration.
* Copernicus heights are orthometric heights above the EGM2008 geoid (CRS
  `EPSG:4326+3855`). They are converted to ellipsoidal heights with the PROJ
  grid `us_nga_egm08_25.tif`, as ADR-004 requires. The grid is fetched with
  pinned checksum by the environment setup. Without the conversion the plane
  is about 50 m wrong at the Dorfberg site, and the footprint scale about
  20 % wrong at 247 m above ground.
* Accuracy: at the Dorfberg site the converted heights differ from the
  reference DSM by +0.37 m (median) with an NMAD of 1.64 m at the footprint
  centres. This is sufficient for selection. The DEM is not used for map
  products.
* Other DEMs (for example SRTM with EGM96 heights, or a national model)
  can be used by configuring their path and CRS; the matching geoid grid is
  then added to `scripts/fetch_proj_grids.sh`.
* Areas without a tile (oceans, a few countries not released to the public)
  have no data; footprints there are rejected (section 1).
* Products derived from the DEM must carry the Copernicus attribution
  (manifest of `fetch_dem.py`). Footprints are not distributed, so the
  mission products are not affected.

### 3. Overlap

The overlap of image `j` with a reference image `i` is the area of the
intersection of their footprint quadrilaterals divided by the area of the
footprint of `i`. Both footprints are expressed in the same planar frame (the
ENU frame of `i`).

### 4. Keyframe selection

An input image becomes a keyframe when all of the following hold:

* its pose is valid (time alignment succeeded; rule defined in D13);
* its tilt from nadir is at most 25° (20° envelope of ADR-001 plus 5° margin),
  which excludes images taken in turns;
* its footprint was computed;
* its overlap with the last keyframe is below the target forward overlap
  (default 70 %, allowed 60–80 %).

### 5. Source view selection

For each keyframe, up to K source views (default 5, ADR-005) are chosen among
all images already received. Non-keyframe images and images of earlier flight
lines are included. A candidate qualifies when:

* its overlap with the keyframe is at least 40 %;
* its baseline-to-height ratio is between 0.05 and 0.6, where height is
  the keyframe's height above the footprint plane;
* the angle between its viewing direction and the keyframe's viewing
  direction is at most 30°.

Qualified candidates are ranked by a score that combines overlap and
triangulation angle, in the spirit of COLMAP's source view selection. At least
one source from a neighbouring flight line is included when one qualifies,
because a different viewing direction improves depth quality.

At the nominal point, the baseline-to-height ratio is about 0.21 between
consecutive keyframes and about 0.35 to the neighbouring line.

### 6. Image pool

* Images at the dense working resolution, their footprints, and the depth
  maps of keyframes are kept in an image pool, so that images of earlier
  flight lines remain available as sources.
* The pool is indexed spatially by footprint. Entries whose footprints are
  farther than a configured distance from the current position are evicted,
  so memory stays bounded (ADR-001 R4).
* Sizing: an RGB image at 2048 × 1366 is about 8.4 MB, so 300 entries need
  about 2.5 GB, within the 16 GB RAM budget of ADR-002. The default quota of
  ADR-013 (2 GB) holds about 240; the pool distance and the quota are tuned
  together in the ADR-005 benchmark.

### 7. Configuration

All thresholds of this record (iterations, nominal height above ground,
target overlap, tilt limit, source criteria, K, pool distance) are
configuration values (cFE tables, D15). The defaults may be tuned in the
ADR-005 benchmark.

## Rationale

* Five rays and a DEM-refined plane give a footprint that follows the terrain
  at little cost, and `σ_h` makes its reliability visible.
* Copernicus GLO-30 covers almost any mission area, is newer and more
  accurate than SRTM in mountainous terrain, and can be downloaded without an
  account; at Dorfberg it agrees with the reference DSM to about 1.6 m
  (NMAD).
* Overlap-based keyframe selection adapts to speed and altitude, unlike a
  fixed image interval.
* Causal source selection with a spatial pool lets earlier flight lines
  contribute, which gives the multi-view stereo better geometry than
  consecutive images alone.

## Alternatives Considered

* **Flat ground plane at a fixed height.** Rejected: wrong on steep terrain
  such as the nominal dataset.
* **SRTM 1 arc-second (SRTMGL1) from NASA Earthdata.** Replaced: needs an
  Earthdata account, is older (2000) and less accurate in mountains (LE90
  9.73 m by specification). It remains usable through configuration.
* **OpenTopography Global DEM API.** Not chosen as the default: needs a
  personal API key, and integration into commercial software needs an
  enterprise key. It also offers DEMs with ellipsoidal heights (for example
  SRTMGL1_E), which would avoid the geoid conversion.
* **The system's own DSM as the terrain prior.** Not chosen: it does not exist
  where the vehicle flies for the first time. It may be added later as a
  refinement.
* **Intersecting each ray with the DEM individually.** Not chosen for now: a
  refined plane is sufficient for overlap estimation. `σ_h` shows where it is
  not.
* **Every n-th image as keyframe.** Rejected: breaks when speed or altitude
  changes.
* **Source views only from the same flight line.** Rejected: loses the
  stronger geometry of neighbouring lines.

## Affected Areas

* [x] algorithm libraries
* [x] configuration/tables
* [x] large data transport
* [x] evaluation and experiments
* [x] documentation

## Impact

* **Pre-mission data:** the Copernicus mosaic of the mission area and the
  PROJ EGM2008 grid must be available on the target. The grid is part of the
  environment setup (D09); the DEM is fetched per mission area with
  `tools/dataset/fetch_dem.py`.
* **D07:** map product generation can use the same footprints to decide
  which tiles an image affects.
* **D11:** frame selection and the image pool belong to the Frame Selection
  application or its library.
* **D12:** the image pool holds working-resolution images and depth maps;
  the transport must allow references into it.
* **D13:** defines when a pose is valid.
* **D16:** footprint rejections, keyframe decisions and pool size are
  reported.

## Open Questions

None.
