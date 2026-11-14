# SELECT_APP

Keyframe and source view selection with the image pool (ADR-006, ADR-012).
For each image with a valid pose it computes the footprint on the terrain
prior, assigns the flight line, decides whether the image is a keyframe,
decodes it to the working resolution into the data store, and for a
keyframe publishes a batch with up to K source views from the pool. The
algorithms are `libs/selection` (with `libs/terrain`) and `libs/imaging`.

## Messages

| Direction | Message | Use |
| --- | --- | --- |
| in | `ALIGN_APP` `ImagePose` | pose and JPEG reference; one count for this application |
| in | `INGEST_APP` `SessionCalibration` | camera model of the session |
| out | `OST` `BATCH_SELECTED_TLM` (`RefinePose` 1) or `BATCH_READY_TLM` (0) | `KeyframeBatch`: keyframe and sources with working images, poses, working-resolution intrinsics, footprint plane height (ellipsoidal) |
| out | `HK_TLM` | counters, pool, flight lines, terrain, decode and selection times, faults |

Commands: no-op (0), reset counters (1), clear latched faults (2).

## Data store references (ADR-013 section 3)

* The pool holds one count of each working image.
* A batch carries one more count of each working image it names, and the
  keyframe's JPEG count from `ImagePose` (for the orthomosaic). Its
  consumer releases them or hands them on. The sources have no JPEG.
* JPEGs of images that do not become keyframes are released after
  decoding.
* Working images: rows packed, 8-bit, interleaved RGB (`WorkChannels` 3)
  or grey (1), of the size in `KeyframeBatch.Intrinsics`.
* A source gets its batch count before the keyframe's working image is
  allocated, so an eviction for memory cannot free it.
* When the working image quota is exhausted, the oldest pool images are
  released (at most 8) before the image is given up (`DATASTORE_QUOTA`).

## Tasks

| Task | Work |
| --- | --- |
| main | commands, housekeeping; `ImagePose` and `SessionCalibration` go into the worker queue in arrival order |
| `SELECT_WORK` | selection, decoding, pool, batches (ADR-015 section 2) |

The worker queue is a ring of the application (32 items) with a mutex and
a counting semaphore: POSIX message queues allow only 10 entries without
privileges. A full queue drops the image with its reason
(`QueueOverflows`); at the nominal point the worker needs about 0.2 s per
image and images arrive every 1.45 s.

## Sessions

`SessionCalibration` starts a session: the pool is released, the working
image converter is created for the camera, and the selector is reset (a
new local frame, flight lines and keyframes). Images of a session without
a calibration are released (`ImagesNoCalibration`).

## Terrain prior

The elevation model, its CRS, the PROJ data and the grid directory are
table values. `/cf/` paths are translated to the host. The mission installs
under `/cf/geodata`: `proj.db`, the EGM2008 grid `us_nga_egm08_25.tif`
(`scripts/fetch_proj_grids.sh`) and the Copernicus mosaic of the mission
area as `dem/terrain.vrt` (`tools/dataset/fetch_dem.py`;
`mission/orthostream_defs/cpu1/install_custom.cmake`). A window of
`TerrainRadiusM` around the flight is held in memory and read again when
the camera comes within half the radius of its edge. Without a usable
model the application raises `TERRAIN_UNAVAILABLE` and releases the
images.

## Configuration

Table `SELECT_APP.ConfigTable` (`fsw/tables/select_app_tbl.c`), loaded once
at startup; the defaults are those of ADR-006:

| Field | Default |
| --- | --- |
| `RefinePose` | 1 (batches to `REFINE_APP`) |
| `WorkWidth`, `WorkChannels` | 2048 px, 3 (RGB) |
| `NominalHeightAglM`, `FootprintIterations` | 250 m, 3 |
| `MaxTiltDeg`, `TargetOverlap` | 25°, 0.7 |
| `MaxSources`, `MinOverlap` | 5, 0.4 |
| `MinBaselineRatio`, `MaxBaselineRatio` | 0.05, 0.6 |
| `MaxViewAngleDeg`, `PreferredTriangulationAngleDeg` | 30°, 10° |
| `LineTurnDeg` | 45° |
| `PoolDistanceM`, `MaxPoolImages` | 600 m, 200 |
| `TerrainRadiusM` | 20 km |
| `DemPath`, `DemCrs` | `/cf/geodata/dem/terrain.vrt`, `EPSG:4326+3855` |
| `ProjDataDir`, `GridDir` | `/cf/geodata/proj`, `/cf/geodata/grids` |

## Events and faults

See [`docs/interfaces/events.md`](../../docs/interfaces/events.md) and
[`docs/interfaces/faults.md`](../../docs/interfaces/faults.md).

## Verification

Injector with Dorfberg images 42–122 (80 images, two flight lines and a
turn), 2026-11-14: 80 images decoded, 36 keyframes, the same keyframes as
`tools/offline_pipeline` on the whole flight except the first (the
session starts at image 42); 20 batches with a source of the neighbouring
line, 135 sources in all; 0.18 s per image (decoding 0.19 s at most,
selection 1 ms, 99 ms for the first image with the terrain window).
