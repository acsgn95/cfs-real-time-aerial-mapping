# ADR-011: Boundary between Algorithm Libraries and cFS Applications

* Status: Accepted; amended by [ADR-012](adr_012_application_boundaries.md) (eighth library `imaging`) and [ADR-013](adr_013_shared_data_store.md) (ninth library `datastore`)
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D10
* Evidence: ADR-005 section 7 (COLMAP only in an algorithm library);
  ADR-009 (C99 applications, C++17 libraries, `OST_` prefix); ADR-010
  (dependencies); the library pattern of a cFS mission on the same machine
  (NavionCore: one C interface header per library, no exceptions or C++
  types across it, libraries independent of cFE)

## Context

The algorithms (ADR-004 to ADR-008) are C++ and depend on COLMAP, OpenCV,
GDAL, PROJ and LASzip. cFS applications are C99 (ADR-009). The benchmark of
ADR-005 and the evaluation have to run the same algorithms without starting
cFS.

## Decision

### 1. Two layers

* **`libs/` (C++17):** the algorithms. No dependency on cFE or OSAL.
* **`apps/` (C99):** thin shells. Software Bus messages, tables, events,
  housekeeping and scheduling.
* Dependencies point one way: applications use libraries, and libraries use
  third-party packages.

### 2. Rules of the C interface

Each library exposes one header, `<library>_c_api.h`, and nothing else to the
applications.

* Objects are opaque handles created and destroyed through the interface.
  Every function returns a status code and can provide a message text.
* No C++ exception crosses the interface; the interface layer catches them.
  No C++ type appears in it.
* The caller owns image, depth and other bulk buffers. A library does not
  make hidden copies of them.
* Configuration structures are plain C structures with the same layout as
  the cFE tables, defined once under `interfaces/`. The validation function
  of a table lives in the library that uses it.
* Naming: functions `OST_<Library>_<Verb>` (for example `OST_Dense_Create`),
  types `OST_<Library>_<Name>_t`. The C++ namespace is
  `orthostream::<library>`.

### 3. Library responsibilities

* Calls are synchronous. The application decides on which task and when a
  call runs (D14). A library does not leave threads running after a call
  returns. Third-party modules may use threads internally during a call.
* Memory is allocated when a handle is created, within configured budgets
  (ADR-001 R4). Each library reports its memory use through a statistics
  function (D16).
* Libraries do not print. They report diagnostics through a callback that
  the application provides and maps to cFE events.

### 4. Libraries

| Library | Content | Records |
| --- | --- | --- |
| `geo` | frames, quaternions, PROJ transformations, camera model, Metashape conversion | ADR-004, ADR-008 |
| `terrain` | SRTM reading through GDAL, EGM96 to ellipsoidal heights | ADR-006 |
| `link` | decoding of the injector's UDP telemetry and TCP image frames; wire format documented under `interfaces/link/` and shared with the Python injector | ADR-003 |
| `time_align` | interpolation of the telemetry stream at image timestamps | ADR-003, D13 |
| `selection` | footprints, overlap, keyframe and source view selection, spatial index of the image pool | ADR-006 |
| `dense` | depth-estimator interface, PatchMatch adapter, SGBM baseline | ADR-005 |
| `map_products` | tiles, DSM fusion, orthomosaic, point cloud, GeoTIFF, LAZ and VRT writing | ADR-007 |

The mapping of libraries to applications is decided in D11.

### 5. COLMAP only in `dense`

* The build option `ORTHOSTREAM_WITH_COLMAP` enables PatchMatch. Without it,
  PatchMatch reports "not available" and SGBM still works.
* CI without a GPU builds with the option off.

### 6. Point cloud files

LAZ files are compressed in chunks and cannot be appended to. The point
cloud of ADR-007 section 4 is therefore written as one LAZ file per keyframe
and tile. Merging and thinning happen offline. This amends ADR-007.

### 7. Running without cFS

* Every library builds outside cFS, with its own GoogleTest tests.
* Small C++ tools under `tools/` run the libraries directly for the ADR-005
  benchmark and for experiments.
* The evaluation tools are Python. They read the written products with the
  GDAL Python bindings and `laspy` with `lazrs`, in their own virtual
  environment. PDAL is not needed.
* No Python bindings are written for the libraries for now.
* The first test: projecting the five ground control points into the
  Dorfberg images reproduces the Metashape marker positions within about
  0.5 px (ADR-008).

## Rationale

* A C interface with opaque handles keeps C++ out of the C99 applications and
  keeps the application shells small.
* Libraries without cFE dependencies can be tested, benchmarked and used in
  experiments without starting the mission.
* Caller-owned buffers and creation-time allocation keep data copies and
  memory use under the control of the execution model.
* GDAL and LASzip cover raster products, SRTM and LAZ without the size and
  packaging cost of PDAL.

## Alternatives Considered

* **C++ in cFS applications.** Rejected in ADR-009.
* **Algorithms inside the applications.** Rejected: they could not be tested
  or benchmarked without cFS.
* **Python bindings for the libraries.** Deferred: the C++ tools and
  file-based evaluation are sufficient now.
* **PDAL for point clouds.** Rejected (ADR-010).
* **Appending to one LAZ file per tile.** Rejected: LAZ does not support
  appending.

## Affected Areas

* [x] algorithm libraries
* [x] cFS application structure
* [x] configuration/tables
* [x] observability and performance measurement
* [x] evaluation and experiments
* [x] documentation

## Impact

* **ADR-007:** section 4 is amended by section 6 of this record.
* **D11:** maps the libraries to applications.
* **D12:** the transport carries references to caller-owned buffers.
* **D14:** decides the tasks on which library calls run.
* **D15:** table structures under `interfaces/` are shared with the
  libraries.
* **Injector:** uses the wire format under `interfaces/link/`.

## Open Questions

None.
