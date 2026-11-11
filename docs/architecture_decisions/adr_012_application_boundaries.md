# ADR-012: Application Boundaries

* Status: Accepted; amended by [ADR-019](adr_019_pose_refinement.md) (optional `REFINE_APP` between `SELECT_APP` and `DENSE_APP`)
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D11
* Revised: 2026-11-11 (reusable components for other cFS missions)
* Evidence: thesis proposal (FB_YL-01) sections 2 and 4.1; ADR-003 to
  ADR-011; OSAL name limits (20 characters including terminator for API and
  file names)

## Context

The proposal splits the workflow into separate cFS applications. Section 4.1
names seven functions: receiving images and camera data, time alignment,
image selection, dense point cloud generation, georeferencing, map products
and recording. Section 2 lists the DSM and the orthomosaic separately and does
not mention image selection.

ADR-011 defines the libraries. This record maps them to applications.

## Decision

### 1. Applications

```text
injector --UDP/TCP--> INGEST_APP --> ALIGN_APP --> SELECT_APP --> DENSE_APP --> GEOREF_APP --> MAPPROD_APP

RECORDER_APP <-- status, timing and health messages of all applications
```

| Application | Responsibility | Libraries |
| --- | --- | --- |
| `INGEST_APP` | receives UDP telemetry and TCP image frames from the injector; stores the JPEG data in the data store (D12); publishes telemetry samples, image arrivals and the session calibration | `link` |
| `ALIGN_APP` | buffers telemetry and interpolates the pose at each image timestamp; rejects invalid associations (rules in D13); publishes the image pose | `time_align`, `geo` |
| `SELECT_APP` | decodes every image to the dense working resolution; computes footprints with the terrain prior (Copernicus DEM); selects keyframes and source views; owns the spatial index of the image pool; publishes keyframe batches | `selection`, `terrain`, `imaging`, `geo` |
| `DENSE_APP` | computes depth, normal and confidence maps for each keyframe batch (PatchMatch or SGBM); runs the GPU work | `dense`, `geo` |
| `GEOREF_APP` | converts depth maps to points and transforms them from `L_k` through ECEF to the product frame `P`; writes the point cloud product (LAZ); publishes georeferenced point sets | `geo`, `map_products` (LAZ writer) |
| `MAPPROD_APP` | fuses points into the DSM, renders the orthomosaic, manages the tile lifecycle and publication | `map_products`, `imaging`, `geo` |
| `RECORDER_APP` | records status, timing and health messages to files for the evaluation; does not record raw images (ADR-001) | — |

The cFS lab applications (`SCH_LAB`, `CI_LAB`, `TO_LAB`) stay in the reference
mission during development. The OrthoStream applications do not depend on
them (ADR-015).

### 1a. `INGEST_APP` is a source adapter

* `INGEST_APP` adapts one data source, the injector link of ADR-003, to the
  OrthoStream messages `TelemetrySample`, `ImageArrived` and
  `SessionCalibration` (ADR-016).
* From `ALIGN_APP` onwards, every application works only with these messages
  and does not know the source.
* Other sources, for example MAVLink telemetry from an autopilot or a camera
  SDK, are added as further adapter applications that publish the same
  messages. The rest of the pipeline stays unchanged.

### 2. Georeferencing is a separate application

`GEOREF_APP` follows the proposal and owns the point cloud product. The
additional Software Bus hop is part of the cFS overhead that the thesis
measures.

### 3. DSM and orthomosaic share one application

Orthomosaic rendering reads the DSM tiles in memory (ADR-007). Splitting the
two products into separate applications would require sharing the tiles
between applications. This follows section 4.1 of the proposal ("map
products") rather than section 2.

### 4. New library: `imaging`

JPEG decoding with DCT scaling (libjpeg-turbo), undistortion and resampling
are used by both `SELECT_APP` (working resolution) and `MAPPROD_APP`
(orthomosaic scale). They form an eighth library, `imaging`, which amends the
list of ADR-011.

### 5. Names

* Application names are short and do not carry the `OST_` prefix. The entry
  point `<NAME>_Main` and the module file `<name>.so` must fit OSAL's name
  limits, which `OST_RECORDER_APP_Main` would exceed.
* The `OST_` prefix remains for project-wide C symbols (ADR-009).
* Directories under `apps/` use the lowercase name (`apps/ingest_app/`).

### 6. Shared data store (input for D12)

The image pool is used by four applications: `SELECT_APP` keeps its index,
`DENSE_APP` reads the working-resolution images, `GEOREF_APP` reads depth
maps, and `MAPPROD_APP` reads keyframe JPEG data. Bulk data therefore lives
in a data store shared between applications. Messages carry references to
it. D12 defines the store.

## Rationale

* Seven applications match the functions named in section 4.1 of the proposal
  and the README.
* Each application has one responsibility and a small set of libraries.
* Keeping DSM and orthomosaic together avoids sharing tile memory between
  applications, which would add complexity without benefit.
* A separate `imaging` library avoids duplicating decoding code in two
  applications.

## Alternatives Considered

* **Georeferencing inside `DENSE_APP` or `MAPPROD_APP`.** Rejected: departs
  from the proposal and leaves the point cloud product without a clear owner.
* **Separate DSM and orthomosaic applications.** Rejected: requires shared
  tile memory.
* **Image decoding in `INGEST_APP`.** Rejected: it would delay reception and
  decode images at a resolution that only selection and dense
  reconstruction need.
* **`OST_` prefix in application names.** Rejected: exceeds OSAL name limits.

## Affected Areas

* [x] cFS application structure
* [x] algorithm libraries
* [x] Software Bus messages
* [x] large data transport
* [x] documentation

## Impact

* **ADR-011:** the library list gains `imaging`.
* **D12:** defines the shared data store.
* **D14:** decides tasks, priorities and child tasks per application.
* **D15:** defines the messages between the applications.
* **README:** the planned architecture table is updated to these names.

## Open Questions

None.
