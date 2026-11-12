# Architecture Decisions

This directory records the architecture decisions of OrthoStream. Each record follows the structure of the "Architecture Decision" issue
template (`.github/ISSUE_TEMPLATE/architecture_decision.md`): context, decision,
rationale, alternatives considered, affected areas, impact and open questions,
plus status, date, backlog item and evidence.

A decision record is written when the architecture changes because of a
decision rather than a clarification. The affected documents are updated in the
same change and point back to the record.

## Record Format

File name: `adr_NNN_short_title.md`. Records are numbered in the order they are
accepted, not in backlog order.

Every record starts with:

```text
# ADR-NNN: Title

* Status: Proposed | Accepted | Superseded by ADR-NNN | Deprecated
* Date: YYYY-MM-DD
* Decided by: <name>
* Backlog item: Dnn
* Evidence: <documents, experiments, measurements or commits that support it>
```

An accepted record may be extended in place when the change does not reverse
its decision, for example a clarification or an added constraint. Such a
change adds a `Revised: <date> (<reason>)` line to the header. When a
decision is reversed or replaced, a new record supersedes or amends it, and
the status line of the old record says so.

## Product Orientation

OrthoStream is designed as a set of reusable cFS components, not only as a
thesis prototype:

* the applications follow the cFS v7 component layout, with default
  configuration headers that a host mission can override (ADR-016);
* the algorithm libraries do not depend on cFE and can be used outside cFS
  (ADR-011);
* dependencies are pinned and built by the project (ADR-010);
* memory is bounded and preallocated (ADR-001, ADR-013);
* no component assumes a given data source or pose quality (ADR-012,
  ADR-019).

The reference mission under `mission/` is the thesis configuration and an
integration example.

## Relationship with the Thesis Proposal

The approved master's thesis proposal (FB_YL-01, dated 2026-10-05) fixes the
scope of this project. Its commitments include:

* camera poses, timestamps and calibration are supplied by the simulator; the
  trajectory is not estimated from imagery;
* the workflow is split into independent cFS applications that exchange data
  through the cFE Software Bus, with OSAL used for tasks, queues, semaphores
  and time services;
* dense reconstruction takes the COLMAP multi-view stereo approach as its
  reference (source view selection, photometric and geometric consistency,
  depth map fusion; Schönberger et al., 2016);
* incremental DSM and orthomosaic generation takes AerialMapper as its
  reference (Hinzmann et al., 2018);
* WGS 84, ECEF and local ENU are used for georeferencing;
* results are compared with an offline COLMAP reconstruction and the
  simulator's reference surface, using the metrics listed in the proposal.

The proposal is the starting point of the scope, not its limit. A decision
that narrows, changes, extends or departs from one of these commitments says
so in its Context section. ADR-019 is such an extension: the poses can
optionally be refined from the imagery.

## Records

| ID | Title | Backlog item | Status |
| --- | --- | --- | --- |
| [ADR-001](adr_001_requirements_and_real_time.md) | Flight envelope, real-time definition and bounded memory | D01 | Accepted, amended by ADR-003 and ADR-019 |
| [ADR-002](adr_002_target_hardware_and_runtime.md) | Target hardware and runtime environment | D02 | Accepted, amended by ADR-009 and ADR-014 |
| [ADR-003](adr_003_data_source_and_injector.md) | Data source and injector interface | D03 | Accepted |
| [ADR-004](adr_004_coordinate_frames_and_conventions.md) | Coordinate frames and conventions | D04 | Accepted, completed by ADR-008 |
| [ADR-005](adr_005_dense_reconstruction_method.md) | Dense reconstruction method | D05 | Accepted; parameters from the benchmark (section 6a); amended by ADR-019 |
| [ADR-006](adr_006_keyframe_and_source_view_selection.md) | Keyframe and source view selection | D06 | Accepted |
| [ADR-007](adr_007_depth_fusion_and_map_products.md) | Depth fusion and incremental map products | D07 | Accepted, amended by ADR-011 |
| [ADR-008](adr_008_offline_reference_and_camera_model.md) | Offline reference products and camera model conventions | D08 | Accepted |
| [ADR-009](adr_009_cfs_version_and_build_structure.md) | cFS version and build structure | D09 | Accepted, amended by ADR-010 |
| [ADR-010](adr_010_third_party_dependency_management.md) | Third-party dependency management | D09 | Accepted |
| [ADR-011](adr_011_library_and_application_boundary.md) | Boundary between algorithm libraries and cFS applications | D10 | Accepted, amended by ADR-012, ADR-013 and ADR-019 |
| [ADR-012](adr_012_application_boundaries.md) | Application boundaries | D11 | Accepted, amended by ADR-019 |
| [ADR-013](adr_013_shared_data_store.md) | Shared data store for bulk data | D12 | Accepted |
| [ADR-014](adr_014_time_model.md) | Time model | D13 | Accepted |
| [ADR-015](adr_015_execution_model.md) | Execution model | D14 | Accepted, amended by ADR-019 |
| [ADR-016](adr_016_messages_commands_and_tables.md) | Messages, commands and tables | D15 | Accepted, amended by ADR-019 |
| [ADR-017](adr_017_observability.md) | Observability and performance measurement | D16 | Accepted |
| [ADR-018](adr_018_experiment_design.md) | Experiment design | D17 | Accepted, amended by ADR-019 |
| [ADR-019](adr_019_pose_refinement.md) | Optional pose refinement | D18 | Accepted |

## Decision Backlog

Decisions are taken in this order because later items depend on earlier ones.
An item may produce one record, several records, or be merged with another
item; the ADR column links the resulting records.

### A. Frame: Requirements, Platform and Data Source

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D01 | Requirements and the meaning of "real time": flight envelope as ranges (altitude, speed, overlap) rather than a single scenario, resulting image rate, map update latency, image resolution, DSM and orthomosaic ground sampling distance; no upper limit on survey area, with memory use independent of the area covered | — | [ADR-001](adr_001_requirements_and_real_time.md) | Accepted |
| D02 | Hardware and runtime environment: simulator and cFS on one or two machines, GPU sharing, operating system, CUDA version | D01 | [ADR-002](adr_002_target_hardware_and_runtime.md) | Accepted |
| D03 | Simulator and data interface: which simulator, which data it provides (images, poses, intrinsics, reference surface, time), live streaming or real-time replay of recorded runs; emulated UAV-to-ground radio link (protocol, bandwidth, latency, loss); replay of the nominal Dorfberg dataset; how altitude and tilt are varied; how the 60 min continuous-operation test is fed | D01, D02 | [ADR-003](adr_003_data_source_and_injector.md) | Accepted |
| D04 | Coordinate frames and conventions: axis definitions, pose representation (e.g. `T_W_C`, quaternion order), ENU origin and its re-anchoring policy for unbounded survey areas, output map projection (e.g. UTM) and zone boundary handling | D01, D03 | [ADR-004](adr_004_coordinate_frames_and_conventions.md) | Accepted |

### B. Algorithms

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D05 | Dense reconstruction method: COLMAP PatchMatch Stereo (CUDA) as a library with known poses, a two-view SGBM baseline, or both; number of source views per depth map | D01, D02, D03 | [ADR-005](adr_005_dense_reconstruction_method.md) | Accepted; benchmark after D09 |
| D06 | Frame and source view selection: footprint overlap, baseline and viewing-angle criteria and thresholds | D04, D05 | [ADR-006](adr_006_keyframe_and_source_view_selection.md) | Accepted |
| D07 | Depth fusion and incremental map products: tiled DSM and orthomosaic layout in a fixed projection, tile finalisation and eviction to keep memory bounded, cell update rule, orthomosaic pixel source selection and blending | D01, D04, D05 | [ADR-007](adr_007_depth_fusion_and_map_products.md) | Accepted |
| D08 | Offline reference workflow (decided: Metashape products of the same flight instead of an offline COLMAP run) and camera model conventions | D05 | [ADR-008](adr_008_offline_reference_and_camera_model.md) | Accepted |

### C. cFS Software Architecture

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D09 | cFS version and build structure: how cFS enters the repository, mission and target definitions, linking C applications with C++ libraries; third-party dependencies | D02 | [ADR-009](adr_009_cfs_version_and_build_structure.md), [ADR-010](adr_010_third_party_dependency_management.md) | Accepted |
| D10 | Boundary between algorithm libraries and cFS application shells, so that algorithms and experiments run without cFS | D09 | [ADR-011](adr_011_library_and_application_boundary.md) | Accepted |
| D11 | Final application boundaries: separate or combined DSM and orthomosaic applications, separate depth fusion application | D05, D07, D10 | [ADR-012](adr_012_application_boundaries.md) | Accepted |
| D12 | Large data transport: shared memory buffer pool, file references or another mechanism for images, depth maps and point clouds | D01, D03, D09 | [ADR-013](adr_013_shared_data_store.md) | Accepted |
| D13 | Time model: simulator time versus cFE time, meaning of message timestamps | D03, D09 | [ADR-014](adr_014_time_model.md) | Accepted |
| D14 | Execution model: scheduler-driven or event-driven applications, child tasks for long-running work, queue limits, backpressure and frame dropping policy | D05, D11, D12 | [ADR-015](adr_015_execution_model.md) | Accepted |
| D15 | Messages, commands and tables: message IDs, payload fields, versioning, configuration through cFE Table Services | D09–D14 | [ADR-016](adr_016_messages_commands_and_tables.md) | Accepted |
| D16 | Observability: housekeeping, events and per-stage timing used for the latency and throughput metrics | D14, D15 | [ADR-017](adr_017_observability.md) | Accepted |

### D. Evaluation

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D17 | Experiment design: scenario matrix, reference surface extraction, metric computation tools | A, B | [ADR-018](adr_018_experiment_design.md) | Accepted |

### E. Optional Extensions

Extensions are implemented after the core pipeline works with `optimized`
poses. The core architecture keeps room for them: no component assumes a
given pose quality, and pose accuracy is always taken from the accuracy
fields of the telemetry.

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D18 | Optional pose refinement before dense reconstruction, chosen before the flight with two configuration values: `refine_pose` (off: batches go directly to `DENSE_APP`; on: `REFINE_APP` refines the poses first) and `precise_position` (on: PPK or RTK positions are kept fixed and only the attitude is estimated; off: positions and attitudes are estimated with priors weighted by the reported accuracy). Sliding-window bundle adjustment with GTSAM 4.3.0 that waits for the neighbouring flight line; latency limit `refine_max_latency`, default 120 s | D05, D06, D11, D14 | [ADR-019](adr_019_pose_refinement.md) | Accepted |

### F. Product Readiness

These items are not needed for the thesis. They are decided before a
production use.

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D19 | Unit tests of the cFS applications with UT-Assert and stubs, and functional tests of the mission | D15 | — | Open |
| D20 | Security of the data link and of command ingest (authentication, replacing `CI_LAB`) | D03, D15 | — | Open |
| D21 | Electronic Data Sheets (EDS) for interoperability with ground systems | D15 | — | Open |
| D22 | Component versioning, packaging and deployment to a production target | D09, D10 | — | Open |
| D23 | Ground station and operator interface, outside the cFS mission: commands, telemetry, health and live map display. Candidates: OpenC3 COSMOS with the cFS plugin of `vendor/cfs/tools` (commands, telemetry, limits, logging, scripted experiments) plus a map view, either as a COSMOS tool or in an installable Electron application that also hosts the COSMOS web interface; a Qt application; or a plain web viewer. Questions: map display of the published tiles (Cloud Optimized GeoTIFF for browser viewers), generation of COSMOS packet definitions from the message headers, licenses (COSMOS open source edition, Qt LGPL), resource use next to the mission. Until decided, QGIS on the published VRT serves as the map display. Decided once the pipeline produces telemetry | D11, D15, D16 | — | Open |
