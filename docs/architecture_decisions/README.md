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

An accepted record is not rewritten. When a decision changes, a new record
supersedes it and only the status line of the old record is updated.

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

A decision that narrows, changes or departs from one of these commitments says
so in its Context section and records the supervisor's agreement under
Evidence.

## Records

| ID | Title | Backlog item | Status |
| --- | --- | --- | --- |
| [ADR-001](adr_001_requirements_and_real_time.md) | Flight envelope, real-time definition and bounded memory | D01 | Accepted |
| [ADR-002](adr_002_target_hardware_and_runtime.md) | Target hardware and runtime environment | D02 | Accepted |

## Decision Backlog

Decisions are taken in this order because later items depend on earlier ones.
An item may produce one record, several records, or be merged with another
item; the ADR column links the resulting records.

### A. Frame: Requirements, Platform and Data Source

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D01 | Requirements and the meaning of "real time": flight envelope as ranges (altitude, speed, overlap) rather than a single scenario, resulting image rate, map update latency, image resolution, DSM and orthomosaic ground sampling distance; no upper limit on survey area, with memory use independent of the area covered | — | [ADR-001](adr_001_requirements_and_real_time.md) | Accepted |
| D02 | Hardware and runtime environment: simulator and cFS on one or two machines, GPU sharing, operating system, CUDA version | D01 | [ADR-002](adr_002_target_hardware_and_runtime.md) | Accepted |
| D03 | Simulator and data interface: which simulator, which data it provides (images, poses, intrinsics, reference surface, time), live streaming or real-time replay of recorded runs; emulated UAV-to-ground radio link (protocol, bandwidth, latency, loss); replay of the nominal Dorfberg dataset; how altitude and tilt are varied; how the 60 min continuous-operation test is fed | D01, D02 | — | Open |
| D04 | Coordinate frames and conventions: axis definitions, pose representation (e.g. `T_W_C`, quaternion order), ENU origin and its re-anchoring policy for unbounded survey areas, output map projection (e.g. UTM) and zone boundary handling | D01, D03 | — | Open |

### B. Algorithms

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D05 | Dense reconstruction method: COLMAP PatchMatch Stereo (CUDA) as a library with known poses, a two-view SGBM baseline, or both; number of source views per depth map | D01, D02, D03 | — | Open |
| D06 | Frame and source view selection: footprint overlap, baseline and viewing-angle criteria and thresholds | D04, D05 | — | Open |
| D07 | Depth fusion and incremental map products: tiled DSM and orthomosaic layout in a fixed projection, tile finalisation and eviction to keep memory bounded, cell update rule, orthomosaic pixel source selection and blending | D01, D04, D05 | — | Open |
| D08 | Offline COLMAP reference workflow: how COLMAP is run with known poses and which settings are used for the comparison | D05 | — | Open |

### C. cFS Software Architecture

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D09 | cFS version and build structure: how cFS enters the repository, mission and target definitions, linking C applications with C++ libraries | D02 | — | Open |
| D10 | Boundary between algorithm libraries and cFS application shells, so that algorithms and experiments run without cFS | D09 | — | Open |
| D11 | Final application boundaries: separate or combined DSM and orthomosaic applications, separate depth fusion application | D05, D07, D10 | — | Open |
| D12 | Large data transport: shared memory buffer pool, file references or another mechanism for images, depth maps and point clouds | D01, D03, D09 | — | Open |
| D13 | Time model: simulator time versus cFE time, meaning of message timestamps | D03, D09 | — | Open |
| D14 | Execution model: scheduler-driven or event-driven applications, child tasks for long-running work, queue limits, backpressure and frame dropping policy | D05, D11, D12 | — | Open |
| D15 | Messages, commands and tables: message IDs, payload fields, versioning, configuration through cFE Table Services | D09–D14 | — | Open |
| D16 | Observability: housekeeping, events and per-stage timing used for the latency and throughput metrics | D14, D15 | — | Open |

### D. Evaluation

| Item | Decision | Depends on | ADR | Status |
| --- | --- | --- | --- | --- |
| D17 | Experiment design: scenario matrix, reference surface extraction, metric computation tools | A, B | — | Open |
