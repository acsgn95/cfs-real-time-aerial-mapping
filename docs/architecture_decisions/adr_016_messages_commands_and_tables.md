# ADR-016: Messages, Commands and Tables

* Status: Accepted
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D15
* Revised: 2026-11-11 (reusable components for other cFS missions)
* Evidence: ADR-003 (link channels), ADR-004 (frames and conventions),
  ADR-011 (table structures shared with libraries), ADR-012 (applications),
  ADR-013 (object references; `datastore` is a cFS library module),
  ADR-014 (time format), ADR-015 (table values for limits and threads);
  cFE Table Services register tables per application; the mission is built
  with `CFE_EDS_ENABLED=OFF` (ADR-009)

## Context

ADR-009 to ADR-015 define the applications, their data and their limits. This
record fixes how messages, commands and configuration are defined. The exact
fields of each message are written in the interface headers when the
applications are implemented, and they must follow this record.

## Decision

### 1. Where definitions live

* Message structures are C headers under `interfaces/messages/`.
* cFS Electronic Data Sheets (EDS) are not used. The mission is built with
  EDS disabled. Adopting EDS later would be a decision of its own.
* Every application follows the cFS v7 component layout, as the cFS sample
  application does. It ships its own default configuration headers under
  `config/` (`default_<app>_topicid_values.h`, `default_<app>_msgids.h`,
  `default_<app>_mission_cfg.h`, `default_<app>_platform_cfg.h` and the table
  definitions). A host mission overrides any of them by placing a file with
  the same name without the `default_` prefix in its `*_defs` directory.
* The default topic identifiers give each application a block of 16, with
  separate ranges for commands and telemetry, and avoid the ranges of cFE and
  the lab applications. A host mission with other ranges overrides them.

### 2. Common elements

* **Payload header:** every payload starts with a schema version, a sequence
  counter and flags.
* **Time:** signed 64-bit integer nanoseconds since the Unix epoch, UTC
  (ADR-014).
* **Data reference:** `OST_ObjectRef_t {object id, generation}` (ADR-013).
* **Poses inside OrthoStream:** position in ECEF (double precision) and the
  camera-to-ECEF quaternion `q_E_C` (Hamilton, w, x, y, z; ADR-004). This is
  global and unambiguous. Geodetic position and `q_N_B` are used only on the
  link, up to `INGEST_APP`.
* **Accuracy:** every pose carries its accuracy fields, taken from the
  telemetry. No component assumes a pose quality.

### 3. Data path messages

| Publisher | Message | Content |
| --- | --- | --- |
| `INGEST_APP` | `TelemetrySample` | time, geodetic position, `q_N_B`, accuracy fields, recorded time |
| `INGEST_APP` | `ImageArrived` | image sequence number, capture time, recorded time, JPEG reference, size |
| `INGEST_APP` | `SessionCalibration` | intrinsics (OpenCV convention), distortion, `T_B_C`, image size |
| `ALIGN_APP` | `ImagePose` | image, pose (`p_E`, `q_E_C`), accuracy, validity and rejection reason |
| `SELECT_APP` | `KeyframeBatch` | keyframe and K sources (references and poses), working-resolution intrinsics, footprint plane height |
| `DENSE_APP` | `DepthReady` | depth set reference, origin of `L_k`, timing |
| `DENSE_APP` | `DenseStatus` | jobs, skipped keyframes, queue state |
| `GEOREF_APP` | `PointsReady` | point set reference, affected tiles |
| `MAPPROD_APP` | `TileUpdated`, `TilePublished` | tile identifiers and times; the latency of ADR-001 R2 is measured from these |
| every application | `<APP>_HK` | counters, queue lengths, statistics |

All messages stay well below the 32 KB Software Bus limit.

### 4. Commands

* Every application accepts the standard cFS commands: no-operation and
  reset counters.
* Additional commands:

  | Application | Commands |
  | --- | --- |
  | `INGEST_APP` | start session, stop session |
  | `MAPPROD_APP` | publish now, finalise all tiles |
  | `RECORDER_APP` | start recording, stop recording, set run identifier |

### 5. Tables

* Every application owns one configuration table. The thresholds and limits
  of the earlier records become table values.
* Table structures live under `interfaces/`. Their validation functions live
  in the libraries that use them (ADR-011).
* Each application ships default table sources under its own `fsw/tables/`.
  A mission may replace them; the reference mission keeps its tables under
  `mission/orthostream_defs/tables/`.
* When experiments start, a tool generates table images from YAML files.
  Each experiment configuration is then stored as a file and can be
  repeated.
* Tables are loaded at startup only. They are not changed while the mission
  runs. Experiments restart the mission, so live changes are not needed.

### 6. Data store sizes

`datastore` is a cFS library module, and cFE tables belong to applications.
The arena size and the quotas per type are therefore platform configuration
values set at build time, as cFE does for its own memory pools. They follow
the same pattern: a default platform configuration header in the library,
overridable by the mission.

### 7. Link wire format

Defined under `interfaces/link/` and shared with the Python injector:

* little-endian, packed binary;
* header: magic word `OSTL`, format version, message type, length, sequence
  number;
* UDP: one telemetry sample per datagram;
* TCP: one frame per image, with an image header followed by the JPEG data;
* no additional checksum. UDP and TCP checksums detect corruption, including
  corruption emulated with `netem`.

## Rationale

* C headers are the standard cFS form and are directly usable by the C99
  applications and the C interfaces of the libraries.
* ECEF positions and `q_E_C` avoid frame ambiguities between applications.
  Working frames (`L_k`) stay internal to the stages that use them.
* Shared table structures with validation in the libraries keep the cFS
  configuration and the host tools consistent.
* Loading tables at startup only keeps the applications simple and
  experiments reproducible.
* A small, versioned binary link format is easy to implement in both C and
  Python.

## Alternatives Considered

* **EDS.** Not now: the mission is built without it, and it adds tooling.
* **Geodetic positions inside OrthoStream.** Rejected: every stage would
  convert them, and heights would mix with angles.
* **Live table updates.** Rejected: not needed for restartable experiments.
* **Data store sizes in a table owned by one application.** Rejected: it
  would couple a shared service to one application.
* **Text formats on the link (for example JSON).** Rejected: larger and
  slower to parse, with no benefit for a binary image stream.

## Affected Areas

* [x] Software Bus messages
* [x] configuration/tables
* [x] simulator interface
* [x] large data transport
* [x] documentation

## Impact

* **Implementation:** `interfaces/messages/`, `interfaces/link/`, the
  configuration headers of each application and its default tables are
  created with the first applications.
* **Injector:** implements the link format of section 7.
* **`TO_LAB`:** its subscription table is extended with the housekeeping and
  status messages for development.
* **D16:** housekeeping and status contents.

## Open Questions

None.
