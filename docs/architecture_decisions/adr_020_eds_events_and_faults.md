# ADR-020: Interface Definitions with EDS, Events and Faults

* Status: Accepted
* Date: 2026-11-13
* Decided by: Ahmet Coşgun
* Backlog item: D21; completes D15 and D16
* Evidence: EDS build of the reference mission on 2026-11-13 (below);
  `vendor/cfs/tools/eds` (EDS tool, EdsLib, the COSMOS writer
  `cfecfs/missionlib/eds/91-write_cosmos_txt.lua`); the OpenC3 COSMOS
  plugin of the cFS bundle (`vendor/cfs/tools/cfs-cosmos-plugin`, setting
  `cfs_eds_enabled`); the fault catalog of NavionCore
  (`interfaces/messages/fault_ids.h`, `fault_event.h`) as a model;
  ADR-021 (COSMOS as the ground system)

## Context

ADR-016 defined the messages as hand-written C headers and built the
mission without Electronic Data Sheets (EDS). Adopting EDS was left to
D21.

The ground system is OpenC3 COSMOS (ADR-021). COSMOS cannot read C
headers. It needs its own definition of every packet: field offsets, types,
units, state names and limits. Hand-written C headers and hand-written COSMOS
definitions would describe the same data twice and would drift apart. A
padding or byte order mistake would shift every later field without any
error.

The earlier records also leave gaps that a ground system exposes:

* event identifiers are not fixed;
* there is no fault catalog. NavionCore has one: fault identifiers with
  severity, state and source, published as a `FaultEvent` message;
* ADR-011 says that every library function returns a status code, but the
  codes are not defined;
* the role of Google Logging (glog), which COLMAP and Ceres bring in, is not
  stated.

## Decision

### 1. EDS is the single source of the interface definitions

* The mission is built with `CFE_EDS_ENABLED=ON`.
* Every OrthoStream application has an EDS file `eds/<app>.xml`. It
  defines the application's commands, telemetry, payloads and table
  structures. The package name equals the application name (for example
  `DENSE_APP`).
* Shared types (payload header, time, object reference, pose with
  accuracy, trace times, fault event) form a package `OST`. It lives in an
  interface-only cFS library module, `ost_common`, because the EDS build
  collects the files from the `eds/` directories of the modules.
* From the EDS files the build generates:
  * the C type definitions used by the applications;
  * the COSMOS command and telemetry definitions (`build/cosmos/`);
  * the EDS database, which the Lua and Python bindings of EdsLib read
    (ground tools, tests).
* Topic identifiers are EDS design parameters. The reference mission
  assigns them in `mission/orthostream_defs/eds/cfe-topicids.xml`, a block
  of 16 per application, as ADR-016 section 1 planned. A host mission
  assigns its own.
* `TO_LAB` encodes telemetry through EdsLib. The packets on the link
  therefore follow the EDS definition (CCSDS headers big-endian, payload in
  the byte order of the EDS configuration), independent of the padding of the
  C structures.
* The component name that an application passes to
  `CFE_Config_GetVersionString` is its EDS package name. cFE reports
  "EDS active" only when the two match. The cFS lab applications pass display
  names ("TO Lab") and are therefore reported as "EDS inactive", although
  their EDS definitions are used.
* OrthoStream applications require an EDS build. A host mission built
  without EDS is not supported. Keeping hand-written headers next to the EDS
  definitions would bring back the duplication that this record removes.

### 2. Link wire format

The link between the injector and `INGEST_APP` (ADR-016 section 7) is not a
Software Bus message and stays as defined there, under `interfaces/link/`.
It is not visible to COSMOS.

### 3. Events

* Each application defines its event identifiers in `<app>_eventids.h`.
  Identifiers are never reused or renumbered.
* Ranges within each application:

  | Range | Use |
  | --- | --- |
  | 1–19 | life cycle and commands: initialisation, no-op, reset, command errors, pipe errors, table load and validation |
  | 20–59 | data path: state changes, dropped items with their reason, rejected poses or batches |
  | 60–79 | fault transitions (section 4) |
  | 80–99 | library diagnostics (section 6) |

* Event types:
  * `CRITICAL`: the application cannot continue its function;
  * `ERROR`: an item or a request failed;
  * `INFORMATION`: state changes and command acknowledgements;
  * `DEBUG`: diagnostics, disabled by default.
* Events are raised on changes of state, not per item. Per-item counts go
  to housekeeping. Events that can repeat are limited with EVS binary
  filters (for example the first 8, then none).
* Event texts fit the cFE limit of 122 characters and start with the
  object they refer to (image sequence number, keyframe, tile).
* `docs/interfaces/events.md` lists all event identifiers. A test checks
  that the identifiers of each application are unique.

### 4. Faults

* A fault catalog `OST_FAULT_*` defines the faults of the mission. Each
  entry has:
  * a fixed numeric identifier;
  * a severity: `INFO`, `WARNING`, `DEGRADED`, `ERROR` or `CRITICAL`;
  * the application that raises it;
  * a clear policy: cleared automatically when the condition is gone, or
    latched until a command clears it.
* A fault changes state between `ACTIVE`, `CLEARED` and `LATCHED`. Each
  change is published once as a `FaultEvent` message (identifier,
  severity, state, source, time, occurrence count, short text). It is also
  raised as an EVS event, for people.
* The housekeeping of each application carries the number of active faults
  and the highest active severity. COSMOS limits are defined on them.
* Every application accepts the command "clear latched faults".
* The first catalog:

  | Fault | Raised by | Severity |
  | --- | --- | --- |
  | telemetry stream stale | `INGEST_APP` | `ERROR` |
  | image stream stale | `INGEST_APP` | `ERROR` |
  | link format or version mismatch | `INGEST_APP` | `CRITICAL` |
  | no telemetry around an image (no pose) | `ALIGN_APP` | `WARNING` |
  | pose accuracy below the configured limit | `ALIGN_APP` | `DEGRADED` |
  | data store quota exhausted | any user of the data store | `ERROR` |
  | keyframes skipped (backlog) | `DENSE_APP` | `DEGRADED` |
  | GPU failure | `DENSE_APP` | `CRITICAL` |
  | refinement rejected (ADR-019 section 5) | `REFINE_APP` | `WARNING` |
  | refinement over its latency limit | `REFINE_APP` | `DEGRADED` |
  | tile write failure | `MAPPROD_APP` | `ERROR` |
  | output disk space low | `MAPPROD_APP`, `RECORDER_APP` | `WARNING` |
  | configuration table rejected | any application | `ERROR` |
  | library internal error | any application | `ERROR` |

### 5. Status codes of the library C interfaces

* All C interfaces (ADR-011) return `OST_Status_t`, a 32-bit signed
  integer: zero for success and negative values for errors, as cFE does.
* A shared header `ost_status.h` defines the codes:

  | Code | Meaning |
  | --- | --- |
  | `OST_SUCCESS` | success |
  | `OST_ERR_INVALID_ARGUMENT` | invalid argument or configuration |
  | `OST_ERR_NOT_INITIALISED` | handle missing or not ready |
  | `OST_ERR_NO_MEMORY` | allocation or memory budget exceeded |
  | `OST_ERR_QUOTA` | data store quota exhausted |
  | `OST_ERR_IO` | file or device error |
  | `OST_ERR_FORMAT` | malformed input data |
  | `OST_ERR_NUMERIC` | numerical failure (no convergence, degenerate geometry) |
  | `OST_ERR_GPU` | GPU or CUDA error |
  | `OST_ERR_TIMEOUT` | time limit exceeded |
  | `OST_ERR_UNSUPPORTED` | feature not built or not supported |
  | `OST_ERR_INTERNAL` | unexpected error, including caught C++ exceptions |

* The C++ exceptions of a library (for example `RefineError`) are mapped to
  these codes in its interface layer. The message text stays available
  through the interface.

### 6. Library diagnostics, and the role of glog

* Libraries report diagnostics through the callback of ADR-011, with a
  level (debug, information, warning, error), a status code and a text.
* The application maps them:
  * warnings and errors become EVS events in the range 80–99, filtered;
  * debug and information messages become EVS `DEBUG` events. These are
    disabled by default and can be enabled for one application from COSMOS
    with the standard EVS command. Detailed logs then appear in the ground
    system only when an operator asks for them.
* Google Logging (glog) is not an observability channel of OrthoStream:
  * it stays the internal logger of COLMAP and Ceres, set to errors only
    (`libs/dense`);
  * an application may additionally write library diagnostics to a log file
    in the run directory (ADR-017 section 4) through glog, for development.
    This is off in release builds;
  * `CHECK`, `DCHECK` and `LOG(FATAL)` are not used in OrthoStream code.
    They call `abort()`, and all cFS applications run in one process, so
    one failed check would stop the whole mission.
* Libraries never terminate the process: no `abort()`, `exit()` or failing
  `assert` in release builds. Third-party code that can terminate the process
  is isolated (COLMAP's `exit()` on CUDA errors, ADR-005 open question).

## Rationale

* EDS is the CCSDS standard for these definitions (CCSDS 876.0) and the
  form that cFS itself uses: cFE and every application of the bundle ship an
  EDS file. The COSMOS plugin of the bundle supports EDS.
* One definition generates the C types, the COSMOS definitions and the
  ground-side bindings, so they cannot drift apart. Encoding through EdsLib
  removes the dependency of the link format on C structure padding.
* The ground system can be replaced without touching the applications. A
  generator for another ground system reads the same EDS files.
* Fixed event identifiers and a fault catalog let COSMOS show, filter and
  limit-check faults. Free-text logs cannot be checked this way.
* Debug diagnostics through EVS `DEBUG` events reach the ground system on
  request without a separate log path, and cost nothing when disabled.
* Banning aborting checks follows from the single-process model of cFS.

## Alternatives Considered

* **Hand-written C headers and hand-written COSMOS definitions (ADR-016).**
  Rejected: two descriptions of the same data, and silent offset errors.
* **C headers checked against COSMOS definitions by a test.** Rejected: the
  check detects drift but does not remove the duplicated work.
* **A project-specific YAML format with its own generator.** Rejected: it
  would reimplement EDS in a non-standard way, and cFE's own messages would
  still need separate definitions.
* **Supporting both EDS and non-EDS host missions, as the cFS sample
  application does.** Rejected for now: it doubles the definitions of every
  application.
* **glog as the logging channel to the ground system.** Rejected: glog
  produces text without identifiers or severity semantics that COSMOS
  understands, and its fatal checks would stop the whole mission.

## Affected Areas

* [x] Software Bus messages
* [x] configuration/tables
* [x] cFS application structure
* [x] observability and performance measurement
* [x] vendor/cFS baseline (build option only; nothing under `vendor/cfs` is changed)
* [x] documentation

## Impact

* **ADR-009:** the mission is built with EDS (`mission/build.sh`). The EDS
  design parameters of the reference mission are under
  `mission/orthostream_defs/eds/` (copied from the cFE sample definitions,
  then extended with the OrthoStream topic identifiers).
* **ADR-010:** Lua 5.4.8 and Expat 2.8.5 are added as build-host tools
  (manifest feature `eds`), and GNU m4 as a system prerequisite.
  * Lua 5.5 (the vcpkg baseline) is not supported by the EDS tool.
  * Expat 2.9.0 deprecates a function that the EDS tool uses, which the
    tool's `-Werror` rejects.
* **ADR-011:** the status codes of section 5 and the diagnostic levels of
  section 6.
* **ADR-016:** section 1 is replaced by section 1 of this record. The
  message contents of sections 2–4 stay and are written in EDS.
* **ADR-017:** the event and fault rules of sections 3 and 4. `FaultEvent`
  messages are also recorded by `RECORDER_APP`.
* **ADR-021:** the generated COSMOS definitions are the basis of the
  OrthoStream COSMOS plugin.

## EDS Build Result (2026-11-13)

* `mission/build.sh` with `CFE_EDS_ENABLED=ON` builds the reference mission
  (cFE, `SCH_LAB`, `CI_LAB`, `TO_LAB`) from a clean build directory.
  * The EDS tool processes all definitions with 0 errors and 0 warnings.
  * The mission starts and reaches the `OPERATIONAL` state.
  * The five cFE services report "EDS active".
* The build generates 26 COSMOS definition files for cFE, `CI_LAB` and
  `TO_LAB` under `build/cosmos/`. They use the header helpers of the cFS
  COSMOS plugin (`cfs_tlm_hdr`), so they fit into it.

## Open Questions

* Python bindings of EdsLib for the injector, `RECORDER_APP` tools and the
  evaluation tools: they need the Python development headers. Decided when
  the first ground-side tool decodes Software Bus messages.
* Support for host missions without EDS, if a user needs it.
