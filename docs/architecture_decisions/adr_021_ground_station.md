# ADR-021: Ground Station and Operator Interface

* Status: Accepted; implementation notes added 2026-11-14 (own instance,
  plugin generator)
* Date: 2026-11-13
* Decided by: Ahmet Coşgun
* Backlog item: D23
* Evidence: the OpenC3 COSMOS plugin of the cFS bundle
  (`vendor/cfs/tools/cfs-cosmos-plugin`: UDP interfaces to `CI_LAB` and
  `TO_LAB`, cFE packet definitions, screens, EVS log protocol, EDS mode);
  OpenC3 installation documentation (Docker or Docker Compose; rootless
  Podman documented; the host install without containers documented only for
  CentOS 7 and RHEL 7); COSMOS 4, the last version that ran without
  containers, is no longer developed; Docker 29.9.3 and Podman 5.7.0 present
  on the development machine; ADR-020 (EDS, generated COSMOS definitions)

## Context

The applications publish housekeeping, events, faults and status (ADR-017,
ADR-020) and accept commands (ADR-016). An operator needs one place to see
the state of the pipeline, to command it and to see the map grow. D23 lists
the candidates: OpenC3 COSMOS with a map view, a Qt application, or a plain
web viewer.

Since version 5, COSMOS is a set of services (operator, command and
telemetry API, script runner, Redis, object storage, web front end) that run
as containers. The project requires that the installer obtains COSMOS by
itself, and that machines without Docker can also be used.

## Decision

### 1. OpenC3 COSMOS as the ground system

* The ground system is OpenC3 COSMOS, open source edition, at one pinned
  release. The release is chosen when COSMOS is first installed and is
  recorded in the installer manifest.
* It uses two plugins:
  * the cFS plugin of the bundle, for cFE and the lab applications;
  * an OrthoStream plugin with the target `ORTHOSTREAM`. It contains the
    command and telemetry definitions that the EDS build generates
    (ADR-020), and its own screens, limits and procedures:
    * a pipeline overview with one row per application: state, queue length,
      items per second, active faults;
    * a fault list built from `FaultEvent` messages;
    * the housekeeping pages of each application;
    * procedures that start, stop and record experiment runs (ADR-018).
* The plugin is built from the generated files. Hand edits are made only to
  screens, limits and procedures, never to packet definitions.

### 2. Link between COSMOS and cFS

* COSMOS sends commands to `CI_LAB` and receives telemetry from `TO_LAB`
  over UDP, as the cFS plugin expects.
* The subscription table of `TO_LAB` lists the OrthoStream housekeeping,
  status and `FaultEvent` messages and the EVS event messages.
* Bulk data (images, depth maps, point sets) never goes to COSMOS. Map tiles
  reach the operator as files (section 4), not as telemetry.
* The lab applications have no authentication. Replacing them for
  production use is D20.

### 3. Desktop application

* The operator uses a desktop application built with Electron. It is not a
  browser tab.
* Its window shows the COSMOS web interface next to OrthoStream's own map
  view.
* It connects either to a local COSMOS or to a COSMOS at a configured
  address on a ground server.
* With a local COSMOS, it starts the COSMOS containers on start-up, waits
  until they are ready and stops them on exit.
* COSMOS itself is not modified. The desktop application only displays it.
  This keeps the OrthoStream code outside the AGPLv3 terms of the open
  source edition.

### 4. Map view

The map view shows the published tiles (ADR-007) as they are updated. It
uses `TileUpdated` and `TilePublished` messages to know which tiles to
reload. Until the view exists, QGIS on the published VRT serves as the map
display.

### 5. Installation and deployment

* **Two sides.** Only the ground side uses containers. The flight side (cFS
  and the libraries) is a native, statically linked program and never needs
  a container runtime.
* **Container runtime.**
  * The installer uses Docker Engine or Podman, whichever is present.
  * On Linux without either, it installs Podman from the distribution
    packages after asking the user. Podman needs no daemon and no root
    rights, and is licensed under Apache-2.0.
  * On Windows and macOS, COSMOS runs in a small Linux virtual machine
    (WSL2 or a Podman machine), which the installer sets up.
  * Docker Desktop is not required. Its license is not free for larger
    organisations.
* **Images.**
  * The installer pulls the official OpenC3 images of the pinned release
    from Docker Hub, by digest.
  * The OrthoStream plugin is installed into these images when COSMOS starts
    for the first time.
  * If images that already contain the plugin turn out to be simpler to
    operate, they are published on Docker Hub under the project's account.
* **Offline installation.** The installer package also contains the pinned
  images as archives (`docker save`). When no network is available, they
  are loaded with `docker load` or `podman load`.
* **Remote ground server.** COSMOS can run once on a ground server. Operator
  machines then need only the desktop application.

### 6. Measurements

COSMOS is not part of the measured system. In the measurements of the
thesis it either runs on another machine, or its resource use is recorded by
the sampler (ADR-017 section 3). Runs without COSMOS remain the reference for
the timing results (ADR-002).

## Implementation Notes (2026-11-14)

* **Pinned release:** OpenC3 COSMOS 7.4.1 (`cosmos-project` tag `v7.4.1`).
* **Own instance.** Another project on the development machine runs its
  own COSMOS. OrthoStream therefore runs a separate instance next to it.
  * `scripts/cosmos_setup.sh` fetches the project into
    `ground/cosmos/runtime/orthostream-cosmos` (not tracked by git) and
    links `ground/cosmos/compose.override.yaml` into it.
  * The instance has its own compose project name, `orthostream-cosmos`,
    so its own containers and volumes.
  * It has its own ports: web interface 2910, telemetry 2244, commands
    1244.
* **Mission ports.** The mission uses the same ports:
  * `TO_LAB` sends to UDP 2244 and `CI_LAB` listens on UDP 1244
    (`mission/orthostream_defs/eds/config.xml`).
  * `TO_LAB` output is enabled with the command
    `TO_LAB_CMD_ENABLE_OUTPUT` (`DEST_IP 127.0.0.1`).
* **Subscriptions** (`mission/orthostream_defs/tables/to_lab_sub.c`):
  * cFE housekeeping and EVS long event messages;
  * `FaultEvent`;
  * the housekeeping and status messages of the applications.
  * During development also the small data path messages
    `TelemetrySample` and `ImageArrived`.
* **Plugin generator.** `tools/ground/generate_cosmos_plugin.py` builds
  the plugin under `build/cosmos_plugin` from the template
  `ground/cosmos/plugin` and the EDS output `build/cosmos`.
  `scripts/cosmos_plugin.sh` generates the plugin, builds the gem and
  installs it.
  * **Message identifiers:** computed from the topic identifiers as cFE
    does: command 0x1800 + offset, telemetry 0x0800 + offset. They were
    checked against the built `SCH_LAB` and `TO_LAB` tables and against
    received packets. The cFS plugin's fixed table of identifiers is not
    used.
  * **Byte order:** packets are big-endian, as `TO_LAB` encodes them
    through EDS. The telemetry header has 4 bytes of alignment after the
    time.
  * **Enumerations:** `STATE` lines are added from the EDS enumerations
    (ADR-020 open question).
  * **Screens:** one housekeeping screen per application and a fault event
    screen.
* **Verified (2026-11-14):** commands from COSMOS reached `CI_LAB`, and
  `INGEST_APP_HK` showed the values of a three-image session. `STATE` was
  displayed as `LISTENING`, `COMMON_HIGHEST_SEVERITY` as `NONE`.
* **Packet time:** shown in 1970 because cFE time is not set. Setting cFE
  time from the host clock is open.

## Rationale

* COSMOS covers commands, telemetry, limits, event logs and scripted
  procedures. The cFS bundle ships a plugin for it, and the EDS build
  generates its packet definitions.
* A desktop application fits an operator's work better than a browser tab.
  It can also manage the local containers and host the map view.
* Podman with an offline image bundle covers machines without Docker and
  sites without internet access.
* The flight side stays free of containers, so the processing machine of
  ADR-002 is not affected.

## Alternatives Considered

* **Yamcs.** A mature ground system that runs as a single Java process
  without containers and supports cFS. Not chosen: COSMOS is preferred, and
  the cFS bundle and the EDS tool support it directly. EDS keeps a later
  change possible.
* **COSMOS 4.** Ran natively without containers, but is no longer
  developed and is not supported by the current cFS plugin. Rejected.
* **COSMOS host install without containers.** Documented only for old
  CentOS and RHEL releases and not supported. Rejected.
* **COSMOS in a browser tab.** Possible, but less suitable for operators and
  cannot manage the local containers. Kept as a fallback.
* **A Qt application.** Rejected: it would reimplement commanding, limits,
  logging and scripting that COSMOS already provides.
* **Requiring Docker Desktop on Windows and macOS.** Rejected: licensing
  for larger organisations, and Podman covers the same need.

## Affected Areas

* [x] ground system and operator interface
* [x] Software Bus messages (`TO_LAB` subscriptions)
* [x] packaging and deployment
* [x] documentation

## Impact

* **ADR-016:** the `TO_LAB` subscriptions become part of the reference
  mission, not only a development aid.
* **ADR-017:** COSMOS is an additional consumer of housekeeping, events and
  faults. `RECORDER_APP` remains the source of the measurement records.
* **D20:** authentication of commands before production use.
* **D22:** the installer and the image bundle are part of packaging.
* **New dependencies:** Electron and Node.js for the desktop application.
  They are added when that application is started, with the user's
  approval.

## Open Questions

* ~~The COSMOS release to pin~~ — 7.4.1 (implementation notes).
* Setting cFE time from the host clock, so that COSMOS shows packet times in
  UTC.
* How the published tiles reach a remote operator (file synchronisation or
  an HTTP tile server on the mission side). Decided with the map view.
* The map library of the map view (for example OpenLayers or MapLibre).
