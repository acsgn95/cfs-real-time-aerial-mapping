# ADR-021: Ground Station and Operator Interface

* Status: Accepted
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

* The COSMOS release to pin.
* How the published tiles reach a remote operator (file synchronisation or
  an HTTP tile server on the mission side). Decided with the map view.
* The map library of the map view (for example OpenLayers or MapLibre).
