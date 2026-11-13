# ADR-009: cFS Version and Build Structure

* Status: Accepted; amended by [ADR-010](adr_010_third_party_dependency_management.md) (third-party dependencies) and [ADR-020](adr_020_eds_events_and_faults.md) (mission built with EDS)
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D09
* Revised: 2026-11-11 (reusable components for other cFS missions)
* Evidence: tags of `nasa/cFS` on 2026-11-10 (latest release `v7.0.1`,
  later only `v7.0.2-rc1`); a cFS `v7.0.1` mission built on the target
  machine with GCC 15.2 and CMake 4.2 (NavionCore, built with `-std=c99`);
  ADR-002 (toolchain risks); ADR-005 (COLMAP integration constraints)

## Context

ADR-002 lists toolchain risks: GCC 15 defaults to C23, CMake 4 rejects old
minimum versions, and COLMAP has to be built with CUDA 13.1. ADR-005 requires
COLMAP to be linked into an algorithm library, not into cFS application code.
The repository has no build yet.

## Decision

### 1. cFS version

cFS `v7.0.1`, the latest release. Release candidates are not used.

### 2. Integration

* cFS is a git submodule at `vendor/cfs`, pinned to the tag `v7.0.1`, with
  its own submodules (cFE, OSAL, PSP, lab applications).
* Nothing under `vendor/cfs` is modified. If a change is unavoidable, it is
  kept as a patch under `vendor/patches/` with a note explaining why.

### 3. Repository layout

| Directory | Content |
| --- | --- |
| `mission/orthostream_defs/` | mission definitions: `targets.cmake`, build options, platform configuration, startup script |
| `mission/build.sh` | configures and builds the mission in one command |
| `apps/` | cFS applications (thin shells, D10) |
| `libs/` | algorithm libraries (D10) |
| `interfaces/` | message definitions (D15) |
| `tools/injector/` | Python injector (ADR-003) |
| `tools/evaluation/` | evaluation tools (D17) |
| `vendor/` | third-party sources as submodules |
| `build/` | build output, not committed |

The repository has two parts:

* **Reusable components:** `apps/`, `libs/` and `interfaces/`. Another cFS
  mission can use them without the rest of the repository.
* **Reference mission:** `mission/orthostream_defs/` and `mission/build.sh`.
  It is the mission used for the thesis, and an example of how a host mission
  integrates the components.

### 4. Build

* The build is configured from `vendor/cfs/cfe` with `MISSION_DEFS`
  pointing to `mission/orthostream_defs` and `CFS_APP_PATH` to `apps/` (and
  to `vendor/cfs/apps` when lab applications are needed).
* One target, `cpu1`: Linux with the POSIX OSAL and the `pc-linux` PSP,
  built natively (`SIMULATION=native`).
* The mission is built with Electronic Data Sheets (`CFE_EDS_ENABLED=ON`,
  ADR-020).
* A host mission integrates OrthoStream by adding `apps/` to its
  `CFS_APP_PATH`, listing the applications in its `targets.cmake`, and
  overriding the default configuration headers in its own `*_defs` directory
  where needed (ADR-016).
* CI builds the applications in a second mission (the cFS sample mission) as
  proof that they do not depend on the reference mission.
* The cFS lab applications are for development only. A host mission uses its
  own scheduler, command ingest and telemetry output.
* The default build type is `debug`. All measurements for the thesis use a
  `release` build. This is added to the measurement conditions of ADR-002.

### 5. Languages

* cFS applications: C99 (`-std=c99 -pedantic -Wall -Werror`). An explicit
  standard avoids the C23 default of GCC 15.
* Algorithm libraries: C++17, unless COLMAP requires a newer standard.
* Applications call libraries through a C interface. The exact boundary is
  decided in D10.

### 6. Naming

* Project-wide C symbols and macros use the prefix `OST_` (OrthoSTream).
  `OS_` belongs to OSAL and is not used.
* The C++ namespace is `orthostream`.
* Application names have the form `NAME_APP` and at most 16 characters,
  because OSAL limits API and file names to 20 characters including the
  module suffix. Final names are decided in D11.

### 7. Third-party dependencies

| Dependency | Source |
| --- | --- |
| OpenCV, GDAL, PROJ, Eigen and the COLMAP build dependencies | Ubuntu packages, installed by the user with `sudo apt` |
| COLMAP | built from source with CUDA, outside the repository in the user's home directory, found through its CMake package |
| Python packages of the injector and tools | a virtual environment per tool, with pinned versions |

The SGBM baseline of ADR-005 runs on the CPU (OpenCV packages without CUDA).
Building OpenCV with CUDA is not needed.

### 8. Setup order

Each step starts only after the user agrees to it.

1. Add the cFS submodule and the mission skeleton; build and run an empty
   mission. No system-wide installation.
2. Install the Ubuntu packages (command prepared, run by the user).
3. Build COLMAP.
4. Prepare the Copernicus DEM of the mission area and the EGM2008 grid (ADR-006).
5. Run the benchmark of ADR-005.

### 9. Continuous integration

GitHub-hosted runners have no GPU. CI will later build cFS and the libraries
with the dense reconstruction module disabled. Until then CI runs the
pre-commit checks only.

## Rationale

* `v7.0.1` is the latest release and is known to build on the target
  toolchain.
* A pinned submodule without local changes keeps the cFS baseline
  reproducible and easy to update.
* Mission definitions and applications inside this repository, connected
  through `MISSION_DEFS` and `CFS_APP_PATH`, follow the standard cFS mission
  layout without copying cFS itself.
* C99 for applications and C++17 for libraries keeps flight-style code simple,
  while the libraries can use COLMAP, OpenCV and Eigen.

## Alternatives Considered

* **`v7.0.2-rc1`.** Rejected: a release candidate.
* **Copying cFS into the repository.** Rejected: hides the baseline version
  and makes updates hard.
* **Building OpenCV with CUDA for a GPU SGBM baseline.** Rejected: long build
  for a baseline whose purpose is comparison.
* **C++ in cFS applications.** Rejected: cFS applications are C; keeping C++
  inside libraries keeps the boundary clear.

## Affected Areas

* [x] cFS application structure
* [x] vendor/cFS baseline
* [x] algorithm libraries
* [x] observability and performance measurement
* [x] documentation

## Impact

* **ADR-002:** release build added to the measurement conditions.
* **ADR-005:** the SGBM baseline runs on the CPU.
* **D10:** defines the C interface between applications and libraries.
* **D11:** application names follow section 6.
* **README and CONTRIBUTING:** build instructions are added after step 1.

## Open Questions

None.
