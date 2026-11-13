# ADR-010: Third-Party Dependency Management

* Status: Accepted; amended by [ADR-020](adr_020_eds_events_and_faults.md) (Lua, Expat and m4 for the EDS build)
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D09 (revisited)
* Revised: 2026-11-11 (setup files: install location, optional COLMAP feature, CUDA architecture;
  OpenGL and Fortran prerequisites found in setup step 2; CI; GTSAM as an
  optional feature)
* Evidence: ADR-009 section 7; vcpkg release `2026.10.16` and its ports
  (GDAL 3.12.4, PROJ 9.9.0, LASzip 3.5.0, OpenCV 4.14.0, Eigen 5.0.1,
  GoogleTest 1.18.0, libjpeg-turbo 3.2.0, COLMAP 3.12.6 with a `cuda`
  feature), checked on 2026-11-11; PDAL is not packaged for Ubuntu 26.04;
  setup step 2 build on 2026-11-11 (below)

## Context

ADR-009 takes OpenCV, GDAL, PROJ, Eigen and the COLMAP build dependencies
from Ubuntu packages. That ties the project to one distribution and release:

* the versions differ or are missing on other systems, for example an
  embedded target;
* the versions change silently with system updates, which harms the
  reproducibility of measurements;
* a deployment depends on the package repositories of the target.

OrthoStream may go to production later. Its dependencies must be pinned and
built by the project itself, as cFS already is.

## Decision

### 1. vcpkg in manifest mode

* vcpkg is a git submodule at `vendor/vcpkg`, pinned to the release tag
  `2026.10.16`.
* A manifest `vcpkg.json` at the repository root lists the dependencies and
  their features. Its `builtin-baseline` is the commit of the vcpkg
  submodule, so the whole dependency tree, including transitive
  dependencies, is pinned by one commit.
* Updating dependencies means moving the submodule and the baseline
  together, in one reviewed change.

### 2. Dependencies

| Dependency | Use | Features |
| --- | --- | --- |
| GDAL | GeoTIFF, COG and VRT products; reading the terrain prior (GeoTIFF/VRT) | default features off; only what these drivers need |
| PROJ | coordinate and vertical datum transformations | default features off; grids are provided as files (no network access) |
| LASzip | LAZ point cloud files | — |
| libjpeg-turbo | JPEG decoding with DCT scaling | — |
| OpenCV | image processing, undistortion, SGBM baseline | only the modules needed (core, imgproc, calib3d, imgcodecs); no CUDA |
| Eigen | linear algebra | — |
| COLMAP | PatchMatch Stereo | `cuda`; no GUI, no CGAL |
| GoogleTest | library tests | — |
| Lua 5.4.8, Expat 2.8.5 | EDS tool on the build host (ADR-020); not linked into the flight software | manifest feature `eds`; versions pinned with `overrides` (Lua 5.5 is not supported by the EDS tool; Expat 2.9.0 deprecates a function the tool uses) |
| GTSAM 4.3.0 | optional pose refinement (ADR-019) | no Boost, no TBB; optional manifest feature `refine`; built without `-march=native`; overlay port without `-Werror` (`cmake/ports/`) |

PDAL is not used. GDAL and LASzip cover the needs of the real-time system,
and the evaluation tools use Python packages (ADR-011).

If the COLMAP port does not build with the system CUDA toolkit, COLMAP is
built from source as a submodule, against the vcpkg-built dependencies.

### 3. Build and linkage

* A project triplet (`cmake/triplets/`) builds release-only static libraries
  with position-independent code. cFS applications are shared modules, so
  the libraries linked into them must be position-independent. Static
  linkage keeps the runtime free of library path issues.
* The dependencies are installed into a project-local prefix,
  `build-deps/vcpkg_installed/<triplet>`. It is separate from `build/`, so a
  clean mission build does not remove them. The mission build passes the
  prefix as `CMAKE_PREFIX_PATH`, which cFS forwards to its architecture
  builds.
* COLMAP is an optional manifest feature (`colmap`), matching the build
  option `ORTHOSTREAM_WITH_COLMAP` (ADR-011). A machine without a GPU builds
  without it.
* GTSAM is an optional manifest feature (`refine`), matching the build
  option `ORTHOSTREAM_WITH_REFINE` for the pose refinement of ADR-019. It needs
  no GPU, so CI builds it.
* The project triplet builds CUDA code for one GPU architecture (86, the
  Ampere GPU of ADR-002) to keep build time short. Another target gets its
  own triplet.
* Ports that need a change are copied to `cmake/ports/` as overlay ports,
  each with one commented change, and are compared with upstream whenever
  `vendor/vcpkg` is updated. GTSAM is the first: GCC 15 reports a false
  warning in its timing utilities that its `-Werror` turns into a build
  failure.
* `scripts/setup_dependencies.sh` checks the system prerequisites,
  bootstraps vcpkg, installs the manifest and fetches the PROJ grids with
  pinned checksums (`scripts/fetch_proj_grids.sh`). Downloads happen only
  in this setup step, never during a build.
* Built packages are cached by vcpkg's binary cache, so a clean build does
  not rebuild unchanged dependencies.

### 4. System prerequisites

Only these come from the system, because they cannot or should not be
vendored:

| Prerequisite | Reason |
| --- | --- |
| C and C++ compiler, CMake, git, Ninja | toolchain |
| curl, zip, unzip, tar, pkg-config | needed by vcpkg itself |
| GNU m4 | the cFS table tool of the EDS build (ADR-020) |
| NVIDIA driver and CUDA toolkit | must match the GPU and the driver |
| OpenGL and GLU development files (`libglvnd-dev`, `libglu1-mesa-dev`), COLMAP only | the OpenGL dispatch library belongs to the driver; GLEW (needed by COLMAP's `cuda` feature) links against it and lists GLU in its pkg-config file, and vcpkg provides GLU only on Windows |
| Fortran compiler (`gfortran`), COLMAP only | LAPACK, used by Ceres and SuiteSparse, is built with the system Fortran compiler |
| Python 3 | injector and evaluation tools, each in its own virtual environment with pinned versions |

The Ubuntu package names are listed in `scripts/system-packages-ubuntu.txt`,
which `scripts/setup_dependencies.sh` checks before building. The NVIDIA
driver and the CUDA toolkit come from NVIDIA's packages and are checked
through `nvcc`.

Building OrthoStream itself needs no `sudo` beyond these prerequisites.

### 5. Licenses

Dependencies are built without optional components that carry licenses
incompatible with Apache-2.0 (for example COLMAP without CGAL). GTSAM
contains files under MPL-2.0, a file-level copyleft license: linking is
allowed, and changes to those files would have to be published. The copyright
files that vcpkg installs are the basis for third-party notices.

### 6. Deployment

How OrthoStream is packaged for a production target is decided later, for
example as an install tree with its libraries or as a container image.
Static linkage and pinned dependencies are the preconditions for both.

## Rationale

* One pinned manifest gives the same dependency versions on every machine and
  over time, as the cFS submodule does for cFS.
* vcpkg resolves transitive dependencies (GDAL needs PROJ, SQLite, libtiff,
  libjpeg, zlib and more). Doing this by hand with submodules would be slow
  and error-prone.
* vcpkg supports other target architectures through triplets, which keeps
  an embedded target possible.
* Minimal features keep build time, binary size and the license surface
  small.

## Alternatives Considered

* **Ubuntu packages (ADR-009 section 7).** Rejected: tied to one
  distribution, versions not pinned.
* **A git submodule and a build script per dependency.** Rejected: transitive
  dependencies would have to be managed by hand.
* **Conan.** Not chosen: comparable capabilities. vcpkg has ports for every
  dependency here, including COLMAP with CUDA.
* **PDAL.** Rejected: not needed next to GDAL and LASzip, pulls in GDAL and a
  large pipeline framework, and is not packaged for Ubuntu 26.04.
* **Shared libraries.** Not chosen: they need library paths at runtime on
  the target.

## Affected Areas

* [x] vendor/cFS baseline
* [x] algorithm libraries
* [x] documentation

## Impact

* **ADR-009:** section 7 is replaced by this record. Setup step 2 becomes
  "bootstrap vcpkg and build the dependencies" and no longer needs `sudo`.
  Step 3 (COLMAP) is part of step 2 when the COLMAP port works.
* **mission/build.sh:** passes the install prefix to the build; the
  dependencies are installed beforehand by `scripts/setup_dependencies.sh`.
* **CI:** `.github/workflows/ci.yml` builds the dependencies and the mission
  on GitHub-hosted runners. They have no GPU, so CI builds without COLMAP.
  vcpkg's binary cache is kept between runs.

## Setup Step 2 Result (2026-11-11)

* The manifest with the `colmap` feature builds on the machine of ADR-002
  (CUDA 13.1, GCC 15.2): 137 packages, about 25 minutes of build time.
  The install prefix is 559 MB.
* COLMAP 3.12.6 builds with CUDA 13.1 from the vcpkg port, so the fallback
  of section 2 (COLMAP as a submodule) is not needed. `libcolmap_mvs_cuda`
  contains code for `sm_86` only, as set by the triplet.
* The static libraries contain no non-PIC relocations (`R_X86_64_32S`),
  checked for GDAL and COLMAP.
* COLMAP needed three system packages not foreseen in section 4: OpenGL and
  GLU development files (for GLEW) and a Fortran compiler (for LAPACK). They
  were added to section 4 and `scripts/system-packages-ubuntu.txt`.
