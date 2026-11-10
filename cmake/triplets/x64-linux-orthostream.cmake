# vcpkg triplet for OrthoStream on x86-64 Linux (ADR-010)
#
# Release-only static libraries with position-independent code: cFS
# applications are shared modules, so every library linked into them must be
# position-independent. Static linkage keeps the runtime free of library
# paths on the target.

set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)

# Release only: halves build time and disk use. Debug builds of OrthoStream
# still link against release dependencies.
set(VCPKG_BUILD_TYPE release)

set(VCPKG_C_FLAGS "-fPIC")
set(VCPKG_CXX_FLAGS "-fPIC")

# The CUDA toolkit comes from the system (ADR-010 section 4).
set(VCPKG_ENV_PASSTHROUGH CUDA_PATH CUDA_HOME)

# GPU architecture for CUDA code (COLMAP): 86 = Ampere, RTX 3050 Laptop of
# ADR-002. Building for one architecture keeps COLMAP's build time short.
# Another target gets its own triplet with its own architecture list.
set(VCPKG_CMAKE_CONFIGURE_OPTIONS "-DCMAKE_CUDA_ARCHITECTURES=86")
