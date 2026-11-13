#!/usr/bin/env bash
# Configures and builds the OrthoStream cFS mission against the vendored cFS
# (vendor/cfs, v7.0.1), native Linux target, without modifying anything under
# vendor/cfs (ADR-009).
#
# Usage:
#   mission/build.sh                 debug build
#   ORTHOSTREAM_BUILD_TYPE=release mission/build.sh
#                                    release build; required for measurements
#
# The cFS lab applications (SCH_LAB, CI_LAB, TO_LAB) come from vendor/cfs/apps.
# CI_LAB accepts commands from anyone on UDP; use it for development only.
#
# The mission is built with Electronic Data Sheets (ADR-020). The EDS tool
# needs Lua 5.4 and Expat on the build host; both come from build-deps
# (vcpkg feature "eds"), never from the system. The tool is built and run on
# the host only; nothing of Lua or Expat is linked into the flight software.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$REPO_ROOT/build"
MISSION_DEFS="$REPO_ROOT/mission/orthostream_defs"
BUILD_TYPE="${ORTHOSTREAM_BUILD_TYPE:-debug}"

if [ ! -f "$REPO_ROOT/vendor/cfs/cfe/CMakeLists.txt" ]; then
  echo "vendor/cfs is missing: run 'git submodule update --init --recursive'" >&2
  exit 1
fi

APP_PATH="$REPO_ROOT/apps:$REPO_ROOT/vendor/cfs/apps"

# Third-party dependencies built by scripts/setup_dependencies.sh (ADR-010).
# cFS passes CMAKE_PREFIX_PATH on to its per-architecture builds.
DEPS_PREFIX="$REPO_ROOT/build-deps/vcpkg_installed/x64-linux-orthostream"

PREFIX_ARGS=()
if [ -d "$DEPS_PREFIX" ]; then
  PREFIX_ARGS+=(-DCMAKE_PREFIX_PATH="$DEPS_PREFIX")
fi

# The OrthoStream libraries (libs/), built and installed by
# scripts/build_libs.sh with the same build type. The cFS modules link them
# statically (ADR-011 section 8); arch_build_custom.cmake adds their prefix.
if [ ! -d "$REPO_ROOT/build-libs/$BUILD_TYPE/install/lib/cmake" ]; then
  echo "OrthoStream libraries missing: run ORTHOSTREAM_BUILD_TYPE=$BUILD_TYPE scripts/build_libs.sh" >&2
  exit 1
fi

# EDS tool dependencies. vcpkg's Lua has no pkg-config file, so the variables
# that tools/eds/edslib would otherwise get from pkg-config are set here.
if [ ! -f "$DEPS_PREFIX/lib/liblua.a" ] || [ ! -f "$DEPS_PREFIX/lib/libexpat.a" ]; then
  echo "Lua or Expat is missing: run scripts/setup_dependencies.sh (feature eds)" >&2
  exit 1
fi
EDS_ARGS=(
  -DCFE_EDS_ENABLED=ON
  -DLUALIB_FOUND=TRUE
  -DLUALIB_INCLUDE_DIRS="$DEPS_PREFIX/include"
  "-DLUALIB_LDFLAGS=$DEPS_PREFIX/lib/liblua.a;m;dl"
  -DEXPAT_LIB="$DEPS_PREFIX/lib/libexpat.a"
)

mkdir -p "$BUILD_DIR"

CFS_APP_PATH="$APP_PATH" cmake \
  -S "$REPO_ROOT/vendor/cfs/cfe" \
  -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=TRUE \
  -DCMAKE_INSTALL_PREFIX=/exe \
  -DSIMULATION=native \
  -DENABLE_UNIT_TESTS=FALSE \
  -DMISSIONCONFIG=orthostream \
  -DMISSION_DEFS="$MISSION_DEFS" \
  "${PREFIX_ARGS[@]}" \
  "${EDS_ARGS[@]}"

make -C "$BUILD_DIR" DESTDIR="$BUILD_DIR" mission-install "$@"

echo ""
echo "Build complete ($BUILD_TYPE). Run with:"
echo "  cd $BUILD_DIR/exe/cpu1 && ./core-cpu1"
