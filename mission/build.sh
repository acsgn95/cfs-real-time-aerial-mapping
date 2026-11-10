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

mkdir -p "$BUILD_DIR"

CFS_APP_PATH="$APP_PATH" cmake \
  -S "$REPO_ROOT/vendor/cfs/cfe" \
  -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=TRUE \
  -DCMAKE_INSTALL_PREFIX=/exe \
  -DSIMULATION=native \
  -DCFE_EDS_ENABLED=OFF \
  -DENABLE_UNIT_TESTS=FALSE \
  -DMISSIONCONFIG=orthostream \
  -DMISSION_DEFS="$MISSION_DEFS"

make -C "$BUILD_DIR" DESTDIR="$BUILD_DIR" mission-install "$@"

echo ""
echo "Build complete ($BUILD_TYPE). Run with:"
echo "  cd $BUILD_DIR/exe/cpu1 && ./core-cpu1"
