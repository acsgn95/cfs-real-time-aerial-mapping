#!/usr/bin/env bash
# Builds and tests the OrthoStream algorithm libraries (libs/) without cFS
# (ADR-011), against the dependencies built by scripts/setup_dependencies.sh.
#
# Usage:
#   scripts/build_libs.sh                    debug build and tests
#   ORTHOSTREAM_BUILD_TYPE=release scripts/build_libs.sh
#   ORTHOSTREAM_WITH_COLMAP=0 scripts/build_libs.sh   without PatchMatch (no GPU)
#   ORTHOSTREAM_WITH_REFINE=0 scripts/build_libs.sh   without pose refinement (no GTSAM)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${ORTHOSTREAM_BUILD_TYPE:-debug}"
BUILD_DIR="$REPO_ROOT/build-libs/$BUILD_TYPE"
DEPS_PREFIX="$REPO_ROOT/build-deps/vcpkg_installed/x64-linux-orthostream"

if [ ! -d "$DEPS_PREFIX" ]; then
  echo "Dependencies missing: run scripts/setup_dependencies.sh first" >&2
  exit 1
fi

# vcpkg's toolchain file adds the wrappers some packages need (for example
# SQLite3 for PROJ). libs/ has no manifest, so the toolchain only reads the
# installed packages and installs nothing.
cmake -S "$REPO_ROOT/libs" -B "$BUILD_DIR" -G Ninja \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_TOOLCHAIN_FILE="$REPO_ROOT/vendor/vcpkg/scripts/buildsystems/vcpkg.cmake" \
  -DVCPKG_INSTALLED_DIR="$REPO_ROOT/build-deps/vcpkg_installed" \
  -DVCPKG_TARGET_TRIPLET=x64-linux-orthostream \
  -DVCPKG_MANIFEST_MODE=OFF \
  -DORTHOSTREAM_WITH_COLMAP="$( [ "${ORTHOSTREAM_WITH_COLMAP:-1}" = "1" ] && echo ON || echo OFF )" \
  -DORTHOSTREAM_WITH_REFINE="$( [ "${ORTHOSTREAM_WITH_REFINE:-1}" = "1" ] && echo ON || echo OFF )" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build "$BUILD_DIR"
ctest --test-dir "$BUILD_DIR" --output-on-failure
