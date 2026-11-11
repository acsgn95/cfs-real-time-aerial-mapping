#!/usr/bin/env bash
# Builds the pinned third-party dependencies of OrthoStream with vcpkg in
# manifest mode (ADR-010). Nothing is installed system-wide and no sudo is
# needed; the result goes to build-deps/ in the repository.
#
# Usage:
#   scripts/setup_dependencies.sh            with COLMAP (CUDA)
#   ORTHOSTREAM_WITH_COLMAP=0 scripts/setup_dependencies.sh
#                                            without COLMAP (no GPU, e.g. CI)
#   ORTHOSTREAM_WITH_REFINE=0 scripts/setup_dependencies.sh
#                                            without GTSAM (pose refinement, D18)
#
# The first run builds every dependency from source and takes a long time
# (GDAL, OpenCV and COLMAP in particular). Later runs reuse vcpkg's binary
# cache.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VCPKG_ROOT="$REPO_ROOT/vendor/vcpkg"
TRIPLET="x64-linux-orthostream"
INSTALL_ROOT="$REPO_ROOT/build-deps/vcpkg_installed"
WITH_COLMAP="${ORTHOSTREAM_WITH_COLMAP:-1}"
WITH_REFINE="${ORTHOSTREAM_WITH_REFINE:-1}"

# System prerequisites (ADR-010 section 4), listed in
# scripts/system-packages-ubuntu.txt. Packages after the "# colmap:" marker
# are only needed with COLMAP.
PACKAGE_LIST="$REPO_ROOT/scripts/system-packages-ubuntu.txt"
missing=()
if command -v dpkg-query >/dev/null 2>&1; then
  while read -r pkg; do
    status="$(dpkg-query -W -f='${db:Status-Status}' "$pkg" 2>/dev/null || true)"
    [ "$status" = "installed" ] || missing+=("$pkg")
  done < <(awk -v colmap="$WITH_COLMAP" '
    /^# colmap:/ { if (colmap != "1") exit }
    /^[[:space:]]*(#|$)/ { next }
    { print $1 }' "$PACKAGE_LIST")
else
  # Not a Debian-based system: check the commands only.
  for tool in cmake git ninja curl zip unzip tar pkg-config; do
    command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
  done
  if [ "$WITH_COLMAP" = "1" ]; then
    command -v gfortran >/dev/null 2>&1 || missing+=("gfortran")
  fi
fi
if [ "$WITH_COLMAP" = "1" ]; then
  command -v nvcc >/dev/null 2>&1 || missing+=("nvcc (CUDA toolkit)")
fi
if [ "${#missing[@]}" -gt 0 ]; then
  echo "Missing system prerequisites: ${missing[*]}" >&2
  echo "See $PACKAGE_LIST" >&2
  exit 1
fi

if [ ! -f "$VCPKG_ROOT/bootstrap-vcpkg.sh" ]; then
  echo "vendor/vcpkg is missing: run 'git submodule update --init vendor/vcpkg'" >&2
  exit 1
fi

if [ ! -x "$VCPKG_ROOT/vcpkg" ]; then
  "$VCPKG_ROOT/bootstrap-vcpkg.sh" -disableMetrics
fi

FEATURES=()
if [ "$WITH_COLMAP" = "1" ]; then
  FEATURES+=(--x-feature=colmap)
  export CUDA_PATH="${CUDA_PATH:-$(dirname "$(dirname "$(command -v nvcc)")")}"
fi
if [ "$WITH_REFINE" = "1" ]; then
  FEATURES+=(--x-feature=refine)
fi

"$VCPKG_ROOT/vcpkg" install \
  --x-manifest-root="$REPO_ROOT" \
  --x-install-root="$INSTALL_ROOT" \
  --triplet="$TRIPLET" \
  --host-triplet="$TRIPLET" \
  "${FEATURES[@]}"

echo ""
echo "Dependencies installed under:"
echo "  $INSTALL_ROOT/$TRIPLET"
echo "mission/build.sh picks them up from there."
