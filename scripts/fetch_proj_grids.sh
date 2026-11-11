#!/usr/bin/env bash
# Downloads the PROJ transformation grids that OrthoStream uses into
# build-deps/proj-grids/ and checks them against pinned SHA-256 sums.
# PROJ network access stays disabled at runtime (ADR-010); grids are
# provided as files.
#
# Usage: scripts/fetch_proj_grids.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GRID_DIR="$REPO_ROOT/build-deps/proj-grids"
BASE_URL="https://cdn.proj.org"

# file name, SHA-256, use
GRIDS=(
  "ch_swisstopo_chgeo2004_ETRS89_LHN95.tif 459e317ba9bf1f744ad920b3fe0d6ee0b1bdfc960cad7d4ea428db7bb229280f Dorfberg evaluation: ETRS89 ellipsoidal to LHN95 heights (ADR-004, ADR-008)"
  "us_nga_egm08_25.tif 4191d471eefebf24091b56dbc604353cb3b8cf8cc70e448bb9ae56a272bef17a terrain prior: Copernicus DEM heights (EGM2008) to ellipsoidal heights (ADR-006)"
)

mkdir -p "$GRID_DIR"
for entry in "${GRIDS[@]}"; do
  read -r name sha _ <<<"$entry"
  target="$GRID_DIR/$name"
  if [ -f "$target" ] && echo "$sha  $target" | sha256sum --check --status; then
    echo "ok        $name"
    continue
  fi
  curl -fsSL --retry 3 -o "$target.part" "$BASE_URL/$name"
  if ! echo "$sha  $target.part" | sha256sum --check --status; then
    rm -f "$target.part"
    echo "checksum mismatch for $name" >&2
    exit 1
  fi
  mv "$target.part" "$target"
  echo "fetched   $name"
done
