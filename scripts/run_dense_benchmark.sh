#!/usr/bin/env bash
# Runs the ADR-005 benchmark matrix with tools/dense_benchmark and writes one
# CSV per configuration to a results directory.
#
# Usage:
#   ORTHOSTREAM_DORFBERG_DIR=/path/to/Dorsberck2 scripts/run_dense_benchmark.sh [results_dir]
#
# Environment:
#   ORTHOSTREAM_DORFBERG_DIR  images and DSM_Dorfberg_20201109_10cm.tif (required)
#   ORTHOSTREAM_BENCH_COUNT   keyframes per configuration (default 20)
#   ORTHOSTREAM_BENCH_WIDTHS, _SOURCES, _ITERATIONS, _MODES  override the levels
#
# Needs a release build (ORTHOSTREAM_BUILD_TYPE=release scripts/build_libs.sh)
# and a GPU that no other process uses.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$REPO_ROOT/build-libs/release/tools/dense_benchmark/dense_benchmark"
DATA="${ORTHOSTREAM_DORFBERG_DIR:?set ORTHOSTREAM_DORFBERG_DIR to the Dorfberg directory}"
POSES="$REPO_ROOT/data/processed/dorfberg_f01/poses_optimized.txt"
# Terrain prior of the mission (ADR-006): tools/dataset/fetch_dem.py --poses "$POSES" --name dorfberg
TERRAIN="$REPO_ROOT/data/raw/dem/cop30/dorfberg.vrt"
OUT="${1:-$REPO_ROOT/results/raw/dense_benchmark/$(date +%Y%m%d_%H%M%S)}"
COUNT="${ORTHOSTREAM_BENCH_COUNT:-20}"
WIDTHS="${ORTHOSTREAM_BENCH_WIDTHS:-1536 2048 2560}"
SOURCES="${ORTHOSTREAM_BENCH_SOURCES:-4 8}"
ITERATIONS="${ORTHOSTREAM_BENCH_ITERATIONS:-3 5}"
MODES="${ORTHOSTREAM_BENCH_MODES:-causal delayed}"

[ -x "$BIN" ] || { echo "missing $BIN: build with ORTHOSTREAM_BUILD_TYPE=release scripts/build_libs.sh" >&2; exit 1; }
[ -f "$POSES" ] || { echo "missing $POSES: run tools/dataset/extract_metashape_poses.py" >&2; exit 1; }
[ -f "$TERRAIN" ] || { echo "missing $TERRAIN: run tools/dataset/fetch_dem.py --poses $POSES --name dorfberg" >&2; exit 1; }
mkdir -p "$OUT"

{
  echo "date: $(date -Iseconds)"
  echo "commit: $(git -C "$REPO_ROOT" rev-parse HEAD)"
  echo "gpu: $(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null || echo unknown)"
  echo "count: $COUNT"
} > "$OUT/manifest.txt"

for width in $WIDTHS; do
  for k in $SOURCES; do
    for it in $ITERATIONS; do
      for mode in $MODES; do
        name="w${width}_k${k}_it${it}_${mode}"
        echo "== $name"
        "$BIN" --poses "$POSES" --images "$DATA" \
          --dsm "$DATA/DSM_Dorfberg_20201109_10cm.tif" --terrain-dem "$TERRAIN" \
          --out "$OUT/$name.csv" --width "$width" --sources "$k" \
          --iterations "$it" --mode "$mode" --count "$COUNT" \
          | tee "$OUT/$name.log"
      done
    done
  done
done
echo "results in $OUT"
