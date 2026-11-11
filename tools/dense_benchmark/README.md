# dense_benchmark

Benchmark of dense reconstruction on keyframe batches of a posed image set
(ADR-005 section 6). For each keyframe it measures:

* the PatchMatch time;
* the peak GPU memory of the process and of the device (NVML);
* the depth accuracy against a reference DSM: height differences (mean,
  median, MAE, RMSE, NMAD, 95th percentile of |e|, share within 0.5 m and
  1 m) and the share of pixels with a depth.

Keyframes and source views are chosen with the `selection` library
(ADR-006). Footprints use the reference DSM instead of SRTM. One run is one
configuration; `scripts/run_dense_benchmark.sh` runs the matrix.

## Build

Built with the libraries when `ORTHOSTREAM_WITH_COLMAP` and
`ORTHOSTREAM_BUILD_TOOLS` are on (the default):

```bash
ORTHOSTREAM_BUILD_TYPE=release scripts/build_libs.sh
```

## Run

```bash
build-libs/release/tools/dense_benchmark/dense_benchmark \
  --poses data/processed/dorfberg_f01/poses_optimized.txt \
  --images /path/to/Dorfberg/images \
  --dsm /path/to/DSM_Dorfberg_20201109_10cm.tif \
  --out results.csv --width 2048 --sources 5 --iterations 5 --count 20
```

`--help` lists all options. Important ones:

| Option | Meaning |
| --- | --- |
| `--mode causal\|delayed` | source views only from images before the keyframe, or also up to the next keyframe (ADR-005 section 2) |
| `--keyframe NAME` | benchmark given keyframes instead of `--count` keyframes spread over the flight |
| `--dry-run` | no PatchMatch: the depth map is rendered from the DSM; checks poses, frames, selection and evaluation without a GPU (median and NMAD should be a few centimetres) |
| `--save-depth DIR` | write depth maps as PFM files |

The first batch runs once before the measurements (warm-up: CUDA context and
memory allocation); its row has `warmup = 1`.

## Notes

* The poses come from `tools/dataset/extract_metashape_poses.py`; the DSM
  is in LV95 with LHN95 heights (`--dsm-crs EPSG:2056+5729`), which needs
  the geoid grid from `scripts/fetch_proj_grids.sh`.
* Height differences are vertical: on steep slopes a small horizontal error
  appears as a larger height error. Forest raises the differences, because
  the DSM and the depth maps see the crowns differently.
* COLMAP ends the process on CUDA errors (ADR-005, open questions). Each CSV
  row is written as soon as its batch finishes, so the rows before a crash
  are kept.
* Geometric consistency and the SGBM baseline are not implemented yet; the
  benchmark covers working width, K, iterations and the processing mode.
* Measurements need a GPU that no other process uses; the device baseline
  column shows memory used by other processes when sampling started.
