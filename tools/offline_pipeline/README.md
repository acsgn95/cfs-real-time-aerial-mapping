# offline_pipeline

The OrthoStream mapping pipeline on a recorded flight, without cFS
(ADR-011 section 7). Keyframes are processed in capture order, as the
mission would process them, but one after the other on one thread:

1. keyframe and source view selection on the terrain prior (ADR-006);
2. working images and PatchMatch depth (ADR-005);
3. DSM fusion, orthomosaic and, optionally, the LAZ point cloud (ADR-007);
4. tile lifecycle: finalising, writing and evicting tiles (ADR-007).

The times of the stages go to `timings.csv`. With `--reference-dsm`, the
DSM tiles are compared with the reference DSM at the end
(`evaluation.txt`).

## Run

```bash
ORTHOSTREAM_BUILD_TYPE=release scripts/build_libs.sh
build-libs/release/tools/offline_pipeline/offline_pipeline \
  --poses data/processed/dorfberg_f01/poses_optimized.txt \
  --images /path/to/Dorfberg/images \
  --terrain-dem data/raw/dem/cop30/dorfberg.vrt \
  --reference-dsm /path/to/DSM_Dorfberg_20201109_10cm.tif \
  --out results/raw/offline/run1 --first 40 --count 30
```

The defaults are the Dorfberg evaluation: products in LV95 with LHN95
heights (`--product-crs EPSG:2056 --vertical-crs EPSG:5729`), which needs the
PROJ grids of the environment setup. `--help` lists all options.

## Output

| Path | Content |
| --- | --- |
| `dsm/`, `dsm.vrt` | DSM tiles (height and variance) and their mosaic |
| `ortho/`, `ortho.vrt` | orthomosaic tiles (RGBA) and their mosaic |
| `points/` | LAZ files per keyframe and tile (`--points`) |
| `state/` | state of evicted tiles (cache, not a product) |
| `timings.csv` | stage times per keyframe |
| `evaluation.txt` | DSM against the reference DSM |

The output directory must be empty, so that tiles of earlier runs do not
enter the evaluation.

## Notes

* The times are those of one thread doing all stages in turn. In the
  mission the stages run in different applications and overlap; the sum of
  the stage times is therefore an upper bound of the latency, and the
  slowest stage bounds the throughput.
* Source views are causal: only images captured before the keyframe.
* Geometric consistency is not implemented yet (ADR-005); depth pixels are
  filtered photometrically, so the DSM contains outliers that later
  observations reject only partly.
