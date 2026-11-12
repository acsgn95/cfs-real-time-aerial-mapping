# ADR-005: Dense Reconstruction Method

* Status: Accepted; parameter values provisional until the benchmark of
  section 6; amended by
  [ADR-019](adr_019_pose_refinement.md) (optional pose refinement before dense
  reconstruction)
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D05
* Revised: 2026-11-11 (dense working resolution given as image width; 2048 × 1366 is 2.8 MP)
* Evidence: thesis proposal (FB_YL-01) sections 2 and 4.2; ADR-001 (keyframe
  interval, latency target, working resolution); ADR-002 (GPU memory budget,
  CUDA 13.1); ADR-004 (working frames `L_k`); AerialMapper source analysis
  (`real-time-mapper-plan/01-architecture/aerial-mapper-analysis.md`:
  two-view rectification with OpenCV SGBM, no multi-view fusion)

## Context

The proposal takes the COLMAP multi-view stereo approach as the reference for
dense reconstruction: source view selection, photometric and geometric
consistency, and depth map fusion (Schönberger et al., 2016). AerialMapper,
the reference for incremental map products, computes depth with two-view
OpenCV SGBM on rectified image pairs and does not fuse views.

ADR-001 leaves about 2.9 s per keyframe at the nominal point, with a 5 s
latency target and a dense working resolution of about 2048 px width (2.8 MP). ADR-002 limits
GPU memory to 5 GB on an RTX 3050 Laptop with CUDA 13.1.

Poses are known (ADR-003), so neither structure from motion nor bundle
adjustment is needed. Only the dense stage of COLMAP is relevant. Optional
pose refinement before the dense stage is decided in ADR-019.

## Decision

### 1. Primary method: COLMAP PatchMatch Stereo as a library

* Depth and normal maps are computed with COLMAP's CUDA PatchMatch Stereo
  module, used as a library with known poses. Feature extraction, matching,
  structure from motion and bundle adjustment are not used.
* Input per keyframe batch: the reference keyframe and K source views
  selected by D06. Images are undistorted and reduced to the dense working
  resolution. Intrinsics are scaled accordingly, and poses are expressed in
  the batch working frame `L_k` (ADR-004).
* Output per keyframe: a depth map, a normal map and a confidence measure in
  the reference camera, at the working resolution.
* The JPEG input can be decoded directly at a reduced scale (ADR-001,
  section 1d) before final resampling to the working resolution.

### 2. Causal processing

Processing is causal by default. When keyframe `k` is processed, only images
received before it exist.

* **Source views** are chosen among images already received, including
  images of neighbouring flight lines.
* **Geometric consistency** for keyframe `k` uses the depth maps already
  computed for its source keyframes. In a stream, these depth maps are
  available from earlier batches, so they do not need to be recomputed as in
  batch COLMAP.
* **Delayed mode (option):** keyframe `k` waits for one more keyframe, so
  that images after it can also serve as sources. This adds one keyframe
  interval of latency. The benchmark compares it with causal mode.

### 3. Depth fusion is part of map product generation

COLMAP's scene-wide fusion (StereoFusion) works on a complete reconstruction
and is not used in the real-time path. Depth maps are fused into the map tiles
incrementally (D07), following the consistency principles of the COLMAP
approach.

### 4. Baseline method: two-view SGBM

* A two-view semi-global matching baseline (OpenCV SGBM, on CPU or CUDA) is
  implemented on rectified consecutive keyframes, as in AerialMapper.
* Both methods implement the same depth-estimator interface: a keyframe batch
  as input, and a depth map with confidence in the reference camera as output.
  They can be swapped by configuration.
* The baseline is used for comparison in the evaluation and as a fallback.

### 5. Provisional parameters

These values are starting points. Section 6 confirms or replaces them.

| Parameter | Provisional value |
| --- | --- |
| Dense working resolution | 2048 px width (2048 × 1366, 2.8 MP, for Dorfberg) |
| Source views K | 5 |
| PatchMatch iterations | 5 (COLMAP default) |
| Geometric consistency | on |
| Processing mode | causal |

### 6. Benchmark plan

The benchmark runs after the cFS development environment is set up (D09),
under the measurement conditions of ADR-002, on keyframe batches of the
Dorfberg flight with `optimized` poses.

| Factor | Levels |
| --- | --- |
| Working resolution (width) | 1536, 2048, 2560 px (1.6, 2.8, 4.4 MP for Dorfberg) |
| Source views K | 4, 8 |
| Geometric consistency | off, on |
| PatchMatch iterations | 3, 5 |
| NCC window radius | 3, 5 (added after the first results, section 6a) |
| Processing mode | causal, delayed |
| Method | PatchMatch, SGBM baseline |

Measured for each run: processing time per keyframe, peak GPU memory, and
depth completeness and accuracy against the reference DSM.
The tool is `tools/dense_benchmark`; `scripts/run_dense_benchmark.sh` runs
the matrix.

**Acceptance:** at the nominal point, the chosen configuration processes a
keyframe in less than the keyframe interval (2.9 s) and stays within 5 GB of
GPU memory, so that ADR-001 R1 and R2 can hold.

**Fallback order** if no configuration meets the acceptance criterion:

1. lower working resolution;
2. fewer source views;
3. fewer iterations;
4. geometric consistency off.

If none of these meets the acceptance criterion, a new record revisits the
method, for example by making SGBM the primary method.

### 6a. First results (2026-11-12)

Two speed sweeps with `tools/dense_benchmark` on Dorfberg, `optimized`
poses, causal mode, Copernicus terrain prior, release build, RTX 3050 Laptop
on mains power. The second sweep uses the same six keyframes (00017, 00024,
00208, 00211, 00386, 00585) for every configuration; one warm-up batch is
not counted. Accuracy is the height difference to the reference DSM;
geometric consistency is not implemented yet. Results are in
`results/raw/dense_benchmark/sweep*`.

| Width | K | Iter. | Window radius | Window step | Samples | Time mean / max | NMAD (median of keyframes) | Valid pixels |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2048 | 5 | 5 | 5 | 1 | 15 | 16.2 / 16.7 s | 0.60 m | 55 % |
| 2048 | 5 | 3 | 5 | 1 | 7 | 5.5 / 5.8 s | 0.58 m | 56 % |
| 2048 | 5 | 3 | 5 | 2 | 7 | 2.4 / 2.5 s | 33.7 m | 49 % |
| 2048 | 5 | 3 | 3 | 1 | 7 | 2.4 / 2.5 s | 0.56 m | 55 % |
| 2048 | 4 | 3 | 3 | 1 | 7 | 2.2 / 2.2 s | 0.40 m | 37 % |
| 1536 | 5 | 3 | 3 | 1 | 7 | 1.5 / 1.6 s | 0.67 m | 61 % |

The first row is from the first sweep (other keyframes); the others are from
the second sweep.

* With COLMAP's default settings a keyframe takes about 16 s, five times
  the 2.9 s budget. The GPU runs at full load (100 %, 1950 MHz, about
  60 W, no throttling); peak GPU memory is 0.6 GB.
* The fallback order of section 6 alone does not reach the budget. The
  window radius does: radius 3 instead of 5 halves the time with no loss of
  accuracy on this data.
* A window step of 2 (every second row and column of the window) reaches
  the budget but ruins the depth of most keyframes (NMAD of tens of
  metres); it is not used.
* Fewer source views give more accurate but less complete depth maps.
* Large outliers remain after photometric filtering: the RMSE is tens of
  metres while the NMAD is below a metre. Geometric consistency or stricter
  filtering is needed (open question below).
* The last keyframe of the flight (00754), with only one consecutive source,
  fails in every configuration.

Candidate for the default: 2048 px, K = 5, 3 iterations, window radius 3,
window step 1, 7 samples: 2.4 s per keyframe, about 17 % below the budget.
To be confirmed with the full matrix and the end-to-end run.

### 7. Integration constraints

* COLMAP is built from source with CUDA 13.1, outside this repository as a
  third-party dependency. If it does not build with CUDA 13.1, a supported
  CUDA 12 toolkit is installed alongside.
* Only the modules needed for PatchMatch Stereo are used. COLMAP is under the
  BSD license, which is compatible with Apache-2.0. Optional components with
  other licenses are not used.
* COLMAP is linked into the algorithm library (D10), not into cFS application
  code. It is not started as a command-line process in the real-time path.
  The command-line tools are acceptable for the offline reference workflow
  (D08).

## Rationale

* PatchMatch Stereo matches the proposal's reference method and gives
  multi-view depth with photometric and geometric consistency.
* Using only the dense module fits known poses. It also follows the idea of
  using COLMAP's GPU modules rather than the full pipeline.
* Causal processing follows from real-time operation. Reusing depth maps of
  earlier keyframes makes geometric consistency cheap in a stream.
* A shared depth-estimator interface makes the SGBM comparison and the
  fallback cheap. It also gives the thesis a measured comparison between the
  AerialMapper approach and the COLMAP approach.
* Fixing the benchmark plan now keeps the measurements systematic.

## Alternatives Considered

* **Full COLMAP pipeline (SfM and MVS).** Rejected: poses are known, and pose
  estimation is out of scope.
* **SGBM only.** Rejected as the primary method: it does not meet the
  proposal's multi-view reference. It is kept as the baseline.
* **A custom GPU plane-sweep MVS.** Rejected: too much implementation work for
  the thesis timeline.
* **COLMAP command-line tools per keyframe.** Rejected for the real-time path:
  process start-up and workspace file I/O per keyframe add latency. They are
  kept for the offline reference (D08).
* **COLMAP StereoFusion for incremental fusion.** Rejected: it is designed
  for a complete scene. Incremental fusion into tiles is decided in D07.

## Affected Areas

* [x] algorithm libraries
* [x] cFS application structure
* [x] time and execution model
* [x] observability and performance measurement
* [x] evaluation and experiments
* [x] vendor/cFS baseline
* [x] documentation

## Impact

* **D06:** source view selection works on images already received (causal),
  with K = 5 as the starting value.
* **D07:** incremental fusion of depth maps into tiles uses depth, normal and
  confidence maps.
* **D08:** the offline reference uses the COLMAP command-line tools with known
  poses.
* **D09:** the build integrates COLMAP and CUDA 13.1. The benchmark of
  section 6 is scheduled right after the environment is set up.
* **D10:** the depth-estimator interface lives in the algorithm library, with
  two implementations.
* **D11 and D14:** the dense stage keeps one GPU job in flight. The delayed
  mode needs a queue of one keyframe.
* **D16:** per-keyframe time and peak GPU memory are recorded.

## Open Questions

* Final parameter values: from the benchmark of section 6; section 6a gives
  the candidate.
* Causal or delayed mode as the default: from the benchmark.
* Does COLMAP build with CUDA 13.1?
  * Resolved 2026-11-11: yes, from the vcpkg port (ADR-010), so section 7's
    out-of-repository build is not needed.
* COLMAP ends the process with `exit()` on any CUDA error
  (`CudaSafeCall`), for example when GPU memory runs out. Inside the cFS
  mission this would stop every application. Options: keep a safety margin
  below the GPU budget and check free memory before each job; patch
  COLMAP through an overlay port to raise an exception instead; or run the
  GPU work in a separate process. To be decided before `DENSE_APP` is
  written (found 2026-11-11).
* Geometric consistency needs depth and normal maps of every source view
  and a first photometric pass of the reference view. Source views that are
  not keyframes have no depth maps (ADR-006). Either geometric consistency
  uses only keyframe sources, or it is dropped; to be decided with the
  benchmark.
