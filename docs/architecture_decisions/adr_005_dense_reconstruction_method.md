# ADR-005: Dense Reconstruction Method

* Status: Accepted; parameter values set by the benchmark of section 6a
  (2026-11-12 and 2026-11-13); amended by
  [ADR-019](adr_019_pose_refinement.md) (optional pose refinement before dense
  reconstruction); amended 2026-11-13: SGBM is the default method and
  PatchMatch the alternative (sections 1, 4 and 6b)
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D05
* Revised: 2026-11-11 (dense working resolution given as image width; 2048 × 1366 is 2.8 MP);
  2026-11-13 (default method SGBM after the comparison of section 6b)
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

ADR-001 leaves about 2.9 s per keyframe at the nominal point, with a
latency target of 5 s (causal) or 10 s (delayed processing) and a dense working resolution of about 2048 px width (2.8 MP). ADR-002 limits
GPU memory to 5 GB on an RTX 3050 Laptop with CUDA 13.1.

Poses are known (ADR-003), so neither structure from motion nor bundle
adjustment is needed. Only the dense stage of COLMAP is relevant. Optional
pose refinement before the dense stage is decided in ADR-019.

## Decision

### 1. Default method: two-view SGBM

Changed on 2026-11-13. Until then PatchMatch (section 4) was the primary
method and SGBM the baseline. The comparison of section 6b reversed the
roles.

* Depth is computed with OpenCV's semi-global matching (StereoSGBM) on the
  CPU, from the reference keyframe and one source view, as in AerialMapper.
* The source view is the one of the batch whose baseline is closest to 0.2
  of the mean depth (about the spacing of consecutive keyframes at the
  nominal point). The pair is rectified with the known poses. Epipolar
  lines along the image columns are handled by transposing the images.
* Pixels whose match falls outside the rectified source image are invalid
  (source mask). Without the mask SGBM matches the black border with itself
  and produces far depths (section 6b).
* Output per keyframe: a depth map and a confidence of 1 for matched pixels.
  The normals point towards the camera, since SGBM gives no surface
  orientation.
* The input, the working resolution, the working frame `L_k`, the terrain
  gate (section 5a) and the processing mode (section 2) are the same as for
  PatchMatch.

### 2. Causal and delayed processing

Processing is delayed by default (changed from causal on 2026-11-13, after
the benchmark of section 6a). In causal mode, when keyframe `k` is processed,
only images received before it exist.

* **Source views** are chosen among images already received, including
  images of neighbouring flight lines.
* **Geometric consistency** for keyframe `k` uses the depth maps already
  computed for its source keyframes. In a stream, these depth maps are
  available from earlier batches, so they do not need to be recomputed as in
  batch COLMAP.
* **Delayed mode (default):** keyframe `k` waits for one more keyframe, so
  that images after it can also serve as sources. This adds one keyframe
  interval of latency (about 2.9 s at the nominal point); ADR-001 R2 allows
  10 s for it. In causal mode the part of the keyframe ahead in the
  direction of flight is seen by no consecutive image, only by one or two
  images of the neighbouring line; there the photometric filter either
  keeps noise or, when stricter, removes most of the half. Causal mode stays
  available for uses that need the lower latency (R2: 5 s).

### 3. Depth fusion is part of map product generation

COLMAP's scene-wide fusion (StereoFusion) works on a complete reconstruction
and is not used in the real-time path. Depth maps are fused into the map tiles
incrementally (D07), following the consistency principles of the COLMAP
approach.

### 4. Alternative method: COLMAP PatchMatch Stereo as a library

* PatchMatch stays implemented behind the same depth-estimator interface and
  is selected by configuration. It is the multi-view method of the proposal
  and is kept for comparison, for scenes where two views are not enough
  (for example steep terrain or occlusions) and for work on geometric
  consistency.
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
* Both methods implement the same depth-estimator interface: a keyframe batch
  as input, and a depth map with confidence in the reference camera as output.

### 5. Parameters

The first values were starting points. The benchmark results of section 6a
replaced them on 2026-11-12 (decision of the project owner); geometric
consistency and the processing mode are still open.

| Parameter | Value | First value |
| --- | --- | --- |
| Method | SGBM (section 1) | PatchMatch |
| SGBM: source view | baseline closest to 0.2 of the mean depth | — |
| SGBM: block size, P1, P2 | 5 px, 8·5², 32·5² (OpenCV 3-way mode) | — |
| SGBM: uniqueness, left-right check, speckle filter | 10 %, 1 px, 100 px window and range 2 | — |
| Dense working resolution | 2048 px width (2048 × 1366, 2.8 MP, for Dorfberg) | same |
| Source views K | 5 | same |
| PatchMatch iterations | 3 | 5 (COLMAP default) |
| NCC window radius | 3 (7 × 7 pixels) | 5 (COLMAP default) |
| NCC window step | 1 | 1 |
| View selection samples | 7 | 15 (COLMAP default) |
| Photometric filter: minimum NCC | 0.3 | 0.1 (COLMAP default) |
| Photometric filter: minimum consistent sources | 2 | 2 |
| Terrain gate | 30 m (section 5a) | none |
| Geometric consistency | open (see open questions) | on |
| Processing mode | delayed (section 2) | causal |

### 5a. Terrain gate

* After PatchMatch, depth pixels whose point lies more than 30 m
  (vertically, in the batch frame `L_k`) from the terrain prior of ADR-006
  are set to invalid. The tolerance is a table value.
* The terrain prior is passed to the dense stage as a height grid over the
  keyframe's footprint in `L_k` (5 m spacing by default, filled by the
  application from the terrain library); the gate interpolates it per
  pixel. It costs about 20 ms per keyframe.
* Rationale: the photometric filter keeps gross outliers that throw points
  tens of metres away. On 40 Dorfberg keyframes the gate lowers the DSM RMSE
  from 37.2 m to 3.4 m and loses 7 % of the area (section 6a). 30 m keeps
  trees, buildings and the error of the terrain prior (NMAD 1.6 m, 95th
  percentile 6 m at Dorfberg) inside the gate.

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
| Method | PatchMatch, SGBM (default since 2026-11-13) |

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
method, for example by making SGBM the primary method. (SGBM became the
default on 2026-11-13 for accuracy, not because PatchMatch missed the
budget; section 6b.)

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

End-to-end runs with `tools/offline_pipeline` on 40 consecutive keyframes
(about 0.4 km², candidate settings, products in LV95 with LHN95 heights),
DSM against the reference DSM:

| Run | Mean | Median | NMAD | RMSE | 95 % of abs. error | Within 1 m | Area |
| --- | --- | --- | --- | --- | --- | --- | --- |
| photometric filter only | −7.53 m | −0.12 m | 0.33 m | 37.2 m | 91.4 m | 79.5 % | 0.42 km² |
| plus terrain gate of 30 m | −0.11 m | −0.10 m | 0.29 m | 3.38 m | 2.97 m | 91.7 % | 0.39 km² |

* The terrain gate drops depth pixels whose point lies more than 30 m
  (vertically) from the terrain prior of ADR-006. It costs about 20 ms per
  keyframe, removes the gross outliers that the photometric filter keeps,
  and loses 7 % of the area. It is a candidate for the dense stage, since
  the terrain prior is available there.
* The comparison images (`dense_benchmark --compare`) show where the
  remaining errors come from: in some keyframes a large part of the depth
  map is noise that passes the photometric filter (`filter_min_ncc` 0.1,
  two consistent sources) while lying within the gate. A stricter filter
  or geometric consistency is needed; this is the next benchmark factor.
* Keyframes at the start of a flight line have at most one causal source
  view and produce no depth; the delayed mode addresses this.

Photometric filter and processing mode, same 40 keyframes, terrain gate on
(results in `test_outputs/offline_pipeline/filter_sweep` of the
development machine):

| Mode | Min. NCC | Min. consistent | Area | NMAD | RMSE | 95 % of abs. error | Within 1 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| causal | 0.1 | 2 | 0.386 km² | 0.285 m | 3.38 m | 2.96 m | 91.7 % |
| causal | 0.5 | 2 | 0.358 km² | 0.266 m | 3.17 m | 2.03 m | 93.2 % |
| causal | 0.3 | 3 | 0.296 km² | 0.288 m | 1.36 m | 0.76 m | 96.7 % |
| delayed | 0.3 | 2 | 0.416 km² | 0.164 m | 1.35 m | 0.57 m | 97.2 % |
| delayed | 0.3 | 3 | 0.367 km² | 0.196 m | 0.94 m | 0.58 m | 97.7 % |

Delayed mode with a minimum NCC of 0.3 and two consistent sources covers
the most area and nearly halves the NMAD; the dense time does not change
(2.3–2.4 s). It became the default on 2026-11-13.

### 6b. SGBM against PatchMatch (2026-11-13)

`tools/offline_pipeline`, Dorfberg, delayed mode, terrain gate 30 m,
2048 px, products in LV95 with LHN95 heights; the same settings for both
methods. SGBM runs on the CPU, PatchMatch on the GPU.

40 keyframes (60 to 99), `optimized` poses, against the reference DSM:

| Method | Area | Median | NMAD | RMSE | 95 % of abs. error | Within 1 m | Dense time mean / max |
| --- | --- | --- | --- | --- | --- | --- | --- |
| PatchMatch | 0.416 km² | −0.080 m | 0.164 m | 1.35 m | 0.57 m | 97.2 % | 2.42 / 3.88 s |
| SGBM, no source mask | 0.401 km² | −0.008 m | 0.070 m | 1.24 m | 0.28 m | 98.1 % | 0.94 / 1.17 s |
| SGBM, source mask | 0.400 km² | −0.007 m | 0.069 m | 0.363 m | 0.25 m | 98.9 % | 0.96 / 1.12 s |

Whole flight (339 keyframes, 1.8 km²):

| Method | Median | NMAD | RMSE | 95 % of abs. error | Within 1 m | Dense time mean / 95 % / max |
| --- | --- | --- | --- | --- | --- | --- |
| PatchMatch | −0.085 m | 0.146 m | 2.38 m | 1.38 m | 94.1 % | 2.55 / 2.77 / 2.91 s |
| SGBM | −0.004 m | 0.062 m | 1.29 m | 0.62 m | 96.2 % | 0.96 / 1.42 / 2.51 s |

Independent check at the five ground control points (GNSS RTK, LHN95; DSM
height minus GNSS height): SGBM mean +0.012 m, RMS 0.098 m; PatchMatch mean
−0.118 m, RMS 0.275 m; the Metashape reference DSM mean +0.042 m, RMS
0.077 m. The reference DSM comes from Metashape's dense matching, which may
favour SGBM; the ground control points do not depend on it and give the
same order.

With refined poses (ADR-019 section 8) both methods keep the same order
(NMAD 0.077 m and 0.168 m).

* SGBM is more accurate than PatchMatch on every measure and on both
  references, and 2.5 times faster, without the GPU.
* PatchMatch has a constant bias of about −8 cm that SGBM does not have
  (open question).
* SGBM leaves thin and complex structures (barriers, vegetation) empty
  instead of filling them with wrong heights.
* Dorfberg is mostly open meadow and gentle slopes. Scenes with steep
  terrain, buildings or occlusions may favour multi-view matching; they are
  not covered by this data.

Decision of the project owner (2026-11-13): SGBM is the default method,
PatchMatch the alternative.

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

* SGBM (since 2026-11-13): more accurate and faster than PatchMatch on the
  Dorfberg data, against the reference DSM and the ground control points
  (section 6b). It runs on the CPU, so the default configuration needs no
  GPU and does not depend on COLMAP.
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
* **SGBM only.** First rejected as the primary method because it does not
  meet the proposal's multi-view reference. Since 2026-11-13 SGBM is the
  default (section 6b), but PatchMatch is kept as the alternative rather
  than removed: it is the proposal's reference method, and other scenes
  may need multi-view matching.
* **PatchMatch as the default (decision of 2026-11-10).** Replaced on
  2026-11-13: less accurate and slower than SGBM on Dorfberg (section 6b).
* **SGBM on the GPU (OpenCV CUDA StereoSGM).** Not now: the CPU version
  already takes about 1 s per keyframe, a third of the budget. It may be
  considered if the CPU is needed by pose refinement (ADR-019).
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
* **SGBM as the default (2026-11-13):**
  * The default dense stage runs on the CPU. The GPU budget of ADR-002 and
    the single GPU job of D11 and D14 apply only when PatchMatch is
    selected.
  * The `exit()` behaviour of COLMAP (open question below) concerns only
    PatchMatch runs.
  * SGBM and pose refinement (ADR-019) both use the CPU. With
    `refine_pose = true` their combined time per keyframe is measured; the
    CPU threads of each are a table value.
  * Geometric consistency is a PatchMatch question; SGBM uses its own
    left-right check (`disp12_max_diff`).

## Open Questions

* Final parameter values: from the benchmark of section 6; section 6a gives
  the candidate.
* Causal or delayed mode as the default.
  * Resolved 2026-11-13: delayed (section 2), with R2 at 10 s.
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
* The −8 cm bias of PatchMatch (section 6b): COLMAP settings or the window
  radius of 3; to be explained if PatchMatch is used for results.
* Geometric consistency needs depth and normal maps of every source view
  and a first photometric pass of the reference view. Source views that are
  not keyframes have no depth maps (ADR-006). Either geometric consistency
  uses only keyframe sources, or it is dropped; to be decided with the
  benchmark.
