# ADR-018: Experiment Design

* Status: Accepted
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D17
* Evidence: thesis proposal (FB_YL-01) section 4.3; ADR-001 (nominal point,
  R1–R4), ADR-003 (injector parameters; altitude and tilt not varied for
  now), ADR-005 (methods and benchmark), ADR-008 (reference products),
  ADR-016 (YAML experiment configurations), ADR-017 (run directories)

## Context

The proposal evaluates accuracy and real-time performance over repeatable
scenarios and compares the results with an offline reference. ADR-003 limits
the variation to what one recorded flight allows: speed and image rate,
overlap, pose errors, timestamp offsets and link conditions. ADR-008 makes the
Metashape products and five ground control points the references.

## Decision

### 1. Approach

* Dataset: Dorfberg Flight 01.
* One factor at a time around the nominal point. A full factorial design is
  not used, because the number of combinations would be too large.
* Every configuration runs three times, because performance varies between
  runs. Results are reported as mean and standard deviation; latency also as
  50th percentile, 95th percentile and maximum.

### 2. Experiment matrix

| Experiment | Factor | Levels | Shows |
| --- | --- | --- | --- |
| E1 Nominal | — | `optimized` poses, time factor 1, all images, ideal link | baseline result |
| E2 Speed and image rate | time factor | 0.5, 1, 1.5, 2, 3 | limit of the real-time envelope (R1–R3) |
| E3 Overlap | image decimation | every image, every 2nd, every 3rd (about 85 %, 70 %, 55 %) | effect of overlap on accuracy |
| E4 Pose errors | attitude error, position noise, pose set | 0.1°, 0.5°, 1°; 0.05 m, 0.5 m, 2 m (standard GNSS); `non-optimized` | the proposal's pose-error experiment |
| E5 Timestamp offset | image timestamp offset | 10, 50, 100, 200 ms | the proposal's timestamp experiment |
| E6 Method and resolution | dense method, working resolution | PatchMatch and SGBM; 1, 2, 3 MP | COLMAP approach against AerialMapper approach |
| E7 Link | `netem` profile | ideal, Wi-Fi, cellular | effect of the link on latency |
| E8 cFS overhead | runtime | the same libraries inside cFS and outside it (`tools/`) | runtime overhead of the cFS architecture |

### 3. Accuracy metrics

References: Metashape products in EPSG:2056 with LHN95 heights (ADR-008).

* **DSM:** mean error, mean absolute error, root mean square error, NMAD
  (robust spread), completeness (share of cells with a value inside the
  flown area).
* **Point cloud:** statistics of the vertical distance from each point to the
  reference DSM.
* **Orthomosaic:** horizontal offsets found by image matching against the
  Metashape orthophoto; position errors at the ground control points.
* **Independent check:** difference between DSM height and GNSS height at
  the five ground control points.
* **Optional stratification:** forest produces large DSM differences (tree
  crowns). Results may additionally be reported by slope class and by a
  simple vegetation mask.

### 4. Performance metrics

As defined in ADR-017: stage durations, Software Bus latency, end-to-end
latency distribution, images processed per second, map update frequency,
queue lengths, skipped keyframes, CPU, GPU, RAM and GPU memory use.

### 5. Tools and reproducibility

* `tools/evaluation` (Python) computes the metrics from each run directory,
  aggregates repetitions, and produces tables and figures.
* Every experiment is defined by a YAML file (ADR-016). Every result can be
  repeated from its configuration and the manifest of its runs.

## Rationale

* One factor at a time isolates each effect and keeps the number of runs
  manageable.
* Three repetitions show the run-to-run variation of performance.
* The matrix covers every variable of the proposal that one recorded flight
  can vary, and adds the comparison of methods and the cFS overhead.
* Robust metrics and stratification keep vegetation from dominating the
  accuracy results.

## Alternatives Considered

* **Full factorial design.** Rejected: too many runs.
* **Single runs per configuration.** Rejected: performance results would not
  show their variation.
* **Altitude and tilt experiments.** Deferred (ADR-003).

## Affected Areas

* [x] evaluation and experiments
* [x] documentation

## Impact

* **Injector:** supports every factor in section 2.
* **`tools/`:** host tools for E8 and evaluation tools for sections 3 to 5.
* **Data:** the orthophoto (ADR-008) is downloaded before the orthomosaic
  evaluation.

## Open Questions

None.
