# ADR-019: Optional Pose Refinement

* Status: Accepted
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D18
* Evidence: ADR-001 section 1b (recorded attitude about 1.5° from the
  adjusted attitude, about 195 px in the image); ADR-003 section 4 (pose
  sets); ADR-010 (GTSAM 4.3.0 as the optional vcpkg feature `refine`, built
  on the development machine); ADR-012, ADR-015 and ADR-016 (applications,
  execution model, tables loaded at startup); ADR-018 E4 (pose-error
  experiment)

## Context

Dense reconstruction with known poses (ADR-005) needs poses that are
accurate to a few pixels. Survey aircraft do not always deliver them:

* With PPK or RTK the positions are accurate to a few centimetres, but the
  attitude of the autopilot is not. At the nominal point the recorded
  attitude is about 1.5° from the adjusted attitude, about 195 px in the
  image (ADR-001).
* With a standard GNSS receiver, such as a u-blox module, the positions are
  also off by metres.

The thesis proposal takes the poses as given and does not estimate them from
the imagery. This record extends it: pose estimation from the imagery is
added as an option, and the core pipeline stays as it is.

The operator knows the pose quality of the aircraft before the flight. The
design is therefore a configuration choice made before the flight, not an
automatic decision during it.

## Decision

### 1. Two configuration values

| Value | Default | Meaning |
| --- | --- | --- |
| `refine_pose` | `false` | `false`: the poses are trusted, and keyframe batches go directly to dense reconstruction. `true`: the poses of every batch are refined from the images first. |
| `precise_position` | `false` | Only used when `refine_pose` is `true`. `true`: positions come from PPK or RTK and are kept fixed; only the attitude is estimated. `false`: positions and attitudes are both estimated, and positions are tied to the telemetry by its reported accuracy. |

* Both values are set in the experiment or mission configuration before the
  flight and loaded with the tables at startup (ADR-016). They do not change
  during a run.
* `refine_pose` belongs to the table of `SELECT_APP`, `precise_position` to
  the table of `REFINE_APP`. The YAML experiment configuration holds both in
  one `pose_refinement` section, and the table tool writes them into the two
  tables.

### 2. Data path

```text
refine_pose = false:  SELECT_APP --KeyframeBatch--------------------------> DENSE_APP
refine_pose = true:   SELECT_APP --KeyframeBatch--> REFINE_APP --KeyframeBatch--> DENSE_APP
```

* `SELECT_APP` publishes the batch under one of two message identifiers:
  "batch selected" when `refine_pose` is `true`, "batch ready" otherwise.
* `REFINE_APP` subscribes to "batch selected". It replaces the poses in the
  batch and publishes it as "batch ready", with the same message structure.
* `DENSE_APP` subscribes only to "batch ready" and does not know whether
  refinement ran.
* Every batch states its pose source: `telemetry`, `refined` or
  `refine_failed` (section 5).
* All stages after `DENSE_APP` take the pose of a keyframe from its batch,
  through the depth set, and never from `ImagePose`. Refined poses therefore
  reach georeferencing and the orthomosaic without further changes.
* Keyframe and source view selection always uses the telemetry poses. An
  attitude error of a few degrees is small compared with the footprint
  overlap tolerances of ADR-006.

### 3. Method

Refinement is a sliding-window bundle adjustment with GNSS priors, a form of
GNSS-aided visual SLAM without loop closure.

1. **Features.** Image features are extracted on the CPU from the
   working-resolution images that `SELECT_APP` already decoded (ADR-013).
   Features are computed once per image and kept while the image is in the
   window. Default: OpenCV SIFT. ORB is the faster alternative. The
   benchmark decides.
2. **Matching.** The keyframe is matched with its source views, and
   consecutive images are matched with each other. Outliers are removed with
   the ratio test and a RANSAC essential-matrix check. The check does not use
   the telemetry attitude, so it also works when the attitude is wrong.
3. **Optimisation.** GTSAM 4.3.0, Levenberg–Marquardt with a Huber loss.
   * Variables: the camera poses of the images in the window, in a local
     working frame (ADR-004).
   * Factors: reprojection of the tie points, with the intrinsics fixed
     (calibrated camera, ADR-008), and priors from the telemetry.
   * `precise_position = true`: positions are held fixed, and only the
     attitudes are variables. The attitude prior uses the reported
     attitude accuracy.
   * `precise_position = false`: positions and attitudes are variables, with
     priors weighted by the reported accuracy fields (ADR-016).
4. **Window.** The window holds the last N keyframes with their source views
   (default N = 8, a table value). An image that leaves the window keeps its
   last refined pose. When it appears in a later batch, that pose is used as
   a tight prior. Memory stays bounded (ADR-001 R4).

GPU: features and optimisation run on the CPU. Refinement therefore does
not compete with PatchMatch for GPU memory (ADR-002, ADR-005).

### 4. Accuracy that can be expected

* With `precise_position = true`, the absolute accuracy follows from the PPK
  or RTK positions. Refinement removes the attitude error.
* With `precise_position = false`, the poses become consistent with each
  other, which is what multi-view stereo needs. The absolute position of the
  map stays limited by the GNSS accuracy.
* Errors that refinement cannot remove: a wrong camera calibration, and, with
  `precise_position = true`, image timestamp offsets, which appear as
  position errors (ADR-018 E5).

### 5. Failure handling

* Checks per batch: a minimum number of inlier matches per image, a maximum
  reprojection RMS, and a correction no larger than a configured multiple
  of the reported accuracy.
* If a check fails, the batch is passed on with its telemetry poses and
  marked `refine_failed`. A batch is never dropped because refinement
  failed. Every failure is counted and reported with its reason.
* In a build without GTSAM (`ORTHOSTREAM_WITH_REFINE=OFF`), `REFINE_APP`
  reports an error event at startup and passes every batch on unchanged,
  marked `refine_failed`.

### 6. Software structure

* New library `refine` (C++17, GTSAM and OpenCV), C interface
  `OST_Refine_*` (ADR-011). Its table structure and validation follow
  ADR-016.
* New application `REFINE_APP`, one child task for the refinement. Priority
  level 3, next to `GEOREF_APP` and `MAPPROD_APP` (ADR-015).
* `REFINE_APP` does not drop batches. If it falls behind, the bounded wait
  of ADR-015 section 3 applies, and loss still happens only at the input of
  `DENSE_APP`.
* Time budget: refinement adds to the latency of each keyframe. It runs on
  the CPU at the same time as the dense work of the previous batch on the
  GPU, so it limits the throughput only if it takes longer than the keyframe
  interval (2.9 s at the nominal point). Provisional target: at most 1 s per
  batch at the nominal point. The benchmark measures it.

### 7. Evaluation (amends ADR-018)

* E4 runs every pose-error level twice, with `refine_pose` off and on:
  * the `non-optimized` pose set (PPK positions, recorded attitude) and the
    attitude errors with `precise_position = true`;
  * the position noise levels (0.5 m, 2 m) with `precise_position = false`.
* Position errors are added as per-image noise and as a slowly varying bias,
  because the errors of a standard GNSS receiver are correlated in time.
* Additional metrics: position and attitude error of the refined poses
  against the Metashape adjusted poses (RMS and maximum), the share of
  batches marked `refine_failed`, and the refinement time per batch.

## Rationale

* Two values match the two situations an operator actually has: poses that
  can be trusted, PPK or RTK with an unreliable attitude, and standard GNSS.
* The operator knows the GNSS equipment before the flight. A fixed
  configuration keeps runs reproducible and the applications simple.
* Routing by message identifier keeps `DENSE_APP` and all later stages
  unchanged. With `refine_pose = false`, the pipeline is exactly the core
  pipeline.
* Survey images are sparse (0.69 Hz at the nominal point) and overlap
  strongly. This suits bundle adjustment over a window better than
  video-rate tracking.
* GTSAM is already built (ADR-010) and offers projection factors, robust
  losses and priors. Its license (BSD-3-Clause) fits the product.
* Keeping positions fixed in the PPK case uses their centimetre accuracy
  instead of letting tie points move them.

## Alternatives Considered

* **Automatic choice from the reported accuracy.** Rejected for now: the
  operator knows the equipment, and a fixed choice is easier to test and to
  reproduce. The accuracy fields still weight the priors.
* **`REFINE_APP` always in the path, passing batches through when
  disabled.** Rejected: an additional hop and an application that must
  always run, even in missions that never refine.
* **Video-rate visual or visual-inertial SLAM (for example ORB-SLAM3,
  VINS-Fusion, OpenVINS, Basalt, Kimera-VIO).** Not now: they need video
  rates and usually an IMU stream, which the data source does not deliver
  (ADR-003). ORB-SLAM3, VINS-Fusion and OpenVINS are GPLv3, which does not
  fit the product license. They may be taken up again for flights without
  GNSS.
* **Loop closure across flight lines.** Not now: neighbouring flight lines
  already share source views within the window (ADR-006). It may be added
  later, for example with iSAM2.
* **Features on the GPU (COLMAP SIFT).** Not now: the GPU is reserved for
  PatchMatch. To be reconsidered if CPU features are too slow.
* **Refining the camera calibration as well.** Rejected: the calibration is
  known (ADR-008), and estimating it in a small window is unstable.

## Affected Areas

* [x] cFS application structure
* [x] algorithm libraries
* [x] Software Bus messages
* [x] configuration/tables
* [x] time and execution model
* [x] evaluation and experiments
* [x] documentation

## Impact

* **ADR-005:** dense reconstruction still uses no bundle adjustment of its
  own. Optional refinement before it comes from this record.
* **ADR-011:** the library list gains `refine`.
* **ADR-012:** the application list gains `REFINE_APP`.
* **ADR-015:** `REFINE_APP` has one child task, at priority level 3.
* **ADR-016:** `KeyframeBatch` is published under "batch selected" or "batch
  ready" and carries a pose source; new table values `refine_pose` and
  `precise_position`.
* **ADR-018:** E4 is run with refinement off and on, and pose accuracy is
  added to the metrics.
* **Benchmark:** the feature type and the refinement time are measured on
  Dorfberg with the `non-optimized` pose set.

## Open Questions

* Which features to use (SIFT or ORB): decided by the benchmark.
* The values of the failure checks of section 5: set from the benchmark.
