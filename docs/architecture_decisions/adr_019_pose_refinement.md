# ADR-019: Optional Pose Refinement

* Status: Accepted
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D18
* Revised: 2026-11-11 (latency limit of 120 s with refinement; refinement
  waits for the next flight line); 2026-11-13 (shared boresight variable,
  feature candidates, implementation notes and first results in section 8)
* Evidence: ADR-001 section 1b (recorded attitude about 1.5° from the
  adjusted attitude, about 195 px in the image); ADR-003 section 4 (pose
  sets); ADR-010 (GTSAM 4.3.0 as the optional vcpkg feature `refine`, built
  on the development machine); ADR-012, ADR-013, ADR-015 and ADR-016
  (applications, data store quotas, execution model, tables loaded at
  startup); ADR-018 E4 (pose-error experiment)

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

* `REFINE_APP` also has a latency limit, `refine_max_latency` (default
  120 s, section 6). It does not change the routing.
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
   * A shared boresight rotation D, one variable for all images: the
     camera of image i is C_i * D, with C_i its pose variable. D absorbs the
     misalignment of the camera mounting, which is common to the flight and
     not random per image (about 1.3° on Dorfberg). Its prior is identity
     with a loose rotation sigma; its translation is held at zero.
4. **Waiting for later images.** A survey block is stiffest when images of
   the neighbouring flight line tie into it. `REFINE_APP` therefore holds a
   batch until the footprint of its keyframe is covered by images of the
   following flight line, or until a waiting limit expires (section 6). Images
   after the keyframe then also contribute tie points.
5. **Window.** The window holds the keyframes and images received within
   the latency limit, with their source views. An image that leaves the
   window keeps its last refined pose. When it appears in a later batch,
   that pose is used as a tight prior. Memory stays bounded (ADR-001 R4).

GPU: features and optimisation run on the CPU. Refinement therefore does
not compete with PatchMatch for GPU memory (ADR-002, ADR-005). With SGBM,
the default dense method since 2026-11-13 (ADR-005), dense reconstruction
also runs on the CPU; refinement and SGBM share it, and their combined time
per keyframe is measured.

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
* Latency: with `refine_pose = true`, map update latency may be much longer
  than the target of ADR-001 R2 (10 s, or 5 s in causal mode), because refinement waits for later
  images. The latency limit is a table value, `refine_max_latency`, default
  120 s; the 95th percentile of the map update latency must stay below it.
  The waiting limit of section 3 is the latency limit minus the measured time
  of refinement and dense reconstruction. With `refine_pose = false`, R2
  keeps its target.
* Throughput (ADR-001 R1) is unchanged: on average, refinement of a batch
  must take less than the keyframe interval (2.9 s at the nominal point), or
  the backlog grows. Refinement runs on the CPU while the GPU computes
  earlier batches. The benchmark measures the time per batch.
* Memory: during the latency limit, keyframe JPEGs wait in the data store
  for the orthomosaic. At the nominal point, 120 s hold about 41 keyframes,
  about 0.8 GB, within the default JPEG quota of 1 GB (ADR-013). Larger
  cameras or longer limits need a larger quota.

### 7. Evaluation (amends ADR-018)

* E4 runs every pose-error level twice, with `refine_pose` off and on:
  * the `non-optimized` pose set (PPK positions, recorded attitude) and the
    attitude errors with `precise_position = true`;
  * the position noise levels (0.5 m, 2 m) with `precise_position = false`.
* Position errors are added as per-image noise and as a slowly varying bias,
  because the errors of a standard GNSS receiver are correlated in time.
* Additional metrics: position and attitude error of the refined poses
  against the Metashape adjusted poses (RMS and maximum), the share of
  batches marked `refine_failed`, the refinement time per batch, and the
  map update latency with refinement.
* The waiting of section 3 is compared with no waiting (causal refinement):
  accuracy against latency.

### 8. Implementation notes and first results (2026-11-13)

* Library `libs/refine` (`ORTHOSTREAM_WITH_REFINE`): a feature extractor
  interface with SIFT as the first implementation, pair matching (ratio
  test 0.8, essential-matrix RANSAC on normalised coordinates), tracks by
  union-find, and the adjustment with GTSAM expression factors, Huber loss
  and Levenberg–Marquardt, solved twice (outliers above 4 px removed in
  between). The checks of section 5 are implemented. The C interface and
  the sliding window follow with `REFINE_APP`.
* OpenCV's SIFT shifts keypoints by +0.25 px in x and y with its default
  upscaling of the first octave. Images of opposite flight lines are turned
  by 180°, so their tie points disagree by 0.5 px and the adjustment turns
  the cameras by about 0.065° (found with a rendered test scene). The
  extractor uses OpenCV's precise upscaling; the error drops to 0.002°.
* `tools/refine_poses` runs block mode: all batches of a block of
  keyframes in one adjustment. It is the upper bound of what the sliding
  window can reach and writes a pose file for `offline_pipeline`.
* First result, Dorfberg non-optimized poses, keyframes 0 to 40 (86 images,
  400 pairs, 30 000 tie points, `precise_position = true`):

  | Poses against Metashape | Attitude median | 95 % | Max | Reprojection RMS |
  | --- | --- | --- | --- | --- |
  | recorded (prior) | 1.51° | 2.47° | 3.32° | 73.8 px |
  | refined | 0.014° | 0.026° | 0.032° | 0.24 px |

  The boresight came out at 1.27°. The positions keep a vertical offset of
  0.39 m against Metashape, which refinement must not and does not remove.
  It was first taken for the antenna lever arm; it is the height datum of
  the reference project (ADR-003 section 4, amended 2026-11-13), and the
  pose set now uses the reference heights. Time: features
  16 s, matching 2.4 s, adjustment 101 s on the CPU for the whole block,
  too slow for the keyframe interval; the window and the number of points
  must be bounded.
* Closing the block-mode evaluation (2026-11-13, keyframes 60 to 99,
  SGBM, against the reference DSM):

  | Run | Median | NMAD | RMSE | Area |
  | --- | --- | --- | --- | --- |
  | Metashape poses | −0.007 m | 0.069 m | 0.363 m | 0.400 km² |
  | refined; ellipsoidal heights, selection with refined poses | +0.342 m | 0.077 m | 1.565 m | 0.379 km² |
  | refined; reference heights (ADR-003 section 4) | −0.001 m | 0.080 m | 1.598 m | 0.380 km² |
  | refined; reference heights, selection with telemetry poses | +0.002 m | 0.079 m | 0.359 m | 0.392 km² |

  * The median offset was the height datum of the reference, not a lever
    arm (ADR-003 section 4).
  * The high RMSE came from the tool, not from refinement: the pipeline
    chose keyframes and source views again with the refined poses, so some
    source views lay outside the refined block and kept their recorded
    attitude. `offline_pipeline --selection-poses` now selects with the
    telemetry poses, as section 2 requires, while depth range, terrain
    gate and footprint follow the batch poses. With refined poses SGBM
    then matches the Metashape poses (RMSE 0.36 m).
  * PatchMatch with the same refined block: median −0.036 m, NMAD 0.190 m,
    RMSE 2.15 m (Metashape poses: 0.164 m and 1.35 m). The additional
    errors lie at both ends of the block, where the refined poses have the
    fewest tie points. Not yet explained (open question).

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
* A map that is correct after two minutes is more useful than a map that is
  wrong after five seconds. Accepting a longer latency lets refinement use
  the neighbouring flight line, which constrains the attitude across the
  track.

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
* **Features on the GPU (COLMAP SIFT).** Not now: the GPU was reserved for
  PatchMatch. With SGBM as the default dense method (ADR-005, 2026-11-13)
  the GPU is free, so this is the first option if CPU features are too
  slow.
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

* **ADR-001:** with `refine_pose = true`, R2 uses `refine_max_latency`
  (default 120 s) instead of its default target.
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

* Which features to use: decided by the benchmark. Candidates: SIFT
  (implemented), ORB, and the learned XFeat (Potje et al., CVPR 2024), which
  is reported faster and more robust than SIFT; XFeat needs a neural network
  runtime (for example ONNX Runtime), a new dependency to be approved.
* Run time of the adjustment (section 8): the sliding window must fit the
  keyframe interval on average.
* The values of the failure checks of section 5: set from the benchmark.
* PatchMatch with refined poses is worse at the ends of a refined block
  than with the Metashape poses, SGBM is not (section 8). To be examined
  with the sliding window, whose ends are always open.
