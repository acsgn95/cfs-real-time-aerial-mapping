# ALIGN_APP

Time alignment of the images with the telemetry stream (ADR-012, ADR-014
section 4). It buffers the telemetry samples of the source adapter,
interpolates the camera pose at each image's capture time and publishes
one `ImagePose` per image, valid or with the reason it is not. The
algorithm is `libs/time_align`.

## Messages

| Direction | Message | Use |
| --- | --- | --- |
| in | `INGEST_APP` `TelemetrySample` | body pose: geodetic position, `q_N_B`, accuracy |
| in | `INGEST_APP` `ImageArrived` | capture time and JPEG reference; one reference count for this application |
| in | `INGEST_APP` `SessionCalibration` | camera mounting `q_B_C`, `t_B_C` |
| out | `IMAGE_POSE_TLM` `ImagePose` | camera centre in ECEF, `q_E_C`, accuracy, result, the two samples used, waiting time |
| out | `HK_TLM` | counters per result, sample verdicts, buffer, waiting times, faults |

* With a valid pose the JPEG reference count goes on to the consumer of
  `ImagePose` (ADR-013 section 3). Otherwise it is released here and the
  reference in the message is empty.
* `ImagePose` comes out in the order of `ImageArrived`, also when a later
  image could be decided first.
* Commands: no-op (0), reset counters (1), clear latched faults (2).

## Alignment

* Both samples around the capture time are converted to ECEF first; the
  body position is interpolated linearly and `q_E_B` with slerp, then the
  mounting gives the camera pose. Interpolating in ECEF avoids mixing the
  local NED frames of two positions.
* No extrapolation: an image waits for the first sample after it, at most
  `MaxWaitMs`, measured on `CLOCK_MONOTONIC` from its arrival.
* Results (`AlignResult`): `VALID`; `NO_SAMPLE_BEFORE` (captured before
  the buffer), `GAP_TOO_LARGE` (samples more than `MaxGapMs` apart),
  `WAIT_EXPIRED` (no sample after it), all three raising
  `POSE_NO_TELEMETRY`; `NO_CALIBRATION`, `QUEUE_FULL`, `CLEARED` (session
  changed while it waited).
* Samples older than the newest one, repeated times and samples without a
  position fix are discarded and counted.
* A message with another session identifier starts a new session: the
  buffer and the mounting are dropped, waiting images come out as
  `CLEARED`.
* A valid pose whose reported accuracy exceeds `MaxPositionStdM` or
  `MaxAttitudeStdDeg` is published with `AccuracyLow` set and raises
  `POSE_ACCURACY_LOW` (auto-clear with the next pose within the limits).

## Task

One task. Commands and input messages share one pipe; after every message,
and at least every `ALIGN_APP_PIPE_TIMEOUT_MS` (50 ms), the images that can
be decided are published. The application therefore needs no wake-up
message. Its priority is that of `INGEST_APP` (ADR-015 section 4).

## Configuration

Table `ALIGN_APP.ConfigTable` (`fsw/tables/align_app_tbl.c`), loaded once at
startup (ADR-016 section 5):

| Field | Default | Meaning |
| --- | --- | --- |
| `MaxGapMs` | 300 | gap between the two samples around an image (three periods at 10 Hz) |
| `MaxWaitMs` | 1000 | waiting time for the next sample |
| `BufferMs` | 10000 | telemetry kept, back from the newest sample |
| `MaxPendingImages` | 64 | images waiting at the same time |
| `MaxPositionStdM` | 5.0 | `POSE_ACCURACY_LOW` above this; 0: no check |
| `MaxAttitudeStdDeg` | 5.0 | `POSE_ACCURACY_LOW` above this; 0: no check (the injector reports 3 degrees) |

Without a valid table the application stays up, raises `TABLE_REJECTED`
and releases the JPEG of every image without publishing a pose.

Platform values (`config/default_align_app_platform_cfg.h`): pipe depth and
per-message limits, receive timeout, images held at the same time.

## Events and faults

See [`docs/interfaces/events.md`](../../docs/interfaces/events.md) and
[`docs/interfaces/faults.md`](../../docs/interfaces/faults.md).

## Verification

Injector with Dorfberg keyframe region 60, 30 images, 10 Hz telemetry,
recorded with `tools/ground/tlm_capture.py` and compared with
`tools/ground/compare_image_poses.py` (2026-11-14): all poses valid and in
order; attitude error median 0.0062°, max 0.0283°; position error median
0.84 mm, max 3.5 mm, the same as the injector's own check of the
interpolation (`--dry-run`: 0.0064°, 0.0283°, 0.9 mm, 3.5 mm). Images
waited 32 ms (median) for the next sample.
