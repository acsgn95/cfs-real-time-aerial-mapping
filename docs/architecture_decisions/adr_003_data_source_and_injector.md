# ADR-003: Data Source and Injector Interface

* Status: Accepted
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D03
* Evidence: ADR-001, ADR-002; Dorfberg Flight 01 dataset (geotags with
  millisecond timestamps, Metashape project with adjusted camera poses)

## Context

ADR-001 takes the Dorfberg Flight 01 dataset as the nominal point. ADR-002
places a Python injector on the same machine as OrthoStream. The injector
emulates a UAV that sends its data to the ground over a radio link.

The thesis proposal describes a UAV simulation environment that provides
images, poses, timestamps, calibration and a reference surface. Replaying a
recorded flight departs from this and needs the supervisor's agreement.

The dataset provides one pose per image, at the camera trigger time. A real
UAV sends its autopilot telemetry as a continuous stream and its images
separately. OrthoStream's Time Alignment application, and the timestamp-offset
experiments in the proposal, only make sense if the two streams are separate.

## Decision

### 1. The injector replays recorded flights in real time

* The injector is a Python program. It reads a dataset from disk and sends it
  as if the UAV were flying now. The recorded timeline is kept, scaled by a
  configurable time factor (1.0 = real time).
* Images are sent as the recorded JPEG files. The injector does not decode or
  re-encode them.
* Reference data (reference DSM, ground control points) is never sent over
  the link. Only the evaluation tools read it.

### 2. Two channels emulate the radio link

| Channel | Transport | Content | Rate |
| --- | --- | --- | --- |
| Telemetry | UDP datagrams | pose samples: time, position, attitude, accuracy; heartbeat | configurable, default 10 Hz |
| Imagery | TCP stream with length-prefixed frames | one frame per image: sequence number, capture timestamp, format, size, JPEG bytes; session start message with camera calibration and camera-to-body transform | one frame per image |

The exact message layouts are defined together with D04 (pose and attitude
conventions) and D13 (time model).

### 3. Telemetry is a continuous stream

The injector interpolates the per-image poses of the dataset into a
continuous trajectory and samples it at the telemetry rate. Images carry their
own capture timestamps. OrthoStream must associate each image with the
telemetry stream itself (Time Alignment), as it would with a real UAV.

### 4. Selectable pose sets

The pose set is selected per run:

| Option | Positions | Attitude | Use |
| --- | --- | --- | --- |
| `optimized` (default) | Metashape bundle-adjusted camera poses | bundle-adjusted | nominal runs |
| `non-optimized` | PPK positions recorded in flight | attitude recorded by the aircraft (about 1.5° from the adjusted attitude) | pose-error experiments |

### 5. Link conditions are emulated outside the injector

* The injector runs in its own network namespace, connected to the host by a
  virtual Ethernet pair.
* Bandwidth, latency, jitter and loss are applied to that pair with Linux
  `tc netem`. Other local traffic is not affected.
* Link profiles (for example, ideal, Wi-Fi, cellular) are named
  configurations, so experiments can be repeated.
* The nominal flight needs about 12.6 MB/s (about 100 Mbit/s) of image data.

### 6. Experiment parameters of the injector

* **Time factor:** scales the timeline, which emulates a different ground
  speed and image rate.
* **Image decimation:** sends every n-th image, which emulates lower forward
  overlap.
* **Pose set:** `optimized` or `non-optimized` (section 4).
* **Controlled errors:** added position and attitude noise or bias, and an
  image timestamp offset.
* **Link profile:** section 5.

### 7. Scope of the evaluation

* **Altitude and camera tilt are not varied for now.** They remain part of
  the design envelope of ADR-001, so no component may assume a fixed altitude
  or a nadir camera. Whether and how they are varied in the evaluation is left
  open and may be taken up again in D17.
* **No dedicated long-duration test.** The 60 min continuous-operation test
  of ADR-001 is dropped. The evaluation uses the recorded flights as they are
  (24 min for the nominal flight). Requirement R4 of ADR-001 still holds; it
  is shown by recording RAM and GPU memory use over the whole nominal flight.

## Rationale

* Replaying recorded JPEG files keeps the injector simple and makes the input
  identical between runs.
* Separate telemetry and imagery channels match real UAV systems. They make
  time alignment and timestamp-offset experiments meaningful.
* UDP fits small, periodic telemetry where a late sample is worthless. TCP
  fits large images that must arrive complete.
* Emulating the link with `netem` on a dedicated interface keeps link
  behaviour out of the injector code and keeps other local traffic
  unaffected.
* The two pose sets turn the recorded aircraft attitude into a realistic
  pose-error case without extra work.

## Alternatives Considered

* **One pose per image, sent with the image.** Rejected: it removes time
  alignment and makes timestamp-offset experiments meaningless.
* **Link delay and loss implemented in the injector.** Rejected: harder to
  validate than `netem`, and it would mix experiment logic into the data
  source.
* **`netem` on the loopback interface.** Rejected: it would slow down all
  local traffic, including cFS ground interfaces.
* **Varying altitude and tilt now, by rendering synthetic views from the
  reference DSM and orthophoto or by adding datasets.** Deferred (section 7).
* **A 60 min looped replay shifted to new ground each loop.** Rejected: not
  needed for the thesis; the recorded flights are evaluated as they are.
* **Shared memory or files between injector and OrthoStream.** Rejected: it
  would not represent a radio link and would hide transfer time from the
  latency measurements.

## Affected Areas

* [x] simulator interface
* [x] coordinate frames and conventions
* [x] time and execution model
* [x] observability and performance measurement
* [x] evaluation and experiments
* [x] documentation

## Impact

* **D04:** Defines the pose representation and attitude convention used on
  the telemetry channel.
* **D11:** The Simulator Interface application receives two channels and
  reassembles TCP frames into images.
* **D12:** Images arrive as JPEG bytes; decoding happens on the ground side.
* **D13:** The time model must relate the injector's emulated UAV time to cFE
  time.
* **D17:** The injector parameters of section 6 are the experiment controls.
  Altitude and tilt variation may be revisited there.
* **ADR-001:** The flight duration row ("tested for at least 60 min") is
  replaced by section 7 of this record.

## Open Questions

* Has the supervisor agreed to recorded-data replay in place of a simulation
  environment?
