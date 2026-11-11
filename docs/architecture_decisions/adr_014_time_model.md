# ADR-014: Time Model

* Status: Accepted
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D13
* Evidence: ADR-001 (R2 latency from the image timestamp), ADR-002 (one
  machine), ADR-003 (separate telemetry and imagery, time factor, timestamp
  offset), ADR-006 (valid pose), ADR-012 (`ALIGN_APP`); Dorfberg geotags
  with Unix millisecond timestamps

## Context

The system needs two kinds of time:

* **measurement time:** when an image was captured and when a pose was
  measured;
* **processing time:** how long each stage takes, and the end-to-end latency.

The nominal dataset was recorded in 2020 and is replayed now, possibly at a
different speed. ADR-006 relies on a "valid pose" that ADR-003 leaves to this
record.

## Decision

### 1. Data time on the link

* The injector maps recorded timestamps to the present:
  `t_emulated = t_start + (t_recorded - t_recorded_0) / time_factor`.
  This corresponds to a UAV that stamps its data with GNSS time while the
  ground station runs on the same UTC time.
* Format: signed 64-bit integer nanoseconds since the Unix epoch, UTC.
* The original recorded time is carried as well, so that results can be
  related to the reference data.

### 2. Time inside OrthoStream

* Measurement timestamps in message payloads are always data time: the
  instant of capture or measurement, never the instant of publication.
* The cFE message header time (time of publication) is not used for
  measurements.
* Stage durations are measured with `CLOCK_MONOTONIC`.

### 3. End-to-end latency (ADR-001 R2)

* Latency = UTC time of the tile update − UTC capture time of the image.
* The injector and OrthoStream run on the same machine and read the same
  system clock, so both times are directly comparable.
* Condition: the system clock must not step during a measurement. The time
  service may only slew it. This is added to the measurement conditions of
  ADR-002.

### 4. Time alignment in `ALIGN_APP`

* The pose at an image timestamp is interpolated between the two telemetry
  samples around it: linear interpolation for position, spherical linear
  interpolation (slerp) for attitude.
* There is no extrapolation. If the next telemetry sample has not arrived,
  the image waits. If it does not arrive within the waiting limit, the image
  is rejected.
* An association is valid when all of the following hold:

  | Rule | Default |
  | --- | --- |
  | gap between the two samples around the image | at most 0.3 s (three periods at 10 Hz) |
  | waiting time for the next sample | at most 1 s |
  | telemetry buffer length | 10 s |
  | sample order | out-of-order and duplicate samples are discarded |

* Every rejected image is counted and reported with its reason.
* A "valid pose" in ADR-006 is a successful association under these rules.
* The defaults are configuration values (D15).

### 5. Relation to the experiment parameters

* **Time factor:** only time is scaled; positions stay the same. The flight
  appears as if flown faster or slower.
* **Timestamp offset:** the injector shifts image timestamps (ADR-003).
  `ALIGN_APP` cannot detect this and interpolates the pose at the wrong
  time. This is the effect the experiment measures.

## Rationale

* UTC data time on the link matches how real UAV systems stamp their data.
* Separating measurement time from publication time keeps Software Bus delays
  out of the geometry.
* A monotonic clock for durations is immune to clock adjustments.
* No extrapolation keeps pose errors bounded by the telemetry rate instead of
  by the vehicle dynamics.

## Alternatives Considered

* **cFE header time as measurement time.** Rejected: it is the time of
  publication.
* **Original 2020 timestamps inside OrthoStream.** Rejected: latency could not
  be measured against the present clock.
* **Extrapolating the pose.** Rejected: unbounded error in turns and after
  telemetry gaps.
* **Higher-order interpolation (for example cubic).** Not chosen: at 10 Hz
  and on straight flight lines, linear interpolation and slerp are
  sufficient. It may be added later as an option.

## Affected Areas

* [x] simulator interface
* [x] time and execution model
* [x] Software Bus messages
* [x] configuration/tables
* [x] observability and performance measurement
* [x] evaluation and experiments
* [x] documentation

## Impact

* **ADR-002:** no clock steps during measurements.
* **Injector:** maps recorded time to the present and carries both times.
* **D14:** `ALIGN_APP` holds images while it waits for telemetry.
* **D15:** timestamps are 64-bit nanoseconds; alignment limits are table
  values.
* **D16:** per-stage monotonic timings and UTC latency are recorded.

## Open Questions

None.
