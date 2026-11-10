# ADR-017: Observability and Performance Measurement

* Status: Accepted
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D16
* Evidence: thesis proposal (FB_YL-01) section 4.3 (performance metrics);
  ADR-001 (R1–R4), ADR-002 (measurement conditions), ADR-014 (time model),
  ADR-015 (drop point, scheduling mode), ADR-016 (messages, housekeeping)

## Context

The proposal evaluates map update latency, images processed per second, map
update frequency, CPU and GPU use, memory use and cFE Software Bus latency.
ADR-001 adds backlog, latency percentiles and skipped keyframes. Earlier
records leave "reported" and "recorded" to this record.

## Decision

### 1. Three channels

| Channel | Content | Rate |
| --- | --- | --- |
| cFE events (EVS) | errors, state changes, dropped data with reasons | when they occur; repeated events are limited with EVS filters |
| Housekeeping (`<APP>_HK`) | counters, queue lengths, data store statistics, library memory use | 1 Hz |
| Trace records | stage timings per image, keyframe and tile | once per item |

### 2. Trace records

* Every data path message carries a trace identifier (image sequence number
  or keyframe identifier).
* It also carries the monotonic entry and exit times of its stage and the UTC
  time of publication.
* `RECORDER_APP` records these messages. There is no separate timing message;
  the timings travel inside the data messages.
* Derived offline:
  * **stage duration** = exit time − entry time of a stage;
  * **Software Bus latency** = entry time of the next stage − exit time of the
    previous stage. This is the cFS overhead metric of the thesis;
  * **end-to-end latency (R2)** = `TileUpdated` time − image capture time,
    with its full distribution including the 95th percentile.

### 3. System resources are sampled externally

* A Python sampler (`tools/evaluation/sampler`) records CPU and GPU use, RAM,
  GPU memory, clock rates and temperatures (ADR-002) at 1 Hz.
* It also covers the injector. No monitoring dependency such as NVML is added
  to the cFS applications.
* The sampler output is joined with the recorder output by UTC time.

### 4. One directory per run

* Each run writes to `runs/<run identifier>/`:
  * one CSV file per message type (rates are low, so CSV is sufficient);
  * the event log;
  * the sampler output;
  * a JSON manifest with everything needed to repeat the run: git commit,
    build type, scheduling mode (ADR-015), table or YAML configuration,
    dependency versions, injector parameters.
* `runs/` is not committed.

### 5. Proposal metrics and their sources

| Proposal metric | Source |
| --- | --- |
| Map update latency | `TileUpdated` time − capture time |
| Images processed per second, map update frequency | trace records |
| Processing queue length, skipped keyframes | housekeeping, `DenseStatus` |
| CPU, GPU, RAM and GPU memory use | sampler |
| cFE Software Bus latency | differences between stage times |

### 6. Low overhead

* Records are per item, never per pixel or point.
* The cFE performance logger (`CFE_ES_PerfLog`) is available as an optional
  fine-grained trace when needed.

## Rationale

* Timings inside the data messages need no extra messages and cannot get out
  of step with the data they describe.
* Monotonic stage times and UTC capture times give both per-stage durations
  and true end-to-end latency on one machine (ADR-014).
* External resource sampling measures the whole machine, including the
  injector, without adding dependencies to the flight software.
* A manifest per run makes every reported number traceable to its
  configuration.

## Alternatives Considered

* **Separate timing messages per stage.** Rejected: doubles the message count
  without adding information.
* **Resource sampling inside `RECORDER_APP` (NVML, `/proc`).** Rejected: adds
  dependencies and misses the injector.
* **A binary or columnar log format (for example Parquet).** Not needed at
  these rates; CSV is readable by every tool.
* **Logging per pixel or point.** Rejected: overhead.

## Affected Areas

* [x] observability and performance measurement
* [x] Software Bus messages
* [x] evaluation and experiments
* [x] documentation

## Impact

* **ADR-016:** data path messages include the trace identifier and stage
  times.
* **`.gitignore`:** `runs/` is excluded.
* **D17:** the evaluation tools read the run directories.

## Open Questions

None.
