# ADR-015: Execution Model

* Status: Accepted; amended by [ADR-019](adr_019_pose_refinement.md) (`REFINE_APP`: one child task, priority level 3)
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D14
* Revised: 2026-11-11 (reusable components for other cFS missions)
* Evidence: ADR-001 (R1–R3), ADR-005 (one GPU job in flight, delayed mode),
  ADR-007 (publication period), ADR-011 (synchronous library calls),
  ADR-012 (applications), ADR-013 (quotas and failed allocations), ADR-014
  (alignment waiting limit); OSAL on Linux applies task priorities only
  with real-time scheduling privileges

## Context

ADR-012 defines seven applications. Their work differs widely in duration:
receiving a telemetry sample takes microseconds, a dense reconstruction takes
seconds. ADR-001 requires sustained throughput (R1), bounded latency (R2) and
no uncontrolled loss (R3). ADR-013 lets allocations fail when a quota is full
and leaves the reaction to this record.

## Decision

### 1. Triggering

* Applications on the data path are event-driven: they wake when a message
  arrives.
* Periodic work is triggered by a 1 Hz wake-up message from the mission's
  scheduler (`SCH_LAB` in the reference mission). The message identifiers of
  the wake-up and of the housekeeping request are configuration values of
  each application, so a host mission can connect its own scheduler. This covers housekeeping, the waiting limits of
  `ALIGN_APP` (ADR-014) and the publication period of `MAPPROD_APP`
  (ADR-007).
* The main loop of every application waits for messages with a short
  timeout (default 100 ms), so that it serves both data messages and
  periodic work on time.

### 2. Long work runs in child tasks

The main task of each application always stays free for commands,
housekeeping and messages. Work that can block or take long runs in child
tasks:

| Application | Child tasks |
| --- | --- |
| `INGEST_APP` | UDP receiver; TCP receiver (blocking sockets) |
| `SELECT_APP` | decoder to the working resolution |
| `DENSE_APP` | GPU worker; one job in flight; in delayed mode one more batch may wait (ADR-005) |
| `GEOREF_APP` | LAZ writer (disk) |
| `MAPPROD_APP` | fusion and orthomosaic worker; publisher (disk) |

Library calls are synchronous (ADR-011). They run on the task that the
application chooses.

### 3. One controlled drop point

* Keyframes are skipped at one place only: the input of `DENSE_APP`.
* Rule "newest wins": while a job runs and a batch is waiting, a newly
  arriving batch replaces the waiting one. The replaced batch is skipped.
  This keeps latency bounded (R2). Every skipped keyframe is counted and
  reported (R3).
* Every other stage does not drop data. It waits for a bounded time
  instead. For example, when the point set quota is full, `GEOREF_APP` waits
  for `MAPPROD_APP` to catch up. This backpressure propagates back to the
  input of `DENSE_APP`, so loss still happens only there.
* `INGEST_APP` must not lose input inside the envelope (R3). Quotas are sized
  for this. If an allocation still fails there, it is reported as an error
  and the image is dropped with a reason.
* If a bounded wait in any other stage expires, the data is dropped and
  reported as an error. This indicates that the configuration is outside the
  real-time envelope.

### 4. Priorities

Relative task priorities, from highest to lowest:

1. `INGEST_APP`, `ALIGN_APP` (input must not be lost)
2. `SELECT_APP`
3. `GEOREF_APP`, `MAPPROD_APP`
4. `DENSE_APP` (mostly waiting for the GPU)

On Linux these priorities take effect only with real-time scheduling
privileges (`CAP_SYS_NICE`). Without them OSAL ignores them. The mode used is
recorded with every measurement.

### 5. CPU use

* OpenCV and COLMAP use internal threads. Each application's thread count for
  these libraries is limited by a table value, so that the libraries together
  do not oversubscribe the CPU.
* Pinning tasks to specific cores (for example performance or efficiency
  cores) is not done now. It may be added after measurements.

## Rationale

* Event-driven processing adds no polling delay to the data path; a 1 Hz tick
  is enough for timeouts, housekeeping and publication.
* Child tasks keep applications responsive while long work runs, which keeps
  commands and housekeeping reliable.
* One drop point makes loss visible and easy to reason about. "Newest wins"
  favours the current position of the vehicle, which is what a live map
  needs.
* Backpressure with bounded waits prevents silent queue growth (R1) without
  unbounded memory (R4).
* Limiting library threads avoids contention that would make timings noisy.

## Alternatives Considered

* **Scheduler-driven polling of every application.** Rejected: adds up to one
  period of delay per stage.
* **Dropping at every stage when full.** Rejected: loss would be scattered and
  hard to attribute.
* **"Oldest wins" at the dense input.** Rejected: latency would grow while the
  system is overloaded.
* **Unbounded queues.** Rejected: violates R1 and R4.
* **Core pinning now.** Deferred until measurements show a need.

## Affected Areas

* [x] cFS application structure
* [x] time and execution model
* [x] configuration/tables
* [x] observability and performance measurement
* [x] documentation

## Impact

* **D15:** wait limits, queue depths, thread counts and the drop policy are
  table values. A status message from `DENSE_APP` reports skipped keyframes.
* **D16:** queue lengths, wait times, drops with reasons and the scheduling
  mode are reported.
* **ADR-002:** the scheduling mode (with or without real-time privileges) is
  part of the recorded measurement conditions.

## Open Questions

None.
