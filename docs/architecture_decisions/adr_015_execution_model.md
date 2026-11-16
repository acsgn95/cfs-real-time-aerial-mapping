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

## Implementation Notes (2026-11-14)

* `ALIGN_APP` uses no wake-up message: its 50 ms pipe timeout enforces the
  waiting limit of ADR-014.
* `SELECT_APP` hands images to its worker through a ring of the
  application with a mutex and a counting semaphore. OSAL queues map to
  POSIX message queues, which allow only 10 entries
  (`/proc/sys/fs/mqueue/msg_max`) without privileges.
* cFE limits a pipe to 50 messages (`CFE_PLATFORM_SB_MAX_PIPE_DEPTH`). A
  data path message lost to a full pipe also loses the data store counts
  it carries, so the per-message limits of the subscriptions are set
  generously.
* `DENSE_APP` (2026-11-14): one child task `DENSE_WORK` (priority level
  4), a ring of the application with the drop rule of section 3, and one
  `DENSE_STATUS` message per batch (done, skipped or failed, with the
  reason).
* **Bursts with pose refinement.** `REFINE_APP` publishes together all
  batches that one window solution covers. On Dorfberg up to 7 batches
  arrive within a millisecond, although on average one batch arrives per
  keyframe interval (2.9 s) and `DENSE_APP` needs about 1 s per batch. With
  one waiting batch, as section 3 first set it, 8 of 30 keyframes were
  skipped in the first run. The number of waiting batches is therefore a
  table value (`MaxWaitingBatches`, platform limit 16) with default 16:
  newest wins only applies when the backlog exceeds a burst. With
  `RefinePose = 0` the batches come one per keyframe and a value of 1
  restores the rule of section 3. A burst of 16 adds at most about 16 s
  of queueing, against the 100 s that refinement already waits.
* `GEOREF_APP` (2026-11-14): one child task `GEOREF_WORK` (priority level
  3) does the conversion, the publication and then the LAZ files of
  section 2, in this order, so that the files are not on the path of the
  map update. On Dorfberg the conversion takes about 0.1 s and the files
  about 0.3 s per keyframe, well within the keyframe interval; a separate
  writer task is not needed. A ring of 16 depth sets without a drop rule;
  a depth set finding it full is dropped with an error (section 3).
* `MAPPROD_APP` (2026-11-14): `MAPPROD_WORK` owns the tiles (the library
  is not thread-safe): fusion, orthomosaic, lifecycle and, every
  publication period, a copy of the changed tiles. `MAPPROD_PUB` (below
  the worker) writes the copies. On the worker, writing held up the map
  update by up to 20 s; ADR-007 section 8 has the details. The period
  comes from the 1 Hz wake-up of `SCH_LAB`; while the publisher still
  writes, changes wait for the next period.
* **Pipe depth (2026-11-16).** The same limit applies to Software Bus
  pipes: OSAL truncates every pipe to `/proc/sys/fs/mqueue/msg_max`
  (10 without privileges), whatever depth `CFE_SB_CreatePipe` asks for.
  `SCH_LAB` sends all housekeeping requests in the same tick, so a pipe
  that subscribes to many housekeeping messages (the recorder, `TO_LAB`)
  receives about 15 at once. `RECORDER_APP` spreads its subscriptions over
  three pipes; `TO_LAB` occasionally drops a housekeeping packet (display
  only). `REFINE_APP` published one window's batches within 0.2 ms, up to
  12 in one recorder run: more than a pipe holds, so the recorder lost 5,
  and `DENSE_APP` would lose a batch with its data store counts.
  `REFINE_APP` therefore spaces its `BATCH_READY` messages by at least
  `REFINE_APP_PUBLISH_GAP_MS` (5 ms; a burst of 12 takes 60 ms, against
  the 100 s that refinement waits). For measurement runs
  `fs.mqueue.msg_max` should still be raised (system setting, needs
  privileges).
* Stopping: cFE deletes the child tasks of an application when it exits.
  `MAPPROD_APP`'s main task therefore waits for its worker (which waits for
  the publisher) before `CFE_ES_ExitApp`. A process ended by a signal runs
  none of this; the command `FINALISE_ALL` ends a flight cleanly.

## Open Questions

None.
