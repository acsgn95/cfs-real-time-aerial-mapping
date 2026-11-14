# REFINE_APP

Pose refinement of the keyframe batches (ADR-019). It takes the batches of
`SELECT_APP` from `BATCH_SELECTED`, refines the poses of the keyframe and
its sources with the sliding window of `libs/refine` (`OST_Refine_*`), and
publishes the batch unchanged otherwise on `BATCH_READY` with pose source
`REFINED`. A batch is never dropped: when refinement fails its checks, or
the batch cannot be queued, it is published with its telemetry poses and
pose source `REFINE_FAILED` (section 5).

## Messages

| Direction | Message | Use |
| --- | --- | --- |
| in | `OST` `BATCH_SELECTED_TLM` | `KeyframeBatch` with working images and telemetry poses |
| in | `INGEST_APP` `SessionCalibration` | a new session: waiting batches are solved, the window is cleared |
| out | `OST` `BATCH_READY_TLM` | the same batch, refined poses, pose source `REFINED` or `REFINE_FAILED` |
| out | `HK_TLM` | batches received, held, refined, failed, covered or by waiting limit; latency; last solution (RMS, boresight, keyframe correction, points, time) |

Commands: no-op (0), reset counters (1), clear latched faults (2), flush
(3: solve and publish all waiting batches now).

The batch's data store counts (one per working image, the keyframe's JPEG)
go on with the published batch. `REFINE_APP` reads the working images under
them (RGB working images are converted to grey for the features).

## Waiting

A batch is solved when an image of a later flight line, flying the other
way, covers its keyframe's footprint centre, or `MaxWaitS` after its
capture (ADR-019 section 3, item 4). The waiting limit is checked every
0.5 s on UTC data time (ADR-014). Up to 64 batches are held; when all slots
are in use every waiting batch is solved at once. A batch published later
than `MaxLatencyS` after its capture raises `REFINE_LATENCY`.

## Tasks

| Task | Work |
| --- | --- |
| main | commands, housekeeping; batches and session changes go into the worker queue (16 items) |
| `REFINE_WORK` | features, sliding-window adjustment, publication (ADR-019 section 6) |

## Configuration

Table `REFINE_APP.ConfigTable` (`fsw/tables/refine_app_tbl.c`); the
reference mission replaces it with `mission/orthostream_defs/tables/refine_app_tbl.c`,
which sets `PrecisePosition` for the PPK positions of Dorfberg.

| Field | Default |
| --- | --- |
| `PrecisePosition`, `EstimateBoresight` | 0 (mission: 1), 1 |
| `MaxLatencyS`, `MaxWaitS`, `WindowS` | 120 s, 100 s, 240 s |
| `MaxWindowImages`, `MaxPoints`, `MaxIterations`, `MaxFeatures` | 16, 2000, 30, 4000 |
| `PositionSigmaM`, `PrecisePositionSigmaM`, `AttitudeSigmaDeg` | 2 m, 0.02 m, 3° (when the image reports none) |
| `BoresightSigmaDeg`, `BoresightSettleBatches`, `HeldBoresightSigmaDeg` | 5°, 4, 0.005° |
| `RefinedAttitudeSigmaDeg` | 0.02° |
| `MinImageObservations`, `MaxReprojectionRmsPx`, `MaxCorrectionSigmas` | 50, 2 px, 5 |

## Events and faults

See [`docs/interfaces/events.md`](../../docs/interfaces/events.md) and
[`docs/interfaces/faults.md`](../../docs/interfaces/faults.md).
`tools/ground/compare_batch_poses.py` compares the poses before and after
refinement with reference poses.
