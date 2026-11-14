# DENSE_APP

Depth maps of the keyframe batches (ADR-005, ADR-012). It takes the
batches of `BATCH_READY` (from `REFINE_APP`, or from `SELECT_APP` without
pose refinement), computes a depth set for each keyframe with
`libs/dense` (`OST_Dense_*`) and publishes it on `DEPTH_READY` for
`GEOREF_APP`. The default method is two-view SGBM on the CPU; PatchMatch on
the GPU is selected by the table.

## Messages

| Direction | Message | Use |
| --- | --- | --- |
| in | `OST` `BATCH_READY_TLM` | `KeyframeBatch`: working images, poses, working camera, footprint plane height |
| out | `DEPTH_READY_TLM` | depth set reference, keyframe (working image, JPEG, pose), camera, origin of `L_k`, depth range, valid and gated pixels, times |
| out | `DENSE_STATUS_TLM` | one per batch: done, skipped or failed, with the reason; queue depth and totals |
| out | `HK_TLM` | counters, last and largest times, last valid share, latency |

Commands: no-op (0), reset counters (1), clear latched faults (2).

## Depth set

A data store object of type `DEPTH_SET` (ADR-013): float32 maps of the
working camera's width × height pixels, row-major, native byte order, one
after the other:

1. depth: z in the keyframe camera, metres; 0 where there is no valid depth;
2. confidence: 0 to 1 (SGBM: 1 for matched pixels);
3. with `StoreNormals` only: normals, x, y, z per pixel, unit vectors in the
   keyframe camera.

At 2048 × 1366 a depth set without normals takes 11.2 MB.

## One job, newest wins

The input is the mission's one controlled drop point (ADR-015 section 3).
`DENSE_WORK` runs one job at a time; at most `MaxWaitingBatches` batches
wait behind it. A batch arriving when the queue is full replaces the
oldest waiting batch, which is skipped: its counts are released,
`KEYFRAMES_SKIPPED` is raised and `DENSE_STATUS` reports reason
`NEWER_BATCH`.

With pose refinement the batches arrive in bursts: `REFINE_APP` publishes
together all batches that one window solution covers (up to 7 at once on
Dorfberg), although on average one batch arrives per keyframe interval.
With one waiting batch, as ADR-015 first set it, most of each burst was
skipped (first run, 2026-11-14). The default is therefore 16, the
platform limit; 1 restores the rule of ADR-015 for operation without
pose refinement.

## A job

1. The working images (RGB or grey) are copied to grey; the counts of the
   working images are not needed for that longer.
2. A depth set is allocated. When the quota is full the job waits up to
   `QuotaWaitMs` (`DATASTORE_QUOTA`, reason `QUOTA` after that).
3. `OST_Dense_Estimate`: batch frame `L_k` (east-north-up at the keyframe's
   camera centre), terrain grid over the footprint, depth range from the
   terrain heights under the footprint ± `DepthMarginM`, SGBM or
   PatchMatch, terrain gate of `TerrainGateM` (ADR-005 section 5a). Without
   terrain under the keyframe the range follows the footprint plane height
   of the batch and the gate is skipped (`TERRAIN_UNAVAILABLE`).
4. The counts of the sources are released. The keyframe's working image
   and JPEG counts and the depth set count go to the consumer of
   `DEPTH_READY` (ADR-013 section 3).

A failed job releases all counts of the batch and reports the reason
(`DEPTH_FAILED`, or `GPU_FAILURE` when PatchMatch finds less free GPU
memory than `PmMinFreeGpuMb`; it is then not started, since COLMAP ends the
process on CUDA errors).

## Tasks

| Task | Work |
| --- | --- |
| main | commands, housekeeping; batches go into the queue (newest wins) |
| `DENSE_WORK` | one job at a time (priority level 4 of ADR-015) |

## Configuration

Table `DENSE_APP.ConfigTable` (`fsw/tables/dense_app_tbl.c`):

| Field | Default |
| --- | --- |
| `Method` | `SGBM` (`PATCHMATCH`) |
| `StoreNormals`, `MaxWaitingBatches`, `Threads`, `QuotaWaitMs` | 0, 16, 0 (OpenCV default), 2000 ms |
| `Sgbm*` | ADR-005 section 5: baseline ratio 0.2, block 5, P1 8·5², P2 32·5², uniqueness 10 %, left-right 1 px, speckle 100 px / 2, 512 disparities |
| `Pm*` | ADR-005 section 5: 3 iterations, radius 3, step 1, 7 samples, NCC 0.3, 2 consistent, 3°; 1024 MB free GPU memory |
| `DepthMarginM`, `TerrainGateM`, `GateSpacingM`, `GateMarginM` | 50 m, 30 m, 5 m, 100 m |
| `TerrainRadiusM`, `DemPath`, `DemCrs`, `ProjDataDir`, `GridDir` | 20 km and the files of `SELECT_APP` under `/cf/geodata` |

The module links `libs/dense` statically. A library build with PatchMatch
(`ORTHOSTREAM_WITH_COLMAP`, the default) makes the module depend on the
CUDA runtime of the system (ADR-010); a build without COLMAP has no CUDA
dependency.

## Events and faults

See [`docs/interfaces/events.md`](../../docs/interfaces/events.md) and
[`docs/interfaces/faults.md`](../../docs/interfaces/faults.md).
`tools/ground/dense_report.py` summarises a capture of `DENSE_STATUS` and
`DEPTH_READY`.
