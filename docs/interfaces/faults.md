# Fault Catalog

The faults of the OrthoStream mission (ADR-020 section 4).

* The identifiers are the EDS enumeration `OST/FaultId` in
  [`apps/ost_common/eds/ost.xml`](../../apps/ost_common/eds/ost.xml).
* Severity and policy are fixed in
  [`ost_fault_catalog.c`](../../apps/ost_common/fsw/src/ost_fault_catalog.c).
* `tests/interfaces/check_interfaces.py` checks that this table, the EDS
  enumeration and the C catalog agree.
* Identifiers are never reused or renumbered.

## States and policies

```text
CLEARED --raise--> ACTIVE --condition gone--> CLEARED   (policy auto-clear)
                          --condition gone--> LATCHED   (policy latch)
CLEARED --momentary occurrence--> LATCHED
LATCHED --raise--> ACTIVE
LATCHED --command "clear latched faults"--> CLEARED
```

* Every change of state is published once as an `OST_FAULT_EVENT` message and
  raised once as an EVS event of the application (event identifiers 60–79).
* Further occurrences while a fault is `ACTIVE` or `LATCHED` only increase its
  occurrence count.
* Faults detected per item (an image, a keyframe, a tile) are momentary
  occurrences and have the policy latch.
* The housekeeping of every application carries the number of active and
  latched faults and the highest severity among them (`OST/HkCommon`).

## Catalog

| Id | Name | Severity | Policy | Raised by | Meaning |
| --- | --- | --- | --- | --- | --- |
| 1 | `TELEMETRY_STALE` | `ERROR` | auto-clear | `INGEST_APP` | no telemetry sample within the stale limit |
| 2 | `IMAGE_STREAM_STALE` | `ERROR` | auto-clear | `INGEST_APP` | no image within the stale limit during a session |
| 3 | `LINK_FORMAT` | `CRITICAL` | latch | `INGEST_APP` | link format or version mismatch |
| 10 | `POSE_NO_TELEMETRY` | `WARNING` | latch | `ALIGN_APP` | no telemetry around an image (no pose) |
| 11 | `POSE_ACCURACY_LOW` | `DEGRADED` | auto-clear | `ALIGN_APP` | pose accuracy below the configured limit |
| 15 | `TERRAIN_UNAVAILABLE` | `ERROR` | auto-clear | `SELECT_APP`, `DENSE_APP` | no terrain prior at the image position: the elevation model or PROJ data could not be opened, or does not cover the flight (no footprints; in `DENSE_APP` no terrain gate) |
| 16 | `IMAGE_DECODE` | `ERROR` | latch | `SELECT_APP` | an image could not be decoded to the working resolution |
| 20 | `DATASTORE_QUOTA` | `ERROR` | latch | any user of the data store | data store quota exhausted |
| 30 | `KEYFRAMES_SKIPPED` | `DEGRADED` | latch | `DENSE_APP` | keyframes skipped because of backlog |
| 31 | `GPU_FAILURE` | `CRITICAL` | latch | `DENSE_APP` | GPU or CUDA failure, or less free GPU memory than PatchMatch needs (it is then not started) |
| 32 | `DEPTH_FAILED` | `ERROR` | latch | `DENSE_APP` | depth estimation failed for a keyframe |
| 33 | `POINTS_FAILED` | `ERROR` | latch | `GEOREF_APP` | a depth set was dropped: georeferencing failed, no point set within the quota wait (ADR-015 section 3), or POINTS_READY not sent |
| 34 | `MAP_UPDATE_FAILED` | `ERROR` | latch | `MAPPROD_APP` | a point set was dropped: fusion failed, unreadable point set, tile grid or CRS different from `GEOREF_APP`, or a full queue |
| 35 | `RECORDING_FAILED` | `ERROR` | latch | `RECORDER_APP` | a run could not be recorded: run directory or recording file not created, or a write failed (recording stops) |
| 40 | `REFINE_REJECTED` | `WARNING` | latch | `REFINE_APP` | refinement result rejected (ADR-019 section 5) |
| 41 | `REFINE_LATENCY` | `DEGRADED` | latch | `REFINE_APP` | refinement over its latency limit |
| 50 | `TILE_WRITE` | `ERROR` | latch | `MAPPROD_APP` | tile write failure |
| 51 | `DISK_SPACE_LOW` | `WARNING` | auto-clear | `MAPPROD_APP`, `RECORDER_APP` | output disk space low |
| 60 | `TABLE_REJECTED` | `ERROR` | latch | any application | configuration table rejected |
| 61 | `LIBRARY_ERROR` | `ERROR` | latch | any application | library internal error |
