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
| 20 | `DATASTORE_QUOTA` | `ERROR` | latch | any user of the data store | data store quota exhausted |
| 30 | `KEYFRAMES_SKIPPED` | `DEGRADED` | latch | `DENSE_APP` | keyframes skipped because of backlog |
| 31 | `GPU_FAILURE` | `CRITICAL` | latch | `DENSE_APP` | GPU or CUDA failure |
| 40 | `REFINE_REJECTED` | `WARNING` | latch | `REFINE_APP` | refinement result rejected (ADR-019 section 5) |
| 41 | `REFINE_LATENCY` | `DEGRADED` | latch | `REFINE_APP` | refinement over its latency limit |
| 50 | `TILE_WRITE` | `ERROR` | latch | `MAPPROD_APP` | tile write failure |
| 51 | `DISK_SPACE_LOW` | `WARNING` | auto-clear | `MAPPROD_APP`, `RECORDER_APP` | output disk space low |
| 60 | `TABLE_REJECTED` | `ERROR` | latch | any application | configuration table rejected |
| 61 | `LIBRARY_ERROR` | `ERROR` | latch | any application | library internal error |
