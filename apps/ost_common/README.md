# ost_common

Shared interface library of the OrthoStream cFS applications (ADR-020).
It is a cFS library module (`CFE_LIB`) and is loaded before the applications:

```text
CFE_LIB, ost_common, OST_Common_Init, OST_COMMON, 0, 0, 0x0, 0;
```

## Content

| File | Content |
| --- | --- |
| `eds/ost.xml` | EDS package `OST`: payload header, times, data store references, poses with accuracy, tile identifier, common housekeeping part, fault catalog enumeration, `FaultEvent`, keyframe batch; the shared topics `FAULT_EVENT_TLM`, `BATCH_SELECTED_TLM`, `BATCH_READY_TLM` |
| `fsw/public_inc/ost_common.h` | entry point, UTC and monotonic time, message identifiers of the shared topics, text copy, mapping of library diagnostics to EVS events |
| `fsw/public_inc/ost_fault.h` | fault catalog lookup and fault sets |
| `../../interfaces/include/ost_status.h` | status codes of the library C interfaces (no cFE dependency) |
| `../../interfaces/include/ost_diag.h` | diagnostic callback of the libraries (no cFE dependency) |

The EDS build generates the C types into `ost_eds_typedefs.h` and the COSMOS
definitions `build/cosmos/ost_eds_*_def.txt`.

## Requirements of the host mission

* Built with `CFE_EDS_ENABLED=ON`.
* Design parameters in its EDS configuration:
  * package `OST`: `MAX_SOURCE_VIEWS`, `FAULT_TEXT_LEN`;
  * package `CFE_MISSION`: `OST_FAULT_EVENT_TLM_TOPICID`,
    `OST_BATCH_SELECTED_TLM_TOPICID`, `OST_BATCH_READY_TLM_TOPICID`, and the
    topic identifiers of the applications it uses.

The reference mission defines them in `mission/orthostream_defs/eds/`.

## Fault sets

An application lists the faults it can raise and keeps one fault set in its
main task:

```c
static const OST_FaultId_Enum_t FAULTS[] = {OST_FaultId_TELEMETRY_STALE, OST_FaultId_LINK_FORMAT};

OST_FaultSet_Init(&Data.Faults, OST_AppId_INGEST_APP, FAULTS, 2);
OST_FaultSet_Raise(&Data.Faults, OST_FaultId_TELEMETRY_STALE, "no sample for 2.0 s");
OST_FaultSet_Resolve(&Data.Faults, OST_FaultId_TELEMETRY_STALE);
OST_FaultSet_Summarise(&Data.Faults, &Data.HkTlm.Payload.Common);
```

The position in the list gives the EVS event identifier (60 + index), so the
list only grows at its end. The catalog is documented in
[`docs/interfaces/faults.md`](../../docs/interfaces/faults.md).
