# Event Identifiers

The cFE event (EVS) identifiers of the OrthoStream applications
(ADR-020 section 3).

* Each application defines its identifiers in
  `apps/<app>/fsw/inc/<app>_eventids.h`.
* Identifiers are never reused or renumbered.
* `tests/interfaces/check_interfaces.py` checks that the identifiers of each
  application are unique, lie in their range and are listed here.

## Ranges

| Range | Use |
| --- | --- |
| 1–19 | life cycle and commands: initialisation, no-op, reset, command errors, pipe errors, table load and validation |
| 20–59 | data path: state changes, dropped items with their reason, rejected poses or batches |
| 60–79 | fault changes, raised by the fault set of `ost_common`: identifier 60 + index of the fault in the application's list |
| 80–99 | library diagnostics: 80 debug, 81 warning, 82 error |

## Types

* `CRITICAL`: the application cannot continue its function.
* `ERROR`: an item or a request failed.
* `INFORMATION`: state changes and command acknowledgements.
* `DEBUG`: diagnostics, disabled by default.

Events are raised on changes of state, not per item. Per-item counts go to
housekeeping. Events that can repeat are limited with EVS binary filters.

## Applications

The sections below are added with the applications. Each lists the
identifiers of `<app>_eventids.h` in a table with the columns Id, Name, Type
and Meaning; the fault events list the application's faults in order.

### `INGEST_APP`

| Id | Name | Type | Meaning |
| --- | --- | --- | --- |
| 1 | `INGEST_APP_INIT_INF_EID` | INFORMATION | initialised, with ports and session state |
| 2 | `INGEST_APP_NOOP_INF_EID` | INFORMATION | no-op command |
| 3 | `INGEST_APP_RESET_INF_EID` | INFORMATION | counters reset |
| 4 | `INGEST_APP_MID_ERR_EID` | ERROR | message with an unknown message identifier |
| 5 | `INGEST_APP_CMD_LEN_ERR_EID` | ERROR | command of wrong length |
| 6 | `INGEST_APP_CC_ERR_EID` | ERROR | unknown command code |
| 7 | `INGEST_APP_PIPE_ERR_EID` | ERROR | Software Bus pipe read error; the application stops |
| 8 | `INGEST_APP_TBL_ERR_EID` | ERROR | configuration table rejected or not loaded |
| 9 | `INGEST_APP_CLEAR_INF_EID` | INFORMATION | latched faults cleared |
| 10 | `INGEST_APP_INIT_ERR_EID` | ERROR | initialisation failed |
| 11 | `INGEST_APP_CMD_REFUSED_EID` | ERROR | start session refused: no valid configuration |
| 20 | `INGEST_APP_START_INF_EID` | INFORMATION | start session: sockets opened |
| 21 | `INGEST_APP_STOP_INF_EID` | INFORMATION | stop session: sockets closed |
| 22 | `INGEST_APP_SOCKET_ERR_EID` | ERROR | socket not opened or receive error (filtered) |
| 23 | `INGEST_APP_CONNECTED_INF_EID` | INFORMATION | image connection accepted |
| 24 | `INGEST_APP_DISCONNECTED_INF_EID` | INFORMATION / ERROR | image connection closed; ERROR if a session was active |
| 25 | `INGEST_APP_SESSION_INF_EID` | INFORMATION | session started: camera, image size, time factor |
| 26 | `INGEST_APP_SESSION_END_INF_EID` | INFORMATION | session ended: images sent, received and dropped, telemetry received and lost |
| 27 | `INGEST_APP_IMAGE_DROPPED_EID` | ERROR | image not stored in the data store (filtered) |
| 28 | `INGEST_APP_FRAME_REJECTED_EID` | ERROR | TCP frame rejected; connection closed (filtered) |
| 29 | `INGEST_APP_TLM_REJECTED_EID` | ERROR | telemetry datagram rejected (filtered) |
| 60 | fault `TELEMETRY_STALE` | | no telemetry sample for `TelemetryStaleMs` during a session |
| 61 | fault `IMAGE_STREAM_STALE` | | no image for `ImageStaleMs` during a session |
| 62 | fault `LINK_FORMAT` | | a datagram or frame failed the checks of `libs/link` |
| 63 | fault `DATASTORE_QUOTA` | | an image could not be stored |
| 64 | fault `TABLE_REJECTED` | | the configuration table was not loaded |

Filtered events are sent for the first 8 occurrences; the counts are in
the housekeeping.
