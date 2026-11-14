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

### `ALIGN_APP`

| Id | Name | Type | Meaning |
| --- | --- | --- | --- |
| 1 | `ALIGN_APP_INIT_INF_EID` | INFORMATION | initialised, with the alignment limits |
| 2 | `ALIGN_APP_NOOP_INF_EID` | INFORMATION | no-op command |
| 3 | `ALIGN_APP_RESET_INF_EID` | INFORMATION | counters reset |
| 4 | `ALIGN_APP_MID_ERR_EID` | ERROR | message with an unknown message identifier |
| 5 | `ALIGN_APP_CMD_LEN_ERR_EID` | ERROR | command of wrong length |
| 6 | `ALIGN_APP_CC_ERR_EID` | ERROR | unknown command code |
| 7 | `ALIGN_APP_PIPE_ERR_EID` | ERROR | Software Bus pipe read error; the application stops |
| 8 | `ALIGN_APP_TBL_ERR_EID` | ERROR | configuration table rejected or not loaded |
| 9 | `ALIGN_APP_CLEAR_INF_EID` | INFORMATION | latched faults cleared |
| 10 | `ALIGN_APP_INIT_ERR_EID` | ERROR | initialisation failed |
| 20 | `ALIGN_APP_SESSION_INF_EID` | INFORMATION | a new session: telemetry buffer and mounting dropped |
| 21 | `ALIGN_APP_CALIBRATION_INF_EID` | INFORMATION | camera mounting of the session set |
| 22 | `ALIGN_APP_CALIBRATION_ERR_EID` | ERROR | camera mounting of the session rejected |
| 23 | `ALIGN_APP_IMAGE_REJECTED_EID` | ERROR | image without a valid pose, with the reason (filtered) |
| 24 | `ALIGN_APP_MSG_LEN_ERR_EID` | ERROR | input message of wrong length (filtered) |
| 25 | `ALIGN_APP_DATASTORE_ERR_EID` | ERROR | JPEG reference not released (filtered) |
| 26 | `ALIGN_APP_LIB_ERR_EID` | ERROR | a call of `libs/time_align` failed (filtered) |
| 60 | fault `POSE_NO_TELEMETRY` | | an image was rejected for missing telemetry: before the buffer, in a gap, or no sample after it |
| 61 | fault `POSE_ACCURACY_LOW` | | the last valid pose was beyond `MaxPositionStdM` or `MaxAttitudeStdDeg` |
| 62 | fault `TABLE_REJECTED` | | the configuration table was not loaded |
| 63 | fault `LIBRARY_ERROR` | | a call of `libs/time_align` failed |

### `SELECT_APP`

| Id | Name | Type | Meaning |
| --- | --- | --- | --- |
| 1 | `SELECT_APP_INIT_INF_EID` | INFORMATION | initialised: working image size, K, causal or delayed sources, destination of the batches |
| 2 | `SELECT_APP_NOOP_INF_EID` | INFORMATION | no-op command |
| 3 | `SELECT_APP_RESET_INF_EID` | INFORMATION | counters reset |
| 4 | `SELECT_APP_MID_ERR_EID` | ERROR | message with an unknown message identifier |
| 5 | `SELECT_APP_CMD_LEN_ERR_EID` | ERROR | command of wrong length |
| 6 | `SELECT_APP_CC_ERR_EID` | ERROR | unknown command code |
| 7 | `SELECT_APP_PIPE_ERR_EID` | ERROR | Software Bus pipe read error; the application stops |
| 8 | `SELECT_APP_TBL_ERR_EID` | ERROR | configuration table rejected or not loaded |
| 9 | `SELECT_APP_CLEAR_INF_EID` | INFORMATION | latched faults cleared |
| 10 | `SELECT_APP_INIT_ERR_EID` | ERROR | initialisation failed |
| 20 | `SELECT_APP_SESSION_INF_EID` | INFORMATION | session calibration applied: camera and working image size |
| 21 | `SELECT_APP_CALIBRATION_ERR_EID` | ERROR | working image converter not created for the session camera |
| 22 | `SELECT_APP_TERRAIN_ERR_EID` | ERROR | terrain prior not usable, or no terrain at an image (filtered) |
| 23 | `SELECT_APP_DECODE_ERR_EID` | ERROR | image not decoded to the working resolution (filtered) |
| 24 | `SELECT_APP_QUOTA_ERR_EID` | ERROR | working image not stored after evicting pool images (filtered) |
| 25 | `SELECT_APP_QUEUE_FULL_EID` | ERROR | image or calibration dropped: worker queue full (filtered) |
| 26 | `SELECT_APP_MSG_LEN_ERR_EID` | ERROR | input message of wrong length (filtered) |
| 27 | `SELECT_APP_LIB_ERR_EID` | ERROR | a call of `libs/selection` failed (filtered) |
| 28 | `SELECT_APP_KEYFRAME_DBG_EID` | DEBUG | keyframe: image, flight line, overlap, sources, plane height |
| 29 | `SELECT_APP_DATASTORE_ERR_EID` | ERROR | data store reference not released (filtered) |
| 60 | fault `TERRAIN_UNAVAILABLE` | | elevation model or PROJ data not usable, or no terrain at the last image |
| 61 | fault `IMAGE_DECODE` | | an image could not be decoded |
| 62 | fault `DATASTORE_QUOTA` | | a working image could not be stored |
| 63 | fault `TABLE_REJECTED` | | the configuration table was not loaded |
| 64 | fault `LIBRARY_ERROR` | | a call of `libs/selection` or `libs/imaging` failed |

### `REFINE_APP`

| Id | Name | Type | Meaning |
| --- | --- | --- | --- |
| 1 | `REFINE_APP_INIT_INF_EID` | INFORMATION | initialised: precise position, window, waiting and latency limits |
| 2 | `REFINE_APP_NOOP_INF_EID` | INFORMATION | no-op command |
| 3 | `REFINE_APP_RESET_INF_EID` | INFORMATION | counters reset |
| 4 | `REFINE_APP_MID_ERR_EID` | ERROR | message with an unknown message identifier |
| 5 | `REFINE_APP_CMD_LEN_ERR_EID` | ERROR | command of wrong length |
| 6 | `REFINE_APP_CC_ERR_EID` | ERROR | unknown command code |
| 7 | `REFINE_APP_PIPE_ERR_EID` | ERROR | Software Bus pipe read error; the application stops |
| 8 | `REFINE_APP_TBL_ERR_EID` | ERROR | configuration table rejected or not loaded |
| 9 | `REFINE_APP_CLEAR_INF_EID` | INFORMATION | latched faults cleared |
| 10 | `REFINE_APP_INIT_ERR_EID` | ERROR | initialisation failed |
| 11 | `REFINE_APP_FLUSH_INF_EID` | INFORMATION / ERROR | flush command accepted, or refused with a full queue |
| 20 | `REFINE_APP_SESSION_INF_EID` | INFORMATION | new session: waiting batches solved, window cleared |
| 21 | `REFINE_APP_FAILED_EID` | ERROR | batch passed on with telemetry poses (`REFINE_FAILED`), with the reason (filtered) |
| 22 | `REFINE_APP_LATENCY_EID` | ERROR | batch published later than `MaxLatencyS` after its capture (filtered) |
| 23 | `REFINE_APP_QUEUE_FULL_EID` | ERROR | batch passed on unrefined: queue full or no table (filtered) |
| 24 | `REFINE_APP_MSG_LEN_ERR_EID` | ERROR | input message of wrong length (filtered) |
| 25 | `REFINE_APP_LIB_ERR_EID` | ERROR | a call of `libs/refine` failed (filtered) |
| 26 | `REFINE_APP_DATASTORE_ERR_EID` | ERROR | data store reference not released (filtered) |
| 27 | `REFINE_APP_BATCH_DBG_EID` | DEBUG | solved batch: covered or waiting limit, waiting time, RMS, window, points, time |
| 28 | `REFINE_APP_HELD_FULL_EID` | ERROR | all hold slots in use: every waiting batch solved at once (filtered) |
| 60 | fault `REFINE_REJECTED` | | a batch failed the checks or could not be refined |
| 61 | fault `REFINE_LATENCY` | | a batch was published later than `MaxLatencyS` |
| 62 | fault `TABLE_REJECTED` | | the configuration table was not loaded |
| 63 | fault `LIBRARY_ERROR` | | a call of `libs/refine` failed |

Filtered events are sent for the first 8 occurrences; the counts are in
the housekeeping.

### `DENSE_APP`

| Id | Name | Type | Meaning |
| --- | --- | --- | --- |
| 1 | `DENSE_APP_INIT_INF_EID` | INFORMATION | initialised: method, waiting batches, terrain gate |
| 2 | `DENSE_APP_NOOP_INF_EID` | INFORMATION | no-op command |
| 3 | `DENSE_APP_RESET_INF_EID` | INFORMATION | counters reset |
| 4 | `DENSE_APP_MID_ERR_EID` | ERROR | message with an unknown message identifier |
| 5 | `DENSE_APP_CMD_LEN_ERR_EID` | ERROR | command of wrong length |
| 6 | `DENSE_APP_CC_ERR_EID` | ERROR | unknown command code |
| 7 | `DENSE_APP_PIPE_ERR_EID` | ERROR | Software Bus pipe read error; the application stops |
| 8 | `DENSE_APP_TBL_ERR_EID` | ERROR | configuration table rejected or not loaded |
| 9 | `DENSE_APP_CLEAR_INF_EID` | INFORMATION | latched faults cleared |
| 10 | `DENSE_APP_INIT_ERR_EID` | ERROR | initialisation failed |
| 20 | `DENSE_APP_SKIPPED_EID` | ERROR / INFORMATION | batch skipped: replaced by a newer batch (newest wins) or no depth set; INFORMATION for a batch without sources, the first keyframe of a flight (filtered) |
| 21 | `DENSE_APP_FAILED_EID` | ERROR | batch failed: no estimator, invalid batch, estimation error, or `DEPTH_READY` not sent (filtered) |
| 22 | `DENSE_APP_QUOTA_EID` | ERROR | no depth set within `QuotaWaitMs` (filtered) |
| 23 | `DENSE_APP_MSG_LEN_ERR_EID` | ERROR | input message of wrong length (filtered) |
| 24 | `DENSE_APP_LIB_ERR_EID` | ERROR | the estimator could not be created (filtered) |
| 25 | `DENSE_APP_DATASTORE_ERR_EID` | ERROR | data store reference not released (filtered) |
| 26 | `DENSE_APP_BATCH_DBG_EID` | DEBUG | depth set published: valid and gated share, depth range, times |
| 27 | `DENSE_APP_TERRAIN_EID` | ERROR | terrain prior not usable, or none under a keyframe: depth range from the footprint plane, no gate (filtered) |
| 28 | `DENSE_APP_GPU_EID` | ERROR | PatchMatch not started: GPU not available or not enough free memory (filtered) |
| 29 | `DENSE_APP_READY_INF_EID` | INFORMATION | estimator created: method, threads, terrain prior |
| 60 | fault `KEYFRAMES_SKIPPED` | | a keyframe was skipped because of backlog |
| 61 | fault `DEPTH_FAILED` | | depth estimation failed for a keyframe |
| 62 | fault `GPU_FAILURE` | | GPU not available or not enough free GPU memory |
| 63 | fault `DATASTORE_QUOTA` | | no depth set within the quota waiting limit |
| 64 | fault `TERRAIN_UNAVAILABLE` | | no terrain prior (files, or under the keyframe) |
| 65 | fault `TABLE_REJECTED` | | the configuration table was not loaded, or names a method that is not built |
| 66 | fault `LIBRARY_ERROR` | | the estimator could not be created |

Filtered events are sent for the first 8 occurrences; the counts are in
the housekeeping and in `DENSE_STATUS`.
