# RECORDER_APP

Measurement records of a run (ADR-012, ADR-017). It subscribes to the
messages of its table (the data path messages with their trace times,
housekeeping, fault events and EVS event messages), encodes each as on the
ground link and writes it with its receive time to the recording of the
current run. No bulk data is recorded (ADR-001): images, depth sets and
point sets stay in the data store.

## A run

`<OutputDir>/<RunId>/` (default `/cf/runs`, that is `build/exe/cpu1/cf/runs`
on the host):

| File | Content |
| --- | --- |
| `recording.ostrec` | header (64 bytes: `OSTREC\0\0`, header size, format 1, start UTC, run identifier) and records: `uint32` length, `int64` receive UTC in ns (little-endian), the message EDS-packed (CCSDS, big-endian), the bytes `TO_LAB` would send |
| `manifest.json` | run identifier, state (`recording`, `stopped`), start and stop UTC, records, bytes, dropped, encoding and write errors, longest write, largest buffer fill, subscribed message identifiers |

`tools/ground/recorder_export.py RUN_DIR` writes the files of ADR-017
section 4 next to it: `csv/<PACKET>.csv` (one per message), `events.log`
and `recording.jsonl` (the format of `tools/ground/tlm_capture.py`, so the
report tools `dense_report.py`, `georef_report.py`, `mapprod_report.py`
read a recording directly). It decodes with the generated COSMOS
definitions and reports a recording that ends in the middle of a record
(the process was killed). The ground tools add what the flight software
does not know: git commit, injector parameters, sampler output.

With `AutoStart` the recording starts with the application, with a run
identifier from the UTC time (`run_YYYYmmdd_HHMMSS`). A run directory that
exists is never overwritten.

## Commands

| Code | Command |
| --- | --- |
| 0, 1, 2 | no-op, reset command counters, clear latched faults |
| 3 | start recording, `RunId` (letters, digits, `-`, `_`, `.`; at most 19 characters, since the run directory is one path component and OSAL limits those to `OS_MAX_FILE_NAME` - 1; empty: from the UTC time); ends a running recording first |
| 4 | stop recording |

`tools/ground/send_command.py RECORDER_APP_CMD_START_RECORDING --param RUN_ID=flight_01`
and `... RECORDER_APP_CMD_STOP_RECORDING` send them without COSMOS.

## Tasks and pipes

| Task | Work |
| --- | --- |
| main | takes the messages off the pipes, encodes them (EDS runtime, as `TO_LAB`) and appends them to a 4 MB buffer; commands, housekeeping, free space every second |
| `REC_WRITE` | writes the buffer to the recording |

A write may block under memory pressure; the pipes are still emptied, and a
message that finds the buffer full is counted (`Dropped`). In the first
runs no message was dropped and the longest write took 0.1 ms.

On Linux, OSAL truncates every Software Bus pipe to
`/proc/sys/fs/mqueue/msg_max` messages (10 without privileges), whatever
depth the application asks for. `SCH_LAB` requests all housekeeping in the
same tick, so about 15 messages arrive at once and a single pipe overflowed
(first smoke test, 2026-11-16). The subscriptions are therefore spread over
three pipes (subscription i on pipe i mod 3); the main task empties all of
them and waits on the first when all are empty (20 ms).

## Disk space and limits

Every second the free space of the output volume is checked:
`DISK_SPACE_LOW` below `WarnFreeMb`; below `MinFreeMb` the recording stops.
It also stops at `MaxRecordingMb` (0: no limit) and on a write failure
(`RECORDING_FAILED`).

## Configuration

Table `RECORDER_APP.ConfigTable` (`fsw/tables/recorder_app_tbl.c`, file
`recorder_tbl.tbl`):

| Field | Default |
| --- | --- |
| `OutputDir`, `AutoStart` | `/cf/runs`, 1 |
| `MinFreeMb`, `WarnFreeMb`, `MaxRecordingMb` | 1024, 10240, 0 |
| `Subscriptions` | message identifier and pipe limit, up to `RECORDER_APP/MAX_SUBSCRIPTIONS` (40); cFE housekeeping and EVS events, `OST` fault events and keyframe batches, every OrthoStream data path message and housekeeping |

The module uses no OrthoStream library: the EDS runtime of the mission
encodes the messages.

## Events and faults

See [`docs/interfaces/events.md`](../../docs/interfaces/events.md) and
[`docs/interfaces/faults.md`](../../docs/interfaces/faults.md).
