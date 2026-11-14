# INGEST_APP

Source adapter for the injector link (ADR-003, ADR-012 section 1a). It
receives autopilot telemetry on UDP and images on TCP, in the format of
[`docs/interfaces/link.md`](../../docs/interfaces/link.md), and publishes the
OrthoStream input messages. From `ALIGN_APP` onwards no application knows
the source.

## Messages

| Interface | Message | When |
| --- | --- | --- |
| `TELEMETRY_SAMPLE_TLM` | `TelemetrySample`: geodetic position, `q_N_B`, accuracy, measurement and recorded time | each valid datagram |
| `IMAGE_ARRIVED_TLM` | `ImageArrived`: capture time, image sequence number, JPEG reference in the data store, size | each stored image |
| `SESSION_CALIBRATION_TLM` | `SessionCalibration`: camera model, `T_B_C`, time factor | at session start |
| `HK_TLM` | housekeeping: session state, counters, ages of the last sample and image, faults | on `SEND_HK` |

`ImageArrived` carries one reference count of its JPEG object, owned by the
consumer (`ALIGN_APP`, ADR-013 section 3).

Commands: no-op (0), reset counters (1), clear latched faults (2), start
session (3, opens the sockets), stop session (4, closes them).

## Tasks

| Task | Work |
| --- | --- |
| main | commands, housekeeping, 1 Hz wake-up: staleness faults |
| `INGEST_UDP` | telemetry datagrams: decode, count sequence gaps, publish |
| `INGEST_TCP` | image connection: session start and end, image frames |

* Images are read from the socket directly into the data store, without a
  copy.
* An image that cannot be stored is read and discarded, counted, and
  reported as `DATASTORE_QUOTA`.
* A frame that fails the checks of `libs/link` closes the connection,
  because the frame boundaries can no longer be trusted. A rejected
  datagram is only counted. Both raise `LINK_FORMAT`.
* One connection is one session: `SESSION_START`, images, `SESSION_END`.

## Configuration

Table `INGEST_APP.ConfigTable` (`fsw/tables/ingest_app_tbl.c`), loaded once
at startup (ADR-016 section 5):

| Field | Default | Meaning |
| --- | --- | --- |
| `UdpPort`, `TcpPort` | 50100, 50101 | ports on all interfaces |
| `TelemetryStaleMs` | 1000 | telemetry gap that raises `TELEMETRY_STALE` during a session |
| `ImageStaleMs` | 30000 | image gap that raises `IMAGE_STREAM_STALE`; the turns of Dorfberg Flight 01 last up to 25 s |
| `AutoStart` | 1 | open the sockets at startup |

Without a valid table the application stays up, raises `TABLE_REJECTED` and
refuses to start a session.

Platform values (`config/default_ingest_app_platform_cfg.h`): pipe depth,
receiver task priority and stack, socket timeout.

## Events and faults

See [`docs/interfaces/events.md`](../../docs/interfaces/events.md) and
[`docs/interfaces/faults.md`](../../docs/interfaces/faults.md).

## Trying it

```bash
mission/build.sh
cd build/exe/cpu1 && ./core-cpu1          # in one terminal
cd tools/injector && python3 -m orthostream_injector \
    --poses ../../data/processed/dorfberg_f01/poses_optimized.txt \
    --captures ../../data/processed/dorfberg_f01/captures.txt \
    --images <image directory> --first 60 --count 20
build/exe/host/cmd_send -D INGEST_APP/Application/CMD.StopSessionCmd
```

The session start and end events report images sent, received and
dropped, and telemetry received and lost.
