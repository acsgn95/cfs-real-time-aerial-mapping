# Injector Link Format

The link between the injector and `INGEST_APP` (ADR-003, ADR-016 section 7).
It emulates the radio link of a survey aircraft: autopilot telemetry on UDP,
images on TCP.

* The definition is
  [`interfaces/link/ost_link_format.h`](../../interfaces/link/ost_link_format.h).
* The injector encodes it in
  [`link.py`](../../tools/injector/orthostream_injector/link.py).
  `libs/link` decodes and validates it for `INGEST_APP`.
* `tests/interfaces/check_interfaces.py` checks that the sizes, offsets and
  type values of the C header and the Python encoder agree.
* [`interfaces/link/test_vectors.txt`](../../interfaces/link/test_vectors.txt)
  holds one encoded message of each type. The Python tests check that the
  encoder still produces them. `libs/link/tests` decodes them and checks the
  values.
* A change of layout increases the format version. Readers reject other
  versions.

## Conventions

| Item | Convention |
| --- | --- |
| Byte order | little-endian, packed, no padding between fields |
| Time | `int64` nanoseconds since the Unix epoch, UTC. Data time is the emulated time of the measurement (ADR-014 section 1). Recorded time is the same instant in the recording. |
| Position | WGS 84 latitude and longitude in degrees, ellipsoidal height in metres |
| Attitude | Hamilton quaternion `q_N_B` (w, x, y, z), from body FRD to local NED (ADR-004) |
| Camera | OpenCV pinhole model with `k1 k2 k3 p1 p2`; the centre of the top-left pixel is at (0, 0). `T_B_C` is given as `q_B_C` and the camera position in the body frame. |
| Accuracy | one standard deviation; position north, east, down in metres; attitude roll, pitch, yaw in degrees |
| Position fix | values of the EDS enumeration `OST/PositionFix` (0 unknown … 5 RTK fixed, 6 PPK) |

## Header (16 bytes)

| Offset | Type | Field |
| --- | --- | --- |
| 0 | 4 bytes | magic word `OSTL` |
| 4 | `uint16` | format version, 1 |
| 6 | `uint16` | message type |
| 8 | `uint32` | payload length in bytes, after the header |
| 12 | `uint32` | sequence number, per channel, +1 per message |

| Type | Name | Channel | Payload length |
| --- | --- | --- | --- |
| 1 | `TELEMETRY` | UDP | 104 |
| 2 | `HEARTBEAT` | UDP | 24 |
| 16 | `SESSION_START` | TCP | 200 |
| 17 | `IMAGE` | TCP | 32 + image size |
| 18 | `SESSION_END` | TCP | 16 |

## UDP: telemetry channel

One message per datagram. The datagram size must equal header plus payload
length.

### `TELEMETRY` (104 bytes)

| Offset | Type | Field |
| --- | --- | --- |
| 0 | `int64` | measurement time (data time) |
| 8 | `int64` | recorded time |
| 16 | `float64` | latitude, deg |
| 24 | `float64` | longitude, deg |
| 32 | `float64` | ellipsoidal height, m |
| 40 | 4 × `float64` | `q_N_B` w, x, y, z |
| 72 | 3 × `float32` | position accuracy north, east, down, m |
| 84 | 3 × `float32` | attitude accuracy roll, pitch, yaw, deg |
| 96 | `uint8` | position fix |
| 97 | `uint8` | flags, 0 |
| 98 | `uint16` | reserved, 0 |
| 100 | `uint32` | session identifier |

### `HEARTBEAT` (24 bytes, once per second)

| Offset | Type | Field |
| --- | --- | --- |
| 0 | `int64` | time of sending |
| 8 | `uint32` | session identifier |
| 12 | `uint32` | telemetry samples sent so far |
| 16 | `uint32` | images sent so far |
| 20 | `uint32` | reserved, 0 |

## TCP: imagery channel

The injector connects to `INGEST_APP`. On one connection it sends one
`SESSION_START`, then one `IMAGE` frame per image, then `SESSION_END`. A
frame is the header followed by its payload. A reader that finds a wrong
magic word has lost the frame boundaries and closes the connection.

### `SESSION_START` (200 bytes)

| Offset | Type | Field |
| --- | --- | --- |
| 0 | `uint32` | session identifier |
| 4 | `uint32` | reserved, 0 |
| 8 | `int64` | data time of the recording start |
| 16 | `int64` | recording start (recorded time of the first image) |
| 24 | `float64` | time factor (1 = real time) |
| 32 | `uint32` | image width, px |
| 36 | `uint32` | image height, px |
| 40 | 4 × `float64` | `fx fy cx cy`, px |
| 72 | 5 × `float64` | `k1 k2 k3 p1 p2` |
| 112 | 4 × `float64` | `q_B_C` w, x, y, z |
| 144 | 3 × `float64` | camera position in the body frame, m |
| 168 | 32 × `char` | camera name, NUL padded |

### `IMAGE` (32 bytes, then the image data)

| Offset | Type | Field |
| --- | --- | --- |
| 0 | `uint32` | session identifier |
| 4 | `uint32` | image sequence number |
| 8 | `int64` | capture time (data time) |
| 16 | `int64` | capture time in the recording |
| 24 | `uint8` | format, 1 = JPEG |
| 25 | 3 × `uint8` | reserved, 0 |
| 28 | `uint32` | image data size; header length = 32 + size |
| 32 | bytes | image data, as recorded (not re-encoded) |

`libs/link` accepts at most 256 MiB of image data per frame.

### `SESSION_END` (16 bytes)

| Offset | Type | Field |
| --- | --- | --- |
| 0 | `uint32` | session identifier |
| 4 | `uint32` | images sent |
| 8 | `int64` | time of sending |

## Validation in `libs/link`

A message is rejected (`OST_ERR_FORMAT`) if any of these holds:

* wrong magic word, unknown type, or a length that does not fit the type;
* a datagram size different from header plus payload, or a TCP type on the
  telemetry channel;
* a latitude beyond ±90°, a longitude beyond ±180°, a value that is not
  finite, or a quaternion whose norm differs from 1 by more than 10⁻⁶;
* a negative accuracy, or a position fix above 6;
* a time factor or focal length that is not positive, or an image size of
  zero;
* an image format other than JPEG, or an image data size that does not
  match the header length.

Another format version gives `OST_ERR_UNSUPPORTED`.

## Test vectors

The values encoded in `test_vectors.txt`, from
[`link_vectors.py`](../../tools/injector/orthostream_injector/link_vectors.py):

| Vector | Content |
| --- | --- |
| `telemetry` | sequence 42, session 7, time 1763078400123456789, recorded 1604917662246000000, 46.8238423310124° N, 9.827941140558746° E, 2582.535681044683 m, `q_N_B` (0.5, 0.5, −0.5, 0.5), accuracy 0.03/0.03/0.05 m and 1/1/3°, fix 6 |
| `heartbeat` | sequence 43, session 7, 10 samples, 1 image |
| `session_start` | session 7, time factor 1, the camera of Dorfberg Flight 01 (7952 × 5304 px), `q_B_C` = 90° about z, camera at (0.1, 0, 0.05) m, name `RX1RII 42MP` |
| `image` | session 7, image 3, 4 bytes of data `FF D8 FF D9` |
| `session_end` | session 7, 1 image |
