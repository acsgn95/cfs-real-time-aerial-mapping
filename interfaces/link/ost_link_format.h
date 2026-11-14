/*
 * Wire format of the link between the injector and INGEST_APP
 * (ADR-003, ADR-016 section 7). Documented in docs/interfaces/link.md;
 * the Python injector (tools/injector/orthostream_injector/link.py) writes
 * the same layout. tests/interfaces/check_interfaces.py compares the two.
 *
 * All fields are little-endian and packed. The offsets below are byte
 * offsets from the start of the payload, which follows the 16-byte header.
 * Readers decode field by field (libs/link); no C struct is overlaid on
 * the bytes.
 *
 * Frames and channels:
 *   UDP telemetry: one datagram = header + TELEMETRY or HEARTBEAT payload
 *   TCP imagery:   SESSION_START, then IMAGE frames, then SESSION_END;
 *                  an IMAGE payload is its fixed part followed by the JPEG
 *
 * Conventions (ADR-004, ADR-014):
 *   time:      int64 nanoseconds since the Unix epoch, UTC (data time)
 *   position:  WGS 84 latitude and longitude in degrees, ellipsoidal height in m
 *   attitude:  Hamilton quaternion q_N_B (w, x, y, z), body FRD to local NED
 *   camera:    OpenCV pinhole with k1 k2 k3 p1 p2; centre of the top-left
 *              pixel at (0, 0); T_B_C as q_B_C and the camera position in
 *              the body frame
 */
#ifndef OST_LINK_FORMAT_H
#define OST_LINK_FORMAT_H

/* Header ------------------------------------------------------------------ */

#define OST_LINK_MAGIC_0 0x4Fu /* 'O' */
#define OST_LINK_MAGIC_1 0x53u /* 'S' */
#define OST_LINK_MAGIC_2 0x54u /* 'T' */
#define OST_LINK_MAGIC_3 0x4Cu /* 'L' */

#define OST_LINK_VERSION 1u

#define OST_LINK_HEADER_SIZE        16u
#define OST_LINK_HEADER_OFF_MAGIC   0u  /* 4 bytes "OSTL" */
#define OST_LINK_HEADER_OFF_VERSION 4u  /* uint16 */
#define OST_LINK_HEADER_OFF_TYPE    6u  /* uint16, OST_LINK_TYPE_* */
#define OST_LINK_HEADER_OFF_LENGTH  8u  /* uint32, payload bytes after the header */
#define OST_LINK_HEADER_OFF_SEQ     12u /* uint32, per channel, +1 per message */

/* Message types ----------------------------------------------------------- */

#define OST_LINK_TYPE_TELEMETRY     1u  /* UDP */
#define OST_LINK_TYPE_HEARTBEAT     2u  /* UDP */
#define OST_LINK_TYPE_SESSION_START 16u /* TCP */
#define OST_LINK_TYPE_IMAGE         17u /* TCP */
#define OST_LINK_TYPE_SESSION_END   18u /* TCP */

/* TELEMETRY: one pose sample of the autopilot ----------------------------- */

#define OST_LINK_TELEMETRY_SIZE          104u
#define OST_LINK_TELEMETRY_OFF_TIME      0u   /* int64 ns, measurement time */
#define OST_LINK_TELEMETRY_OFF_RECORDED  8u   /* int64 ns, time in the recording */
#define OST_LINK_TELEMETRY_OFF_LAT       16u  /* float64 deg */
#define OST_LINK_TELEMETRY_OFF_LON       24u  /* float64 deg */
#define OST_LINK_TELEMETRY_OFF_HEIGHT    32u  /* float64 m, ellipsoidal */
#define OST_LINK_TELEMETRY_OFF_Q_N_B     40u  /* 4 x float64, w x y z */
#define OST_LINK_TELEMETRY_OFF_STD_POS   72u  /* 3 x float32 m, north east down */
#define OST_LINK_TELEMETRY_OFF_STD_ATT   84u  /* 3 x float32 deg, roll pitch yaw */
#define OST_LINK_TELEMETRY_OFF_FIX       96u  /* uint8, OST_PositionFix (EDS) */
#define OST_LINK_TELEMETRY_OFF_FLAGS     97u  /* uint8, 0 */
#define OST_LINK_TELEMETRY_OFF_RESERVED  98u  /* uint16, 0 */
#define OST_LINK_TELEMETRY_OFF_SESSION   100u /* uint32 */

/* HEARTBEAT: sent once per second on the telemetry channel ---------------- */

#define OST_LINK_HEARTBEAT_SIZE         24u
#define OST_LINK_HEARTBEAT_OFF_TIME     0u  /* int64 ns, time of sending */
#define OST_LINK_HEARTBEAT_OFF_SESSION  8u  /* uint32 */
#define OST_LINK_HEARTBEAT_OFF_SAMPLES  12u /* uint32, telemetry samples sent */
#define OST_LINK_HEARTBEAT_OFF_IMAGES   16u /* uint32, images sent */
#define OST_LINK_HEARTBEAT_OFF_RESERVED 20u /* uint32, 0 */

/* SESSION_START: first frame on a TCP connection -------------------------- */

#define OST_LINK_CAMERA_NAME_LEN 32u

#define OST_LINK_SESSION_START_SIZE         200u
#define OST_LINK_SESSION_START_OFF_SESSION  0u   /* uint32 */
#define OST_LINK_SESSION_START_OFF_RESERVED 4u   /* uint32, 0 */
#define OST_LINK_SESSION_START_OFF_START    8u   /* int64 ns, data time of the recording start */
#define OST_LINK_SESSION_START_OFF_RECORDED 16u  /* int64 ns, recording start */
#define OST_LINK_SESSION_START_OFF_FACTOR   24u  /* float64, time factor (1 = real time) */
#define OST_LINK_SESSION_START_OFF_WIDTH    32u  /* uint32 px */
#define OST_LINK_SESSION_START_OFF_HEIGHT   36u  /* uint32 px */
#define OST_LINK_SESSION_START_OFF_K        40u  /* 4 x float64 px: fx fy cx cy */
#define OST_LINK_SESSION_START_OFF_DIST     72u  /* 5 x float64: k1 k2 k3 p1 p2 */
#define OST_LINK_SESSION_START_OFF_Q_B_C    112u /* 4 x float64, w x y z */
#define OST_LINK_SESSION_START_OFF_T_B_C    144u /* 3 x float64 m, camera in body */
#define OST_LINK_SESSION_START_OFF_NAME     168u /* char[32], NUL padded */

/* IMAGE: fixed part, followed by DATA_SIZE bytes of image data ------------ */

#define OST_LINK_IMAGE_FORMAT_JPEG 1u

#define OST_LINK_IMAGE_SIZE          32u /* fixed part only */
#define OST_LINK_IMAGE_OFF_SESSION   0u  /* uint32 */
#define OST_LINK_IMAGE_OFF_SEQ       4u  /* uint32, image sequence number */
#define OST_LINK_IMAGE_OFF_CAPTURE   8u  /* int64 ns, capture time */
#define OST_LINK_IMAGE_OFF_RECORDED  16u /* int64 ns, capture time in the recording */
#define OST_LINK_IMAGE_OFF_FORMAT    24u /* uint8, OST_LINK_IMAGE_FORMAT_* */
#define OST_LINK_IMAGE_OFF_RESERVED  25u /* 3 x uint8, 0 */
#define OST_LINK_IMAGE_OFF_DATA_SIZE 28u /* uint32, header length = 32 + DATA_SIZE */

/* SESSION_END: last frame on a TCP connection ----------------------------- */

#define OST_LINK_SESSION_END_SIZE        16u
#define OST_LINK_SESSION_END_OFF_SESSION 0u /* uint32 */
#define OST_LINK_SESSION_END_OFF_IMAGES  4u /* uint32, images sent */
#define OST_LINK_SESSION_END_OFF_TIME    8u /* int64 ns, time of sending */

#endif /* OST_LINK_FORMAT_H */
