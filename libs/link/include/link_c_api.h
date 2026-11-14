/*
 * C interface of the OrthoStream link library (ADR-011, ADR-012, ADR-016
 * section 7): decoding and validation of the injector's UDP telemetry
 * datagrams and TCP image frames. The byte layout is interfaces/link/
 * ost_link_format.h; docs/interfaces/link.md describes it.
 *
 * The library does no input or output. INGEST_APP reads the bytes from its
 * sockets and calls these functions:
 *
 *   UDP: OST_Link_DecodeDatagram on each datagram
 *   TCP: read OST_LINK_HEADER_SIZE bytes, OST_Link_DecodeHeader, then the
 *        payload of the type: SESSION_START and SESSION_END completely,
 *        IMAGE only its fixed part of OST_LINK_IMAGE_SIZE bytes, followed
 *        by data_size bytes of image data read directly into the data store
 *
 * Status codes (ost_status.h):
 *   OST_ERR_FORMAT        malformed message: magic, type, length or a
 *                         field out of range (not finite, quaternion not of
 *                         unit length, latitude beyond 90 degrees, ...)
 *   OST_ERR_UNSUPPORTED   other format version
 *   OST_ERR_INVALID_ARGUMENT  NULL pointer or a buffer shorter than needed
 * On failure, *reason (if not NULL) points to a static English text naming
 * the check that failed. All functions are thread-safe and allocate nothing.
 */

#ifndef ORTHOSTREAM_LINK_C_API_H
#define ORTHOSTREAM_LINK_C_API_H

#include <stddef.h>
#include <stdint.h>

#include "ost_link_format.h"
#include "ost_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Largest image a frame may announce. */
#define OST_LINK_MAX_IMAGE_BYTES (256u * 1024u * 1024u)

typedef struct
{
    uint16_t version;
    uint16_t type;     /* OST_LINK_TYPE_* */
    uint32_t length;   /* payload bytes after the header */
    uint32_t sequence;
} OST_Link_Header_t;

typedef struct
{
    int64_t  time_ns;
    int64_t  recorded_ns;
    double   latitude_deg;
    double   longitude_deg;
    double   height_m;
    double   q_n_b[4];          /* w x y z */
    float    std_position_m[3]; /* north east down */
    float    std_attitude_deg[3]; /* roll pitch yaw */
    uint8_t  fix;               /* OST_PositionFix values */
    uint8_t  flags;
    uint32_t session_id;
} OST_Link_Telemetry_t;

typedef struct
{
    int64_t  time_ns;
    uint32_t session_id;
    uint32_t samples_sent;
    uint32_t images_sent;
} OST_Link_Heartbeat_t;

typedef struct
{
    OST_Link_Header_t header;
    union
    {
        OST_Link_Telemetry_t telemetry; /* header.type == OST_LINK_TYPE_TELEMETRY */
        OST_Link_Heartbeat_t heartbeat; /* header.type == OST_LINK_TYPE_HEARTBEAT */
    } u;
} OST_Link_Datagram_t;

typedef struct
{
    uint32_t session_id;
    int64_t  start_ns;
    int64_t  recorded_start_ns;
    double   time_factor;
    uint32_t width;
    uint32_t height;
    double   fx, fy, cx, cy;
    double   k1, k2, k3, p1, p2;
    double   q_b_c[4]; /* w x y z */
    double   t_b_c[3]; /* camera position in the body frame, m */
    char     camera_name[OST_LINK_CAMERA_NAME_LEN + 1];
} OST_Link_SessionStart_t;

typedef struct
{
    uint32_t session_id;
    uint32_t image_seq;
    int64_t  capture_ns;
    int64_t  recorded_ns;
    uint8_t  format;    /* OST_LINK_IMAGE_FORMAT_* */
    uint32_t data_size; /* bytes of image data after the fixed part */
} OST_Link_Image_t;

typedef struct
{
    uint32_t session_id;
    uint32_t images_sent;
    int64_t  time_ns;
} OST_Link_SessionEnd_t;

/* Decodes and checks a header: magic, version, known type, and a length
 * that fits the type (exact for fixed-size messages; for IMAGE at least the
 * fixed part and at most OST_LINK_MAX_IMAGE_BYTES of data). */
OST_Status_t OST_Link_DecodeHeader(const uint8_t *bytes, size_t size, OST_Link_Header_t *header, const char **reason);

/* One UDP datagram: header and a TELEMETRY or HEARTBEAT payload; size must
 * be exactly header plus payload. */
OST_Status_t OST_Link_DecodeDatagram(const uint8_t *bytes, size_t size, OST_Link_Datagram_t *datagram,
                                     const char **reason);

/* TCP payloads, after OST_Link_DecodeHeader. size is the number of bytes at
 * payload: header->length for SESSION_START and SESSION_END, and
 * OST_LINK_IMAGE_SIZE for IMAGE. The header must be of the matching type. */
OST_Status_t OST_Link_DecodeSessionStart(const OST_Link_Header_t *header, const uint8_t *payload, size_t size,
                                         OST_Link_SessionStart_t *session, const char **reason);
OST_Status_t OST_Link_DecodeImage(const OST_Link_Header_t *header, const uint8_t *payload, size_t size,
                                  OST_Link_Image_t *image, const char **reason);
OST_Status_t OST_Link_DecodeSessionEnd(const OST_Link_Header_t *header, const uint8_t *payload, size_t size,
                                       OST_Link_SessionEnd_t *end, const char **reason);

#ifdef __cplusplus
}
#endif

#endif /* ORTHOSTREAM_LINK_C_API_H */
