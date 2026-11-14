// Decoding of the injector link (ADR-016 section 7). Fields are read one by
// one from little-endian bytes, so the code does not depend on the host's
// byte order or structure padding.

#include "link_c_api.h"

#include <cmath>
#include <cstring>

namespace {

class Reader
{
public:
    explicit Reader(const uint8_t *bytes) : p_(bytes) {}

    uint8_t u8(size_t off) const { return p_[off]; }

    uint16_t u16(size_t off) const
    {
        return static_cast<uint16_t>(p_[off] | (p_[off + 1] << 8));
    }

    uint32_t u32(size_t off) const
    {
        uint32_t v = 0;
        for (int i = 3; i >= 0; --i)
        {
            v = (v << 8) | p_[off + static_cast<size_t>(i)];
        }
        return v;
    }

    uint64_t u64(size_t off) const
    {
        uint64_t v = 0;
        for (int i = 7; i >= 0; --i)
        {
            v = (v << 8) | p_[off + static_cast<size_t>(i)];
        }
        return v;
    }

    int64_t i64(size_t off) const
    {
        const uint64_t v = u64(off);
        int64_t        r;
        std::memcpy(&r, &v, sizeof r);
        return r;
    }

    double f64(size_t off) const
    {
        const uint64_t v = u64(off);
        double         r;
        std::memcpy(&r, &v, sizeof r);
        return r;
    }

    float f32(size_t off) const
    {
        const uint32_t v = u32(off);
        float          r;
        std::memcpy(&r, &v, sizeof r);
        return r;
    }

private:
    const uint8_t *p_;
};

OST_Status_t fail(OST_Status_t status, const char *text, const char **reason)
{
    if (reason != nullptr)
    {
        *reason = text;
    }
    return status;
}

OST_Status_t ok(const char **reason)
{
    if (reason != nullptr)
    {
        *reason = "";
    }
    return OST_SUCCESS;
}

bool isFinite(double v)
{
    return std::isfinite(v);
}

// Python normalises in double precision; 1e-6 leaves room for any encoder.
bool unitQuaternion(const double q[4])
{
    for (int i = 0; i < 4; ++i)
    {
        if (!isFinite(q[i]))
        {
            return false;
        }
    }
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return std::fabs(n - 1.0) <= 1e-6;
}

bool validStd(float v)
{
    return std::isfinite(v) && v >= 0.0f;
}

OST_Status_t decodeTelemetry(const Reader &r, OST_Link_Telemetry_t *t, const char **reason)
{
    t->time_ns       = r.i64(OST_LINK_TELEMETRY_OFF_TIME);
    t->recorded_ns   = r.i64(OST_LINK_TELEMETRY_OFF_RECORDED);
    t->latitude_deg  = r.f64(OST_LINK_TELEMETRY_OFF_LAT);
    t->longitude_deg = r.f64(OST_LINK_TELEMETRY_OFF_LON);
    t->height_m      = r.f64(OST_LINK_TELEMETRY_OFF_HEIGHT);
    for (size_t i = 0; i < 4; ++i)
    {
        t->q_n_b[i] = r.f64(OST_LINK_TELEMETRY_OFF_Q_N_B + 8 * i);
    }
    for (size_t i = 0; i < 3; ++i)
    {
        t->std_position_m[i]   = r.f32(OST_LINK_TELEMETRY_OFF_STD_POS + 4 * i);
        t->std_attitude_deg[i] = r.f32(OST_LINK_TELEMETRY_OFF_STD_ATT + 4 * i);
    }
    t->fix        = r.u8(OST_LINK_TELEMETRY_OFF_FIX);
    t->flags      = r.u8(OST_LINK_TELEMETRY_OFF_FLAGS);
    t->session_id = r.u32(OST_LINK_TELEMETRY_OFF_SESSION);

    if (!isFinite(t->latitude_deg) || std::fabs(t->latitude_deg) > 90.0)
    {
        return fail(OST_ERR_FORMAT, "telemetry latitude out of range", reason);
    }
    if (!isFinite(t->longitude_deg) || std::fabs(t->longitude_deg) > 180.0)
    {
        return fail(OST_ERR_FORMAT, "telemetry longitude out of range", reason);
    }
    if (!isFinite(t->height_m))
    {
        return fail(OST_ERR_FORMAT, "telemetry height not finite", reason);
    }
    if (!unitQuaternion(t->q_n_b))
    {
        return fail(OST_ERR_FORMAT, "telemetry attitude not a unit quaternion", reason);
    }
    for (int i = 0; i < 3; ++i)
    {
        if (!validStd(t->std_position_m[i]) || !validStd(t->std_attitude_deg[i]))
        {
            return fail(OST_ERR_FORMAT, "telemetry accuracy negative or not finite", reason);
        }
    }
    if (t->fix > 6)
    {
        return fail(OST_ERR_FORMAT, "telemetry position fix unknown", reason);
    }
    return ok(reason);
}

void decodeHeartbeat(const Reader &r, OST_Link_Heartbeat_t *h)
{
    h->time_ns      = r.i64(OST_LINK_HEARTBEAT_OFF_TIME);
    h->session_id   = r.u32(OST_LINK_HEARTBEAT_OFF_SESSION);
    h->samples_sent = r.u32(OST_LINK_HEARTBEAT_OFF_SAMPLES);
    h->images_sent  = r.u32(OST_LINK_HEARTBEAT_OFF_IMAGES);
}

OST_Status_t checkPayload(const OST_Link_Header_t *header, uint16_t type, const uint8_t *payload, size_t size,
                          size_t needed, const void *out, const char **reason)
{
    if (header == nullptr || payload == nullptr || out == nullptr)
    {
        return fail(OST_ERR_INVALID_ARGUMENT, "null argument", reason);
    }
    if (header->type != type)
    {
        return fail(OST_ERR_INVALID_ARGUMENT, "header of another message type", reason);
    }
    if (size < needed)
    {
        return fail(OST_ERR_INVALID_ARGUMENT, "payload buffer too short", reason);
    }
    return OST_SUCCESS;
}

}  // namespace

extern "C" {

OST_Status_t OST_Link_DecodeHeader(const uint8_t *bytes, size_t size, OST_Link_Header_t *header, const char **reason)
{
    if (bytes == nullptr || header == nullptr)
    {
        return fail(OST_ERR_INVALID_ARGUMENT, "null argument", reason);
    }
    if (size < OST_LINK_HEADER_SIZE)
    {
        return fail(OST_ERR_FORMAT, "shorter than a header", reason);
    }
    if (bytes[0] != OST_LINK_MAGIC_0 || bytes[1] != OST_LINK_MAGIC_1 || bytes[2] != OST_LINK_MAGIC_2 ||
        bytes[3] != OST_LINK_MAGIC_3)
    {
        return fail(OST_ERR_FORMAT, "magic word is not OSTL", reason);
    }
    const Reader r(bytes);
    header->version  = r.u16(OST_LINK_HEADER_OFF_VERSION);
    header->type     = r.u16(OST_LINK_HEADER_OFF_TYPE);
    header->length   = r.u32(OST_LINK_HEADER_OFF_LENGTH);
    header->sequence = r.u32(OST_LINK_HEADER_OFF_SEQ);
    if (header->version != OST_LINK_VERSION)
    {
        return fail(OST_ERR_UNSUPPORTED, "unsupported format version", reason);
    }

    uint32_t expected = 0;
    switch (header->type)
    {
        case OST_LINK_TYPE_TELEMETRY:
            expected = OST_LINK_TELEMETRY_SIZE;
            break;
        case OST_LINK_TYPE_HEARTBEAT:
            expected = OST_LINK_HEARTBEAT_SIZE;
            break;
        case OST_LINK_TYPE_SESSION_START:
            expected = OST_LINK_SESSION_START_SIZE;
            break;
        case OST_LINK_TYPE_SESSION_END:
            expected = OST_LINK_SESSION_END_SIZE;
            break;
        case OST_LINK_TYPE_IMAGE:
            if (header->length <= OST_LINK_IMAGE_SIZE)
            {
                return fail(OST_ERR_FORMAT, "image frame without image data", reason);
            }
            if (header->length - OST_LINK_IMAGE_SIZE > OST_LINK_MAX_IMAGE_BYTES)
            {
                return fail(OST_ERR_FORMAT, "image larger than the limit", reason);
            }
            return ok(reason);
        default:
            return fail(OST_ERR_FORMAT, "unknown message type", reason);
    }
    if (header->length != expected)
    {
        return fail(OST_ERR_FORMAT, "length does not match the message type", reason);
    }
    return ok(reason);
}

OST_Status_t OST_Link_DecodeDatagram(const uint8_t *bytes, size_t size, OST_Link_Datagram_t *datagram,
                                     const char **reason)
{
    if (datagram == nullptr)
    {
        return fail(OST_ERR_INVALID_ARGUMENT, "null argument", reason);
    }
    const OST_Status_t status = OST_Link_DecodeHeader(bytes, size, &datagram->header, reason);
    if (status != OST_SUCCESS)
    {
        return status;
    }
    const OST_Link_Header_t &h = datagram->header;
    if (h.type != OST_LINK_TYPE_TELEMETRY && h.type != OST_LINK_TYPE_HEARTBEAT)
    {
        return fail(OST_ERR_FORMAT, "message type not sent on the telemetry channel", reason);
    }
    if (size != OST_LINK_HEADER_SIZE + static_cast<size_t>(h.length))
    {
        return fail(OST_ERR_FORMAT, "datagram size does not match the header length", reason);
    }
    const Reader payload(bytes + OST_LINK_HEADER_SIZE);
    if (h.type == OST_LINK_TYPE_TELEMETRY)
    {
        return decodeTelemetry(payload, &datagram->u.telemetry, reason);
    }
    decodeHeartbeat(payload, &datagram->u.heartbeat);
    return ok(reason);
}

OST_Status_t OST_Link_DecodeSessionStart(const OST_Link_Header_t *header, const uint8_t *payload, size_t size,
                                         OST_Link_SessionStart_t *s, const char **reason)
{
    const OST_Status_t status = checkPayload(header, OST_LINK_TYPE_SESSION_START, payload, size,
                                             OST_LINK_SESSION_START_SIZE, s, reason);
    if (status != OST_SUCCESS)
    {
        return status;
    }
    const Reader r(payload);
    s->session_id        = r.u32(OST_LINK_SESSION_START_OFF_SESSION);
    s->start_ns          = r.i64(OST_LINK_SESSION_START_OFF_START);
    s->recorded_start_ns = r.i64(OST_LINK_SESSION_START_OFF_RECORDED);
    s->time_factor       = r.f64(OST_LINK_SESSION_START_OFF_FACTOR);
    s->width             = r.u32(OST_LINK_SESSION_START_OFF_WIDTH);
    s->height            = r.u32(OST_LINK_SESSION_START_OFF_HEIGHT);
    s->fx                = r.f64(OST_LINK_SESSION_START_OFF_K + 0);
    s->fy                = r.f64(OST_LINK_SESSION_START_OFF_K + 8);
    s->cx                = r.f64(OST_LINK_SESSION_START_OFF_K + 16);
    s->cy                = r.f64(OST_LINK_SESSION_START_OFF_K + 24);
    s->k1                = r.f64(OST_LINK_SESSION_START_OFF_DIST + 0);
    s->k2                = r.f64(OST_LINK_SESSION_START_OFF_DIST + 8);
    s->k3                = r.f64(OST_LINK_SESSION_START_OFF_DIST + 16);
    s->p1                = r.f64(OST_LINK_SESSION_START_OFF_DIST + 24);
    s->p2                = r.f64(OST_LINK_SESSION_START_OFF_DIST + 32);
    for (size_t i = 0; i < 4; ++i)
    {
        s->q_b_c[i] = r.f64(OST_LINK_SESSION_START_OFF_Q_B_C + 8 * i);
    }
    for (size_t i = 0; i < 3; ++i)
    {
        s->t_b_c[i] = r.f64(OST_LINK_SESSION_START_OFF_T_B_C + 8 * i);
    }
    std::memcpy(s->camera_name, payload + OST_LINK_SESSION_START_OFF_NAME, OST_LINK_CAMERA_NAME_LEN);
    s->camera_name[OST_LINK_CAMERA_NAME_LEN] = '\0';

    if (!isFinite(s->time_factor) || s->time_factor <= 0.0)
    {
        return fail(OST_ERR_FORMAT, "time factor not positive", reason);
    }
    if (s->width == 0 || s->height == 0)
    {
        return fail(OST_ERR_FORMAT, "image size zero", reason);
    }
    const double values[] = {s->fx, s->fy, s->cx, s->cy, s->k1, s->k2, s->k3, s->p1, s->p2,
                             s->t_b_c[0], s->t_b_c[1], s->t_b_c[2]};
    for (double v : values)
    {
        if (!isFinite(v))
        {
            return fail(OST_ERR_FORMAT, "calibration value not finite", reason);
        }
    }
    if (s->fx <= 0.0 || s->fy <= 0.0)
    {
        return fail(OST_ERR_FORMAT, "focal length not positive", reason);
    }
    if (!unitQuaternion(s->q_b_c))
    {
        return fail(OST_ERR_FORMAT, "camera mounting not a unit quaternion", reason);
    }
    return ok(reason);
}

OST_Status_t OST_Link_DecodeImage(const OST_Link_Header_t *header, const uint8_t *payload, size_t size,
                                  OST_Link_Image_t *image, const char **reason)
{
    const OST_Status_t status =
        checkPayload(header, OST_LINK_TYPE_IMAGE, payload, size, OST_LINK_IMAGE_SIZE, image, reason);
    if (status != OST_SUCCESS)
    {
        return status;
    }
    const Reader r(payload);
    image->session_id  = r.u32(OST_LINK_IMAGE_OFF_SESSION);
    image->image_seq   = r.u32(OST_LINK_IMAGE_OFF_SEQ);
    image->capture_ns  = r.i64(OST_LINK_IMAGE_OFF_CAPTURE);
    image->recorded_ns = r.i64(OST_LINK_IMAGE_OFF_RECORDED);
    image->format      = r.u8(OST_LINK_IMAGE_OFF_FORMAT);
    image->data_size   = r.u32(OST_LINK_IMAGE_OFF_DATA_SIZE);
    if (image->format != OST_LINK_IMAGE_FORMAT_JPEG)
    {
        return fail(OST_ERR_FORMAT, "image format not JPEG", reason);
    }
    if (static_cast<uint64_t>(image->data_size) + OST_LINK_IMAGE_SIZE != header->length)
    {
        return fail(OST_ERR_FORMAT, "image data size does not match the header length", reason);
    }
    return ok(reason);
}

OST_Status_t OST_Link_DecodeSessionEnd(const OST_Link_Header_t *header, const uint8_t *payload, size_t size,
                                       OST_Link_SessionEnd_t *end, const char **reason)
{
    const OST_Status_t status = checkPayload(header, OST_LINK_TYPE_SESSION_END, payload, size,
                                             OST_LINK_SESSION_END_SIZE, end, reason);
    if (status != OST_SUCCESS)
    {
        return status;
    }
    const Reader r(payload);
    end->session_id  = r.u32(OST_LINK_SESSION_END_OFF_SESSION);
    end->images_sent = r.u32(OST_LINK_SESSION_END_OFF_IMAGES);
    end->time_ns     = r.i64(OST_LINK_SESSION_END_OFF_TIME);
    return ok(reason);
}

}  // extern "C"
