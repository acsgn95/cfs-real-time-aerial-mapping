#include <limits>
// Decodes the shared byte vectors of interfaces/link/test_vectors.txt, which
// the Python encoder produces (tools/injector), and checks malformed input.

#include <gtest/gtest.h>

#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "link_c_api.h"

extern "C" int ost_link_c_api_usage(const uint8_t *datagram, size_t size);

namespace {

using Bytes = std::vector<uint8_t>;

std::map<std::string, Bytes> loadVectors()
{
    std::ifstream                 in(OST_LINK_TEST_VECTORS);
    std::map<std::string, Bytes>  vectors;
    std::string                   line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream ss(line);
        std::string        name, hex;
        ss >> name >> hex;
        Bytes bytes;
        for (size_t i = 0; i + 1 < hex.size(); i += 2)
        {
            bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
        }
        vectors[name] = bytes;
    }
    return vectors;
}

const std::map<std::string, Bytes> &vectors()
{
    static const std::map<std::string, Bytes> v = loadVectors();
    return v;
}

void putU32(Bytes &b, size_t off, uint32_t v)
{
    for (size_t i = 0; i < 4; ++i)
    {
        b.at(off + i) = static_cast<uint8_t>(v >> (8 * i));
    }
}

void putF64(Bytes &b, size_t off, double v)
{
    uint64_t u;
    std::memcpy(&u, &v, sizeof u);
    for (size_t i = 0; i < 8; ++i)
    {
        b.at(off + i) = static_cast<uint8_t>(u >> (8 * i));
    }
}

}  // namespace

TEST(Link, VectorsArePresent)
{
    EXPECT_EQ(vectors().size(), 5u);
}

TEST(Link, TelemetryVector)
{
    const Bytes        &b = vectors().at("telemetry");
    OST_Link_Datagram_t d;
    const char         *reason = nullptr;
    ASSERT_EQ(OST_Link_DecodeDatagram(b.data(), b.size(), &d, &reason), OST_SUCCESS) << reason;
    EXPECT_EQ(d.header.version, 1u);
    EXPECT_EQ(d.header.type, OST_LINK_TYPE_TELEMETRY);
    EXPECT_EQ(d.header.length, OST_LINK_TELEMETRY_SIZE);
    EXPECT_EQ(d.header.sequence, 42u);
    const OST_Link_Telemetry_t &t = d.u.telemetry;
    EXPECT_EQ(t.time_ns, 1763078400123456789LL);
    EXPECT_EQ(t.recorded_ns, 1604917662246000000LL);
    EXPECT_EQ(t.latitude_deg, 46.8238423310124);
    EXPECT_EQ(t.longitude_deg, 9.827941140558746);
    EXPECT_EQ(t.height_m, 2582.535681044683);
    EXPECT_EQ(t.q_n_b[0], 0.5);
    EXPECT_EQ(t.q_n_b[2], -0.5);
    EXPECT_EQ(t.std_position_m[0], 0.03f);
    EXPECT_EQ(t.std_position_m[2], 0.05f);
    EXPECT_EQ(t.std_attitude_deg[2], 3.0f);
    EXPECT_EQ(t.fix, 6u);
    EXPECT_EQ(t.session_id, 7u);
    EXPECT_EQ(ost_link_c_api_usage(b.data(), b.size()), 0);
}

TEST(Link, HeartbeatVector)
{
    const Bytes        &b = vectors().at("heartbeat");
    OST_Link_Datagram_t d;
    ASSERT_EQ(OST_Link_DecodeDatagram(b.data(), b.size(), &d, nullptr), OST_SUCCESS);
    EXPECT_EQ(d.header.type, OST_LINK_TYPE_HEARTBEAT);
    EXPECT_EQ(d.u.heartbeat.time_ns, 1763078401000000000LL);
    EXPECT_EQ(d.u.heartbeat.session_id, 7u);
    EXPECT_EQ(d.u.heartbeat.samples_sent, 10u);
    EXPECT_EQ(d.u.heartbeat.images_sent, 1u);
}

TEST(Link, SessionStartVector)
{
    const Bytes      &b = vectors().at("session_start");
    OST_Link_Header_t h;
    ASSERT_EQ(OST_Link_DecodeHeader(b.data(), b.size(), &h, nullptr), OST_SUCCESS);
    ASSERT_EQ(b.size(), OST_LINK_HEADER_SIZE + h.length);
    OST_Link_SessionStart_t s;
    const char             *reason = nullptr;
    ASSERT_EQ(OST_Link_DecodeSessionStart(&h, b.data() + OST_LINK_HEADER_SIZE, h.length, &s, &reason), OST_SUCCESS)
        << reason;
    EXPECT_EQ(s.session_id, 7u);
    EXPECT_EQ(s.start_ns, 1763078400000000000LL);
    EXPECT_EQ(s.recorded_start_ns, 1604917662246000000LL);
    EXPECT_EQ(s.time_factor, 1.0);
    EXPECT_EQ(s.width, 7952u);
    EXPECT_EQ(s.height, 5304u);
    EXPECT_EQ(s.fx, 7466.670812412861);
    EXPECT_EQ(s.cy, 2637.8310591480517);
    EXPECT_EQ(s.k3, -0.09318594764822886);
    EXPECT_EQ(s.p2, -4.833588644189338e-05);
    EXPECT_EQ(s.q_b_c[3], 0.7071067811865476);
    EXPECT_EQ(s.t_b_c[0], 0.1);
    EXPECT_EQ(s.t_b_c[2], 0.05);
    EXPECT_STREQ(s.camera_name, "RX1RII 42MP");
}

TEST(Link, ImageVector)
{
    const Bytes      &b = vectors().at("image");
    OST_Link_Header_t h;
    ASSERT_EQ(OST_Link_DecodeHeader(b.data(), b.size(), &h, nullptr), OST_SUCCESS);
    EXPECT_EQ(h.length, OST_LINK_IMAGE_SIZE + 4u);
    OST_Link_Image_t img;
    ASSERT_EQ(OST_Link_DecodeImage(&h, b.data() + OST_LINK_HEADER_SIZE, OST_LINK_IMAGE_SIZE, &img, nullptr),
              OST_SUCCESS);
    EXPECT_EQ(img.session_id, 7u);
    EXPECT_EQ(img.image_seq, 3u);
    EXPECT_EQ(img.capture_ns, 1763078401580000000LL);
    EXPECT_EQ(img.recorded_ns, 1604917663826000000LL);
    EXPECT_EQ(img.format, OST_LINK_IMAGE_FORMAT_JPEG);
    ASSERT_EQ(img.data_size, 4u);
    const uint8_t *data = b.data() + OST_LINK_HEADER_SIZE + OST_LINK_IMAGE_SIZE;
    EXPECT_EQ(data[0], 0xFF);
    EXPECT_EQ(data[3], 0xD9);
}

TEST(Link, SessionEndVector)
{
    const Bytes      &b = vectors().at("session_end");
    OST_Link_Header_t h;
    ASSERT_EQ(OST_Link_DecodeHeader(b.data(), b.size(), &h, nullptr), OST_SUCCESS);
    OST_Link_SessionEnd_t e;
    ASSERT_EQ(OST_Link_DecodeSessionEnd(&h, b.data() + OST_LINK_HEADER_SIZE, h.length, &e, nullptr), OST_SUCCESS);
    EXPECT_EQ(e.session_id, 7u);
    EXPECT_EQ(e.images_sent, 1u);
    EXPECT_EQ(e.time_ns, 1763078402000000000LL);
}

TEST(Link, MalformedHeaders)
{
    const Bytes       good = vectors().at("telemetry");
    OST_Link_Header_t h;
    const char       *reason = nullptr;

    Bytes b = good;
    b[0]    = 'X';
    EXPECT_EQ(OST_Link_DecodeHeader(b.data(), b.size(), &h, &reason), OST_ERR_FORMAT);
    EXPECT_STREQ(reason, "magic word is not OSTL");

    b    = good;
    b[4] = 2;  // version
    EXPECT_EQ(OST_Link_DecodeHeader(b.data(), b.size(), &h, nullptr), OST_ERR_UNSUPPORTED);

    b    = good;
    b[6] = 99;  // type
    EXPECT_EQ(OST_Link_DecodeHeader(b.data(), b.size(), &h, nullptr), OST_ERR_FORMAT);

    b = good;
    putU32(b, OST_LINK_HEADER_OFF_LENGTH, OST_LINK_TELEMETRY_SIZE + 1);
    EXPECT_EQ(OST_Link_DecodeHeader(b.data(), b.size(), &h, nullptr), OST_ERR_FORMAT);

    EXPECT_EQ(OST_Link_DecodeHeader(good.data(), OST_LINK_HEADER_SIZE - 1, &h, nullptr), OST_ERR_FORMAT);
    EXPECT_EQ(OST_Link_DecodeHeader(nullptr, 16, &h, nullptr), OST_ERR_INVALID_ARGUMENT);

    Bytes img = vectors().at("image");
    putU32(img, OST_LINK_HEADER_OFF_LENGTH, OST_LINK_IMAGE_SIZE);  // no data
    EXPECT_EQ(OST_Link_DecodeHeader(img.data(), img.size(), &h, nullptr), OST_ERR_FORMAT);
    putU32(img, OST_LINK_HEADER_OFF_LENGTH, OST_LINK_IMAGE_SIZE + OST_LINK_MAX_IMAGE_BYTES + 1);
    EXPECT_EQ(OST_Link_DecodeHeader(img.data(), img.size(), &h, &reason), OST_ERR_FORMAT);
    EXPECT_STREQ(reason, "image larger than the limit");
}

TEST(Link, MalformedDatagrams)
{
    const Bytes         good = vectors().at("telemetry");
    OST_Link_Datagram_t d;
    const char         *reason = nullptr;
    const size_t        p      = OST_LINK_HEADER_SIZE;

    // Truncated and padded datagrams.
    EXPECT_EQ(OST_Link_DecodeDatagram(good.data(), good.size() - 1, &d, nullptr), OST_ERR_FORMAT);
    Bytes b = good;
    b.push_back(0);
    EXPECT_EQ(OST_Link_DecodeDatagram(b.data(), b.size(), &d, nullptr), OST_ERR_FORMAT);

    b = good;
    putF64(b, p + OST_LINK_TELEMETRY_OFF_LAT, 91.0);
    EXPECT_EQ(OST_Link_DecodeDatagram(b.data(), b.size(), &d, &reason), OST_ERR_FORMAT);
    EXPECT_STREQ(reason, "telemetry latitude out of range");

    b = good;
    putF64(b, p + OST_LINK_TELEMETRY_OFF_Q_N_B, 0.6);  // |q| != 1
    EXPECT_EQ(OST_Link_DecodeDatagram(b.data(), b.size(), &d, &reason), OST_ERR_FORMAT);
    EXPECT_STREQ(reason, "telemetry attitude not a unit quaternion");

    b = good;
    putF64(b, p + OST_LINK_TELEMETRY_OFF_HEIGHT, std::numeric_limits<double>::quiet_NaN());
    EXPECT_EQ(OST_Link_DecodeDatagram(b.data(), b.size(), &d, nullptr), OST_ERR_FORMAT);

    b                                  = good;
    b[p + OST_LINK_TELEMETRY_OFF_FIX] = 7;
    EXPECT_EQ(OST_Link_DecodeDatagram(b.data(), b.size(), &d, nullptr), OST_ERR_FORMAT);

    // A TCP message type is not accepted on the telemetry channel.
    const Bytes &end = vectors().at("session_end");
    EXPECT_EQ(OST_Link_DecodeDatagram(end.data(), end.size(), &d, &reason), OST_ERR_FORMAT);
    EXPECT_STREQ(reason, "message type not sent on the telemetry channel");
}

TEST(Link, MalformedTcpPayloads)
{
    const Bytes      ss = vectors().at("session_start");
    OST_Link_Header_t h;
    ASSERT_EQ(OST_Link_DecodeHeader(ss.data(), ss.size(), &h, nullptr), OST_SUCCESS);
    OST_Link_SessionStart_t s;
    const char             *reason = nullptr;
    const size_t            p      = OST_LINK_HEADER_SIZE;

    Bytes b = ss;
    putF64(b, p + OST_LINK_SESSION_START_OFF_FACTOR, 0.0);
    EXPECT_EQ(OST_Link_DecodeSessionStart(&h, b.data() + p, h.length, &s, &reason), OST_ERR_FORMAT);
    EXPECT_STREQ(reason, "time factor not positive");

    b = ss;
    putF64(b, p + OST_LINK_SESSION_START_OFF_K, -1.0);
    EXPECT_EQ(OST_Link_DecodeSessionStart(&h, b.data() + p, h.length, &s, nullptr), OST_ERR_FORMAT);

    b = ss;
    putF64(b, p + OST_LINK_SESSION_START_OFF_Q_B_C, 1.0);
    EXPECT_EQ(OST_Link_DecodeSessionStart(&h, b.data() + p, h.length, &s, nullptr), OST_ERR_FORMAT);

    // Name without a terminating NUL in the frame is still terminated.
    b = ss;
    std::memset(b.data() + p + OST_LINK_SESSION_START_OFF_NAME, 'A', OST_LINK_CAMERA_NAME_LEN);
    ASSERT_EQ(OST_Link_DecodeSessionStart(&h, b.data() + p, h.length, &s, nullptr), OST_SUCCESS);
    EXPECT_EQ(std::strlen(s.camera_name), OST_LINK_CAMERA_NAME_LEN);

    // Short buffer and wrong header type.
    EXPECT_EQ(OST_Link_DecodeSessionStart(&h, ss.data() + p, h.length - 1, &s, nullptr), OST_ERR_INVALID_ARGUMENT);
    const Bytes      im = vectors().at("image");
    OST_Link_Header_t hi;
    ASSERT_EQ(OST_Link_DecodeHeader(im.data(), im.size(), &hi, nullptr), OST_SUCCESS);
    EXPECT_EQ(OST_Link_DecodeSessionStart(&hi, im.data() + p, 200, &s, nullptr), OST_ERR_INVALID_ARGUMENT);

    // Image: data size must match the header length; format must be JPEG.
    OST_Link_Image_t img;
    b = im;
    putU32(b, p + OST_LINK_IMAGE_OFF_DATA_SIZE, 5);
    EXPECT_EQ(OST_Link_DecodeImage(&hi, b.data() + p, OST_LINK_IMAGE_SIZE, &img, &reason), OST_ERR_FORMAT);
    EXPECT_STREQ(reason, "image data size does not match the header length");
    b                                = im;
    b[p + OST_LINK_IMAGE_OFF_FORMAT] = 2;
    EXPECT_EQ(OST_Link_DecodeImage(&hi, b.data() + p, OST_LINK_IMAGE_SIZE, &img, nullptr), OST_ERR_FORMAT);
}
