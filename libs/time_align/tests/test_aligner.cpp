// Tests of the time alignment (ADR-014 section 4).

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "orthostream/time_align/aligner.hpp"
#include "time_align_c_api.h"

extern "C" int ost_time_align_c_api_usage(void);

namespace ta = orthostream::time_align;

namespace
{

constexpr double       kPi = 3.14159265358979323846;
constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kS  = 1'000'000'000;
constexpr std::int64_t kT0 = 1'763'100'000LL * kS;  // a UTC time in 2025

double deg(double rad)
{
    return rad * 180.0 / kPi;
}

/// ECEF to geodetic (degrees), iterative; reference for the test trajectory.
void ecefToGeodetic(const Eigen::Vector3d &p, double *lat, double *lon, double *h)
{
    const double a  = 6378137.0;
    const double f  = 1.0 / 298.257223563;
    const double e2 = f * (2.0 - f);
    const double r  = std::hypot(p.x(), p.y());
    double       phi = std::atan2(p.z(), r * (1.0 - e2));
    double       n   = a;
    for (int i = 0; i < 10; ++i)
    {
        n   = a / std::sqrt(1.0 - e2 * std::sin(phi) * std::sin(phi));
        *h  = r / std::cos(phi) - n;
        phi = std::atan2(p.z(), r * (1.0 - e2 * n / (n + *h)));
    }
    *lat = deg(phi);
    *lon = deg(std::atan2(p.y(), p.x()));
}

/// A body flying a straight line at constant velocity and turning at a
/// constant rate about a fixed axis: linear interpolation and slerp in ECEF
/// are exact for it.
struct Truth
{
    Eigen::Vector3d    p0;
    Eigen::Vector3d    v;      ///< m/s, ECEF
    Eigen::Quaterniond q0;     ///< q_E_B at t = 0
    Eigen::Vector3d    omega;  ///< rad/s, body frame

    Truth()
    {
        p0 = ta::geodeticToEcef(47.12, 9.43, 900.0);
        const Eigen::Matrix3d R_E_N = ta::rotationEcefFromNed(47.12, 9.43);
        v  = R_E_N * Eigen::Vector3d(12.0, 13.0, -0.5);
        // Heading 47 degrees, slight roll and pitch.
        const Eigen::Quaterniond q_N_B = Eigen::AngleAxisd(0.82, Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(-0.05, Eigen::Vector3d::UnitX());
        q0    = Eigen::Quaterniond(R_E_N) * q_N_B;
        omega = Eigen::Vector3d(0.01, -0.02, 0.15);
    }

    Eigen::Vector3d position(double t) const { return p0 + t * v; }

    Eigen::Quaterniond attitude(double t) const
    {
        const Eigen::Vector3d w = omega * t;
        return q0 * Eigen::Quaterniond(Eigen::AngleAxisd(w.norm(), w.normalized()));
    }

    ta::Sample sample(std::int64_t t_ns) const
    {
        const double t = static_cast<double>(t_ns - kT0) * 1e-9;
        ta::Sample   s;
        s.time_ns = t_ns;
        ecefToGeodetic(position(t), &s.latitude_deg, &s.longitude_deg, &s.height_m);
        const Eigen::Matrix3d R_E_N = ta::rotationEcefFromNed(s.latitude_deg, s.longitude_deg);
        s.q_N_B = (Eigen::Quaterniond(R_E_N.transpose()) * attitude(t)).normalized();
        s.accuracy.position_std_north_m  = 0.02f;
        s.accuracy.attitude_std_yaw_deg  = 3.0f;
        s.accuracy.fix                   = 6;
        return s;
    }
};

ta::Mounting wingtraMounting()
{
    Eigen::Matrix3d R_B_C;
    R_B_C << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    ta::Mounting m;
    m.q_B_C = Eigen::Quaterniond(R_B_C);
    m.t_B_C = Eigen::Vector3d(0.1, -0.02, 0.05);
    return m;
}

void feed(ta::Aligner &a, const Truth &truth, std::int64_t from_ns, std::int64_t to_ns, std::int64_t period_ns)
{
    for (std::int64_t t = from_ns; t <= to_ns; t += period_ns)
    {
        ASSERT_EQ(a.addSample(truth.sample(t)), ta::SampleVerdict::accepted);
    }
}

double angleDeg(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b)
{
    return deg(a.angularDistance(b));
}

}  // namespace

TEST(Geodesy, EcefOfReferencePoints)
{
    const Eigen::Vector3d equator = ta::geodeticToEcef(0.0, 0.0, 0.0);
    EXPECT_NEAR(equator.x(), 6378137.0, 1e-6);
    EXPECT_NEAR(equator.y(), 0.0, 1e-6);
    const Eigen::Vector3d pole = ta::geodeticToEcef(90.0, 0.0, 100.0);
    EXPECT_NEAR(pole.z(), 6356752.314245 + 100.0, 1e-5);

    double lat = 0, lon = 0, h = 0;
    ecefToGeodetic(ta::geodeticToEcef(47.5, 9.25, 1234.5), &lat, &lon, &h);
    EXPECT_NEAR(lat, 47.5, 1e-10);
    EXPECT_NEAR(lon, 9.25, 1e-10);
    EXPECT_NEAR(h, 1234.5, 1e-6);
}

TEST(Geodesy, NedAxesAtTheOrigin)
{
    const Eigen::Matrix3d R = ta::rotationEcefFromNed(0.0, 0.0);
    EXPECT_TRUE(R.col(0).isApprox(Eigen::Vector3d::UnitZ()));   // north
    EXPECT_TRUE(R.col(1).isApprox(Eigen::Vector3d::UnitY()));   // east
    EXPECT_TRUE(R.col(2).isApprox(-Eigen::Vector3d::UnitX()));  // down
    const Eigen::Matrix3d S = ta::rotationEcefFromNed(47.1, 9.4);
    EXPECT_TRUE((S.transpose() * S).isIdentity(1e-12));
    EXPECT_NEAR(S.determinant(), 1.0, 1e-12);
}

TEST(Aligner, InterpolatesTheCameraPose)
{
    const Truth  truth;
    ta::Aligner  a({});
    const auto   m = wingtraMounting();
    a.setMounting(m);
    feed(a, truth, kT0, kT0 + 5 * kS, 100 * kMs);

    const std::vector<std::int64_t> captures = {kT0 + 1234 * kMs, kT0 + 2500 * kMs, kT0 + 3999 * kMs + 7};
    for (std::size_t i = 0; i < captures.size(); ++i)
    {
        a.addImage(i, captures[i], 0);
    }
    const auto poses = a.poll(0);
    ASSERT_EQ(poses.size(), captures.size());
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
        const double             t     = static_cast<double>(captures[i] - kT0) * 1e-9;
        const Eigen::Quaterniond q_E_B = truth.attitude(t);
        const Eigen::Vector3d    p_E_C = truth.position(t) + q_E_B * m.t_B_C;
        const Eigen::Quaterniond q_E_C = q_E_B * m.q_B_C;
        EXPECT_EQ(poses[i].id, i);
        EXPECT_EQ(poses[i].result, ta::Result::valid);
        EXPECT_LT((poses[i].p_E - p_E_C).norm(), 1e-6);
        EXPECT_LT(angleDeg(poses[i].q_E_C, q_E_C), 1e-7);
        EXPECT_GE(poses[i].q_E_C.w(), 0.0);
        EXPECT_LE(poses[i].before_ns, captures[i]);
        EXPECT_GE(poses[i].after_ns, captures[i]);
        EXPECT_EQ(poses[i].after_ns - poses[i].before_ns, i == 1 ? 0 : 100 * kMs);
        EXPECT_FLOAT_EQ(poses[i].accuracy.attitude_std_yaw_deg, 3.0f);
        EXPECT_EQ(poses[i].accuracy.fix, 6);
    }
}

TEST(Aligner, WaitsForTheNextSampleWithoutExtrapolating)
{
    const Truth truth;
    ta::Aligner a({});
    a.setMounting(wingtraMounting());
    feed(a, truth, kT0, kT0 + kS, 100 * kMs);

    a.addImage(1, kT0 + 1050 * kMs, 0);
    EXPECT_TRUE(a.poll(500 * kMs).empty());
    EXPECT_EQ(a.stats().pending_images, 1u);

    ASSERT_EQ(a.addSample(truth.sample(kT0 + 1100 * kMs)), ta::SampleVerdict::accepted);
    const auto poses = a.poll(600 * kMs);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0].result, ta::Result::valid);
    EXPECT_EQ(poses[0].waited_ns, 600 * kMs);
}

TEST(Aligner, RejectsWhenTheWaitExpires)
{
    const Truth truth;
    ta::Aligner a({});
    a.setMounting(wingtraMounting());
    feed(a, truth, kT0, kT0 + kS, 100 * kMs);

    a.addImage(1, kT0 + 1050 * kMs, 0);
    EXPECT_TRUE(a.poll(kS).empty());  // exactly the limit: still waiting
    const auto poses = a.poll(kS + 1);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0].result, ta::Result::wait_expired);
    EXPECT_EQ(poses[0].before_ns, kT0 + kS);
}

TEST(Aligner, RejectsAGapAndAnImageBeforeTheBuffer)
{
    const Truth truth;
    ta::Aligner a({});
    a.setMounting(wingtraMounting());
    feed(a, truth, kT0, kT0 + kS, 100 * kMs);
    feed(a, truth, kT0 + 1400 * kMs, kT0 + 2 * kS, 100 * kMs);  // 0.4 s gap

    a.addImage(1, kT0 - 10 * kMs, 0);
    a.addImage(2, kT0 + 1200 * kMs, 0);
    a.addImage(3, kT0 + 1350 * kMs + 300 * kMs, 0);
    const auto poses = a.poll(0);
    ASSERT_EQ(poses.size(), 3u);
    EXPECT_EQ(poses[0].result, ta::Result::no_sample_before);
    EXPECT_EQ(poses[1].result, ta::Result::gap_too_large);
    EXPECT_EQ(poses[1].before_ns, kT0 + kS);
    EXPECT_EQ(poses[1].after_ns, kT0 + 1400 * kMs);
    EXPECT_EQ(poses[2].result, ta::Result::valid);
}

TEST(Aligner, GapEqualToTheLimitIsValid)
{
    const Truth     truth;
    ta::AlignConfig c;
    c.max_gap_ns = 300 * kMs;
    ta::Aligner a(c);
    a.setMounting(wingtraMounting());
    ASSERT_EQ(a.addSample(truth.sample(kT0)), ta::SampleVerdict::accepted);
    ASSERT_EQ(a.addSample(truth.sample(kT0 + 300 * kMs)), ta::SampleVerdict::accepted);
    a.addImage(1, kT0 + 100 * kMs, 0);
    const auto poses = a.poll(0);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0].result, ta::Result::valid);
}

TEST(Aligner, DiscardsOutOfOrderDuplicateAndInvalidSamples)
{
    const Truth truth;
    ta::Aligner a({});
    EXPECT_EQ(a.addSample(truth.sample(kT0 + kS)), ta::SampleVerdict::accepted);
    EXPECT_EQ(a.addSample(truth.sample(kT0 + kS)), ta::SampleVerdict::duplicate);
    EXPECT_EQ(a.addSample(truth.sample(kT0 + 900 * kMs)), ta::SampleVerdict::out_of_order);

    ta::Sample no_fix = truth.sample(kT0 + 2 * kS);
    no_fix.accuracy.fix = ta::kFixNoFix;
    EXPECT_EQ(a.addSample(no_fix), ta::SampleVerdict::invalid);
    ta::Sample bad_q = truth.sample(kT0 + 2 * kS);
    bad_q.q_N_B.coeffs() *= 1.01;
    EXPECT_EQ(a.addSample(bad_q), ta::SampleVerdict::invalid);
    ta::Sample nan = truth.sample(kT0 + 2 * kS);
    nan.height_m = std::nan("");
    EXPECT_EQ(a.addSample(nan), ta::SampleVerdict::invalid);

    const ta::AlignStats s = a.stats();
    EXPECT_EQ(s.samples_accepted, 1u);
    EXPECT_EQ(s.samples_duplicate, 1u);
    EXPECT_EQ(s.samples_out_of_order, 1u);
    EXPECT_EQ(s.samples_invalid, 3u);
}

TEST(Aligner, KeepsTheInputOrder)
{
    const Truth truth;
    ta::Aligner a({});
    a.setMounting(wingtraMounting());
    feed(a, truth, kT0, kT0 + kS, 100 * kMs);

    a.addImage(1, kT0 + 1050 * kMs, 0);  // waits
    a.addImage(2, kT0 + 500 * kMs, 0);   // known, but after image 1
    EXPECT_TRUE(a.poll(0).empty());
    ASSERT_EQ(a.addSample(truth.sample(kT0 + 1100 * kMs)), ta::SampleVerdict::accepted);
    const auto poses = a.poll(0);
    ASSERT_EQ(poses.size(), 2u);
    EXPECT_EQ(poses[0].id, 1u);
    EXPECT_EQ(poses[1].id, 2u);
}

TEST(Aligner, WaitsForTheCalibration)
{
    const Truth truth;
    ta::Aligner a({});
    feed(a, truth, kT0, kT0 + kS, 100 * kMs);
    a.addImage(1, kT0 + 500 * kMs, 0);
    a.addImage(2, kT0 + 600 * kMs, 2 * kS);
    EXPECT_TRUE(a.poll(kS).empty());

    auto poses = a.poll(kS + 1);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0].result, ta::Result::no_calibration);

    a.setMounting(wingtraMounting());
    poses = a.poll(kS + 2);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0].result, ta::Result::valid);
}

TEST(Aligner, LimitsTheWaitingImages)
{
    const Truth     truth;
    ta::AlignConfig c;
    c.max_pending = 2;
    ta::Aligner a(c);
    a.setMounting(wingtraMounting());
    feed(a, truth, kT0, kT0 + kS, 100 * kMs);
    a.addImage(1, kT0 + 2 * kS, 0);
    a.addImage(2, kT0 + 2 * kS + 1, 0);
    a.addImage(3, kT0 + 2 * kS + 2, 0);
    EXPECT_TRUE(a.poll(0).empty());
    const auto poses = a.poll(2 * kS);
    ASSERT_EQ(poses.size(), 3u);
    EXPECT_EQ(poses[0].result, ta::Result::wait_expired);
    EXPECT_EQ(poses[1].result, ta::Result::wait_expired);
    EXPECT_EQ(poses[2].result, ta::Result::queue_full);
}

TEST(Aligner, ClearReleasesTheWaitingImages)
{
    const Truth truth;
    ta::Aligner a({});
    a.setMounting(wingtraMounting());
    feed(a, truth, kT0, kT0 + kS, 100 * kMs);
    a.addImage(1, kT0 + 2 * kS, 0);
    a.clear();
    EXPECT_FALSE(a.hasMounting());
    EXPECT_EQ(a.stats().buffered_samples, 0u);
    const auto poses = a.poll(0);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0].result, ta::Result::cleared);
}

TEST(Aligner, BufferKeepsOneSampleBeforeItsLimit)
{
    const Truth     truth;
    ta::AlignConfig c;
    c.buffer_ns = 2 * kS;
    ta::Aligner a(c);
    a.setMounting(wingtraMounting());
    feed(a, truth, kT0, kT0 + 5 * kS, 100 * kMs);
    ta::AlignStats s = a.stats();
    EXPECT_EQ(s.buffered_samples, 21u);
    EXPECT_EQ(s.buffer_span_ns, 2 * kS);

    // A 20 s telemetry gap: the last sample before it stays, so an image in
    // the gap is a gap, not older than the buffer.
    ASSERT_EQ(a.addSample(truth.sample(kT0 + 25 * kS)), ta::SampleVerdict::accepted);
    s = a.stats();
    EXPECT_EQ(s.buffered_samples, 2u);
    a.addImage(1, kT0 + 10 * kS, 0);
    const auto poses = a.poll(0);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0].result, ta::Result::gap_too_large);
}

TEST(Aligner, RejectsInvalidConfiguration)
{
    ta::AlignConfig c;
    c.max_gap_ns = 0;
    EXPECT_THROW(ta::Aligner a(c), std::invalid_argument);
    ta::Aligner  b({});
    ta::Mounting m;
    m.q_B_C.coeffs() << 0.0, 0.0, 0.0, 2.0;
    EXPECT_THROW(b.setMounting(m), std::invalid_argument);
}

TEST(CApi, AlignsFromC)
{
    EXPECT_EQ(ost_time_align_c_api_usage(), 0);
}

TEST(CApi, QueuesBeyondTheCapacity)
{
    const Truth        truth;
    OST_Align_Config_t c;
    OST_Align_DefaultConfig(&c);
    EXPECT_EQ(c.max_gap_ns, 300 * kMs);
    EXPECT_EQ(c.max_wait_ns, kS);
    EXPECT_EQ(c.buffer_ns, 10 * kS);
    OST_Align_t *h = nullptr;
    ASSERT_EQ(OST_Align_Create(&c, &h), OST_SUCCESS);

    const double q[4] = {1, 0, 0, 0};
    const double t[3] = {0, 0, 0};
    ASSERT_EQ(OST_Align_SetMounting(h, q, t), OST_SUCCESS);
    const double bad_q[4] = {2, 0, 0, 0};
    EXPECT_EQ(OST_Align_SetMounting(h, bad_q, t), OST_ERR_INVALID_ARGUMENT);
    EXPECT_STRNE(OST_Align_LastError(h), "");

    for (std::int64_t k = 0; k <= 10; ++k)
    {
        const ta::Sample   s = truth.sample(kT0 + k * 100 * kMs);
        OST_Align_Sample_t cs{};
        cs.time_ns       = s.time_ns;
        cs.latitude_deg  = s.latitude_deg;
        cs.longitude_deg = s.longitude_deg;
        cs.height_m      = s.height_m;
        cs.q_ned_from_body[0] = s.q_N_B.w();
        cs.q_ned_from_body[1] = s.q_N_B.x();
        cs.q_ned_from_body[2] = s.q_N_B.y();
        cs.q_ned_from_body[3] = s.q_N_B.z();
        cs.accuracy.fix  = 6;
        int verdict      = -1;
        ASSERT_EQ(OST_Align_AddSample(h, &cs, &verdict), OST_SUCCESS);
        EXPECT_EQ(verdict, OST_ALIGN_SAMPLE_ACCEPTED);
    }
    for (std::uint64_t id = 0; id < 3; ++id)
    {
        ASSERT_EQ(OST_Align_AddImage(h, id, kT0 + static_cast<std::int64_t>(id) * 200 * kMs + 50 * kMs, 0), OST_SUCCESS);
    }
    OST_Align_ImagePose_t out[2];
    uint32_t              n = 0;
    ASSERT_EQ(OST_Align_Poll(h, 0, out, 2, &n), OST_SUCCESS);
    ASSERT_EQ(n, 2u);
    EXPECT_EQ(out[0].id, 0u);
    EXPECT_EQ(out[1].id, 1u);
    OST_Align_Stats_t st;
    ASSERT_EQ(OST_Align_GetStats(h, &st), OST_SUCCESS);
    EXPECT_EQ(st.pending_images, 1u);
    ASSERT_EQ(OST_Align_Poll(h, 0, out, 2, &n), OST_SUCCESS);
    ASSERT_EQ(n, 1u);
    EXPECT_EQ(out[0].id, 2u);
    EXPECT_EQ(out[0].result, OST_ALIGN_RESULT_VALID);
    EXPECT_STREQ(OST_Align_ResultName(OST_ALIGN_RESULT_GAP_TOO_LARGE), "telemetry gap around the capture");

    OST_Align_Destroy(h);
}
