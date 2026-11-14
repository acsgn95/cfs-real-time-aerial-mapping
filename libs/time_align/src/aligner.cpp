// Time alignment of images with the telemetry stream (aligner.hpp).

#include "orthostream/time_align/aligner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace orthostream::time_align
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kA  = 6378137.0;              // WGS 84 semi-major axis
constexpr double kF  = 1.0 / 298.257223563;    // WGS 84 flattening
constexpr double kE2 = kF * (2.0 - kF);

double rad(double deg)
{
    return deg * kPi / 180.0;
}

Eigen::Quaterniond canonical(const Eigen::Quaterniond &q)
{
    Eigen::Quaterniond n = q.normalized();
    if (n.w() < 0.0)
    {
        n.coeffs() = -n.coeffs();
    }
    return n;
}

bool finite(double v)
{
    return std::isfinite(v);
}

}  // namespace

Accuracy Accuracy::worse(const Accuracy &a, const Accuracy &b)
{
    Accuracy r;
    r.position_std_north_m   = std::max(a.position_std_north_m, b.position_std_north_m);
    r.position_std_east_m    = std::max(a.position_std_east_m, b.position_std_east_m);
    r.position_std_down_m    = std::max(a.position_std_down_m, b.position_std_down_m);
    r.attitude_std_roll_deg  = std::max(a.attitude_std_roll_deg, b.attitude_std_roll_deg);
    r.attitude_std_pitch_deg = std::max(a.attitude_std_pitch_deg, b.attitude_std_pitch_deg);
    r.attitude_std_yaw_deg   = std::max(a.attitude_std_yaw_deg, b.attitude_std_yaw_deg);
    r.fix                    = std::min(a.fix, b.fix);
    return r;
}

const char *resultName(Result result)
{
    switch (result)
    {
        case Result::valid:            return "valid";
        case Result::no_sample_before: return "no telemetry before the capture";
        case Result::gap_too_large:    return "telemetry gap around the capture";
        case Result::wait_expired:     return "no telemetry after the capture";
        case Result::no_calibration:   return "no camera calibration";
        case Result::queue_full:       return "too many images waiting";
        case Result::cleared:          return "session changed";
    }
    return "unknown";
}

Eigen::Vector3d geodeticToEcef(double latitude_deg, double longitude_deg, double height_m)
{
    const double lat = rad(latitude_deg);
    const double lon = rad(longitude_deg);
    const double s   = std::sin(lat);
    const double n   = kA / std::sqrt(1.0 - kE2 * s * s);
    return {(n + height_m) * std::cos(lat) * std::cos(lon), (n + height_m) * std::cos(lat) * std::sin(lon),
            (n * (1.0 - kE2) + height_m) * s};
}

Eigen::Matrix3d rotationEcefFromNed(double latitude_deg, double longitude_deg)
{
    const double sl = std::sin(rad(latitude_deg));
    const double cl = std::cos(rad(latitude_deg));
    const double so = std::sin(rad(longitude_deg));
    const double co = std::cos(rad(longitude_deg));
    Eigen::Matrix3d R;
    // Columns: north, east and down in ECEF.
    R << -sl * co, -so, -cl * co,
         -sl * so,  co, -cl * so,
          cl,      0.0, -sl;
    return R;
}

Aligner::Aligner(const AlignConfig &config) : config_(config)
{
    if (config.max_gap_ns <= 0 || config.max_wait_ns < 0 || config.buffer_ns <= 0 || config.max_pending == 0)
    {
        throw std::invalid_argument("time_align: gap, buffer and queue limits must be positive, the wait not negative");
    }
}

void Aligner::setMounting(const Mounting &mounting)
{
    const double n = mounting.q_B_C.norm();
    if (!mounting.t_B_C.allFinite() || !mounting.q_B_C.coeffs().allFinite() || std::abs(n - 1.0) > 1e-6)
    {
        throw std::invalid_argument("time_align: mounting not finite or not a unit quaternion");
    }
    mounting_       = mounting;
    mounting_.q_B_C = canonical(mounting.q_B_C);
    has_mounting_   = true;
}

SampleVerdict Aligner::addSample(const Sample &s)
{
    const Eigen::Quaterniond &q = s.q_N_B;
    if (!finite(s.latitude_deg) || !finite(s.longitude_deg) || !finite(s.height_m) || !q.coeffs().allFinite() ||
        std::abs(q.norm() - 1.0) > 1e-6 || std::abs(s.latitude_deg) > 90.0 || s.accuracy.fix == kFixNoFix)
    {
        ++stats_.samples_invalid;
        return SampleVerdict::invalid;
    }
    if (!samples_.empty())
    {
        if (s.time_ns == samples_.back().time_ns)
        {
            ++stats_.samples_duplicate;
            return SampleVerdict::duplicate;
        }
        if (s.time_ns < samples_.back().time_ns)
        {
            ++stats_.samples_out_of_order;
            return SampleVerdict::out_of_order;
        }
    }

    Buffered b;
    b.time_ns  = s.time_ns;
    b.p_E      = geodeticToEcef(s.latitude_deg, s.longitude_deg, s.height_m);
    b.q_E_B    = canonical(Eigen::Quaterniond(rotationEcefFromNed(s.latitude_deg, s.longitude_deg)) * q);
    b.accuracy = s.accuracy;
    samples_.push_back(b);
    ++stats_.samples_accepted;

    // Keep the samples of the last buffer_ns, and one sample at or before
    // that limit, so that an image in a long gap is reported as a gap and
    // not as being older than the buffer.
    const std::int64_t cutoff = s.time_ns - config_.buffer_ns;
    while (samples_.size() >= 2 && samples_[1].time_ns <= cutoff)
    {
        samples_.pop_front();
    }
    return SampleVerdict::accepted;
}

void Aligner::addImage(std::uint64_t id, std::int64_t capture_ns, std::int64_t arrival_ns)
{
    const std::size_t waiting =
        static_cast<std::size_t>(std::count_if(pending_.begin(), pending_.end(), [](const Pending &p) { return !p.decided; }));
    Pending p;
    p.pose.id         = id;
    p.pose.capture_ns = capture_ns;
    p.arrival_ns      = arrival_ns;
    if (waiting >= config_.max_pending)
    {
        p.pose.result = Result::queue_full;
        p.decided     = true;
    }
    pending_.push_back(p);
}

void Aligner::cameraPose(const Buffered &a, const Buffered &b, double u, ImagePose *pose) const
{
    const Eigen::Vector3d    p_E_B = a.p_E + u * (b.p_E - a.p_E);
    const Eigen::Quaterniond q_E_B = a.q_E_B.slerp(u, b.q_E_B).normalized();
    pose->p_E      = p_E_B + q_E_B * mounting_.t_B_C;
    pose->q_E_C    = canonical(q_E_B * mounting_.q_B_C);
    pose->accuracy = Accuracy::worse(a.accuracy, b.accuracy);
    pose->before_ns = a.time_ns;
    pose->after_ns  = b.time_ns;
}

bool Aligner::decide(Pending &p, std::int64_t now_ns) const
{
    ImagePose         &pose    = p.pose;
    const bool         expired = now_ns - p.arrival_ns > config_.max_wait_ns;
    const std::int64_t t       = pose.capture_ns;

    pose.waited_ns = std::max<std::int64_t>(0, now_ns - p.arrival_ns);

    auto it = std::lower_bound(samples_.begin(), samples_.end(), t,
                               [](const Buffered &s, std::int64_t v) { return s.time_ns < v; });
    if (it == samples_.end())
    {
        // No sample at or after the capture yet: wait (no extrapolation).
        if (!expired)
        {
            return false;
        }
        pose.result = Result::wait_expired;
        if (!samples_.empty())
        {
            pose.before_ns = samples_.back().time_ns;
        }
        return true;
    }
    if (it->time_ns != t)
    {
        if (it == samples_.begin())
        {
            pose.result   = Result::no_sample_before;
            pose.after_ns = it->time_ns;
            return true;
        }
        const Buffered &a = *(it - 1);
        if (it->time_ns - a.time_ns > config_.max_gap_ns)
        {
            pose.result    = Result::gap_too_large;
            pose.before_ns = a.time_ns;
            pose.after_ns  = it->time_ns;
            return true;
        }
    }
    if (!has_mounting_)
    {
        if (!expired)
        {
            return false;
        }
        pose.result = Result::no_calibration;
        return true;
    }
    if (it->time_ns == t)
    {
        cameraPose(*it, *it, 0.0, &pose);
    }
    else
    {
        const Buffered &a = *(it - 1);
        const double    u = static_cast<double>(t - a.time_ns) / static_cast<double>(it->time_ns - a.time_ns);
        cameraPose(a, *it, u, &pose);
    }
    pose.result = Result::valid;
    return true;
}

std::vector<ImagePose> Aligner::poll(std::int64_t now_ns)
{
    for (Pending &p : pending_)
    {
        if (!p.decided)
        {
            p.decided = decide(p, now_ns);
        }
    }
    std::vector<ImagePose> out;
    while (!pending_.empty() && pending_.front().decided)
    {
        out.push_back(pending_.front().pose);
        pending_.pop_front();
    }
    return out;
}

void Aligner::clear()
{
    samples_.clear();
    has_mounting_ = false;
    mounting_     = Mounting{};
    for (Pending &p : pending_)
    {
        if (!p.decided)
        {
            p.pose.result = Result::cleared;
            p.decided     = true;
        }
    }
}

AlignStats Aligner::stats() const
{
    AlignStats s       = stats_;
    s.buffered_samples = samples_.size();
    s.buffer_span_ns   = samples_.empty() ? 0 : samples_.back().time_ns - samples_.front().time_ns;
    s.pending_images   = pending_.size();
    return s;
}

}  // namespace orthostream::time_align
