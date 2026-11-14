// Time alignment of images with the telemetry stream (ADR-014 section 4).
//
// Telemetry samples (body pose: geodetic position and q_N_B) are buffered.
// The camera pose at an image's capture time is interpolated between the two
// samples around it: linear interpolation of the body position in ECEF and
// slerp of q_E_B, then the camera mounting (q_B_C, t_B_C) is applied. Both
// samples are converted to ECEF first, so the interpolation does not mix the
// local NED frames of two positions.
//
// There is no extrapolation. An image whose next sample has not arrived
// waits; it is rejected when the waiting limit expires. An association is
// valid when the two samples are at most max_gap_ns apart. Out-of-order and
// duplicate samples are discarded.
//
// Times: sample and capture times are data time (UTC ns, ADR-014 section 2).
// The waiting time is measured in the clock of arrival_ns and now_ns, which
// the caller chooses (CLOCK_MONOTONIC in ALIGN_APP).

#ifndef ORTHOSTREAM_TIME_ALIGN_ALIGNER_HPP
#define ORTHOSTREAM_TIME_ALIGN_ALIGNER_HPP

#include <cstdint>
#include <deque>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orthostream::time_align
{

struct AlignConfig
{
    std::int64_t max_gap_ns  = 300'000'000;     ///< gap between the two samples around an image
    std::int64_t max_wait_ns = 1'000'000'000;   ///< waiting time for the next sample
    std::int64_t buffer_ns   = 10'000'000'000;  ///< telemetry kept, back from the newest sample
    std::size_t  max_pending = 64;              ///< images waiting at the same time
};

/// Reported accuracy, one standard deviation (OST/PoseAccuracy).
struct Accuracy
{
    float         position_std_north_m = 0.0f;
    float         position_std_east_m  = 0.0f;
    float         position_std_down_m  = 0.0f;
    float         attitude_std_roll_deg  = 0.0f;
    float         attitude_std_pitch_deg = 0.0f;
    float         attitude_std_yaw_deg   = 0.0f;
    std::uint8_t  fix = 0;  ///< OST/PositionFix

    /// The worse of two accuracies, per component (the larger deviation and
    /// the lower fix).
    static Accuracy worse(const Accuracy &a, const Accuracy &b);
};

/// Position fix values of OST/PositionFix used here.
constexpr std::uint8_t kFixNoFix = 1;

struct Sample
{
    std::int64_t       time_ns       = 0;  ///< measurement time
    double             latitude_deg  = 0.0;
    double             longitude_deg = 0.0;
    double             height_m      = 0.0;  ///< ellipsoidal
    Eigen::Quaterniond q_N_B = Eigen::Quaterniond::Identity();  ///< body to local NED
    Accuracy           accuracy;
};

/// Camera mounting from the session calibration.
struct Mounting
{
    Eigen::Quaterniond q_B_C = Eigen::Quaterniond::Identity();  ///< camera to body
    Eigen::Vector3d    t_B_C = Eigen::Vector3d::Zero();         ///< camera centre in the body frame, m
};

enum class SampleVerdict
{
    accepted,
    out_of_order,  ///< older than the newest sample
    duplicate,     ///< same time as the newest sample
    invalid,       ///< not finite, not a unit quaternion, or no position fix
};

/// Outcome of an image (values are those of ALIGN_APP/AlignResult).
enum class Result : std::uint8_t
{
    valid            = 0,
    no_sample_before = 1,  ///< captured before the oldest buffered sample
    gap_too_large    = 2,  ///< the samples around it are more than max_gap_ns apart
    wait_expired     = 3,  ///< no sample after it within max_wait_ns
    no_calibration   = 4,  ///< no camera mounting within max_wait_ns
    queue_full       = 5,  ///< max_pending images were already waiting
    cleared          = 6,  ///< waiting when the aligner was cleared (session change)
};

const char *resultName(Result result);

struct ImagePose
{
    std::uint64_t      id         = 0;
    std::int64_t       capture_ns = 0;
    Result             result     = Result::valid;
    Eigen::Vector3d    p_E        = Eigen::Vector3d::Zero();  ///< camera centre, ECEF
    Eigen::Quaterniond q_E_C      = Eigen::Quaterniond::Identity();  ///< camera to ECEF, w >= 0
    Accuracy           accuracy;
    std::int64_t       before_ns = 0;  ///< sample times used; 0 if none
    std::int64_t       after_ns  = 0;
    std::int64_t       waited_ns = 0;  ///< from arrival to the outcome
};

struct AlignStats
{
    std::uint64_t samples_accepted     = 0;
    std::uint64_t samples_out_of_order = 0;
    std::uint64_t samples_duplicate    = 0;
    std::uint64_t samples_invalid      = 0;
    std::size_t   buffered_samples     = 0;
    std::int64_t  buffer_span_ns       = 0;
    std::size_t   pending_images       = 0;
};

/// Geodetic (WGS 84, degrees, ellipsoidal height) to ECEF; closed form, no PROJ.
Eigen::Vector3d geodeticToEcef(double latitude_deg, double longitude_deg, double height_m);

/// Rotation R_E_N from local north-east-down at the given position to ECEF.
Eigen::Matrix3d rotationEcefFromNed(double latitude_deg, double longitude_deg);

class Aligner
{
public:
    explicit Aligner(const AlignConfig &config);

    const AlignConfig &config() const { return config_; }

    void setMounting(const Mounting &mounting);
    bool hasMounting() const { return has_mounting_; }

    SampleVerdict addSample(const Sample &sample);

    /// Queues an image. arrival_ns is in the clock of poll(now_ns).
    void addImage(std::uint64_t id, std::int64_t capture_ns, std::int64_t arrival_ns);

    /// Decides the images that can be decided at now_ns and returns them in
    /// the order they were added. An image whose outcome is known waits for
    /// the images added before it, so the output order is the input order.
    std::vector<ImagePose> poll(std::int64_t now_ns);

    /// New session: discards the samples and the mounting; the waiting
    /// images are returned by the next poll with Result::cleared.
    void clear();

    AlignStats stats() const;

private:
    struct Buffered
    {
        std::int64_t       time_ns = 0;
        Eigen::Vector3d    p_E     = Eigen::Vector3d::Zero();  ///< body origin
        Eigen::Quaterniond q_E_B   = Eigen::Quaterniond::Identity();
        Accuracy           accuracy;
    };
    struct Pending
    {
        ImagePose    pose;
        std::int64_t arrival_ns = 0;
        bool         decided    = false;
    };

    bool decide(Pending &p, std::int64_t now_ns) const;
    void cameraPose(const Buffered &a, const Buffered &b, double u, ImagePose *pose) const;

    AlignConfig          config_;
    Mounting             mounting_;
    bool                 has_mounting_ = false;
    std::deque<Buffered> samples_;
    std::deque<Pending>  pending_;
    AlignStats           stats_;
};

}  // namespace orthostream::time_align

#endif  // ORTHOSTREAM_TIME_ALIGN_ALIGNER_HPP
