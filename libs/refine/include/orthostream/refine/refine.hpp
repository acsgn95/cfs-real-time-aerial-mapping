// Optional pose refinement (ADR-019): image features, matching, tie point
// tracks and a bundle adjustment with telemetry priors (GTSAM).
//
// The images are the undistorted working images of the dense stage, so the
// camera is a pinhole camera without distortion. Poses are camera-to-world
// transforms T_W_C in a local metric frame W (ADR-004 section 4); OpenCV
// camera axes (x right, y down, z forward).

#ifndef ORTHOSTREAM_REFINE_REFINE_HPP
#define ORTHOSTREAM_REFINE_REFINE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orthostream/geo/camera.hpp"

namespace orthostream::refine
{

class RefineError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

struct RefineConfig
{
    // Priors from the telemetry (ADR-019 sections 1 and 3).
    bool   precise_position       = false;  ///< PPK/RTK: positions held by a tight prior
    double position_sigma         = 2.0;    ///< m, per axis, when precise_position is false
    double precise_position_sigma = 0.02;   ///< m, per axis, when precise_position is true
    double attitude_sigma_deg     = 3.0;    ///< per axis, of each image's recorded attitude
    /// One rotation shared by all images, applied in the camera frame:
    /// R_W_C = R_W_C(recorded) * R_boresight. It absorbs the misalignment of
    /// the camera mounting, which is common to the whole flight.
    bool   estimate_boresight     = true;
    double boresight_sigma_deg    = 5.0;

    // Tie point measurements.
    double pixel_sigma          = 1.0;  ///< pixels
    double huber_threshold      = 2.0;  ///< in units of pixel_sigma
    double outlier_threshold_px = 4.0;  ///< observations dropped after the first solution
    double min_triangulation_deg = 1.0; ///< largest ray angle of a track
    std::size_t max_points      = 30000; ///< longest tracks are kept
    int    max_iterations       = 100;

    // Features and matching.
    int    max_features        = 4000;
    double ratio_test          = 0.8;
    double ransac_threshold_px = 1.5;
    int    min_pair_inliers    = 30;   ///< pairs with fewer inliers contribute nothing

    // Checks (ADR-019 section 5).
    std::size_t min_image_observations = 50;
    double      max_reprojection_rms_px = 2.0;
    double      max_correction_sigmas   = 5.0;  ///< per-image correction against its prior sigma
};

/// A grey 8-bit image; rows are stride bytes apart.
struct GreyImage
{
    const std::uint8_t *data   = nullptr;
    int                 width  = 0;
    int                 height = 0;
    std::size_t         stride = 0;
};

/// Keypoints (pixel coordinates) and their descriptors, one row each.
struct Features
{
    std::vector<Eigen::Vector2d> keypoints;
    int                          descriptor_size = 0;  ///< values per descriptor
    std::vector<float>           descriptors;           ///< keypoints.size() x descriptor_size
};

/// Image feature front end. SIFT is the first implementation; others (ORB,
/// learned features such as XFeat) implement the same interface, and the
/// benchmark decides (ADR-019 open questions).
class FeatureExtractor
{
public:
    virtual ~FeatureExtractor() = default;
    virtual Features    extract(const GreyImage &image) = 0;
    virtual std::string name() const                    = 0;
};

/// OpenCV SIFT with at most max_features keypoints.
std::unique_ptr<FeatureExtractor> makeSiftExtractor(int max_features);

/// One observation of a tie point: image index and pixel position.
struct Observation
{
    std::size_t     image = 0;
    Eigen::Vector2d pixel;
};
using Track = std::vector<Observation>;

/// Inlier matches between two images, as keypoint index pairs.
struct PairMatches
{
    std::size_t                                     image_a = 0;
    std::size_t                                     image_b = 0;
    std::size_t                                     candidates = 0;  ///< matches after the ratio test
    std::vector<std::pair<std::size_t, std::size_t>> inliers;
};

/// Matches the descriptors of two images (ratio test) and keeps the matches
/// consistent with one essential matrix (RANSAC). The check uses no pose,
/// so it also works when the recorded attitude is wrong.
PairMatches matchPair(const Features &a, const Features &b, const geo::CameraModel &camera,
                      const RefineConfig &config);

/// Joins pairwise matches into tracks (union-find). A track that contains
/// two keypoints of the same image is inconsistent and dropped; tracks have
/// at least two observations.
std::vector<Track> buildTracks(const std::vector<std::vector<Eigen::Vector2d>> &keypoints,
                               const std::vector<PairMatches>                  &matches);

struct AdjustmentResult
{
    std::vector<Eigen::Isometry3d> T_W_C;      ///< refined poses, boresight included
    Eigen::Matrix3d                boresight = Eigen::Matrix3d::Identity();
    std::vector<std::size_t>       observations_per_image;
    std::size_t                    points       = 0;  ///< triangulated tie points used
    std::size_t                    observations = 0;  ///< after outlier removal
    std::size_t                    rejected     = 0;  ///< observations removed as outliers
    double                         rms_before_px = 0.0;  ///< at the priors, points triangulated from them
    double                         rms_after_px  = 0.0;
    double max_rotation_correction_deg = 0.0;  ///< largest per-image change, boresight excluded
    double max_position_correction_m   = 0.0;
};

/// Bundle adjustment with GNSS/attitude priors: triangulates the tracks
/// from the prior poses, solves with Levenberg-Marquardt and a Huber loss,
/// removes outliers and solves again. The camera must have no distortion.
AdjustmentResult adjust(const geo::CameraModel &camera, const std::vector<Eigen::Isometry3d> &priors,
                        const std::vector<Track> &tracks, const RefineConfig &config);

enum class PoseSource
{
    telemetry,      ///< refinement off
    refined,
    refine_failed,  ///< checks failed; the telemetry poses are passed on
};

struct RefineResult
{
    PoseSource                     source = PoseSource::refine_failed;
    std::string                    reason;  ///< why refinement failed, empty otherwise
    std::vector<Eigen::Isometry3d> T_W_C;   ///< refined, or the priors if it failed
    AdjustmentResult               adjustment;
    std::size_t                    pairs_used   = 0;
    std::size_t                    pairs_tried  = 0;
    std::size_t                    tracks       = 0;
    double                         features_s   = 0.0;
    double                         matching_s   = 0.0;
    double                         adjustment_s = 0.0;
};

/// Refinement of one window of images: add the images with their prior
/// poses, add the pairs to match, then solve.
class Refiner
{
public:
    Refiner(const geo::CameraModel &camera, const RefineConfig &config,
            std::unique_ptr<FeatureExtractor> extractor = nullptr);
    ~Refiner();

    Refiner(const Refiner &)            = delete;
    Refiner &operator=(const Refiner &) = delete;

    /// Extracts the features of an image; returns its index.
    std::size_t addImage(const GreyImage &image, const Eigen::Isometry3d &T_W_C_prior);

    /// Matches the given image pairs (in parallel).
    void addPairs(const std::vector<std::pair<std::size_t, std::size_t>> &pairs);

    std::size_t     images() const;
    const Features &features(std::size_t image) const;

    /// Builds the tracks and runs the adjustment and the checks of
    /// ADR-019 section 5.
    RefineResult solve();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orthostream::refine

#endif  // ORTHOSTREAM_REFINE_REFINE_HPP
