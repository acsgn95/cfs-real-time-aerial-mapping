// Sliding-window refinement (ADR-019 section 3): the form that runs during
// the flight. Images arrive one by one with their telemetry poses; keyframe
// batches arrive from the selection. A batch is refined when the next
// flight line covers its keyframe or when its waiting limit expires.
//
// Each batch is solved in a local window: the images whose footprints lie
// near the keyframe's, at most max_window_images of them. Features are
// extracted once per image and pair matches are kept, so every image and
// every pair is processed once however many windows use it. Images refined
// before enter with their refined pose as a tight prior. The shared
// boresight is estimated in the first batches and then held.
//
// Poses are T_W_C in a local metric frame W whose z axis points up (the
// tools use the flight frame; REFINE_APP a local ENU frame).

#ifndef ORTHOSTREAM_REFINE_SLIDING_HPP
#define ORTHOSTREAM_REFINE_SLIDING_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "orthostream/refine/refine.hpp"

namespace orthostream::refine
{

struct SlidingConfig
{
    RefineConfig refine;  ///< priors, features, matching and checks

    // Defaults from the benchmark of 2026-11-14 (Dorfberg, keyframes 60-80,
    // 2048 px): 24 images and 4000 points took 4.4 s per batch, 16 images
    // and 2000 points 2.1 s with the same accuracy (keyframe interval 2.9 s).
    std::size_t max_window_images     = 16;    ///< images of one local solution
    std::size_t max_points            = 2000;  ///< tie points of one local solution
    int         max_iterations        = 30;
    double      relative_error_tol    = 1e-5;
    double      window_radius_factor  = 1.2;   ///< x footprint width: images near the keyframe
    double      pair_distance_factor  = 0.8;   ///< x footprint width: pairs that are matched

    /// Prior of an image refined before (ADR-019 section 3, item 5).
    double refined_attitude_sigma_deg = 0.02;
    double refined_position_sigma_m   = 0.05;  ///< unused with precise_position

    int    boresight_settle_batches  = 4;      ///< refined batches before the boresight is held
    double held_boresight_sigma_deg  = 0.005;

    /// Waiting (section 4): a batch waits until an image of a later flight
    /// line, flying the other way, sees its keyframe's footprint centre, at
    /// most max_wait_s after the keyframe's capture.
    double max_wait_s               = 100.0;
    double coverage_distance_factor = 0.4;     ///< x footprint width
    double line_turn_deg            = 45.0;    ///< a new line starts after this change of direction

    /// Images older than this, and not part of a waiting batch, leave the
    /// window (memory bound, ADR-001 R4). They keep their last refined pose.
    double window_s = 240.0;
};

struct SlidingImage
{
    std::uint64_t     id      = 0;
    std::int64_t      time_ns = 0;  ///< capture time
    GreyImage         image;        ///< working image; only used during addImage
    Eigen::Isometry3d T_W_C = Eigen::Isometry3d::Identity();  ///< telemetry pose
    double attitude_sigma_deg = 0.0;  ///< reported accuracy; 0: the configured one
    double position_sigma_m   = 0.0;  ///< reported accuracy; 0: the configured one
};

struct SlidingBatch
{
    std::uint64_t              id       = 0;
    std::uint64_t              keyframe = 0;  ///< image id
    std::vector<std::uint64_t> images;        ///< keyframe and source views, image ids
    double                     ground_z = 0.0;  ///< footprint plane height in W
};

struct SlidingResult
{
    std::uint64_t batch_id = 0;
    PoseSource    source   = PoseSource::refine_failed;
    std::string   reason;  ///< why refinement failed, empty otherwise
    /// Poses of the batch images, in the order of SlidingBatch::images:
    /// refined, or the telemetry poses if refinement failed.
    std::vector<Eigen::Isometry3d> T_W_C;

    bool        covered        = false;  ///< false: solved because the waiting limit expired
    double      waited_s       = 0.0;    ///< data time from the keyframe capture to the solution
    std::size_t window_images  = 0;
    std::size_t pairs          = 0;
    std::size_t new_pairs      = 0;      ///< matched for this batch
    std::size_t points         = 0;
    std::size_t observations   = 0;
    int         iterations     = 0;
    double      rms_before_px  = 0.0;
    double      rms_after_px   = 0.0;
    double      boresight_deg  = 0.0;    ///< angle of the boresight used or estimated
    bool        boresight_held = false;
    double      matching_s     = 0.0;
    double      adjustment_s   = 0.0;
};

struct SlidingStats
{
    std::size_t images          = 0;  ///< in the window
    std::size_t pending_batches = 0;
    std::size_t cached_pairs    = 0;
    std::size_t refined_batches = 0;
    std::size_t failed_batches  = 0;
    double      features_s      = 0.0;  ///< total feature extraction time
};

class SlidingRefiner
{
public:
    SlidingRefiner(const geo::CameraModel &camera, const SlidingConfig &config,
                   std::unique_ptr<FeatureExtractor> extractor = nullptr);
    ~SlidingRefiner();

    SlidingRefiner(const SlidingRefiner &)            = delete;
    SlidingRefiner &operator=(const SlidingRefiner &) = delete;

    /// Extracts the features of an image. Ids must be unique. Images may
    /// arrive out of capture order (the source views of a batch can be older
    /// than images added before).
    void addImage(const SlidingImage &image);

    /// A batch whose images were all added before.
    void addBatch(const SlidingBatch &batch);

    /// Solves the waiting batches that are ready at data time now_ns
    /// (normally the capture time of the latest image), oldest first. With
    /// flush, all waiting batches are solved (end of the session).
    std::vector<SlidingResult> poll(std::int64_t now_ns, bool flush = false);

    /// The image is in the window (added and not yet pruned).
    bool hasImage(std::uint64_t id) const;

    /// Last refined pose of an image in the window, if any.
    bool refinedPose(std::uint64_t id, Eigen::Isometry3d *T_W_C) const;

    SlidingStats stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orthostream::refine

#endif  // ORTHOSTREAM_REFINE_SLIDING_HPP
