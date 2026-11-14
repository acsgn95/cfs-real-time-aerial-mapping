// Incremental keyframe and source view selection with an image pool, as
// SELECT_APP runs it (ADR-006): images arrive one by one with their poses.
//
// For each image: footprint on the terrain prior, flight line, keyframe
// decision, and for a keyframe its source views among the images in the
// pool. Causal mode: only images that arrived before it. Delayed mode
// (ADR-005 section 2, the default there): the keyframe waits until the
// next keyframe arrives, and its sources are chosen among all images
// before that one, also those after the keyframe. Images with a footprint
// join the pool. The pool is bounded by a distance from the
// current position and by a number of images; evicted images are returned
// to the caller, who owns their data (SELECT_APP: the working images in
// the data store).
//
// Geometry runs in a local ENU frame at the first image. The terrain and
// the geodetic conversion are given as functions, so the selector has no
// files (SELECT_APP: libs/terrain; tests: a synthetic surface).

#ifndef ORTHOSTREAM_SELECTION_SELECTOR_HPP
#define ORTHOSTREAM_SELECTION_SELECTOR_HPP

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

#include "orthostream/geo/camera.hpp"
#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/frames.hpp"
#include "orthostream/selection/footprint.hpp"
#include "orthostream/selection/views.hpp"

namespace orthostream::selection
{

/// The terrain point at the horizontal position of an ECEF point, in ECEF,
/// or nothing where there is no terrain.
using SurfaceFunction = std::function<std::optional<Eigen::Vector3d>(const Eigen::Vector3d &p_E)>;

/// WGS 84 geodetic coordinates of an ECEF point.
using GeodeticFunction = std::function<geo::Geodetic(const Eigen::Vector3d &p_E)>;

struct SelectorConfig
{
    FootprintConfig  footprint;
    KeyframeConfig   keyframe;
    SourceViewConfig sources;
    double           line_turn_deg   = 45.0;   ///< a new flight line after this change of direction
    double           pool_distance_m = 600.0;  ///< footprint centres farther from the current position leave the pool
    std::size_t      max_pool        = 200;    ///< images in the pool
    bool             delayed         = false;  ///< sources also from images up to the next keyframe
};

/// The sources of a keyframe whose selection waited (delayed mode).
struct CompletedKeyframe
{
    std::uint64_t           id = 0;
    std::vector<SourceView> sources;
};

struct Decision
{
    bool        footprint    = false;  ///< a footprint was computed
    bool        keyframe     = false;
    double      tilt_deg     = 0.0;
    double      overlap_last = -1.0;   ///< with the last keyframe; -1 if none or no footprint
    int         flight_line  = 0;
    double      plane_height = 0.0;    ///< ellipsoidal height of the footprint plane at its centre
    double      height_spread = 0.0;   ///< spread of the terrain heights in the footprint
    double      width_m      = 0.0;
    double      height_m     = 0.0;
    Eigen::Vector3d centre_E = Eigen::Vector3d::Zero();  ///< footprint centre, ECEF
    std::vector<SourceView> sources;  ///< keyframes only; empty for a deferred keyframe
    bool deferred = false;  ///< delayed mode: a keyframe whose sources come with the next keyframe or flush()
    /// Delayed mode: the waiting keyframe completed by this image (the image
    /// is the next keyframe).
    std::optional<CompletedKeyframe> completed;
};

struct SelectorStats
{
    std::size_t images        = 0;
    std::size_t footprints    = 0;
    std::size_t keyframes     = 0;
    std::size_t pool          = 0;
    std::size_t evicted       = 0;
    int         flight_lines  = 0;
};

class Selector
{
public:
    Selector(const geo::CameraModel &camera, const SelectorConfig &config, SurfaceFunction surface,
             GeodeticFunction geodetic);

    /// Decides an image with pose T_E_C (camera to ECEF). Ids must be unique.
    Decision add(std::uint64_t id, const Eigen::Isometry3d &T_E_C);

    /// Delayed mode: completes the waiting keyframe with the images in the
    /// pool now (end of a flight, or no next keyframe in time). Nothing if no
    /// keyframe is waiting.
    std::optional<CompletedKeyframe> flush();

    /// Delayed mode: whether a keyframe is waiting, and its id.
    std::optional<std::uint64_t> waitingKeyframe() const;

    /// Pool images whose footprint centre is farther than pool_distance_m
    /// from the latest camera position, and the oldest beyond max_pool.
    std::vector<std::uint64_t> evict();

    /// The oldest image of the pool, removed (for memory pressure).
    std::optional<std::uint64_t> evictOldest();

    /// Removes an image from the pool (for example when its data could not
    /// be stored). Returns false if it is not in the pool.
    bool remove(std::uint64_t id);

    /// New session: empties the pool, forgets the frame, the flight lines,
    /// the last and the waiting keyframe, and uses the given camera.
    void reset(const geo::CameraModel &camera);

    bool               hasFrame() const { return frame_ != nullptr; }
    const geo::LocalFrame &frame() const { return *frame_; }
    SelectorStats      stats() const;

private:
    std::optional<double> terrainHeight(double x, double y) const;
    CompletedKeyframe     complete(const View &keyframe) const;

    geo::CameraModel                 camera_;
    SelectorConfig                   config_;
    SurfaceFunction                  surface_;
    GeodeticFunction                 geodetic_;
    std::unique_ptr<geo::LocalFrame> frame_;
    KeyframeSelector                 keyframes_;
    std::optional<Footprint>         last_keyframe_;
    std::deque<View>                 pool_;  ///< oldest first
    std::optional<View>              waiting_;  ///< delayed mode: keyframe without sources yet
    std::optional<Eigen::Vector3d>   last_position_;     ///< local frame
    std::optional<Eigen::Vector2d>   line_direction_;
    int                              line_            = 0;
    double                           terrain_guess_z_ = 0.0;  ///< height for terrain queries (local z)
    SelectorStats                    stats_;
};

}  // namespace orthostream::selection

#endif  // ORTHOSTREAM_SELECTION_SELECTOR_HPP
