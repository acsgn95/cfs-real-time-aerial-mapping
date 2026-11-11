// Keyframe and source view selection (ADR-006 sections 4 and 5).

#ifndef ORTHOSTREAM_SELECTION_VIEWS_HPP
#define ORTHOSTREAM_SELECTION_VIEWS_HPP

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

#include "orthostream/selection/footprint.hpp"

namespace orthostream::selection
{

struct KeyframeConfig
{
    double max_tilt_deg   = 25.0;  ///< 20° envelope of ADR-001 plus 5° margin
    double target_overlap = 0.7;   ///< forward overlap between keyframes, 0.6–0.8
};

/// Decides for each image, in order of arrival, whether it becomes a
/// keyframe: tilt within the limit, footprint available, and overlap with
/// the last keyframe below the target.
class KeyframeSelector
{
public:
    explicit KeyframeSelector(const KeyframeConfig &config);

    /// The pose must be valid (time alignment succeeded). Returns true if
    /// the image becomes a keyframe; it is then the new last keyframe.
    bool consider(const Eigen::Isometry3d &T_L_C, const std::optional<Footprint> &footprint);

    void reset();

private:
    KeyframeConfig           config_;
    std::optional<Footprint> last_;
};

struct SourceViewConfig
{
    int    max_sources                       = 5;     ///< K
    double min_overlap                       = 0.4;
    double min_baseline_ratio                = 0.05;  ///< baseline / height above the footprint plane
    double max_baseline_ratio                = 0.6;
    double max_view_angle_deg                = 30.0;  ///< between the optical axes
    double preferred_triangulation_angle_deg = 10.0;  ///< angles above it score fully
};

/// An image that can serve as a keyframe or a source view.
struct View
{
    std::uint64_t     id = 0;
    Eigen::Isometry3d T_L_C;
    Footprint         footprint;
    int               flight_line = 0;
};

struct SourceView
{
    std::uint64_t id                      = 0;
    double        overlap                 = 0.0;
    double        baseline_ratio          = 0.0;
    double        triangulation_angle_deg = 0.0;  ///< at the keyframe's footprint centre
    double        score                   = 0.0;
    bool          other_line              = false;
};

/// Chooses up to max_sources source views for a keyframe among candidates.
/// The caller passes only images that may be used (for example images
/// received before the keyframe in causal mode). Candidates are ranked by
/// overlap times min(1, triangulation angle / preferred angle). If no chosen
/// view belongs to another flight line but a qualified one does, the best of
/// them replaces the last chosen view. Sorted by descending score, except
/// for that replacement, which comes last.
std::vector<SourceView> selectSourceViews(const View                &keyframe,
                                          const std::vector<View>   &candidates,
                                          const SourceViewConfig    &config);

/// Splits camera positions in order of capture into flight lines: a new line
/// starts when the horizontal direction of travel deviates from the
/// direction at the start of the current line by more than max_turn_deg.
/// Images taken in a turn form short lines of their own.
std::vector<int> assignFlightLines(const std::vector<Eigen::Vector3d> &positions, double max_turn_deg = 45.0);

}  // namespace orthostream::selection

#endif  // ORTHOSTREAM_SELECTION_VIEWS_HPP
