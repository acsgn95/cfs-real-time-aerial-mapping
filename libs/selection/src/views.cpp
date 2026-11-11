#include "orthostream/selection/views.hpp"

#include <algorithm>
#include <cmath>

namespace orthostream::selection
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double deg(double rad)
{
    return rad * 180.0 / kPi;
}

double angleBetween(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

}  // namespace

KeyframeSelector::KeyframeSelector(const KeyframeConfig &config) : config_(config)
{
    if (!(config.max_tilt_deg > 0.0) || !(config.target_overlap > 0.0) || !(config.target_overlap < 1.0))
    {
        throw SelectionError("invalid keyframe configuration");
    }
}

bool KeyframeSelector::consider(const Eigen::Isometry3d &T_L_C, const std::optional<Footprint> &footprint)
{
    if (!footprint || deg(tiltFromNadir(T_L_C)) > config_.max_tilt_deg)
    {
        return false;
    }
    if (last_ && overlapFraction(*last_, *footprint) >= config_.target_overlap)
    {
        return false;
    }
    last_ = footprint;
    return true;
}

void KeyframeSelector::reset()
{
    last_.reset();
}

std::vector<SourceView> selectSourceViews(const View &keyframe, const std::vector<View> &candidates,
                                          const SourceViewConfig &config)
{
    if (config.max_sources <= 0 || !(config.min_baseline_ratio >= 0.0)
        || !(config.max_baseline_ratio > config.min_baseline_ratio) || !(config.preferred_triangulation_angle_deg > 0.0))
    {
        throw SelectionError("invalid source view configuration");
    }
    const Eigen::Vector3d C_k    = keyframe.T_L_C.translation();
    const Eigen::Vector3d axis_k = keyframe.T_L_C.linear().col(2);
    const Eigen::Vector3d X      = keyframe.footprint.centre;
    const double          height = C_k.z() - keyframe.footprint.plane_height;
    if (!(height > 0.0))
    {
        throw SelectionError("keyframe is not above its footprint plane");
    }

    std::vector<SourceView> qualified;
    for (const View &v : candidates)
    {
        if (v.id == keyframe.id)
        {
            continue;
        }
        const Eigen::Vector3d C_j = v.T_L_C.translation();
        SourceView            s;
        s.id             = v.id;
        s.overlap        = overlapFraction(keyframe.footprint, v.footprint);
        s.baseline_ratio = (C_j - C_k).norm() / height;
        if (s.overlap < config.min_overlap || s.baseline_ratio < config.min_baseline_ratio
            || s.baseline_ratio > config.max_baseline_ratio
            || deg(angleBetween(axis_k, v.T_L_C.linear().col(2))) > config.max_view_angle_deg)
        {
            continue;
        }
        s.triangulation_angle_deg = deg(angleBetween(C_k - X, C_j - X));
        s.score      = s.overlap * std::min(1.0, s.triangulation_angle_deg / config.preferred_triangulation_angle_deg);
        s.other_line = v.flight_line != keyframe.flight_line;
        qualified.push_back(s);
    }
    std::stable_sort(qualified.begin(), qualified.end(),
                     [](const SourceView &a, const SourceView &b) { return a.score > b.score; });

    const std::size_t       k = static_cast<std::size_t>(config.max_sources);
    std::vector<SourceView> chosen(qualified.begin(), qualified.begin() + static_cast<std::ptrdiff_t>(std::min(k, qualified.size())));
    const bool has_other = std::any_of(chosen.begin(), chosen.end(), [](const SourceView &s) { return s.other_line; });
    if (!has_other)
    {
        const auto best_other = std::find_if(qualified.begin(), qualified.end(),
                                             [](const SourceView &s) { return s.other_line; });
        if (best_other != qualified.end())
        {
            if (chosen.size() == k)
            {
                chosen.back() = *best_other;
            }
            else
            {
                chosen.push_back(*best_other);
            }
        }
    }
    return chosen;
}

std::vector<int> assignFlightLines(const std::vector<Eigen::Vector3d> &positions, double max_turn_deg)
{
    std::vector<int> lines(positions.size(), 0);
    int              line = 0;
    std::optional<Eigen::Vector2d> line_dir;
    for (std::size_t i = 1; i < positions.size(); ++i)
    {
        const Eigen::Vector2d step = (positions[i] - positions[i - 1]).head<2>();
        if (step.norm() > 0.0)
        {
            const Eigen::Vector2d dir = step.normalized();
            if (!line_dir)
            {
                line_dir = dir;
            }
            else if (deg(std::acos(std::clamp(dir.dot(*line_dir), -1.0, 1.0))) > max_turn_deg)
            {
                ++line;
                line_dir = dir;
            }
        }
        lines[i] = line;
    }
    return lines;
}

}  // namespace orthostream::selection
