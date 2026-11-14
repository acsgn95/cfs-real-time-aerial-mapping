// Incremental selection with an image pool (selector.hpp).

#include "orthostream/selection/selector.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace orthostream::selection
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double deg(double rad)
{
    return rad * 180.0 / kPi;
}

}  // namespace

Selector::Selector(const geo::CameraModel &camera, const SelectorConfig &config, SurfaceFunction surface,
                   GeodeticFunction geodetic)
    : camera_(camera), config_(config), surface_(std::move(surface)), geodetic_(std::move(geodetic)),
      keyframes_(config.keyframe)
{
    if (!surface_ || !geodetic_)
    {
        throw SelectionError("selector: surface and geodetic functions are required");
    }
    if (!(config.pool_distance_m > 0.0) || config.max_pool == 0 || !(config.line_turn_deg > 0.0) ||
        config.sources.max_sources <= 0 || camera.width <= 0 || camera.height <= 0)
    {
        throw SelectionError("selector: invalid configuration or camera");
    }
}

std::optional<double> Selector::terrainHeight(double x, double y) const
{
    const std::optional<Eigen::Vector3d> s = surface_(frame_->toEcef({x, y, terrain_guess_z_}));
    if (!s)
    {
        return std::nullopt;
    }
    return frame_->toLocal(*s).z();
}

Decision Selector::add(std::uint64_t id, const Eigen::Isometry3d &T_E_C)
{
    if (!T_E_C.matrix().allFinite())
    {
        throw SelectionError("selector: pose not finite");
    }
    if (!frame_)
    {
        const geo::Geodetic g = geodetic_(T_E_C.translation());
        frame_ = std::make_unique<geo::LocalFrame>(T_E_C.translation(), g.latitude_rad, g.longitude_rad);
    }
    ++stats_.images;

    View view;
    view.id    = id;
    view.T_L_C = frame_->poseInLocal(T_E_C);
    const Eigen::Vector3d C = view.T_L_C.translation();

    // Flight line (as assignFlightLines, one image at a time).
    if (last_position_)
    {
        const Eigen::Vector2d step = (C - *last_position_).head<2>();
        if (step.norm() > 0.0)
        {
            const Eigen::Vector2d dir = step.normalized();
            if (!line_direction_)
            {
                line_direction_ = dir;
            }
            else if (deg(std::acos(std::clamp(dir.dot(*line_direction_), -1.0, 1.0))) > config_.line_turn_deg)
            {
                ++line_;
                line_direction_ = dir;
            }
        }
    }
    last_position_   = C;
    view.flight_line = line_;

    Decision d;
    d.flight_line = line_;
    d.tilt_deg    = deg(tiltFromNadir(view.T_L_C));

    terrain_guess_z_ = C.z() - config_.footprint.nominal_height_agl;
    const TerrainHeight              terrain = [this](double x, double y) { return terrainHeight(x, y); };
    const std::optional<Footprint>   fp      = computeFootprint(camera_, view.T_L_C, terrain, config_.footprint);
    if (fp)
    {
        d.footprint     = true;
        d.height_spread = fp->height_spread;
        d.width_m       = fp->width_m;
        d.height_m      = fp->height_m;
        const Eigen::Vector3d centre_plane(fp->centre.x(), fp->centre.y(), fp->plane_height);
        d.centre_E      = frame_->toEcef(fp->centre);
        d.plane_height  = geodetic_(frame_->toEcef(centre_plane)).height_m;
        if (last_keyframe_)
        {
            d.overlap_last = overlapFraction(*last_keyframe_, *fp);
        }
        ++stats_.footprints;
    }

    d.keyframe = keyframes_.consider(view.T_L_C, fp);
    if (fp)
    {
        view.footprint = *fp;
    }
    if (d.keyframe)
    {
        last_keyframe_ = fp;
        ++stats_.keyframes;
        if (config_.delayed)
        {
            // The waiting keyframe gets the images before this one.
            if (waiting_)
            {
                d.completed = complete(*waiting_);
            }
            waiting_   = view;
            d.deferred = true;
        }
        else
        {
            d.sources = complete(view).sources;
        }
    }
    if (fp)
    {
        pool_.push_back(view);
    }
    stats_.flight_lines = line_ + 1;
    return d;
}

CompletedKeyframe Selector::complete(const View &keyframe) const
{
    const std::vector<View> candidates(pool_.begin(), pool_.end());
    CompletedKeyframe       c;
    c.id      = keyframe.id;
    c.sources = selectSourceViews(keyframe, candidates, config_.sources);
    return c;
}

std::optional<CompletedKeyframe> Selector::flush()
{
    if (!waiting_)
    {
        return std::nullopt;
    }
    const CompletedKeyframe c = complete(*waiting_);
    waiting_.reset();
    return c;
}

std::optional<std::uint64_t> Selector::waitingKeyframe() const
{
    if (!waiting_)
    {
        return std::nullopt;
    }
    return waiting_->id;
}

std::vector<std::uint64_t> Selector::evict()
{
    std::vector<std::uint64_t> out;
    if (!last_position_)
    {
        return out;
    }
    const Eigen::Vector2d here = last_position_->head<2>();
    for (auto it = pool_.begin(); it != pool_.end();)
    {
        if ((it->footprint.centre.head<2>() - here).norm() > config_.pool_distance_m)
        {
            out.push_back(it->id);
            it = pool_.erase(it);
        }
        else
        {
            ++it;
        }
    }
    while (pool_.size() > config_.max_pool)
    {
        out.push_back(pool_.front().id);
        pool_.pop_front();
    }
    stats_.evicted += out.size();
    return out;
}

std::optional<std::uint64_t> Selector::evictOldest()
{
    if (pool_.empty())
    {
        return std::nullopt;
    }
    const std::uint64_t id = pool_.front().id;
    pool_.pop_front();
    ++stats_.evicted;
    return id;
}

bool Selector::remove(std::uint64_t id)
{
    const auto it = std::find_if(pool_.begin(), pool_.end(), [id](const View &v) { return v.id == id; });
    if (it == pool_.end())
    {
        return false;
    }
    pool_.erase(it);
    return true;
}

void Selector::reset(const geo::CameraModel &camera)
{
    if (camera.width <= 0 || camera.height <= 0)
    {
        throw SelectionError("selector: invalid camera");
    }
    camera_ = camera;
    frame_.reset();
    keyframes_.reset();
    last_keyframe_.reset();
    waiting_.reset();
    pool_.clear();
    last_position_.reset();
    line_direction_.reset();
    line_  = 0;
    stats_ = SelectorStats{};
}

SelectorStats Selector::stats() const
{
    SelectorStats s = stats_;
    s.pool          = pool_.size();
    return s;
}

}  // namespace orthostream::selection
