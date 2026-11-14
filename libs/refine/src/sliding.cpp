// Sliding-window refinement (ADR-019 section 3); see sliding.hpp.

#include "orthostream/refine/sliding.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <sstream>

#include <opencv2/core.hpp>

namespace orthostream::refine
{
namespace
{

using Clock = std::chrono::steady_clock;
constexpr double kDeg = M_PI / 180.0;

double since(Clock::time_point t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

struct Image
{
    std::uint64_t     id      = 0;
    std::int64_t      time_ns = 0;
    Eigen::Isometry3d telemetry;
    double            attitude_sigma_deg = 0.0;
    double            position_sigma_m   = 0.0;
    Features          features;
    int               line = 0;
    Eigen::Vector2d   direction = Eigen::Vector2d::Zero();  ///< horizontal direction of travel
    bool              refined   = false;
    Eigen::Isometry3d X_refined;  ///< pose variable of the last solution
};

struct Footprint
{
    Eigen::Vector2d centre;
    double          width = 0.0;
};

}  // namespace

struct SlidingRefiner::Impl
{
    geo::CameraModel                  camera;
    SlidingConfig                     config;
    std::unique_ptr<FeatureExtractor> extractor;

    std::map<std::uint64_t, Image>                                  images;
    std::vector<std::uint64_t>                                      order;  ///< ids by capture time
    std::map<std::pair<std::uint64_t, std::uint64_t>, PairMatches>  pairs;  ///< inliers as keypoint indices
    std::deque<SlidingBatch>                                        pending;

    double          ground_z      = 0.0;
    Eigen::Matrix3d boresight     = Eigen::Matrix3d::Identity();
    int             settled       = 0;  ///< refined batches that contributed to the boresight
    SlidingStats    stats;

    std::optional<Footprint> footprint(const Image &im) const
    {
        const Eigen::Isometry3d &T = im.refined ? im.X_refined : im.telemetry;
        const Eigen::Vector3d    c = T.translation();
        const Eigen::Vector3d    r = T.linear() * Eigen::Vector3d::UnitZ();
        if (r.z() > -0.1 || c.z() <= ground_z)
        {
            return std::nullopt;  // not looking down, or below the ground
        }
        const double t = (ground_z - c.z()) / r.z();
        return Footprint { (c + t * r).head<2>(), t * camera.width / camera.fx };
    }

    PosePrior prior(const Image &im) const
    {
        const RefineConfig &rc = config.refine;
        if (im.refined)
        {
            return { im.X_refined, config.refined_attitude_sigma_deg,
                     rc.precise_position ? rc.precise_position_sigma : config.refined_position_sigma_m };
        }
        const double att = im.attitude_sigma_deg > 0.0 ? im.attitude_sigma_deg : rc.attitude_sigma_deg;
        double       pos = rc.precise_position ? rc.precise_position_sigma : rc.position_sigma;
        if (!rc.precise_position && im.position_sigma_m > 0.0)
        {
            pos = im.position_sigma_m;
        }
        return { im.telemetry, att, pos };
    }

    /// The keyframe's footprint is seen by an image of a later flight line
    /// that flies the other way.
    bool covered(const Image &kf, const Footprint &f) const
    {
        for (auto it = order.rbegin(); it != order.rend(); ++it)
        {
            const Image &im = images.at(*it);
            if (im.time_ns <= kf.time_ns)
            {
                break;
            }
            if (im.line == kf.line || im.direction.dot(kf.direction) > -0.5)
            {
                continue;
            }
            const auto g = footprint(im);
            if (g && (g->centre - f.centre).norm() < config.coverage_distance_factor * f.width)
            {
                return true;
            }
        }
        return false;
    }

    /// Directions of travel and flight lines of the images in time order: a
    /// new line starts when the direction turns away from the direction at
    /// the start of the current line. Images in the window are not
    /// consecutive captures (only the images of batches arrive), so the
    /// labels are recomputed whenever an image is added.
    void relabel()
    {
        int             l = 0;
        Eigen::Vector2d start = Eigen::Vector2d::Zero();
        Image          *prev  = nullptr;
        for (std::uint64_t id : order)
        {
            Image &im = images.at(id);
            if (prev != nullptr)
            {
                const Eigen::Vector2d d = (im.telemetry.translation() - prev->telemetry.translation()).head<2>();
                im.direction            = d.norm() > 1.0 ? Eigen::Vector2d(d.normalized()) : prev->direction;
                if (prev->direction.isZero())
                {
                    prev->direction = im.direction;
                }
                if (start.isZero())
                {
                    start = im.direction;
                }
                else if (!im.direction.isZero() &&
                         std::acos(std::clamp(im.direction.dot(start), -1.0, 1.0)) > config.line_turn_deg * kDeg)
                {
                    ++l;
                    start = im.direction;
                }
            }
            im.line = l;
            prev    = &im;
        }
    }

    SlidingResult solve(const SlidingBatch &batch, std::int64_t now_ns, bool is_covered);
    void          prune(std::int64_t now_ns);
};

SlidingRefiner::SlidingRefiner(const geo::CameraModel &camera, const SlidingConfig &config,
                               std::unique_ptr<FeatureExtractor> extractor)
    : impl_(std::make_unique<Impl>())
{
    impl_->camera    = camera;
    impl_->config    = config;
    impl_->extractor = extractor ? std::move(extractor) : makeSiftExtractor(config.refine.max_features);
}

SlidingRefiner::~SlidingRefiner() = default;

void SlidingRefiner::addImage(const SlidingImage &in)
{
    Impl &m = *impl_;
    if (in.image.width != m.camera.width || in.image.height != m.camera.height)
    {
        throw RefineError("image size differs from the camera");
    }
    if (m.images.count(in.id) != 0)
    {
        throw RefineError("image added twice");
    }
    Image im;
    im.id                 = in.id;
    im.time_ns            = in.time_ns;
    im.telemetry          = in.T_W_C;
    im.attitude_sigma_deg = in.attitude_sigma_deg;
    im.position_sigma_m   = in.position_sigma_m;
    const auto t0         = Clock::now();
    im.features           = m.extractor->extract(in.image);
    m.stats.features_s += since(t0);

    const auto pos = std::upper_bound(m.order.begin(), m.order.end(), in.time_ns, [&](std::int64_t t, std::uint64_t id) {
        return t < m.images.at(id).time_ns;
    });
    m.order.insert(pos, im.id);
    m.images.emplace(im.id, std::move(im));
    m.relabel();
}

void SlidingRefiner::addBatch(const SlidingBatch &batch)
{
    for (std::uint64_t id : batch.images)
    {
        if (impl_->images.count(id) == 0)
        {
            throw RefineError("batch refers to an image that was not added");
        }
    }
    if (batch.images.empty() || batch.images.front() != batch.keyframe)
    {
        throw RefineError("the first image of a batch must be its keyframe");
    }
    impl_->ground_z = batch.ground_z;
    impl_->pending.push_back(batch);
}

std::vector<SlidingResult> SlidingRefiner::poll(std::int64_t now_ns, bool flush)
{
    Impl                      &m = *impl_;
    std::vector<SlidingResult> out;
    while (!m.pending.empty())
    {
        const SlidingBatch &b  = m.pending.front();
        const Image        &kf = m.images.at(b.keyframe);
        const double        waited = static_cast<double>(now_ns - kf.time_ns) * 1e-9;
        const auto          f      = m.footprint(kf);
        const bool          cov    = f && m.covered(kf, *f);
        if (!(flush || cov || waited >= m.config.max_wait_s))
        {
            break;  // batches are solved in order
        }
        out.push_back(m.solve(b, now_ns, cov));
        m.pending.pop_front();
    }
    m.prune(now_ns);
    return out;
}

SlidingResult SlidingRefiner::Impl::solve(const SlidingBatch &batch, std::int64_t now_ns, bool is_covered)
{
    SlidingResult r;
    r.batch_id = batch.id;
    r.covered  = is_covered;
    const Image &kf = images.at(batch.keyframe);
    r.waited_s      = static_cast<double>(now_ns - kf.time_ns) * 1e-9;
    for (std::uint64_t id : batch.images)
    {
        r.T_W_C.push_back(images.at(id).telemetry);
    }
    auto fail = [&](const std::string &why) {
        r.reason = why;
        ++stats.failed_batches;
        return r;
    };
    const auto f = footprint(kf);
    if (!f)
    {
        return fail("keyframe does not look at the ground plane");
    }

    // Local window: the batch, then the images nearest to the keyframe's
    // footprint centre.
    std::vector<std::uint64_t>             sel(batch.images.begin(), batch.images.end());
    std::set<std::uint64_t>                in_sel(sel.begin(), sel.end());
    std::vector<std::pair<double, std::uint64_t>> near;
    std::map<std::uint64_t, Footprint>     fp;
    for (std::uint64_t id : order)
    {
        const auto g = footprint(images.at(id));
        if (!g)
        {
            continue;
        }
        fp[id]          = *g;
        const double dd = (g->centre - f->centre).norm();
        if (in_sel.count(id) == 0 && dd < config.window_radius_factor * f->width)
        {
            near.emplace_back(dd, id);
        }
    }
    std::sort(near.begin(), near.end());
    for (const auto &[dd, id] : near)
    {
        if (sel.size() >= config.max_window_images)
        {
            break;
        }
        sel.push_back(id);
    }
    r.window_images = sel.size();
    std::map<std::uint64_t, std::size_t> local;
    for (std::size_t i = 0; i < sel.size(); ++i)
    {
        local[sel[i]] = i;
    }

    // Pairs: overlapping footprints, and all pairs within the batch.
    std::vector<std::pair<std::uint64_t, std::uint64_t>> want;
    for (std::size_t a = 0; a < sel.size(); ++a)
    {
        for (std::size_t b = a + 1; b < sel.size(); ++b)
        {
            const bool in_batch = a < batch.images.size() && b < batch.images.size();
            const auto fa = fp.find(sel[a]), fb = fp.find(sel[b]);
            const bool close = fa != fp.end() && fb != fp.end() &&
                               (fa->second.centre - fb->second.centre).norm() < config.pair_distance_factor * f->width;
            if (in_batch || close)
            {
                want.emplace_back(std::min(sel[a], sel[b]), std::max(sel[a], sel[b]));
            }
        }
    }
    std::vector<std::pair<std::uint64_t, std::uint64_t>> todo;
    for (const auto &p : want)
    {
        if (pairs.count(p) == 0)
        {
            todo.push_back(p);
        }
    }
    const auto t0 = Clock::now();
    std::vector<PairMatches> found(todo.size());
    cv::parallel_for_(cv::Range(0, static_cast<int>(todo.size())), [&](const cv::Range &range) {
        for (int k = range.start; k < range.end; ++k)
        {
            const auto &[a, b] = todo[static_cast<std::size_t>(k)];
            found[static_cast<std::size_t>(k)] =
                matchPair(images.at(a).features, images.at(b).features, camera, config.refine);
        }
    });
    for (std::size_t k = 0; k < todo.size(); ++k)
    {
        pairs[todo[k]] = std::move(found[k]);
    }
    r.matching_s = since(t0);
    r.new_pairs  = todo.size();
    r.pairs      = want.size();

    std::vector<PairMatches> used;
    for (const auto &p : want)
    {
        const PairMatches &pm = pairs.at(p);
        if (pm.inliers.size() >= static_cast<std::size_t>(config.refine.min_pair_inliers))
        {
            PairMatches l = pm;
            l.image_a     = local.at(p.first);
            l.image_b     = local.at(p.second);
            used.push_back(std::move(l));
        }
    }
    std::vector<std::vector<Eigen::Vector2d>> keypoints;
    std::vector<PosePrior>                    priors;
    for (std::uint64_t id : sel)
    {
        keypoints.push_back(images.at(id).features.keypoints);
        priors.push_back(prior(images.at(id)));
    }
    const std::vector<Track> tracks = buildTracks(keypoints, used);

    RefineConfig rc       = config.refine;
    rc.max_points         = config.max_points;
    rc.max_iterations     = config.max_iterations;
    rc.relative_error_tol = config.relative_error_tol;
    const bool     held   = settled >= config.boresight_settle_batches;
    BoresightPrior bp;
    bp.R           = boresight;
    bp.sigma_deg   = held ? config.held_boresight_sigma_deg : rc.boresight_sigma_deg;
    r.boresight_held = held;

    const auto       t1 = Clock::now();
    AdjustmentResult a;
    try
    {
        a = adjust(camera, priors, tracks, rc, bp);
    }
    catch (const RefineError &e)
    {
        r.adjustment_s = since(t1);
        return fail(e.what());
    }
    r.adjustment_s  = since(t1);
    r.points        = a.points;
    r.observations  = a.observations;
    r.iterations    = a.iterations;
    r.rms_before_px = a.rms_before_px;
    r.rms_after_px  = a.rms_after_px;
    r.boresight_deg = Eigen::AngleAxisd(a.boresight).angle() / kDeg;

    // Checks of section 5, for the images of the batch.
    std::ostringstream why;
    for (std::size_t i = 0; i < batch.images.size() && why.str().empty(); ++i)
    {
        const PosePrior &p = priors[i];
        if (a.observations_per_image[i] < rc.min_image_observations)
        {
            why << "image " << batch.images[i] << " has " << a.observations_per_image[i] << " observations";
        }
        else if (a.rotation_correction_deg[i] > rc.max_correction_sigmas * p.attitude_sigma_deg &&
                 a.rotation_correction_deg[i] > 0.01)
        {
            why << "image " << batch.images[i] << ": attitude correction " << a.rotation_correction_deg[i] << " deg";
        }
        else if (a.position_correction_m[i] > rc.max_correction_sigmas * p.position_sigma_m &&
                 a.position_correction_m[i] > 0.01)
        {
            why << "image " << batch.images[i] << ": position correction " << a.position_correction_m[i] << " m";
        }
    }
    if (why.str().empty() && a.rms_after_px > rc.max_reprojection_rms_px)
    {
        why << "reprojection RMS " << a.rms_after_px << " px";
    }
    if (!why.str().empty())
    {
        return fail(why.str());
    }

    for (std::size_t i = 0; i < sel.size(); ++i)
    {
        Image &im    = images.at(sel[i]);
        im.refined   = true;
        im.X_refined = a.X_W_C[i];
    }
    for (std::size_t i = 0; i < batch.images.size(); ++i)
    {
        r.T_W_C[i] = a.T_W_C[i];
    }
    if (!held && rc.estimate_boresight)
    {
        boresight = a.boresight;
        ++settled;
    }
    r.source = PoseSource::refined;
    ++stats.refined_batches;
    return r;
}

void SlidingRefiner::Impl::prune(std::int64_t now_ns)
{
    std::set<std::uint64_t> needed;
    for (const SlidingBatch &b : pending)
    {
        needed.insert(b.images.begin(), b.images.end());
    }
    const auto limit = now_ns - static_cast<std::int64_t>(config.window_s * 1e9);
    std::vector<std::uint64_t> keep;
    for (std::uint64_t id : order)
    {
        if (images.at(id).time_ns < limit && needed.count(id) == 0)
        {
            images.erase(id);
        }
        else
        {
            keep.push_back(id);
        }
    }
    order.swap(keep);
    for (auto it = pairs.begin(); it != pairs.end();)
    {
        if (images.count(it->first.first) == 0 || images.count(it->first.second) == 0)
        {
            it = pairs.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

bool SlidingRefiner::hasImage(std::uint64_t id) const
{
    return impl_->images.count(id) != 0;
}

bool SlidingRefiner::refinedPose(std::uint64_t id, Eigen::Isometry3d *T_W_C) const
{
    const auto it = impl_->images.find(id);
    if (it == impl_->images.end() || !it->second.refined)
    {
        return false;
    }
    if (T_W_C != nullptr)
    {
        Eigen::Isometry3d D = Eigen::Isometry3d::Identity();
        if (impl_->config.refine.estimate_boresight)
        {
            D.linear() = impl_->boresight;
        }
        *T_W_C = it->second.X_refined * D;
    }
    return true;
}

SlidingStats SlidingRefiner::stats() const
{
    SlidingStats s     = impl_->stats;
    s.images           = impl_->images.size();
    s.pending_batches  = impl_->pending.size();
    s.cached_pairs     = impl_->pairs.size();
    return s;
}

}  // namespace orthostream::refine
