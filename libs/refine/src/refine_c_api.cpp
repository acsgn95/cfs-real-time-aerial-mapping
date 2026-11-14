// C interface of the sliding-window refinement (refine_c_api.h).

#include "refine_c_api.h"

#include <cmath>
#include <cstring>
#include <deque>
#include <exception>
#include <memory>
#include <new>
#include <optional>
#include <string>

#include "orthostream/geo/frames.hpp"
#include "orthostream/refine/sliding.hpp"

namespace rf = orthostream::refine;
namespace geo = orthostream::geo;

struct OST_Refine
{
    std::unique_ptr<rf::SlidingRefiner> refiner;
    rf::SlidingConfig                   config;
    std::optional<geo::LocalFrame>      frame;
    double                              origin_height = 0.0;  ///< ellipsoidal height of the frame origin
    std::deque<OST_Refine_Result_t>     queue;
    std::string                         error;
};

namespace
{

constexpr double kA  = 6378137.0;
constexpr double kF  = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);

/// Geodetic latitude, longitude and height of an ECEF point (iterative; the
/// local frame needs no PROJ).
void geodetic(const Eigen::Vector3d &p, double *lat, double *lon, double *h)
{
    const double r = std::hypot(p.x(), p.y());
    *lon           = std::atan2(p.y(), p.x());
    double phi     = std::atan2(p.z(), r * (1.0 - kE2));
    double n       = kA;
    for (int i = 0; i < 6; ++i)
    {
        n   = kA / std::sqrt(1.0 - kE2 * std::sin(phi) * std::sin(phi));
        *h  = r / std::cos(phi) - n;
        phi = std::atan2(p.z(), r * (1.0 - kE2 * n / (n + *h)));
    }
    *lat = phi;
}

Eigen::Isometry3d toIsometry(const OST_Refine_Pose_t &p)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::Quaterniond(p.q_ecef_from_camera[0], p.q_ecef_from_camera[1], p.q_ecef_from_camera[2],
                                    p.q_ecef_from_camera[3])
                     .normalized()
                     .toRotationMatrix();
    T.translation() = Eigen::Vector3d(p.position_ecef[0], p.position_ecef[1], p.position_ecef[2]);
    return T;
}

OST_Refine_Pose_t toPose(const Eigen::Isometry3d &T)
{
    OST_Refine_Pose_t  p;
    Eigen::Quaterniond q(T.linear());
    if (q.w() < 0.0)
    {
        q.coeffs() *= -1.0;
    }
    for (int k = 0; k < 3; ++k)
    {
        p.position_ecef[k] = T.translation()[k];
    }
    p.q_ecef_from_camera[0] = q.w();
    p.q_ecef_from_camera[1] = q.x();
    p.q_ecef_from_camera[2] = q.y();
    p.q_ecef_from_camera[3] = q.z();
    return p;
}

OST_Status_t fail(OST_Refine_t *h, OST_Status_t status, const std::string &text)
{
    if (h != nullptr)
    {
        h->error = text;
    }
    return status;
}

template <typename F>
OST_Status_t guarded(OST_Refine_t *h, F &&f)
{
    try
    {
        if (h != nullptr)
        {
            h->error.clear();
        }
        return f();
    }
    catch (const rf::RefineError &e)
    {
        return fail(h, OST_ERR_INVALID_ARGUMENT, e.what());
    }
    catch (const std::bad_alloc &)
    {
        return fail(h, OST_ERR_NO_MEMORY, "out of memory");
    }
    catch (const std::exception &e)
    {
        return fail(h, OST_ERR_INTERNAL, e.what());
    }
    catch (...)
    {
        return fail(h, OST_ERR_INTERNAL, "unknown exception");
    }
}

}  // namespace

extern "C" {

void OST_Refine_DefaultConfig(OST_Refine_Config_t *c)
{
    if (c == nullptr)
    {
        return;
    }
    const rf::SlidingConfig s;
    std::memset(c, 0, sizeof(*c));
    c->precise_position           = s.refine.precise_position ? 1 : 0;
    c->estimate_boresight         = s.refine.estimate_boresight ? 1 : 0;
    c->position_sigma_m           = s.refine.position_sigma;
    c->precise_position_sigma_m   = s.refine.precise_position_sigma;
    c->attitude_sigma_deg         = s.refine.attitude_sigma_deg;
    c->boresight_sigma_deg        = s.refine.boresight_sigma_deg;
    c->boresight_settle_batches   = static_cast<uint32_t>(s.boresight_settle_batches);
    c->held_boresight_sigma_deg   = s.held_boresight_sigma_deg;
    c->refined_attitude_sigma_deg = s.refined_attitude_sigma_deg;
    c->max_features               = s.refine.max_features;
    c->max_window_images          = static_cast<uint32_t>(s.max_window_images);
    c->max_points                 = static_cast<uint32_t>(s.max_points);
    c->max_iterations             = s.max_iterations;
    c->max_wait_s                 = s.max_wait_s;
    c->window_s                   = s.window_s;
    c->min_image_observations     = static_cast<uint32_t>(s.refine.min_image_observations);
    c->max_reprojection_rms_px    = s.refine.max_reprojection_rms_px;
    c->max_correction_sigmas      = s.refine.max_correction_sigmas;
}

OST_Status_t OST_Refine_Create(const OST_Refine_Config_t *c, const OST_Refine_Camera_t *cam, OST_Refine_t **handle)
{
    if (c == nullptr || cam == nullptr || handle == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *handle = nullptr;
    if (cam->width <= 0 || cam->height <= 0 || !(cam->fx > 0.0) || !(cam->fy > 0.0) || c->max_window_images < 2 ||
        c->max_points == 0 || c->max_iterations <= 0 || c->max_features <= 0 || !(c->max_wait_s >= 0.0) ||
        !(c->window_s > c->max_wait_s))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    try
    {
        auto               h = std::make_unique<OST_Refine>();
        rf::SlidingConfig &s = h->config;
        s.refine.precise_position        = c->precise_position != 0;
        s.refine.estimate_boresight      = c->estimate_boresight != 0;
        s.refine.position_sigma          = c->position_sigma_m;
        s.refine.precise_position_sigma  = c->precise_position_sigma_m;
        s.refine.attitude_sigma_deg      = c->attitude_sigma_deg;
        s.refine.boresight_sigma_deg     = c->boresight_sigma_deg;
        s.refine.max_features            = c->max_features;
        s.refine.min_image_observations  = c->min_image_observations;
        s.refine.max_reprojection_rms_px = c->max_reprojection_rms_px;
        s.refine.max_correction_sigmas   = c->max_correction_sigmas;
        s.boresight_settle_batches       = static_cast<int>(c->boresight_settle_batches);
        s.held_boresight_sigma_deg       = c->held_boresight_sigma_deg;
        s.refined_attitude_sigma_deg     = c->refined_attitude_sigma_deg;
        s.max_window_images              = c->max_window_images;
        s.max_points                     = c->max_points;
        s.max_iterations                 = c->max_iterations;
        s.max_wait_s                     = c->max_wait_s;
        s.window_s                       = c->window_s;

        geo::CameraModel m;
        m.width  = cam->width;
        m.height = cam->height;
        m.fx     = cam->fx;
        m.fy     = cam->fy;
        m.cx     = cam->cx;
        m.cy     = cam->cy;
        h->refiner = std::make_unique<rf::SlidingRefiner>(m, s);
        *handle    = h.release();
        return OST_SUCCESS;
    }
    catch (const std::bad_alloc &)
    {
        return OST_ERR_NO_MEMORY;
    }
    catch (...)
    {
        return OST_ERR_INTERNAL;
    }
}

void OST_Refine_Destroy(OST_Refine_t *handle)
{
    delete handle;
}

const char *OST_Refine_LastError(const OST_Refine_t *handle)
{
    return handle != nullptr ? handle->error.c_str() : "no handle";
}

OST_Status_t OST_Refine_HasImage(const OST_Refine_t *h, uint64_t id, int *has)
{
    if (h == nullptr || has == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *has = h->refiner->hasImage(id) ? 1 : 0;
    return OST_SUCCESS;
}

OST_Status_t OST_Refine_AddImage(OST_Refine_t *h, const OST_Refine_Image_t *image)
{
    if (h == nullptr || image == nullptr || image->grey == nullptr)
    {
        return fail(h, OST_ERR_INVALID_ARGUMENT, "null argument");
    }
    return guarded(h, [&] {
        const Eigen::Isometry3d T_E_C = toIsometry(image->pose);
        if (!h->frame)
        {
            double lat = 0, lon = 0, height = 0;
            geodetic(T_E_C.translation(), &lat, &lon, &height);
            h->frame.emplace(T_E_C.translation(), lat, lon);
            h->origin_height = height;
        }
        rf::SlidingImage si;
        si.id                 = image->id;
        si.time_ns            = image->capture_utc_ns;
        si.image              = { image->grey, image->width, image->height, image->stride };
        si.T_W_C              = h->frame->poseInLocal(T_E_C);
        si.attitude_sigma_deg = image->attitude_sigma_deg;
        si.position_sigma_m   = image->position_sigma_m;
        h->refiner->addImage(si);
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Refine_AddBatch(OST_Refine_t *h, uint64_t batch_id, const uint64_t *ids, uint32_t count,
                                 double plane_height_m)
{
    if (h == nullptr || ids == nullptr || count == 0 || count > OST_REFINE_MAX_BATCH_IMAGES)
    {
        return fail(h, OST_ERR_INVALID_ARGUMENT, "invalid batch");
    }
    if (!h->frame)
    {
        return fail(h, OST_ERR_INVALID_ARGUMENT, "no image added");
    }
    return guarded(h, [&] {
        rf::SlidingBatch b;
        b.id       = batch_id;
        b.keyframe = ids[0];
        b.images.assign(ids, ids + count);
        // Heights above the origin; the curvature of the earth is neglected
        // (0.3 m at 2 km), the footprints only choose the window.
        b.ground_z = plane_height_m - h->origin_height;
        h->refiner->addBatch(b);
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Refine_Poll(OST_Refine_t *h, int64_t now_utc_ns, int flush, OST_Refine_Result_t *results,
                             uint32_t capacity, uint32_t *count)
{
    if (h == nullptr || count == nullptr || (capacity > 0 && results == nullptr))
    {
        return fail(h, OST_ERR_INVALID_ARGUMENT, "null argument");
    }
    *count = 0;
    return guarded(h, [&] {
        if (h->frame)
        {
            for (const rf::SlidingResult &r : h->refiner->poll(now_utc_ns, flush != 0))
            {
                OST_Refine_Result_t o;
                std::memset(&o, 0, sizeof(o));
                o.batch_id       = r.batch_id;
                o.refined        = r.source == rf::PoseSource::refined ? 1 : 0;
                o.covered        = r.covered ? 1 : 0;
                o.boresight_held = r.boresight_held ? 1 : 0;
                o.count          = static_cast<uint32_t>(r.T_W_C.size());
                for (std::size_t k = 0; k < r.T_W_C.size() && k < OST_REFINE_MAX_BATCH_IMAGES; ++k)
                {
                    o.poses[k] = toPose(h->frame->ecefFromLocal() * r.T_W_C[k]);
                }
                o.waited_s      = r.waited_s;
                o.rms_px        = r.rms_after_px;
                o.boresight_deg = r.boresight_deg;
                o.matching_s    = r.matching_s;
                o.adjustment_s  = r.adjustment_s;
                o.window_images = static_cast<uint32_t>(r.window_images);
                o.points        = static_cast<uint32_t>(r.points);
                o.iterations    = static_cast<uint32_t>(r.iterations);
                std::strncpy(o.reason, r.reason.c_str(), OST_REFINE_REASON_LEN - 1);
                h->queue.push_back(o);
            }
        }
        while (*count < capacity && !h->queue.empty())
        {
            results[(*count)++] = h->queue.front();
            h->queue.pop_front();
        }
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Refine_GetStats(const OST_Refine_t *h, OST_Refine_Stats_t *stats)
{
    if (h == nullptr || stats == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    const rf::SlidingStats s = h->refiner->stats();
    stats->images            = static_cast<uint32_t>(s.images);
    stats->pending_batches   = static_cast<uint32_t>(s.pending_batches);
    stats->cached_pairs      = static_cast<uint32_t>(s.cached_pairs);
    stats->refined_batches   = static_cast<uint32_t>(s.refined_batches);
    stats->failed_batches    = static_cast<uint32_t>(s.failed_batches);
    stats->queued_results    = static_cast<uint32_t>(h->queue.size());
    stats->features_s        = s.features_s;
    return OST_SUCCESS;
}

}  // extern "C"
