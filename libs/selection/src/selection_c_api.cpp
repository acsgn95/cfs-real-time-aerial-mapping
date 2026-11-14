// C interface of the selection (selection_c_api.h).

#include "selection_c_api.h"

#include <cmath>
#include <cstring>
#include <deque>
#include <exception>
#include <memory>
#include <new>
#include <string>

#include "orthostream/selection/selector.hpp"
#include "orthostream/terrain/terrain.hpp"

namespace sel = orthostream::selection;
namespace tr  = orthostream::terrain;
namespace geo = orthostream::geo;

struct OST_Select
{
    std::unique_ptr<tr::TerrainPrior> terrain;
    std::unique_ptr<sel::Selector>    selector;
    double                            terrain_radius = 0.0;
    std::uint32_t                     terrain_loads  = 0;
    std::deque<std::uint64_t>         evicted;  ///< not yet returned
    std::string                       error;
};

namespace
{

geo::CameraModel toModel(const OST_Geo_Camera_t &c)
{
    geo::CameraModel m;
    m.width  = c.width;
    m.height = c.height;
    m.fx     = c.fx;
    m.fy     = c.fy;
    m.cx     = c.cx;
    m.cy     = c.cy;
    m.k1     = c.k1;
    m.k2     = c.k2;
    m.k3     = c.k3;
    m.p1     = c.p1;
    m.p2     = c.p2;
    return m;
}

sel::SelectorConfig toConfig(const OST_Select_Config_t &c)
{
    sel::SelectorConfig s;
    s.footprint.nominal_height_agl              = c.nominal_height_agl_m;
    s.footprint.iterations                      = c.footprint_iterations;
    s.keyframe.max_tilt_deg                     = c.max_tilt_deg;
    s.keyframe.target_overlap                   = c.target_overlap;
    s.sources.max_sources                       = c.max_sources;
    s.sources.min_overlap                       = c.min_overlap;
    s.sources.min_baseline_ratio                = c.min_baseline_ratio;
    s.sources.max_baseline_ratio                = c.max_baseline_ratio;
    s.sources.max_view_angle_deg                = c.max_view_angle_deg;
    s.sources.preferred_triangulation_angle_deg = c.preferred_triangulation_angle_deg;
    s.line_turn_deg                             = c.line_turn_deg;
    s.pool_distance_m                           = c.pool_distance_m;
    s.max_pool                                  = c.max_pool_images;
    return s;
}

template <typename F>
OST_Status_t guarded(OST_Select_t *handle, F &&f)
{
    try
    {
        handle->error.clear();
        return f();
    }
    catch (const sel::SelectionError &e)
    {
        handle->error = e.what();
        return OST_ERR_INVALID_ARGUMENT;
    }
    catch (const std::bad_alloc &)
    {
        handle->error = "out of memory";
        return OST_ERR_NO_MEMORY;
    }
    catch (const std::exception &e)
    {
        handle->error = e.what();
        return OST_ERR_INTERNAL;
    }
    catch (...)
    {
        handle->error = "unknown exception";
        return OST_ERR_INTERNAL;
    }
}

}  // namespace

extern "C" {

void OST_Select_DefaultConfig(OST_Select_Config_t *config)
{
    if (config == nullptr)
    {
        return;
    }
    const sel::SelectorConfig d;
    std::memset(config, 0, sizeof(*config));
    config->nominal_height_agl_m              = d.footprint.nominal_height_agl;
    config->footprint_iterations              = d.footprint.iterations;
    config->max_sources                       = d.sources.max_sources;
    config->max_tilt_deg                      = d.keyframe.max_tilt_deg;
    config->target_overlap                    = d.keyframe.target_overlap;
    config->min_overlap                       = d.sources.min_overlap;
    config->min_baseline_ratio                = d.sources.min_baseline_ratio;
    config->max_baseline_ratio                = d.sources.max_baseline_ratio;
    config->max_view_angle_deg                = d.sources.max_view_angle_deg;
    config->preferred_triangulation_angle_deg = d.sources.preferred_triangulation_angle_deg;
    config->line_turn_deg                     = d.line_turn_deg;
    config->pool_distance_m                   = d.pool_distance_m;
    config->max_pool_images                   = static_cast<uint32_t>(d.max_pool);
    config->terrain_radius_m                  = 20000.0;
}

OST_Status_t OST_Select_Create(const OST_Select_Config_t *config, const OST_Select_Terrain_t *terrain,
                               const OST_Geo_Camera_t *camera, OST_Select_t **handle)
{
    if (config == nullptr || terrain == nullptr || camera == nullptr || handle == nullptr ||
        terrain->dem_path == nullptr || terrain->dem_crs == nullptr || terrain->proj_data_dir == nullptr ||
        config->max_sources > OST_SELECT_MAX_SOURCES || !(config->terrain_radius_m > 0.0))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *handle = nullptr;
    try
    {
        auto h            = std::make_unique<OST_Select>();
        h->terrain_radius = config->terrain_radius_m;
        tr::TerrainConfig tc;
        tc.dem_path      = terrain->dem_path;
        tc.dem_crs       = terrain->dem_crs;
        tc.proj_data_dir = terrain->proj_data_dir;
        if (terrain->grid_dir != nullptr && terrain->grid_dir[0] != '\0')
        {
            tc.grid_dirs.push_back(terrain->grid_dir);
        }
        try
        {
            h->terrain = std::make_unique<tr::TerrainPrior>(tc);
        }
        catch (const tr::TerrainError &)
        {
            return OST_ERR_IO;
        }
        tr::TerrainPrior *t = h->terrain.get();
        h->selector         = std::make_unique<sel::Selector>(
            toModel(*camera), toConfig(*config), [t](const Eigen::Vector3d &p) { return t->surface(p); },
            [t](const Eigen::Vector3d &p) { return t->geodetic(p); });
        *handle = h.release();
        return OST_SUCCESS;
    }
    catch (const sel::SelectionError &)
    {
        return OST_ERR_INVALID_ARGUMENT;
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

void OST_Select_Destroy(OST_Select_t *handle)
{
    delete handle;
}

const char *OST_Select_LastError(const OST_Select_t *handle)
{
    return handle == nullptr ? "no handle" : handle->error.c_str();
}

OST_Status_t OST_Select_AddImage(OST_Select_t *handle, uint64_t id, const double position_ecef[3],
                                 const double q_ecef_from_camera[4], OST_Select_Decision_t *decision)
{
    if (handle == nullptr || position_ecef == nullptr || q_ecef_from_camera == nullptr || decision == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(handle, [&] {
        const Eigen::Quaterniond q(q_ecef_from_camera[0], q_ecef_from_camera[1], q_ecef_from_camera[2],
                                   q_ecef_from_camera[3]);
        if (std::abs(q.norm() - 1.0) > 1e-6)
        {
            throw sel::SelectionError("selector: q_E_C is not a unit quaternion");
        }
        Eigen::Isometry3d T_E_C = Eigen::Isometry3d::Identity();
        T_E_C.linear()          = q.normalized().toRotationMatrix();
        T_E_C.translation()     = Eigen::Vector3d(position_ecef[0], position_ecef[1], position_ecef[2]);

        // Terrain window around the flight; read again near its edge.
        std::memset(decision, 0, sizeof(*decision));
        bool terrain_ok = handle->terrain->loaded() &&
                          handle->terrain->distanceFromWindowCentre(T_E_C.translation()) < 0.5 * handle->terrain_radius;
        if (!terrain_ok)
        {
            try
            {
                handle->terrain->load(T_E_C.translation(), handle->terrain_radius);
                ++handle->terrain_loads;
                terrain_ok = true;
            }
            catch (const tr::TerrainError &e)
            {
                handle->error = e.what();  // reported through terrain_available
            }
        }

        const sel::Decision d       = handle->selector->add(id, T_E_C);
        decision->footprint         = d.footprint ? 1 : 0;
        decision->keyframe          = d.keyframe ? 1 : 0;
        decision->terrain_available = terrain_ok ? 1 : 0;
        decision->flight_line       = d.flight_line;
        decision->tilt_deg          = d.tilt_deg;
        decision->overlap_last      = d.overlap_last;
        decision->plane_height_m    = d.plane_height;
        decision->height_spread_m   = d.height_spread;
        decision->width_m           = d.width_m;
        decision->height_m          = d.height_m;
        for (int i = 0; i < 3; ++i)
        {
            decision->centre_ecef[i] = d.centre_E[i];
        }
        decision->source_count = 0;
        for (const sel::SourceView &s : d.sources)
        {
            if (decision->source_count == OST_SELECT_MAX_SOURCES)
            {
                break;
            }
            OST_Select_Source_t &o    = decision->sources[decision->source_count++];
            o.id                      = s.id;
            o.overlap                 = s.overlap;
            o.baseline_ratio          = s.baseline_ratio;
            o.triangulation_angle_deg = s.triangulation_angle_deg;
            o.score                   = s.score;
            o.other_line              = s.other_line ? 1 : 0;
        }
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Select_Evict(OST_Select_t *handle, uint64_t *ids, uint32_t capacity, uint32_t *count)
{
    if (handle == nullptr || count == nullptr || (ids == nullptr && capacity > 0))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *count = 0;
    return guarded(handle, [&] {
        for (std::uint64_t id : handle->selector->evict())
        {
            handle->evicted.push_back(id);
        }
        while (*count < capacity && !handle->evicted.empty())
        {
            ids[(*count)++] = handle->evicted.front();
            handle->evicted.pop_front();
        }
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Select_EvictOldest(OST_Select_t *handle, uint64_t *id, int *found)
{
    if (handle == nullptr || id == nullptr || found == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(handle, [&] {
        const std::optional<std::uint64_t> e = handle->selector->evictOldest();
        *found                               = e ? 1 : 0;
        *id                                  = e ? *e : 0;
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Select_Remove(OST_Select_t *handle, uint64_t id, int *found)
{
    if (handle == nullptr || found == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(handle, [&] {
        *found = handle->selector->remove(id) ? 1 : 0;
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Select_Reset(OST_Select_t *handle, const OST_Geo_Camera_t *camera)
{
    if (handle == nullptr || camera == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(handle, [&] {
        handle->selector->reset(toModel(*camera));
        handle->evicted.clear();
        return OST_SUCCESS;
    });
}

OST_Status_t OST_Select_GetStats(const OST_Select_t *handle, OST_Select_Stats_t *stats)
{
    if (handle == nullptr || stats == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    const sel::SelectorStats s = handle->selector->stats();
    std::memset(stats, 0, sizeof(*stats));
    stats->images         = static_cast<uint32_t>(s.images);
    stats->footprints     = static_cast<uint32_t>(s.footprints);
    stats->keyframes      = static_cast<uint32_t>(s.keyframes);
    stats->pool           = static_cast<uint32_t>(s.pool);
    stats->evicted        = static_cast<uint32_t>(s.evicted);
    stats->flight_lines   = static_cast<uint32_t>(s.flight_lines);
    stats->terrain_loads  = handle->terrain_loads;
    stats->terrain_loaded = handle->terrain->loaded() ? 1 : 0;
    return OST_SUCCESS;
}

}  // extern "C"
