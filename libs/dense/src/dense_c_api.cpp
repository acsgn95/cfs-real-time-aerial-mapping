// C interface of the dense reconstruction (dense_c_api.h).

#include "dense_c_api.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "orthostream/dense/patch_match.hpp"
#include "orthostream/dense/sgbm.hpp"
#include "orthostream/geo/frames.hpp"
#include "orthostream/terrain/terrain.hpp"

namespace dn  = orthostream::dense;
namespace tr  = orthostream::terrain;
namespace geo = orthostream::geo;

struct OST_Dense
{
    OST_Dense_Config_t                  config {};
    std::unique_ptr<tr::TerrainPrior>   terrain;
    std::unique_ptr<dn::DepthEstimator> estimator;
    dn::SgbmEstimator                  *sgbm          = nullptr;  ///< estimator, when SGBM
    std::uint32_t                       terrain_loads = 0;
    std::vector<float>                  normals;                  ///< when the caller wants none
    std::string                         error;
};

namespace
{

using Clock = std::chrono::steady_clock;

double since(Clock::time_point t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

class BatchError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

dn::SgbmConfig sgbmConfig(const OST_Dense_Config_t &c)
{
    dn::SgbmConfig s;
    s.target_baseline_ratio = c.sgbm_target_baseline_ratio;
    s.block_size            = c.sgbm_block_size;
    s.p1_factor             = c.sgbm_p1_factor;
    s.p2_factor             = c.sgbm_p2_factor;
    s.uniqueness_ratio      = c.sgbm_uniqueness_ratio;
    s.disp12_max_diff       = c.sgbm_disp12_max_diff;
    s.speckle_window        = c.sgbm_speckle_window;
    s.speckle_range         = c.sgbm_speckle_range;
    s.max_disparities       = c.sgbm_max_disparities;
    return s;
}

dn::PatchMatchConfig patchMatchConfig(const OST_Dense_Config_t &c)
{
    dn::PatchMatchConfig p;
    p.gpu_index                      = c.pm_gpu_index;
    p.num_iterations                 = c.pm_iterations;
    p.window_radius                  = c.pm_window_radius;
    p.window_step                    = c.pm_window_step;
    p.num_samples                    = c.pm_samples;
    p.filter_min_num_consistent      = c.pm_filter_min_consistent;
    p.filter_min_ncc                 = c.pm_filter_min_ncc;
    p.filter_min_triangulation_angle = c.pm_filter_min_angle_deg;
    return p;
}

bool validConfig(const OST_Dense_Config_t &c)
{
    return (c.method == OST_DENSE_METHOD_SGBM || c.method == OST_DENSE_METHOD_PATCHMATCH) && c.threads >= 0
           && c.depth_margin_m >= 0.0 && c.terrain_gate_m >= 0.0 && c.gate_spacing_m > 0.0 && c.gate_margin_m >= 0.0
           && c.terrain_radius_m > 0.0 && c.pm_min_free_gpu_mb >= 0.0;
}

Eigen::Isometry3d toPose(const OST_Dense_View_t &v)
{
    const Eigen::Quaterniond q(v.q_ecef_from_camera[0], v.q_ecef_from_camera[1], v.q_ecef_from_camera[2],
                               v.q_ecef_from_camera[3]);
    if (!(std::abs(q.norm() - 1.0) < 1e-6))
    {
        throw BatchError("q_E_C is not a unit quaternion");
    }
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear()          = q.normalized().toRotationMatrix();
    T.translation()     = Eigen::Vector3d(v.position_ecef[0], v.position_ecef[1], v.position_ecef[2]);
    if (!T.matrix().allFinite())
    {
        throw BatchError("pose not finite");
    }
    return T;
}

// Viewing rays of the image corners and the principal point in L_k.
std::vector<Eigen::Vector3d> cornerRays(const geo::CameraModel &cam, const Eigen::Matrix3d &R_L_C)
{
    const double                 w = cam.width, h = cam.height;
    std::vector<Eigen::Vector3d> rays;
    for (const Eigen::Vector2d &p : { Eigen::Vector2d(-0.5, -0.5), Eigen::Vector2d(w - 0.5, -0.5),
                                      Eigen::Vector2d(w - 0.5, h - 0.5), Eigen::Vector2d(-0.5, h - 0.5),
                                      Eigen::Vector2d(cam.cx, cam.cy) })
    {
        rays.push_back(R_L_C * Eigen::Vector3d((p.x() - cam.cx) / cam.fx, (p.y() - cam.cy) / cam.fy, 1.0));
    }
    return rays;
}

// Terrain heights (L_k z) on a grid over the footprint of the reference
// camera on the plane z = plane_z, plus a margin (ADR-005 section 5a). The
// camera centre is the origin of L_k.
dn::HeightGrid heightGrid(const tr::TerrainPrior *terrain, const geo::LocalFrame &L,
                          const std::vector<Eigen::Vector3d> &rays, double plane_z, double spacing, double margin,
                          double &x0, double &y0, double &x1, double &y1)
{
    x0 = y0 = std::numeric_limits<double>::infinity();
    x1 = y1 = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < 4; ++i)
    {
        const Eigen::Vector3d p = rays[i] * (plane_z / rays[i].z());
        x0 = std::min(x0, p.x()), y0 = std::min(y0, p.y());
        x1 = std::max(x1, p.x()), y1 = std::max(y1, p.y());
    }
    dn::HeightGrid g;
    g.spacing = spacing;
    g.x0      = x0 - margin;
    g.y0      = y0 - margin;
    g.nx      = static_cast<int>(std::ceil((x1 - x0 + 2 * margin) / spacing)) + 1;
    g.ny      = static_cast<int>(std::ceil((y1 - y0 + 2 * margin) / spacing)) + 1;
    if (static_cast<double>(g.nx) * g.ny > 4.0e6)
    {
        throw BatchError("terrain grid too large: the footprint is implausible");
    }
    g.heights.assign(static_cast<std::size_t>(g.nx) * g.ny, std::numeric_limits<float>::quiet_NaN());
    if (terrain == nullptr)
    {
        return g;
    }
    for (int j = 0; j < g.ny; ++j)
    {
        for (int i = 0; i < g.nx; ++i)
        {
            const Eigen::Vector3d p_E = L.toEcef({ g.x0 + i * spacing, g.y0 + j * spacing, plane_z });
            if (const std::optional<Eigen::Vector3d> s = terrain->surface(p_E))
            {
                g.heights[static_cast<std::size_t>(j) * g.nx + i] = static_cast<float>(L.toLocal(*s).z());
            }
        }
    }
    return g;
}

template <typename F>
OST_Status_t guarded(OST_Dense_t *handle, F &&f)
{
    try
    {
        handle->error.clear();
        return f();
    }
    catch (const BatchError &e)
    {
        handle->error = e.what();
        return OST_ERR_INVALID_ARGUMENT;
    }
    catch (const dn::NotAvailableError &e)
    {
        handle->error = e.what();
        return OST_ERR_UNSUPPORTED;
    }
    catch (const dn::DenseError &e)
    {
        handle->error = e.what();
        return OST_ERR_NUMERIC;
    }
    catch (const cv::Exception &e)
    {
        handle->error = e.what();
        return OST_ERR_NUMERIC;
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

void OST_Dense_DefaultConfig(OST_Dense_Config_t *config)
{
    if (config == nullptr)
    {
        return;
    }
    const dn::SgbmConfig       s;
    const dn::PatchMatchConfig p;
    std::memset(config, 0, sizeof(*config));
    config->method                     = OST_DENSE_METHOD_SGBM;
    config->threads                    = 0;
    config->sgbm_target_baseline_ratio = s.target_baseline_ratio;
    config->sgbm_block_size            = s.block_size;
    config->sgbm_p1_factor             = s.p1_factor;
    config->sgbm_p2_factor             = s.p2_factor;
    config->sgbm_uniqueness_ratio      = s.uniqueness_ratio;
    config->sgbm_disp12_max_diff       = s.disp12_max_diff;
    config->sgbm_speckle_window        = s.speckle_window;
    config->sgbm_speckle_range         = s.speckle_range;
    config->sgbm_max_disparities       = s.max_disparities;
    config->pm_gpu_index               = p.gpu_index;
    config->pm_iterations              = p.num_iterations;
    config->pm_window_radius           = p.window_radius;
    config->pm_window_step             = p.window_step;
    config->pm_samples                 = p.num_samples;
    config->pm_filter_min_consistent   = p.filter_min_num_consistent;
    config->pm_filter_min_ncc          = p.filter_min_ncc;
    config->pm_filter_min_angle_deg    = p.filter_min_triangulation_angle;
    config->pm_min_free_gpu_mb         = 1024.0;  // peak use at 2048 px is about 0.6 GB (ADR-005 section 6a)
    config->depth_margin_m             = 50.0;
    config->terrain_gate_m             = 30.0;
    config->gate_spacing_m             = 5.0;
    config->gate_margin_m              = 100.0;
    config->terrain_radius_m           = 20000.0;
}

int OST_Dense_PatchMatchAvailable(void)
{
    return dn::patchMatchAvailable() ? 1 : 0;
}

OST_Status_t OST_Dense_Create(const OST_Dense_Config_t *config, const OST_Dense_Terrain_t *terrain,
                              OST_Dense_t **handle)
{
    if (config == nullptr || terrain == nullptr || handle == nullptr || terrain->dem_path == nullptr
        || terrain->dem_crs == nullptr || terrain->proj_data_dir == nullptr || !validConfig(*config))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *handle = nullptr;
    try
    {
        auto h    = std::make_unique<OST_Dense>();
        h->config = *config;
        if (config->method == OST_DENSE_METHOD_SGBM)
        {
            auto s  = std::make_unique<dn::SgbmEstimator>(sgbmConfig(*config));
            h->sgbm = s.get();
            h->estimator = std::move(s);
        }
        else
        {
            h->estimator = std::make_unique<dn::PatchMatchEstimator>(patchMatchConfig(*config));
        }
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
        if (config->threads > 0)
        {
            cv::setNumThreads(config->threads);
        }
        *handle = h.release();
        return OST_SUCCESS;
    }
    catch (const dn::NotAvailableError &)
    {
        return OST_ERR_UNSUPPORTED;
    }
    catch (const dn::DenseError &)
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

void OST_Dense_Destroy(OST_Dense_t *handle)
{
    delete handle;
}

const char *OST_Dense_LastError(const OST_Dense_t *handle)
{
    return handle == nullptr ? "no handle" : handle->error.c_str();
}

OST_Status_t OST_Dense_Estimate(OST_Dense_t *handle, const OST_Dense_Camera_t *camera,
                                const OST_Dense_View_t *reference, const OST_Dense_View_t *sources,
                                uint32_t source_count, double plane_height_m, const OST_Dense_Output_t *output,
                                OST_Dense_Result_t *result)
{
    if (handle == nullptr || camera == nullptr || reference == nullptr || (sources == nullptr && source_count > 0)
        || output == nullptr || result == nullptr || output->depth == nullptr || output->confidence == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(result, 0, sizeof(*result));
    result->source_index = -1;
    return guarded(handle, [&] {
        const OST_Dense_Config_t &cfg = handle->config;
        if (source_count == 0 || source_count > OST_DENSE_MAX_SOURCES)
        {
            throw BatchError("a batch needs 1 to 16 source views");
        }
        if (!std::isfinite(plane_height_m))
        {
            throw BatchError("footprint plane height not finite");
        }
        geo::CameraModel cam;
        cam.width  = camera->width;
        cam.height = camera->height;
        cam.fx     = camera->fx;
        cam.fy     = camera->fy;
        cam.cx     = camera->cx;
        cam.cy     = camera->cy;
        if (cam.width <= 0 || cam.height <= 0 || !(cam.fx > 0.0) || !(cam.fy > 0.0))
        {
            throw BatchError("invalid camera");
        }

        // Batch frame L_k at the keyframe's camera centre (ADR-004 section 4).
        auto                    t0    = Clock::now();
        const Eigen::Isometry3d T_E_R = toPose(*reference);
        const geo::Geodetic     g     = handle->terrain->geodetic(T_E_R.translation());
        const geo::LocalFrame   L(T_E_R.translation(), g.latitude_rad, g.longitude_rad);
        for (int i = 0; i < 3; ++i)
        {
            result->origin_ecef[i] = T_E_R.translation()[i];
        }
        result->latitude_rad  = g.latitude_rad;
        result->longitude_rad = g.longitude_rad;
        result->height_m      = g.height_m;

        dn::DepthBatch batch;
        batch.camera          = cam;
        batch.reference.T_L_C = L.poseInLocal(T_E_R);
        batch.reference.image = { reference->grey, reference->stride };
        for (uint32_t i = 0; i < source_count; ++i)
        {
            dn::BatchView v;
            v.T_L_C = L.poseInLocal(toPose(sources[i]));
            v.image = { sources[i].grey, sources[i].stride };
            batch.sources.push_back(v);
        }

        // Terrain window: read again when the keyframe nears its edge.
        bool terrain_ok = handle->terrain->loaded()
                          && handle->terrain->distanceFromWindowCentre(T_E_R.translation()) < 0.5 * cfg.terrain_radius_m;
        if (!terrain_ok)
        {
            try
            {
                handle->terrain->load(T_E_R.translation(), cfg.terrain_radius_m);
                ++handle->terrain_loads;
                terrain_ok = true;
            }
            catch (const tr::TerrainError &e)
            {
                handle->error = e.what();  // reported through terrain_available
            }
        }
        result->terrain_loads = handle->terrain_loads;

        // Footprint on the plane, terrain grid over it, depth range from the
        // terrain heights under the footprint (as tools/common Flight).
        const std::vector<Eigen::Vector3d> rays = cornerRays(cam, batch.reference.T_L_C.linear());
        double                             down_min = std::numeric_limits<double>::infinity(), down_max = 0.0;
        for (const Eigen::Vector3d &r : rays)
        {
            down_min = std::min(down_min, -r.z());
            down_max = std::max(down_max, -r.z());
        }
        if (!(down_min > 0.05))
        {
            throw BatchError("camera looks too far from nadir");
        }
        const double plane_z = plane_height_m - g.height_m;
        if (!(plane_z < -1.0))
        {
            throw BatchError("footprint plane not below the camera");
        }
        double               fx0, fy0, fx1, fy1;
        const dn::HeightGrid grid = heightGrid(terrain_ok ? handle->terrain.get() : nullptr, L, rays, plane_z,
                                               cfg.gate_spacing_m, cfg.gate_margin_m, fx0, fy0, fx1, fy1);
        double lo = std::numeric_limits<double>::infinity(), hi = -std::numeric_limits<double>::infinity();
        std::size_t known = 0;
        for (int j = 0; j < grid.ny; ++j)
        {
            for (int i = 0; i < grid.nx; ++i)
            {
                const double x = grid.x0 + i * grid.spacing, y = grid.y0 + j * grid.spacing;
                const float  h = grid.heights[static_cast<std::size_t>(j) * grid.nx + i];
                if (std::isfinite(h) && x >= fx0 && x <= fx1 && y >= fy0 && y <= fy1)
                {
                    lo = std::min(lo, static_cast<double>(h));
                    hi = std::max(hi, static_cast<double>(h));
                    ++known;
                }
            }
        }
        result->terrain_available = known > 0 ? 1 : 0;
        if (known == 0)
        {
            lo = hi = plane_z;
        }
        batch.depth_min     = std::max(1.0, (-hi - cfg.depth_margin_m) / down_max);
        batch.depth_max     = (-lo + cfg.depth_margin_m) / down_min;
        result->depth_min_m = batch.depth_min;
        result->depth_max_m = batch.depth_max;
        result->setup_s     = since(t0);

        // Depth estimation.
        const std::size_t n = static_cast<std::size_t>(cam.width) * cam.height;
        float            *normals = output->normals;
        if (normals == nullptr)
        {
            handle->normals.resize(3 * n);
            normals = handle->normals.data();
        }
        const dn::DepthOutput out { output->depth, normals, output->confidence };
        if (handle->sgbm != nullptr)
        {
            result->source_index = static_cast<int32_t>(handle->sgbm->chooseSource(batch));
        }
        else if (cfg.pm_min_free_gpu_mb > 0.0)
        {
            const std::optional<std::size_t> free_bytes = dn::gpuFreeMemory(cfg.pm_gpu_index);
            if (!free_bytes || static_cast<double>(*free_bytes) < cfg.pm_min_free_gpu_mb * 1048576.0)
            {
                handle->error = free_bytes ? "GPU memory below the configured minimum" : "GPU not available";
                return OST_ERR_GPU;
            }
        }
        t0 = Clock::now();
        handle->estimator->estimate(batch, out);
        result->estimate_s = since(t0);

        // Terrain gate (section 5a).
        t0 = Clock::now();
        if (cfg.terrain_gate_m > 0.0 && result->terrain_available)
        {
            result->gated_pixels = static_cast<uint32_t>(dn::applyTerrainGate(batch, grid, cfg.terrain_gate_m, out));
            result->gate_applied = 1;
        }
        result->gate_s       = since(t0);
        result->valid_pixels = static_cast<uint32_t>(
            std::count_if(output->depth, output->depth + n, [](float d) { return d > 0.0f; }));
        return OST_SUCCESS;
    });
}

}  // extern "C"
