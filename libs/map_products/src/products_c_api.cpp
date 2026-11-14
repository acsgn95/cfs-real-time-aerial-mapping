// C interface of the map products: DSM fusion, orthomosaic, lifecycle
// (map_products_c_api.h, OST_Map_Products*).

#include "map_products_c_api.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "orthostream/geo/camera.hpp"
#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/frames.hpp"
#include "orthostream/map_products/dsm.hpp"
#include "orthostream/map_products/lifecycle.hpp"
#include "orthostream/map_products/ortho.hpp"
#include "orthostream/map_products/point_set.hpp"
#include "orthostream/map_products/writer.hpp"

namespace mp  = orthostream::map_products;
namespace geo = orthostream::geo;
namespace fs  = std::filesystem;

struct OST_Map_Products
{
    // Declaration order is construction order: everything refers to the
    // frame, the manager to the products and writers.
    double                                render_margin = 30.0;
    std::unique_ptr<mp::ProductFrame>     frame;
    // The writers' own frame: the DSM writer converts heights through PROJ
    // on the publisher's task, and PROJ objects are not thread-safe.
    std::unique_ptr<mp::ProductFrame>     writer_frame;
    std::unique_ptr<geo::GeodeticConverter> geodetic;
    std::unique_ptr<mp::DsmFusion>        dsm;
    std::unique_ptr<mp::OrthoRenderer>    ortho;
    std::unique_ptr<mp::DsmWriter>        dsm_writer;
    std::unique_ptr<mp::OrthoWriter>      ortho_writer;
    std::unique_ptr<mp::TileManager>      manager;
    std::vector<mp::TileSpan>             spans;
    std::vector<float>                    heights;
    std::string                           error;
    std::string                           publish_error;  // WriteSnapshot, on its own task

    ~OST_Map_Products()
    {
        // The manager installs loaders on the products: it goes first.
        manager.reset();
    }
};

struct OST_Map_Snapshot
{
    mp::TileSnapshot tiles;
};

namespace
{

using Clock = std::chrono::steady_clock;

double since(Clock::time_point t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

class ArgumentError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

std::string text(const char *s)
{
    return s != nullptr ? s : "";
}

template <typename F>
OST_Status_t guarded(OST_Map_Products_t *handle, F &&f)
{
    try
    {
        handle->error.clear();
        f();
        return OST_SUCCESS;
    }
    catch (const ArgumentError &e)
    {
        handle->error = e.what();
        return OST_ERR_INVALID_ARGUMENT;
    }
    catch (const fs::filesystem_error &e)
    {
        handle->error = e.what();
        return OST_ERR_IO;
    }
    catch (const mp::MapProductsError &e)
    {
        // Writers and state files report through it; argument errors are
        // caught above.
        handle->error = e.what();
        return OST_ERR_IO;
    }
    catch (const geo::GeoError &e)
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

Eigen::Isometry3d pose(const OST_Map_Pose_t &p)
{
    const Eigen::Quaterniond q(p.q_ecef_from_camera[0], p.q_ecef_from_camera[1], p.q_ecef_from_camera[2],
                               p.q_ecef_from_camera[3]);
    if (!(std::abs(q.norm() - 1.0) < 1e-6))
    {
        throw ArgumentError("q_E_C is not a unit quaternion");
    }
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear()          = q.normalized().toRotationMatrix();
    T.translation()     = Eigen::Vector3d(p.position_ecef[0], p.position_ecef[1], p.position_ecef[2]);
    if (!T.matrix().allFinite())
    {
        throw ArgumentError("pose not finite");
    }
    return T;
}

geo::CameraModel fullCamera(const OST_Map_FullCamera_t &c)
{
    if (c.width <= 0 || c.height <= 0 || !(c.fx > 0.0) || !(c.fy > 0.0))
    {
        throw ArgumentError("invalid camera");
    }
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

void listTiles(const std::vector<mp::TileId> &ids, OST_Map_TileSpan_t *tiles, uint32_t max_tiles)
{
    if (tiles == nullptr)
    {
        return;
    }
    for (std::size_t k = 0; k < ids.size() && k < max_tiles; ++k)
    {
        tiles[k].column = static_cast<int32_t>(ids[k].ix);
        tiles[k].row    = static_cast<int32_t>(ids[k].iy);
        tiles[k].first  = 0;
        tiles[k].count  = 0;
    }
}

// Footprint of the camera on the ellipsoidal height h in P: the image
// corners and edge midpoints, undistorted, intersected with the level plane
// through the point below the camera. Empty if a ray does not reach the
// ground.
Eigen::AlignedBox2d footprint(const OST_Map_Products &h, const geo::CameraModel &cam, const Eigen::Isometry3d &T,
                              double ground)
{
    const geo::Geodetic   g  = h.geodetic->ecefToGeodetic(T.translation());
    const Eigen::Vector3d up = geo::rotationEcefFromEnu(g.latitude_rad, g.longitude_rad).col(2);
    const double          w = cam.width, ht = cam.height;
    Eigen::AlignedBox2d   box;
    for (const Eigen::Vector2d &px :
         { Eigen::Vector2d(-0.5, -0.5), Eigen::Vector2d(w / 2, -0.5), Eigen::Vector2d(w - 0.5, -0.5),
           Eigen::Vector2d(w - 0.5, ht / 2), Eigen::Vector2d(w - 0.5, ht - 0.5), Eigen::Vector2d(w / 2, ht - 0.5),
           Eigen::Vector2d(-0.5, ht - 0.5), Eigen::Vector2d(-0.5, ht / 2) })
    {
        std::optional<Eigen::Vector2d> n = geo::undistortPixel(cam, px);
        if (!n)
        {
            n = Eigen::Vector2d((px.x() - cam.cx) / cam.fx, (px.y() - cam.cy) / cam.fy);
        }
        const Eigen::Vector3d dir  = T.linear() * Eigen::Vector3d(n->x(), n->y(), 1.0);
        const double          down = -dir.dot(up);
        if (!(down > 1e-3) || !(g.height_m > ground))
        {
            return Eigen::AlignedBox2d();
        }
        const Eigen::Vector3d p = T.translation() + dir * ((g.height_m - ground) / down);
        box.extend(h.frame->fromEcef(p).head<2>());
    }
    return box;
}

}  // namespace

extern "C" {

void OST_Map_ProductsDefaultConfig(OST_Map_ProductsConfig_t *config)
{
    if (config == nullptr)
    {
        return;
    }
    std::memset(config, 0, sizeof(*config));
    config->projected_crs       = "EPSG:2056";
    config->vertical_crs        = "EPSG:5729";
    config->proj_data_dir       = "";
    config->grid_dir            = "";
    config->output_dir          = "";
    config->cell_size_m         = 0.26;
    config->tile_cells          = 512;
    config->min_observations    = 3;
    config->rejection_sigmas    = 3.0;
    config->min_sigma_m         = 0.25;
    config->min_confidence      = 0.0;
    config->ortho_pixel_m       = 0.066;
    config->surface_smoothing_m = 1.0;
    config->switch_margin_deg   = 2.0;
    config->render_margin_m     = 30.0;
    config->max_active_tiles    = 64;
    config->finalise_distance_m = 600.0;
    config->defer_writes        = 1;
}

OST_Status_t OST_Map_ProductsCreate(const OST_Map_ProductsConfig_t *config, OST_Map_Products_t **handle)
{
    if (handle == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *handle = nullptr;
    if (config == nullptr || text(config->projected_crs).empty() || text(config->proj_data_dir).empty()
        || text(config->output_dir).empty() || !(config->cell_size_m > 0.0) || config->tile_cells <= 0
        || config->min_observations < 1 || !(config->rejection_sigmas > 0.0) || !(config->min_sigma_m >= 0.0)
        || !(config->min_confidence >= 0.0) || !(config->min_confidence <= 1.0) || !(config->ortho_pixel_m > 0.0)
        || !(config->surface_smoothing_m >= 0.0) || !(config->switch_margin_deg >= 0.0)
        || !(config->render_margin_m >= 0.0) || config->max_active_tiles < 1 || !(config->finalise_distance_m > 0.0))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    try
    {
        auto                     h = std::make_unique<OST_Map_Products>();
        std::vector<std::string> grids;
        if (!text(config->grid_dir).empty())
        {
            grids.push_back(config->grid_dir);
        }
        mp::DsmFusionConfig dc;
        dc.grid.cell_size    = config->cell_size_m;
        dc.grid.tile_cells   = config->tile_cells;
        dc.min_observations  = config->min_observations;
        dc.rejection_sigmas  = config->rejection_sigmas;
        dc.min_sigma         = config->min_sigma_m;
        dc.min_confidence    = config->min_confidence;
        mp::OrthoConfig oc;
        oc.pixel_size        = config->ortho_pixel_m;
        oc.surface_smoothing = config->surface_smoothing_m;
        oc.switch_margin_deg = config->switch_margin_deg;
        mp::TileLifecycleConfig lc;
        lc.max_active_tiles  = static_cast<std::size_t>(config->max_active_tiles);
        lc.finalise_distance = config->finalise_distance_m;
        lc.state_directory   = (fs::path(config->output_dir) / "state").string();
        lc.defer_writes      = config->defer_writes != 0;
        h->render_margin     = config->render_margin_m;
        try
        {
            h->frame    = std::make_unique<mp::ProductFrame>(config->projected_crs, config->proj_data_dir, grids);
            h->writer_frame = std::make_unique<mp::ProductFrame>(config->projected_crs, config->proj_data_dir, grids);
            h->geodetic = std::make_unique<geo::GeodeticConverter>(config->proj_data_dir);
            h->dsm      = std::make_unique<mp::DsmFusion>(dc, *h->frame);
            h->ortho    = std::make_unique<mp::OrthoRenderer>(oc, *h->dsm, *h->frame);
            h->dsm_writer = std::make_unique<mp::DsmWriter>(
                mp::DsmWriterConfig { config->output_dir, config->projected_crs, text(config->vertical_crs),
                                      config->proj_data_dir, grids },
                *h->writer_frame);
            h->ortho_writer = std::make_unique<mp::OrthoWriter>(
                mp::OrthoWriterConfig { config->output_dir, config->projected_crs, config->proj_data_dir });
            fs::create_directories(lc.state_directory);
            h->manager = std::make_unique<mp::TileManager>(lc, *h->dsm, *h->ortho, *h->dsm_writer, *h->ortho_writer);
        }
        catch (const geo::GeoError &)
        {
            return OST_ERR_IO;
        }
        catch (const fs::filesystem_error &)
        {
            return OST_ERR_IO;
        }
        *handle = h.release();
        return OST_SUCCESS;
    }
    catch (const mp::MapProductsError &)
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

void OST_Map_ProductsDestroy(OST_Map_Products_t *handle)
{
    delete handle;
}

const char *OST_Map_ProductsLastError(const OST_Map_Products_t *handle)
{
    return handle != nullptr ? handle->error.c_str() : "no handle";
}

OST_Status_t OST_Map_ProductsAddPoints(OST_Map_Products_t *handle, const OST_Map_PointSet_t *set,
                                       OST_Map_TileSpan_t *tiles, uint32_t max_tiles, OST_Map_FusionResult_t *result)
{
    if (handle == nullptr || set == nullptr || result == nullptr || (set->count > 0 && set->points == nullptr)
        || (set->tile_count > 0 && set->tiles == nullptr))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(result, 0, sizeof(*result));
    return guarded(handle,
                   [&]
                   {
                       const auto t0 = Clock::now();
                       handle->spans.resize(set->tile_count);
                       for (uint32_t k = 0; k < set->tile_count; ++k)
                       {
                           if (static_cast<uint64_t>(set->tiles[k].first) + set->tiles[k].count > set->count)
                           {
                               throw ArgumentError("tile span outside the point set");
                           }
                           handle->spans[k].id    = { set->tiles[k].column, set->tiles[k].row };
                           handle->spans[k].first = set->tiles[k].first;
                           handle->spans[k].count = set->tiles[k].count;
                       }
                       mp::PointSetView view;
                       view.points     = reinterpret_cast<const mp::PointRecord *>(set->points);
                       view.count      = set->count;
                       view.origin     = Eigen::Vector3d(set->origin[0], set->origin[1], set->origin[2]);
                       view.tiles      = handle->spans.data();
                       view.tile_count = handle->spans.size();
                       const mp::FusionStats s = handle->dsm->addPoints(view);
                       handle->manager->markChanged(s.tiles);

                       // Median height from a subsample, for the footprint.
                       handle->heights.clear();
                       const std::size_t step = std::max<std::size_t>(1, set->count / 4096);
                       for (std::size_t i = 0; i < set->count; i += step)
                       {
                           handle->heights.push_back(set->points[i].height);
                       }
                       if (!handle->heights.empty())
                       {
                           auto mid = handle->heights.begin() + static_cast<std::ptrdiff_t>(handle->heights.size() / 2);
                           std::nth_element(handle->heights.begin(), mid, handle->heights.end());
                           result->height_median = set->origin[2] + *mid;
                       }
                       listTiles(s.tiles, tiles, max_tiles);
                       result->points     = static_cast<uint32_t>(s.points);
                       result->added      = static_cast<uint32_t>(s.added);
                       result->rejected   = static_cast<uint32_t>(s.rejected);
                       result->tile_count = static_cast<uint32_t>(s.tiles.size());
                       if (!s.extent.isEmpty())
                       {
                           result->extent[0] = s.extent.min().x();
                           result->extent[1] = s.extent.min().y();
                           result->extent[2] = s.extent.max().x();
                           result->extent[3] = s.extent.max().y();
                       }
                       result->fuse_s = since(t0);
                   });
}

OST_Status_t OST_Map_ProductsRender(OST_Map_Products_t *handle, const OST_Map_RgbImage_t *image,
                                    const OST_Map_FullCamera_t *camera, const OST_Map_Pose_t *pose_in,
                                    double ground_height, const double *extent, OST_Map_TileSpan_t *tiles,
                                    uint32_t max_tiles, OST_Map_RenderResult_t *result)
{
    if (handle == nullptr || image == nullptr || image->data == nullptr || camera == nullptr || pose_in == nullptr
        || result == nullptr || image->width <= 0 || image->height <= 0
        || image->stride < static_cast<size_t>(3 * image->width) || !std::isfinite(ground_height))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(result, 0, sizeof(*result));
    return guarded(handle,
                   [&]
                   {
                       const auto              t0   = Clock::now();
                       const geo::CameraModel  full = fullCamera(*camera);
                       const Eigen::Isometry3d T    = pose(*pose_in);
                       Eigen::AlignedBox2d     box  = footprint(*handle, full, T, ground_height);
                       if (!box.isEmpty())
                       {
                           box.min().array() -= handle->render_margin;
                           box.max().array() += handle->render_margin;
                       }
                       if (extent != nullptr)
                       {
                           const Eigen::AlignedBox2d limit(Eigen::Vector2d(extent[0], extent[1]),
                                                           Eigen::Vector2d(extent[2], extent[3]));
                           box = box.isEmpty() ? limit : box.intersection(limit);
                       }
                       if (box.isEmpty())
                       {
                           result->render_s = since(t0);
                           return;  // nothing to render
                       }
                       const geo::CameraModel cam = geo::scaledCamera(full, image->width, image->height);
                       const mp::RenderStats  s   = handle->ortho->render(
                           { image->data, image->width, image->height, image->stride }, cam, T, box);
                       handle->manager->markChanged(s.tiles);
                       listTiles(s.tiles, tiles, max_tiles);
                       result->pixels     = static_cast<uint32_t>(s.pixels);
                       result->tile_count = static_cast<uint32_t>(s.tiles.size());
                       result->render_s   = since(t0);
                   });
}

OST_Status_t OST_Map_ProductsUpdate(OST_Map_Products_t *handle, const double vehicle_ecef[3],
                                    OST_Map_WriteResult_t *result)
{
    if (handle == nullptr || vehicle_ecef == nullptr || result == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(result, 0, sizeof(*result));
    return guarded(handle,
                   [&]
                   {
                       const auto            t0 = Clock::now();
                       const Eigen::Vector3d v(vehicle_ecef[0], vehicle_ecef[1], vehicle_ecef[2]);
                       if (!v.allFinite())
                       {
                           throw ArgumentError("vehicle position not finite");
                       }
                       result->tiles        = static_cast<uint32_t>(
                           handle->manager->update(handle->frame->fromEcef(v).head<2>()).size());
                       result->active_tiles = static_cast<uint32_t>(handle->manager->activeTiles());
                       result->write_s      = since(t0);
                   });
}

OST_Status_t OST_Map_ProductsPublish(OST_Map_Products_t *handle, OST_Map_WriteResult_t *result)
{
    if (handle == nullptr || result == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(result, 0, sizeof(*result));
    return guarded(handle,
                   [&]
                   {
                       const auto t0        = Clock::now();
                       result->tiles        = static_cast<uint32_t>(handle->manager->publish());
                       result->active_tiles = static_cast<uint32_t>(handle->manager->activeTiles());
                       result->write_s      = since(t0);
                   });
}

OST_Status_t OST_Map_ProductsSnapshot(OST_Map_Products_t *handle, OST_Map_Snapshot_t **snapshot, uint32_t *tiles)
{
    if (handle == nullptr || snapshot == nullptr || tiles == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *snapshot = nullptr;
    *tiles    = 0;
    return guarded(handle,
                   [&]
                   {
                       auto s = std::make_unique<OST_Map_Snapshot>();
                       s->tiles = handle->manager->snapshot();
                       *tiles   = static_cast<uint32_t>(s->tiles.tiles.size());
                       if (*tiles > 0)
                       {
                           *snapshot = s.release();
                       }
                   });
}

OST_Status_t OST_Map_ProductsWriteSnapshot(OST_Map_Products_t *handle, OST_Map_Snapshot_t *snapshot,
                                           OST_Map_WriteResult_t *result)
{
    std::unique_ptr<OST_Map_Snapshot> owned(snapshot);
    if (handle == nullptr || result == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(result, 0, sizeof(*result));
    if (!owned)
    {
        return OST_SUCCESS;
    }
    // Not guarded(): that writes handle->error, which belongs to the owner.
    const auto t0 = Clock::now();
    try
    {
        handle->publish_error.clear();
        result->tiles   = static_cast<uint32_t>(handle->manager->write(owned->tiles));
        result->write_s = since(t0);
        return OST_SUCCESS;
    }
    catch (const std::bad_alloc &)
    {
        handle->publish_error = "out of memory";
        return OST_ERR_NO_MEMORY;
    }
    catch (const std::exception &e)
    {
        handle->publish_error = e.what();
    }
    catch (...)
    {
        handle->publish_error = "unknown exception";
    }
    result->write_s = since(t0);
    return OST_ERR_IO;
}

const char *OST_Map_ProductsPublishError(const OST_Map_Products_t *handle)
{
    return handle != nullptr ? handle->publish_error.c_str() : "no handle";
}

OST_Status_t OST_Map_ProductsFinaliseAll(OST_Map_Products_t *handle, OST_Map_WriteResult_t *result)
{
    if (handle == nullptr || result == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(result, 0, sizeof(*result));
    return guarded(handle,
                   [&]
                   {
                       const auto t0        = Clock::now();
                       result->tiles        = static_cast<uint32_t>(handle->manager->finaliseAll());
                       result->active_tiles = static_cast<uint32_t>(handle->manager->activeTiles());
                       result->write_s      = since(t0);
                   });
}

OST_Status_t OST_Map_ProductsGetStats(const OST_Map_Products_t *handle, OST_Map_ProductsStats_t *stats)
{
    if (handle == nullptr || stats == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(stats, 0, sizeof(*stats));
    stats->active_tiles  = static_cast<uint32_t>(handle->manager->activeTiles());
    stats->changed_tiles = static_cast<uint32_t>(handle->manager->changedTiles());
    const uint64_t cells = static_cast<uint64_t>(handle->dsm->config().grid.tile_cells);
    const uint64_t px    = static_cast<uint64_t>(handle->ortho->tilePixels());
    // DSM: weight, mean, M2 (float) and count (uint16) per cell; orthomosaic:
    // RGB and the viewing angle (uint16) per pixel.
    std::size_t dsm_tiles = handle->dsm->tiles().size(), ortho_tiles = handle->ortho->tiles().size();
    for (const auto &e : handle->manager->deferredTiles())
    {
        dsm_tiles += e.dsm ? 1 : 0;
        ortho_tiles += e.ortho ? 1 : 0;
    }
    stats->deferred_tiles = static_cast<uint32_t>(handle->manager->deferredTiles().size());
    stats->memory_bytes   = dsm_tiles * cells * cells * 14u + ortho_tiles * px * px * 5u;
    return OST_SUCCESS;
}

}  // extern "C"
