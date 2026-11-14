// C interface of the map products (map_products_c_api.h).

#include "map_products_c_api.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <new>
#include <string>

#include "orthostream/geo/crs.hpp"
#include "orthostream/map_products/point_cloud.hpp"
#include "orthostream/map_products/point_set.hpp"

namespace mp  = orthostream::map_products;
namespace geo = orthostream::geo;

static_assert(sizeof(OST_Map_Point_t) == sizeof(mp::PointRecord), "point record layout");
static_assert(offsetof(OST_Map_Point_t, weight) == offsetof(mp::PointRecord, weight), "point record layout");
static_assert(offsetof(OST_Map_Point_t, rgb) == offsetof(mp::PointRecord, rgb), "point record layout");

struct OST_Map_Georef
{
    // The builder and the writer refer to the frame: it is declared first.
    std::unique_ptr<mp::ProductFrame>     frame;
    std::unique_ptr<mp::PointSetBuilder>  builder;
    std::unique_ptr<mp::PointCloudWriter> writer;
    std::vector<mp::TileSpan>             spans;
    std::string                           error;
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

geo::CameraModel camera(const OST_Map_Camera_t &c)
{
    if (c.width <= 0 || c.height <= 0 || !(c.fx > 0.0) || !(c.fy > 0.0) || !std::isfinite(c.cx)
        || !std::isfinite(c.cy))
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
    return m;
}

mp::DepthMapView depthMap(const OST_Map_DepthMap_t &m)
{
    if (m.depth == nullptr || m.confidence == nullptr)
    {
        throw ArgumentError("depth map without depth or confidence");
    }
    return { m.depth, m.confidence };
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

// Runs f and maps exceptions to status codes, keeping their text.
template <typename F>
OST_Status_t guarded(OST_Map_Georef_t *handle, OST_Status_t map_error, F &&f)
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
    catch (const mp::MapProductsError &e)
    {
        handle->error = e.what();
        return map_error;
    }
    catch (const geo::GeoError &e)
    {
        handle->error = e.what();
        return OST_ERR_NUMERIC;
    }
    catch (const std::filesystem::filesystem_error &e)
    {
        handle->error = e.what();
        return OST_ERR_IO;
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

void OST_Map_GeorefDefaultConfig(OST_Map_GeorefConfig_t *config)
{
    if (config == nullptr)
    {
        return;
    }
    std::memset(config, 0, sizeof(*config));
    config->projected_crs  = "EPSG:2056";
    config->vertical_crs   = "EPSG:5729";
    config->proj_data_dir  = "";
    config->grid_dir       = "";
    config->output_dir     = "";
    config->cell_size_m    = 0.26;
    config->tile_cells     = 512;
    config->min_confidence = 0.0;
}

OST_Status_t OST_Map_GeorefCreate(const OST_Map_GeorefConfig_t *config, OST_Map_Georef_t **handle)
{
    if (handle == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    *handle = nullptr;
    if (config == nullptr || text(config->projected_crs).empty() || text(config->proj_data_dir).empty()
        || !(config->cell_size_m > 0.0) || config->tile_cells <= 0 || !(config->min_confidence >= 0.0)
        || !(config->min_confidence <= 1.0))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    try
    {
        auto                     h = std::make_unique<OST_Map_Georef>();
        std::vector<std::string> grids;
        if (!text(config->grid_dir).empty())
        {
            grids.push_back(config->grid_dir);
        }
        mp::TileGridConfig grid;
        grid.cell_size  = config->cell_size_m;
        grid.tile_cells = config->tile_cells;
        try
        {
            h->frame = std::make_unique<mp::ProductFrame>(config->projected_crs, config->proj_data_dir, grids);
            if (!text(config->output_dir).empty())
            {
                mp::PointCloudWriterConfig pc;
                pc.directory      = config->output_dir;
                pc.projected_crs  = config->projected_crs;
                pc.vertical_crs   = text(config->vertical_crs);
                pc.proj_data_dir  = config->proj_data_dir;
                pc.grid_dirs      = grids;
                pc.grid           = grid;
                pc.min_confidence = static_cast<float>(config->min_confidence);
                h->writer         = std::make_unique<mp::PointCloudWriter>(pc, *h->frame);
            }
        }
        catch (const geo::GeoError &)
        {
            return OST_ERR_IO;  // PROJ data, grids or CRS not usable
        }
        catch (const std::filesystem::filesystem_error &)
        {
            return OST_ERR_IO;  // output directory
        }
        h->builder = std::make_unique<mp::PointSetBuilder>(*h->frame, grid, config->min_confidence);
        *handle    = h.release();
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

void OST_Map_GeorefDestroy(OST_Map_Georef_t *handle)
{
    delete handle;
}

const char *OST_Map_GeorefLastError(const OST_Map_Georef_t *handle)
{
    return handle != nullptr ? handle->error.c_str() : "no handle";
}

OST_Status_t OST_Map_GeorefCount(OST_Map_Georef_t *handle, const OST_Map_DepthMap_t *map, uint32_t *points)
{
    if (handle == nullptr || map == nullptr || points == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    return guarded(handle, OST_ERR_INVALID_ARGUMENT,
                   [&]
                   {
                       *points = static_cast<uint32_t>(handle->builder->count(depthMap(*map), camera(map->camera)));
                   });
}

OST_Status_t OST_Map_GeorefConvert(OST_Map_Georef_t *handle, const OST_Map_DepthMap_t *map,
                                   const OST_Map_Image_t *image, const OST_Map_Pose_t *pose_in,
                                   OST_Map_Point_t *points, uint32_t capacity, OST_Map_TileSpan_t *tiles,
                                   uint32_t max_tiles, OST_Map_GeorefResult_t *result)
{
    if (handle == nullptr || map == nullptr || pose_in == nullptr || points == nullptr || tiles == nullptr
        || result == nullptr)
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(result, 0, sizeof(*result));
    return guarded(handle, OST_ERR_INVALID_ARGUMENT,
                   [&]
                   {
                       const auto        t0 = Clock::now();
                       mp::WorkImageView view;
                       if (image != nullptr && image->data != nullptr)
                       {
                           view.data     = image->data;
                           view.stride   = image->stride;
                           view.channels = image->channels;
                       }
                       handle->spans.resize(max_tiles);
                       const mp::PointSetStats s = handle->builder->build(
                           depthMap(*map), camera(map->camera), view, pose(*pose_in),
                           reinterpret_cast<mp::PointRecord *>(points), capacity, handle->spans.data(), max_tiles);
                       for (std::size_t k = 0; k < s.tiles; ++k)
                       {
                           const mp::TileSpan &t = handle->spans[k];
                           tiles[k].column       = static_cast<int32_t>(t.id.ix);
                           tiles[k].row          = static_cast<int32_t>(t.id.iy);
                           tiles[k].first        = t.first;
                           tiles[k].count        = t.count;
                       }
                       result->points     = static_cast<uint32_t>(s.points);
                       result->tile_count = static_cast<uint32_t>(s.tiles);
                       for (int i = 0; i < 3; ++i)
                       {
                           result->origin[i] = s.origin[i];
                       }
                       result->height_min = s.height_min;
                       result->height_max = s.height_max;
                       result->convert_s  = since(t0);
                   });
}

OST_Status_t OST_Map_GeorefWriteLaz(OST_Map_Georef_t *handle, uint32_t keyframe, const OST_Map_PointSet_t *set,
                                    OST_Map_LazResult_t *result)
{
    if (handle == nullptr || set == nullptr || result == nullptr
        || (set->count > 0 && set->points == nullptr) || (set->tile_count > 0 && set->tiles == nullptr))
    {
        return OST_ERR_INVALID_ARGUMENT;
    }
    std::memset(result, 0, sizeof(*result));
    if (!handle->writer)
    {
        handle->error = "no output directory";
        return OST_ERR_UNSUPPORTED;
    }
    return guarded(handle, OST_ERR_IO,
                   [&]
                   {
                       const auto t0 = Clock::now();
                       handle->spans.resize(set->tile_count);
                       for (uint32_t k = 0; k < set->tile_count; ++k)
                       {
                           handle->spans[k].id    = { set->tiles[k].column, set->tiles[k].row };
                           handle->spans[k].first = set->tiles[k].first;
                           handle->spans[k].count = set->tiles[k].count;
                           if (static_cast<uint64_t>(set->tiles[k].first) + set->tiles[k].count > set->count)
                           {
                               throw ArgumentError("tile span outside the point set");
                           }
                       }
                       mp::PointSetView view;
                       view.points     = reinterpret_cast<const mp::PointRecord *>(set->points);
                       view.count      = set->count;
                       view.origin     = Eigen::Vector3d(set->origin[0], set->origin[1], set->origin[2]);
                       view.tiles      = handle->spans.data();
                       view.tile_count = handle->spans.size();
                       const mp::PointCloudStats s = handle->writer->write(keyframe, view);
                       result->files               = static_cast<uint32_t>(s.files.size());
                       result->points              = static_cast<uint32_t>(s.points);
                       result->write_s             = since(t0);
                   });
}

}  // extern "C"
