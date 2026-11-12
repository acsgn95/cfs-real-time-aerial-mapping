#include "orthostream/map_products/point_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>

#include <laszip/laszip_api.h>

#include "depth_points.hpp"
#include "vertical_datum.hpp"

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;

constexpr double kScale = 0.001;  // millimetre coordinates

std::string laszipError(laszip_POINTER p)
{
    laszip_CHAR *msg = nullptr;
    laszip_get_error(p, &msg);
    return msg != nullptr ? msg : "unknown LASzip error";
}

// GeoKeyDirectoryTag (LAS 1.2 "LASF_Projection" record 34735).
std::vector<laszip_U8> geoKeys(int projected_epsg, int vertical_epsg)
{
    std::vector<std::uint16_t> keys = { 1, 1, 0, 0,                                     // header, count below
                                        1024, 0, 1, 1,                                  // GTModelType: projected
                                        3072, 0, 1, static_cast<std::uint16_t>(projected_epsg) };
    if (vertical_epsg != 0)
    {
        keys.insert(keys.end(), { 4096, 0, 1, static_cast<std::uint16_t>(vertical_epsg),   // VerticalCSType
                                  4099, 0, 1, 9001 });                                      // vertical units: metre
    }
    keys[3] = static_cast<std::uint16_t>(keys.size() / 4 - 1);
    std::vector<laszip_U8> bytes(keys.size() * 2);
    for (std::size_t i = 0; i < keys.size(); ++i)  // little-endian
    {
        bytes[2 * i]     = static_cast<laszip_U8>(keys[i] & 0xFF);
        bytes[2 * i + 1] = static_cast<laszip_U8>(keys[i] >> 8);
    }
    return bytes;
}

// One open LAZ file of a tile.
struct TileFile
{
    laszip_POINTER      writer = nullptr;
    laszip_point       *point  = nullptr;
    fs::path            path, tmp;
    detail::OffsetGrid  offsets { 0, 0, 1 };
    double              x0 = 0, y0 = 0, z0 = 0;
};

}  // namespace

struct PointCloudWriter::Impl
{
    PointCloudWriterConfig config;
    const ProductFrame    &frame;
    detail::VerticalDatum  datum;
    std::vector<laszip_U8> keys;

    Impl(const PointCloudWriterConfig &c, const ProductFrame &f)
        : config(c), frame(f), datum(f, c.projected_crs, c.vertical_crs, c.proj_data_dir, c.grid_dirs),
          keys(geoKeys(datum.projectedEpsg(), datum.verticalEpsg()))
    {
    }

    TileFile open(const TileId &id, std::uint32_t keyframe, double height)
    {
        TileFile     f;
        const double T = config.grid.cell_size * config.grid.tile_cells;
        const double west = static_cast<double>(id.ix) * T, north = static_cast<double>(id.iy + 1) * T;
        f.offsets         = datum.offsets(west, north, T, height);
        f.x0              = west;
        f.y0              = north - T;
        f.z0              = std::round(height + f.offsets.at(west + T / 2, north - T / 2));
        f.path            = fs::path(config.directory) / "points" / fileName(id, keyframe);
        f.tmp             = fs::path(f.path).concat(".tmp");

        laszip_header *h = nullptr;
        if (laszip_create(&f.writer) != 0 || laszip_get_header_pointer(f.writer, &h) != 0)
        {
            throw MapProductsError("cannot create a LASzip writer");
        }
        h->version_major            = 1;
        h->version_minor            = 2;
        h->point_data_format        = 2;
        h->point_data_record_length = 26;
        h->x_scale_factor = h->y_scale_factor = h->z_scale_factor = kScale;
        h->x_offset                 = f.x0;
        h->y_offset                 = f.y0;
        h->z_offset                 = f.z0;
        std::snprintf(h->generating_software, sizeof(h->generating_software), "OrthoStream");
        std::snprintf(h->system_identifier, sizeof(h->system_identifier), "OrthoStream keyframe %u", keyframe);
        if (laszip_add_vlr(f.writer, "LASF_Projection", 34735, static_cast<laszip_U16>(keys.size()),
                           "GeoKeyDirectoryTag", keys.data())
                != 0
            || laszip_open_writer(f.writer, f.tmp.c_str(), 1) != 0 || laszip_get_point_pointer(f.writer, &f.point) != 0)
        {
            const std::string msg = laszipError(f.writer);
            laszip_destroy(f.writer);
            throw MapProductsError("cannot open " + f.tmp.string() + ": " + msg);
        }
        return f;
    }

    void close(TileFile &f)
    {
        const bool ok = laszip_close_writer(f.writer) == 0;
        laszip_destroy(f.writer);
        f.writer = nullptr;
        if (!ok)
        {
            fs::remove(f.tmp);
            throw MapProductsError("writing " + f.path.string() + " failed");
        }
        fs::rename(f.tmp, f.path);
    }
};

PointCloudWriter::PointCloudWriter(const PointCloudWriterConfig &config, const ProductFrame &frame)
    : impl_(std::make_unique<Impl>(config, frame))
{
    if (config.directory.empty() || !(config.grid.cell_size > 0.0) || config.grid.tile_cells <= 0)
    {
        throw MapProductsError("invalid point cloud writer configuration");
    }
    fs::create_directories(fs::path(config.directory) / "points");
}

PointCloudWriter::~PointCloudWriter() = default;

std::string PointCloudWriter::fileName(const TileId &id, std::uint32_t keyframe)
{
    char name[96];
    std::snprintf(name, sizeof(name), "points_%lld_%lld_kf%08u.laz", static_cast<long long>(id.ix),
                  static_cast<long long>(id.iy), keyframe);
    return name;
}

PointCloudStats PointCloudWriter::write(std::uint32_t keyframe, const DepthMapView &map,
                                        const geo::CameraModel &depth_camera, const RgbImageView &image,
                                        const geo::CameraModel &image_camera, const Eigen::Isometry3d &T_E_C)
{
    if (image.data == nullptr || image.width != image_camera.width || image.height != image_camera.height)
    {
        throw MapProductsError("invalid keyframe image");
    }
    const double                T = impl_->config.grid.cell_size * impl_->config.grid.tile_cells;
    std::map<TileId, TileFile>  files;
    TileFile                   *current = nullptr;
    TileId                      current_id;
    PointCloudStats             stats;
    try
    {
        stats.points = detail::forEachDepthPoint(
            map, depth_camera, T_E_C, impl_->frame, impl_->config.min_confidence,
            [&](std::size_t i, const Eigen::Vector3d &p_C, const Eigen::Vector3d &p)
            {
                const TileId id { static_cast<std::int64_t>(std::floor(p.x() / T)),
                                  static_cast<std::int64_t>(std::floor(p.y() / T)) };
                if (current == nullptr || !(current_id == id))
                {
                    auto it = files.find(id);
                    if (it == files.end())
                    {
                        it = files.emplace(id, impl_->open(id, keyframe, p.z())).first;
                    }
                    current    = &it->second;
                    current_id = id;
                }
                laszip_point &pt = *current->point;
                const double  z  = p.z() + current->offsets.at(p.x(), p.y());
                pt.X             = static_cast<laszip_I32>(std::lround((p.x() - current->x0) / kScale));
                pt.Y             = static_cast<laszip_I32>(std::lround((p.y() - current->y0) / kScale));
                pt.Z             = static_cast<laszip_I32>(std::lround((z - current->z0) / kScale));
                pt.intensity     = static_cast<laszip_U16>(std::lround(std::clamp(map.confidence[i], 0.0f, 1.0f) * 65535.0f));
                pt.return_number     = 1;
                pt.number_of_returns = 1;
                pt.point_source_ID   = static_cast<laszip_U16>(keyframe & 0xFFFF);
                pt.rgb[0] = pt.rgb[1] = pt.rgb[2] = 0;
                if (const auto uv = geo::projectPoint(image_camera, p_C))
                {
                    const long u = std::lround(uv->x()), v = std::lround(uv->y());
                    if (u >= 0 && v >= 0 && u < image.width && v < image.height)
                    {
                        const std::uint8_t *px = image.data + static_cast<std::size_t>(v) * image.stride + 3 * u;
                        for (int c = 0; c < 3; ++c)
                        {
                            pt.rgb[c] = static_cast<laszip_U16>(px[c] * 257);  // 8 to 16 bit
                        }
                    }
                }
                if (laszip_write_point(current->writer) != 0 || laszip_update_inventory(current->writer) != 0)
                {
                    throw MapProductsError("writing a point failed: " + laszipError(current->writer));
                }
            });
        for (auto &[id, f] : files)
        {
            impl_->close(f);
            stats.files.push_back(f.path.string());
        }
    }
    catch (...)
    {
        for (auto &[id, f] : files)
        {
            if (f.writer != nullptr)
            {
                laszip_close_writer(f.writer);
                laszip_destroy(f.writer);
                fs::remove(f.tmp);
            }
        }
        throw;
    }
    return stats;
}

}  // namespace orthostream::map_products
