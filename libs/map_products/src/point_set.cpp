#include "orthostream/map_products/point_set.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "depth_points.hpp"

namespace orthostream::map_products
{

PointSetBuilder::PointSetBuilder(const ProductFrame &frame, const TileGridConfig &grid, double min_confidence)
    : frame_(frame), grid_(grid), min_confidence_(min_confidence)
{
    if (!(grid.cell_size > 0.0) || grid.tile_cells <= 0 || !(min_confidence >= 0.0))
    {
        throw MapProductsError("invalid point set configuration");
    }
}

std::size_t PointSetBuilder::count(const DepthMapView &map, const geo::CameraModel &camera) const
{
    if (map.depth == nullptr || map.confidence == nullptr || camera.width <= 0 || camera.height <= 0)
    {
        throw MapProductsError("invalid depth map");
    }
    const std::size_t n     = static_cast<std::size_t>(camera.width) * camera.height;
    std::size_t       count = 0;
    for (std::size_t i = 0; i < n; ++i)
    {
        // The rule of detail::forEachDepthPoint.
        count += map.depth[i] > 0.0f && map.confidence[i] > 0.0f && map.confidence[i] >= min_confidence_;
    }
    return count;
}

PointSetStats PointSetBuilder::build(const DepthMapView &map, const geo::CameraModel &camera,
                                     const WorkImageView &image, const Eigen::Isometry3d &T_E_C,
                                     PointRecord *points, std::size_t capacity, TileSpan *tiles,
                                     std::size_t max_tiles)
{
    if (points == nullptr || tiles == nullptr
        || (image.data != nullptr
            && ((image.channels != 1 && image.channels != 3)
                || image.stride < static_cast<std::size_t>(camera.width) * image.channels)))
    {
        throw MapProductsError("invalid point set buffers or image");
    }
    const double T = grid_.cell_size * grid_.tile_cells;

    PointSetStats stats;
    const Eigen::Vector3d centre = frame_.fromEcef(T_E_C.translation());
    stats.origin = Eigen::Vector3d(std::round(centre.x()), std::round(centre.y()), std::round(centre.z()));
    stats.height_min = std::numeric_limits<double>::infinity();
    stats.height_max = -std::numeric_limits<double>::infinity();

    // Pass 1: points in pixel order with their tile.
    std::vector<TileId> ids;  // tiles seen, in order of appearance
    unsorted_.clear();
    tile_of_.clear();
    TileId        last {};
    std::uint32_t last_index = 0;
    bool          have_last  = false;
    stats.points             = detail::forEachDepthPoint(
        map, camera, T_E_C, frame_, min_confidence_,
        [&](std::size_t i, const Eigen::Vector3d &, const Eigen::Vector3d &p)
        {
            const TileId id { static_cast<std::int64_t>(std::floor(p.x() / T)),
                              static_cast<std::int64_t>(std::floor(p.y() / T)) };
            if (!have_last || !(id == last))
            {
                const auto it = std::find(ids.begin(), ids.end(), id);
                last_index    = static_cast<std::uint32_t>(it - ids.begin());
                if (it == ids.end())
                {
                    ids.push_back(id);
                }
                last      = id;
                have_last = true;
            }
            PointRecord r;
            r.east   = static_cast<float>(p.x() - stats.origin.x());
            r.north  = static_cast<float>(p.y() - stats.origin.y());
            r.height = static_cast<float>(p.z() - stats.origin.z());
            r.weight = map.confidence[i];
            if (image.data != nullptr)
            {
                const std::size_t   u  = i % static_cast<std::size_t>(camera.width);
                const std::size_t   v  = i / static_cast<std::size_t>(camera.width);
                const std::uint8_t *px = image.data + v * image.stride + u * image.channels;
                for (int c = 0; c < 3; ++c)
                {
                    r.rgb[c] = px[image.channels == 3 ? c : 0];
                }
            }
            stats.height_min = std::min(stats.height_min, p.z());
            stats.height_max = std::max(stats.height_max, p.z());
            unsorted_.push_back(r);
            tile_of_.push_back(last_index);
        });
    if (stats.points == 0)
    {
        stats.height_min = stats.height_max = 0.0;
        return stats;
    }
    if (stats.points > capacity)
    {
        throw MapProductsError(std::to_string(stats.points) + " points, capacity " + std::to_string(capacity));
    }
    if (ids.size() > max_tiles)
    {
        throw MapProductsError(std::to_string(ids.size()) + " tiles, at most " + std::to_string(max_tiles));
    }

    // Pass 2: counting sort by tile, tiles in TileId order.
    std::vector<std::uint32_t> order(ids.size());
    for (std::uint32_t k = 0; k < order.size(); ++k)
    {
        order[k] = k;
    }
    std::sort(order.begin(), order.end(), [&ids](std::uint32_t a, std::uint32_t b) { return ids[a] < ids[b]; });
    std::vector<std::uint32_t> counts(ids.size(), 0), next(ids.size(), 0);
    for (const std::uint32_t t : tile_of_)
    {
        ++counts[t];
    }
    std::uint32_t first = 0;
    for (std::size_t k = 0; k < order.size(); ++k)
    {
        const std::uint32_t t = order[k];
        tiles[k].id           = ids[t];
        tiles[k].first        = first;
        tiles[k].count        = counts[t];
        next[t]               = first;
        first += counts[t];
    }
    for (std::size_t j = 0; j < unsorted_.size(); ++j)
    {
        points[next[tile_of_[j]]++] = unsorted_[j];
    }
    stats.tiles = ids.size();
    return stats;
}

}  // namespace orthostream::map_products
