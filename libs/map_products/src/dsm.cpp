#include "orthostream/map_products/dsm.hpp"

#include "depth_points.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace orthostream::map_products
{
namespace
{

double tileSize(const TileGridConfig &g)
{
    return g.cell_size * g.tile_cells;
}

void checkGrid(const TileGridConfig &g)
{
    if (!(g.cell_size > 0.0) || g.tile_cells <= 0)
    {
        throw MapProductsError("invalid tile grid");
    }
}

}  // namespace

DsmTile::DsmTile(const TileId &id, const TileGridConfig &grid, double reference_height)
    : id_(id), cells_(grid.tile_cells), cell_size_(grid.cell_size), reference_height_(reference_height)
{
    checkGrid(grid);
    const double T = tileSize(grid);
    origin_e_      = static_cast<double>(id.ix) * T;
    origin_n_      = static_cast<double>(id.iy + 1) * T;
    const std::size_t n = static_cast<std::size_t>(cells_) * cells_;
    weight_.assign(n, 0.0f);
    mean_.assign(n, 0.0f);
    m2_.assign(n, 0.0f);
    count_.assign(n, 0);
}

DsmTile::Update DsmTile::add(int col, int row, double height, float weight, const DsmFusionConfig &config)
{
    const std::size_t i = index(col, row);
    const double      h = height - reference_height_;
    const double      W = weight_[i];
    const double      m = mean_[i];
    if (count_[i] >= config.min_observations && W > 0.0)
    {
        const double sigma = std::sqrt(std::max(0.0, static_cast<double>(m2_[i]) / W));
        if (std::abs(h - m) > config.rejection_sigmas * std::max(sigma, config.min_sigma))
        {
            return Update::rejected;
        }
    }
    const double W_new = W + weight;
    const double delta = h - m;
    const double m_new = m + (weight / W_new) * delta;
    weight_[i]         = static_cast<float>(W_new);
    mean_[i]           = static_cast<float>(m_new);
    m2_[i]             = static_cast<float>(m2_[i] + weight * delta * (h - m_new));
    if (count_[i] < std::numeric_limits<std::uint16_t>::max())
    {
        ++count_[i];
    }
    return Update::added;
}

std::optional<double> DsmTile::height(int col, int row) const
{
    const std::size_t i = index(col, row);
    if (!(weight_[i] > 0.0f))
    {
        return std::nullopt;
    }
    return reference_height_ + mean_[i];
}

std::optional<double> DsmTile::variance(int col, int row) const
{
    const std::size_t i = index(col, row);
    if (!(weight_[i] > 0.0f))
    {
        return std::nullopt;
    }
    return std::max(0.0, static_cast<double>(m2_[i]) / weight_[i]);
}

std::size_t DsmTile::filledCells() const
{
    return static_cast<std::size_t>(std::count_if(weight_.begin(), weight_.end(), [](float w) { return w > 0.0f; }));
}

DsmFusion::DsmFusion(const DsmFusionConfig &config, const ProductFrame &frame) : config_(config), frame_(frame)
{
    checkGrid(config.grid);
    if (config.min_observations < 1 || !(config.rejection_sigmas > 0.0) || !(config.min_sigma >= 0.0))
    {
        throw MapProductsError("invalid DSM fusion configuration");
    }
}

DsmTile &DsmFusion::tileFor(const TileId &id, double reference_height)
{
    auto it = tiles_.find(id);
    if (it == tiles_.end())
    {
        std::unique_ptr<DsmTile> t = loader_ ? loader_(id) : nullptr;
        if (!t)
        {
            t = std::make_unique<DsmTile>(id, config_.grid, std::round(reference_height));
        }
        it = tiles_.emplace(id, std::move(t)).first;
    }
    return *it->second;
}

std::unique_ptr<DsmTile> DsmFusion::release(const TileId &id)
{
    const auto it = tiles_.find(id);
    if (it == tiles_.end())
    {
        return nullptr;
    }
    std::unique_ptr<DsmTile> t = std::move(it->second);
    tiles_.erase(it);
    return t;
}

const DsmTile *DsmFusion::tile(const TileId &id) const
{
    const auto it = tiles_.find(id);
    return it == tiles_.end() ? nullptr : it->second.get();
}

FusionStats DsmFusion::addDepthMap(const DepthMapView &map, const geo::CameraModel &camera,
                                   const Eigen::Isometry3d &T_E_C)
{
    const double     T    = tileSize(config_.grid);
    const double     cs   = config_.grid.cell_size;
    const int        n    = config_.grid.tile_cells;
    DsmTile         *tile = nullptr;
    std::set<TileId> touched;
    FusionStats      stats;
    stats.points = detail::forEachDepthPoint(
        map, camera, T_E_C, frame_, config_.min_confidence,
        [&](std::size_t i, const Eigen::Vector3d &, const Eigen::Vector3d &p)
        {
            stats.extent.extend(p.head<2>());
            const TileId id { static_cast<std::int64_t>(std::floor(p.x() / T)),
                              static_cast<std::int64_t>(std::floor(p.y() / T)) };
            if (tile == nullptr || !(tile->id() == id))
            {
                tile = &tileFor(id, p.z());
                touched.insert(id);
            }
            const int col = std::clamp(static_cast<int>(std::floor((p.x() - tile->originEasting()) / cs)), 0, n - 1);
            const int row = std::clamp(static_cast<int>(std::floor((tile->originNorthing() - p.y()) / cs)), 0, n - 1);
            if (tile->add(col, row, p.z(), map.confidence[i], config_) == DsmTile::Update::added)
            {
                ++stats.added;
            }
            else
            {
                ++stats.rejected;
            }
        });
    stats.tiles.assign(touched.begin(), touched.end());
    return stats;
}

std::optional<double> DsmFusion::heightAt(double easting, double northing) const
{
    const double   T  = tileSize(config_.grid);
    const TileId   id { static_cast<std::int64_t>(std::floor(easting / T)),
                      static_cast<std::int64_t>(std::floor(northing / T)) };
    const DsmTile *t = tile(id);
    if (t == nullptr)
    {
        return std::nullopt;
    }
    const double cs  = config_.grid.cell_size;
    const int    n   = config_.grid.tile_cells;
    const int    col = std::clamp(static_cast<int>(std::floor((easting - t->originEasting()) / cs)), 0, n - 1);
    const int    row = std::clamp(static_cast<int>(std::floor((t->originNorthing() - northing) / cs)), 0, n - 1);
    return t->height(col, row);
}

}  // namespace orthostream::map_products
