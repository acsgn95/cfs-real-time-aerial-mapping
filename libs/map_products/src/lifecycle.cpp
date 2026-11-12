#include "orthostream/map_products/lifecycle.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;

}  // namespace

TileManager::TileManager(const TileLifecycleConfig &config, DsmFusion &dsm, OrthoRenderer &ortho,
                         DsmWriter &dsm_writer, OrthoWriter &ortho_writer)
    : config_(config), dsm_(dsm), ortho_(ortho), dsm_writer_(dsm_writer), ortho_writer_(ortho_writer)
{
    if (config.max_active_tiles == 0 || !(config.finalise_distance > 0.0) || config.state_directory.empty())
    {
        throw MapProductsError("invalid tile lifecycle configuration");
    }
    const TileGridConfig &g = dsm.config().grid;
    tile_size_              = g.cell_size * g.tile_cells;
    fs::create_directories(config.state_directory);

    // Reload written tiles when a keyframe touches them again.
    dsm_.setTileLoader(
        [this](const TileId &id) -> std::unique_ptr<DsmTile>
        {
            const std::string path = statePath("dsm", id);
            if (saved_.count(id) == 0 || !fs::exists(path))
            {
                return nullptr;
            }
            written_.erase(id);
            return DsmTile::load(path);
        });
    ortho_.setTileLoader(
        [this](const TileId &id) -> std::unique_ptr<OrthoTile>
        {
            const std::string path = statePath("ortho", id);
            if (saved_.count(id) == 0 || !fs::exists(path))
            {
                return nullptr;
            }
            written_.erase(id);
            return OrthoTile::load(path);
        });
}

std::string TileManager::statePath(const char *kind, const TileId &id) const
{
    char name[96];
    std::snprintf(name, sizeof(name), "%s_%lld_%lld.state", kind, static_cast<long long>(id.ix),
                  static_cast<long long>(id.iy));
    return (fs::path(config_.state_directory) / name).string();
}

void TileManager::markChanged(const std::vector<TileId> &tiles)
{
    changed_.insert(tiles.begin(), tiles.end());
}

TileState TileManager::state(const TileId &id) const
{
    if (dsm_.tile(id) != nullptr || ortho_.tile(id) != nullptr)
    {
        return TileState::active;
    }
    return written_.count(id) ? TileState::written : TileState::unknown;
}

std::size_t TileManager::activeTiles() const
{
    std::set<TileId> ids;
    for (const auto &[id, t] : dsm_.tiles())
    {
        ids.insert(id);
    }
    for (const auto &[id, t] : ortho_.tiles())
    {
        ids.insert(id);
    }
    return ids.size();
}

void TileManager::finalise(const TileId &id)
{
    if (std::unique_ptr<DsmTile> t = dsm_.release(id))
    {
        dsm_writer_.write(*t);
        t->save(statePath("dsm", id));
        dsm_written_ = true;
    }
    if (std::unique_ptr<OrthoTile> t = ortho_.release(id))
    {
        ortho_writer_.write(*t);
        t->save(statePath("ortho", id));
        ortho_written_ = true;
    }
    changed_.erase(id);
    written_.insert(id);
    saved_.insert(id);
}

std::vector<TileId> TileManager::update(const Eigen::Vector2d &vehicle)
{
    // Active tiles by distance of their centre from the vehicle.
    std::vector<std::pair<double, TileId>> by_distance;
    std::set<TileId>                       ids;
    for (const auto &[id, t] : dsm_.tiles())
    {
        ids.insert(id);
    }
    for (const auto &[id, t] : ortho_.tiles())
    {
        ids.insert(id);
    }
    for (const TileId &id : ids)
    {
        const Eigen::Vector2d centre((static_cast<double>(id.ix) + 0.5) * tile_size_,
                                     (static_cast<double>(id.iy) + 0.5) * tile_size_);
        by_distance.emplace_back((centre - vehicle).norm(), id);
    }
    std::sort(by_distance.begin(), by_distance.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });  // farthest first

    std::vector<TileId> finalised;
    std::size_t         active = by_distance.size();
    for (const auto &[distance, id] : by_distance)
    {
        if (distance <= config_.finalise_distance && active <= config_.max_active_tiles)
        {
            break;
        }
        finalise(id);
        finalised.push_back(id);
        --active;
    }
    if (!finalised.empty())
    {
        writeIndexes();
    }
    return finalised;
}

std::size_t TileManager::publish()
{
    std::size_t written = 0;
    for (const TileId &id : changed_)
    {
        if (const DsmTile *t = dsm_.tile(id))
        {
            dsm_writer_.write(*t);
            dsm_written_ = true;
            ++written;
        }
        if (const OrthoTile *t = ortho_.tile(id))
        {
            ortho_writer_.write(*t);
            ortho_written_ = true;
        }
    }
    changed_.clear();
    if (written > 0)
    {
        writeIndexes();
    }
    return written;
}

std::size_t TileManager::finaliseAll()
{
    std::set<TileId> ids;
    for (const auto &[id, t] : dsm_.tiles())
    {
        ids.insert(id);
    }
    for (const auto &[id, t] : ortho_.tiles())
    {
        ids.insert(id);
    }
    for (const TileId &id : ids)
    {
        finalise(id);
    }
    if (!ids.empty())
    {
        writeIndexes();
    }
    return ids.size();
}

void TileManager::writeIndexes()
{
    if (dsm_written_)
    {
        dsm_writer_.writeIndex();
    }
    if (ortho_written_)
    {
        ortho_writer_.writeIndex();
    }
}

}  // namespace orthostream::map_products
