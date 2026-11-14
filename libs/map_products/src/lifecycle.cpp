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
    for (const TileId &id : tiles)
    {
        ++version_[id];
    }
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

void TileManager::finalise(const TileId &id, bool defer)
{
    const std::uint64_t        version = version_[id];
    std::unique_ptr<DsmTile>   dsm     = dsm_.release(id);
    std::unique_ptr<OrthoTile> ortho   = ortho_.release(id);
    if (dsm)
    {
        dsm->save(statePath("dsm", id));
    }
    if (ortho)
    {
        ortho->save(statePath("ortho", id));
    }
    changed_.erase(id);
    written_.insert(id);
    saved_.insert(id);
    if (defer)
    {
        TileSnapshot::Entry e;
        e.id      = id;
        e.version = version;
        e.dsm     = std::move(dsm);
        e.ortho   = std::move(ortho);
        deferred_.push_back(std::move(e));
        return;
    }
    std::lock_guard<std::mutex> lock(disk_);
    auto                        it      = on_disk_.find(id);
    const bool                  current = it != on_disk_.end() && it->second >= version;
    if (dsm && !current)
    {
        dsm_writer_.write(*dsm);
        dsm_written_ = true;
    }
    if (ortho && !current)
    {
        ortho_writer_.write(*ortho);
        ortho_written_ = true;
    }
    on_disk_[id] = version;
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
        finalise(id, config_.defer_writes);
        finalised.push_back(id);
        --active;
    }
    if (!finalised.empty() && !config_.defer_writes)
    {
        writeIndexes();
    }
    return finalised;
}

std::size_t TileManager::publish()
{
    return write(snapshot());
}

TileSnapshot TileManager::snapshot()
{
    // Finalised tiles first: a tile reloaded and changed since comes after
    // them in a newer version.
    TileSnapshot s;
    s.tiles = std::move(deferred_);
    deferred_.clear();
    for (const TileId &id : changed_)
    {
        TileSnapshot::Entry e;
        e.id      = id;
        e.version = version_[id];
        if (const DsmTile *t = dsm_.tile(id))
        {
            e.dsm = std::make_unique<DsmTile>(*t);
        }
        if (const OrthoTile *t = ortho_.tile(id))
        {
            e.ortho = std::make_unique<OrthoTile>(*t);
        }
        if (e.dsm || e.ortho)
        {
            s.tiles.push_back(std::move(e));
        }
    }
    changed_.clear();
    return s;
}

std::size_t TileManager::write(const TileSnapshot &snapshot)
{
    std::size_t written = 0;
    for (const TileSnapshot::Entry &e : snapshot.tiles)
    {
        // One tile at a time: the owner waits at most one tile write when it
        // finalises.
        std::lock_guard<std::mutex> lock(disk_);
        auto                        it = on_disk_.find(e.id);
        if (it != on_disk_.end() && it->second >= e.version)
        {
            continue;  // finalised meanwhile, at least as new
        }
        if (e.dsm)
        {
            dsm_writer_.write(*e.dsm);
            dsm_written_ = true;
        }
        if (e.ortho)
        {
            ortho_writer_.write(*e.ortho);
            ortho_written_ = true;
        }
        on_disk_[e.id] = e.version;
        ++written;
    }
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
        finalise(id, true);
    }
    // The deferred tiles of earlier finalising as well; changed_ is empty.
    write(snapshot());
    writeIndexes();
    return ids.size();
}

void TileManager::writeIndexes()
{
    std::lock_guard<std::mutex> lock(disk_);
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
