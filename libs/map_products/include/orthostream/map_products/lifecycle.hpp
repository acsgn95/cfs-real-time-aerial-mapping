// Tile lifecycle and publication (ADR-007 sections 5 and 6).
//
//   active:  in memory, receives updates;
//   written: finalised, written to disk and evicted from memory.
//
// A tile is finalised when it is farther than a configured distance from the
// vehicle, or, when there are more active tiles than the budget, the tile
// farthest from the vehicle first. Finalising writes the map products and a
// state file, and evicts the tile. If a later keyframe touches a written
// tile, it is reloaded from its state file and becomes active again.
// Changed active tiles are written periodically (publication) so that the
// map can be viewed during the flight. Publication may run on another task:
// snapshot() copies the changed tiles on the task that owns the products,
// write() writes the copies elsewhere.

#ifndef ORTHOSTREAM_MAP_PRODUCTS_LIFECYCLE_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_LIFECYCLE_HPP

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "orthostream/map_products/dsm.hpp"
#include "orthostream/map_products/ortho.hpp"
#include "orthostream/map_products/writer.hpp"

namespace orthostream::map_products
{

struct TileLifecycleConfig
{
    std::size_t max_active_tiles  = 64;     ///< DSM tiles in memory (about 4 MB each, plus the orthomosaic)
    double      finalise_distance = 600.0;  ///< metres from the vehicle to the tile centre
    std::string state_directory;            ///< state files of evicted tiles
    /// Finalised tiles are evicted and their state saved at once, but their
    /// products are written with the next snapshot (publication on another
    /// task), so that the owner does not wait for the writing. Otherwise
    /// (default, offline) update() writes them itself.
    bool        defer_writes      = false;
};

enum class TileState
{
    unknown,  ///< never touched
    active,
    written,
};

/// Copies of changed active tiles, for writing on another task.
struct TileSnapshot
{
    struct Entry
    {
        TileId                     id;
        std::uint64_t              version = 0;  ///< changes of the tile when copied
        std::unique_ptr<DsmTile>   dsm;          ///< may be null
        std::unique_ptr<OrthoTile> ortho;        ///< may be null
    };
    std::vector<Entry> tiles;
};

/// Drives the lifecycle of the tiles of a DsmFusion and an OrthoRenderer.
/// It installs tile loaders on both, so it must outlive them in use. Not
/// thread-safe: all calls come from the task that owns the products, except
/// write(TileSnapshot), which one other task may call concurrently.
class TileManager
{
public:
    TileManager(const TileLifecycleConfig &config, DsmFusion &dsm, OrthoRenderer &ortho, DsmWriter &dsm_writer,
                OrthoWriter &ortho_writer);

    /// Records tiles changed by a keyframe (from FusionStats and RenderStats).
    void markChanged(const std::vector<TileId> &tiles);

    /// Finalises tiles by distance and budget; returns them.
    std::vector<TileId> update(const Eigen::Vector2d &vehicle);

    /// Writes the changed active tiles and the mosaic indexes; returns the
    /// number of tiles written. Equals write(snapshot()).
    std::size_t publish();

    /// Copies the changed active tiles and clears the changed set.
    TileSnapshot snapshot();

    /// Writes a snapshot and the mosaic indexes; returns the number of
    /// tiles written. A tile finalised meanwhile in the same or a newer
    /// version is not written again, so an older copy never replaces a
    /// newer file. Safe to call from one other task while the owner works,
    /// provided the writers share no PROJ objects with the products: the DSM
    /// writer converts heights through its ProductFrame, so it needs a frame
    /// of its own (PROJ objects are not thread-safe).
    std::size_t write(const TileSnapshot &snapshot);

    /// Finalises every active tile (end of the flight) and writes it, with
    /// the finalised tiles still waiting for a snapshot; returns the number
    /// of tiles finalised now.
    std::size_t finaliseAll();

    TileState   state(const TileId &id) const;
    std::size_t activeTiles() const;
    std::size_t changedTiles() const { return changed_.size(); }
    /// Finalised tiles whose products wait for the next snapshot.
    const std::vector<TileSnapshot::Entry> &deferredTiles() const { return deferred_; }

private:
    void        finalise(const TileId &id, bool defer);
    std::string statePath(const char *kind, const TileId &id) const;
    void        writeIndexes();

    TileLifecycleConfig config_;
    DsmFusion          &dsm_;
    OrthoRenderer      &ortho_;
    DsmWriter          &dsm_writer_;
    OrthoWriter        &ortho_writer_;
    double              tile_size_;
    std::set<TileId>    changed_;
    std::set<TileId>    written_;  // finalised and not reloaded
    std::set<TileId>    saved_;    // with a state file from this run
    std::map<TileId, std::uint64_t> version_;  // changes per tile (owner)
    std::vector<TileSnapshot::Entry> deferred_; // finalised, products not yet written (owner)

    // Product files: written by the owner (finalise) and by write().
    mutable std::mutex              disk_;
    std::map<TileId, std::uint64_t> on_disk_;  // version of the written files
    bool                            dsm_written_   = false;
    bool                            ortho_written_ = false;
};

}  // namespace orthostream::map_products

#endif  // ORTHOSTREAM_MAP_PRODUCTS_LIFECYCLE_HPP
