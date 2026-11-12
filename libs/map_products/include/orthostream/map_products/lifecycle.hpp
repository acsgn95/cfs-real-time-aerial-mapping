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
// map can be viewed during the flight.

#ifndef ORTHOSTREAM_MAP_PRODUCTS_LIFECYCLE_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_LIFECYCLE_HPP

#include <cstddef>
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
};

enum class TileState
{
    unknown,  ///< never touched
    active,
    written,
};

/// Drives the lifecycle of the tiles of a DsmFusion and an OrthoRenderer.
/// It installs tile loaders on both, so it must outlive them in use. Not
/// thread-safe: all calls come from the task that owns the products.
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
    /// number of tiles written.
    std::size_t publish();

    /// Finalises every active tile (end of the flight); returns their number.
    std::size_t finaliseAll();

    TileState   state(const TileId &id) const;
    std::size_t activeTiles() const;

private:
    void        finalise(const TileId &id);
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
    bool                dsm_written_   = false;
    bool                ortho_written_ = false;
};

}  // namespace orthostream::map_products

#endif  // ORTHOSTREAM_MAP_PRODUCTS_LIFECYCLE_HPP
