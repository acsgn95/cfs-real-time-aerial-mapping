// DSM fusion into tiles (ADR-007 sections 1 and 2).

#ifndef ORTHOSTREAM_MAP_PRODUCTS_DSM_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_DSM_HPP

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

#include "orthostream/geo/camera.hpp"
#include "orthostream/map_products/product_frame.hpp"

namespace orthostream::map_products
{

struct TileGridConfig
{
    double cell_size  = 0.26;  ///< metres; DSM resolution in P
    int    tile_cells = 512;   ///< cells per tile side
};

/// Tile index on the regular grid of P. Tile (ix, iy) covers easting
/// [ix, ix + 1) * T and northing [iy, iy + 1) * T with T = cell size x cells.
struct TileId
{
    std::int64_t ix = 0;
    std::int64_t iy = 0;
    bool operator<(const TileId &o) const { return ix < o.ix || (ix == o.ix && iy < o.iy); }
    bool operator==(const TileId &o) const { return ix == o.ix && iy == o.iy; }
};

struct DsmFusionConfig
{
    TileGridConfig grid;
    int    min_observations = 3;     ///< observations before outliers are rejected
    double rejection_sigmas = 3.0;   ///< k: reject |h - mean| > k * max(sigma, min_sigma)
    double min_sigma        = 0.25;  ///< metres; floor of sigma for the rejection test
    double min_confidence   = 0.0;   ///< depth pixels below it are ignored
};

/// Cell statistics of one tile. Heights are ellipsoidal and stored relative
/// to the tile's reference height. Mean and spread are updated with the
/// weighted form of Welford's method, which keeps single precision accurate;
/// it is equivalent to keeping the weighted sums of heights and squared
/// heights (ADR-007).
class DsmTile
{
public:
    DsmTile(const TileId &id, const TileGridConfig &grid, double reference_height);

    const TileId &id() const { return id_; }
    int           cells() const { return cells_; }
    double        cellSize() const { return cell_size_; }
    double        referenceHeight() const { return reference_height_; }
    /// Easting and northing of the outer corner of cell (0, 0), which is the
    /// north-west corner: rows run southwards as in a GeoTIFF.
    double originEasting() const { return origin_e_; }
    double originNorthing() const { return origin_n_; }

    enum class Update
    {
        added,
        rejected
    };
    /// Adds an observation to cell (col, row).
    Update add(int col, int row, double height, float weight, const DsmFusionConfig &config);

    std::size_t                index(int col, int row) const { return static_cast<std::size_t>(row) * cells_ + col; }
    std::optional<double>      height(int col, int row) const;    ///< weighted mean, ellipsoidal
    std::optional<double>      variance(int col, int row) const;  ///< weighted variance
    std::uint16_t              observations(int col, int row) const { return count_[index(col, row)]; }
    float                      weight(int col, int row) const { return weight_[index(col, row)]; }
    std::size_t                filledCells() const;

private:
    TileId                     id_;
    int                        cells_;
    double                     cell_size_;
    double                     reference_height_;
    double                     origin_e_;
    double                     origin_n_;
    std::vector<float>         weight_;  // sum of weights
    std::vector<float>         mean_;    // relative to reference_height_
    std::vector<float>         m2_;      // sum of w * (h - mean_old) * (h - mean_new)
    std::vector<std::uint16_t> count_;
};

/// Depth map of a keyframe at the dense working resolution: z in the camera,
/// 0 where invalid; confidence in [0, 1] (ADR-005). Row-major, camera size.
struct DepthMapView
{
    const float *depth      = nullptr;
    const float *confidence = nullptr;
};

struct FusionStats
{
    std::size_t          points   = 0;  ///< valid depth pixels
    std::size_t          added    = 0;
    std::size_t          rejected = 0;  ///< outliers
    std::vector<TileId>  tiles;         ///< tiles touched, sorted
    Eigen::AlignedBox2d  extent;        ///< easting and northing of the fused points
};

/// Fuses depth maps into DSM tiles of P. Tiles are created on first use;
/// their lifecycle (finalising, writing, eviction) follows in a later step.
class DsmFusion
{
public:
    DsmFusion(const DsmFusionConfig &config, const ProductFrame &frame);

    /// camera: pinhole camera of the depth map; T_E_C: camera to ECEF.
    FusionStats addDepthMap(const DepthMapView &map, const geo::CameraModel &camera, const Eigen::Isometry3d &T_E_C);

    const std::map<TileId, std::unique_ptr<DsmTile>> &tiles() const { return tiles_; }
    const DsmTile                                    *tile(const TileId &id) const;
    const DsmFusionConfig                            &config() const { return config_; }

    /// Ellipsoidal height of the DSM at a point of P, nothing where empty.
    std::optional<double> heightAt(double easting, double northing) const;

private:
    DsmTile &tileFor(const TileId &id, double reference_height);

    DsmFusionConfig                            config_;
    const ProductFrame                        &frame_;
    std::map<TileId, std::unique_ptr<DsmTile>> tiles_;
};

}  // namespace orthostream::map_products

#endif  // ORTHOSTREAM_MAP_PRODUCTS_DSM_HPP
