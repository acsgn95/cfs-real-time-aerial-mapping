// Orthomosaic rendering (ADR-007 section 3): every pixel takes its colour
// from the keyframe that sees its ground point closest to nadir; no
// blending, no occlusion test.

#ifndef ORTHOSTREAM_MAP_PRODUCTS_ORTHO_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_ORTHO_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <map>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "orthostream/geo/camera.hpp"
#include "orthostream/map_products/dsm.hpp"

namespace orthostream::map_products
{

struct OrthoConfig
{
    /// Target pixel size in metres. The tile of the DSM grid is divided into
    /// a whole number of pixels, so the pixel size used may differ slightly
    /// (OrthoTile::pixelSize).
    double pixel_size = 0.066;
    /// Radius in metres of the median filter applied to the DSM before it
    /// is used as the orthomosaic surface; 0 uses the DSM as it is. Noise
    /// and small, tall objects in the DSM otherwise make straight edges in
    /// the orthomosaic wavy. The DSM product is not changed.
    double surface_smoothing = 1.0;
    /// A rendered pixel takes the colour of a later keyframe only if that
    /// keyframe sees the point at least this many degrees closer to nadir.
    /// Avoids switching between keyframes of nearly the same angle, which
    /// splits tall objects into parts of different perspective.
    double switch_margin_deg = 2.0;
};

class OrthoTile
{
public:
    static constexpr std::uint16_t kEmpty = 0xFFFF;

    OrthoTile(const TileId &id, double tile_size, int pixels);

    const TileId &id() const { return id_; }
    int           pixels() const { return pixels_; }
    double        pixelSize() const { return pixel_size_; }
    double        originEasting() const { return origin_e_; }   ///< north-west corner
    double        originNorthing() const { return origin_n_; }

    std::size_t index(int col, int row) const { return static_cast<std::size_t>(row) * pixels_ + col; }
    /// Interleaved RGB, row-major, rows southwards.
    const std::vector<std::uint8_t> &rgb() const { return rgb_; }
    /// Viewing angle from nadir of the source pixel, in 0.01 degrees;
    /// kEmpty where nothing was rendered.
    std::uint16_t angle(int col, int row) const { return angle_[index(col, row)]; }

    /// Sets the pixel if the angle is smaller than the current one.
    bool set(int col, int row, const std::uint8_t rgb[3], std::uint16_t angle);
    std::size_t filledPixels() const;

    /// Complete pixel state (colours and viewing angles) in a binary file,
    /// for reloading an evicted tile. Not a map product.
    void                              save(const std::string &path) const;
    static std::unique_ptr<OrthoTile> load(const std::string &path);

private:
    OrthoTile() = default;

    TileId                     id_;
    int                        pixels_;
    double                     pixel_size_;
    double                     origin_e_;
    double                     origin_n_;
    std::vector<std::uint8_t>  rgb_;
    std::vector<std::uint16_t> angle_;
};

/// 8-bit RGB keyframe image owned by the caller.
struct RgbImageView
{
    const std::uint8_t *data   = nullptr;
    int                 width  = 0;
    int                 height = 0;
    std::size_t         stride = 0;  ///< bytes per row
};

struct RenderStats
{
    std::size_t         nodes   = 0;  ///< DSM nodes projected into the image
    std::size_t         pixels  = 0;  ///< pixels with a colour from this keyframe
    std::vector<TileId> tiles;        ///< tiles changed, sorted
};

/// Renders keyframes into orthomosaic tiles on the tile grid of the DSM.
/// Ground heights come from the DSM; the projection is evaluated at the DSM
/// cell corners and interpolated for the pixels in between.
class OrthoRenderer
{
public:
    OrthoRenderer(const OrthoConfig &config, const DsmFusion &dsm, const ProductFrame &frame);

    /// image: the keyframe at any scale; camera: its model at that scale,
    /// including distortion; T_E_C: camera to ECEF; extent: area of P to
    /// render (for example the extent of the keyframe's fused points).
    RenderStats render(const RgbImageView &image, const geo::CameraModel &camera, const Eigen::Isometry3d &T_E_C,
                       const Eigen::AlignedBox2d &extent);

    const std::map<TileId, std::unique_ptr<OrthoTile>> &tiles() const { return tiles_; }
    const OrthoTile                                    *tile(const TileId &id) const;
    int    tilePixels() const { return tile_pixels_; }

    using TileLoader = std::function<std::unique_ptr<OrthoTile>(const TileId &)>;
    void setTileLoader(TileLoader loader) { loader_ = std::move(loader); }
    std::unique_ptr<OrthoTile> release(const TileId &id);
    double pixelSize() const { return tile_size_ / tile_pixels_; }

private:
    OrthoTile &tileFor(const TileId &id);

    const DsmFusion                              &dsm_;
    const ProductFrame                           &frame_;
    double                                        tile_size_;
    int                                           tile_pixels_;
    double                                        smoothing_;
    int                                           margin_;  // 0.01 degrees
    std::map<TileId, std::unique_ptr<OrthoTile>>  tiles_;
    TileLoader                                    loader_;
};

}  // namespace orthostream::map_products

#endif  // ORTHOSTREAM_MAP_PRODUCTS_ORTHO_HPP
