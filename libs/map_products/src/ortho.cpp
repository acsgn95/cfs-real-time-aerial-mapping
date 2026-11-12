#include "orthostream/map_products/ortho.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace orthostream::map_products
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Projection of one DSM cell corner into the keyframe.
struct Node
{
    float u = 0.0f, v = 0.0f, angle = 0.0f;
    bool  valid = false;
};

}  // namespace

OrthoTile::OrthoTile(const TileId &id, double tile_size, int pixels)
    : id_(id), pixels_(pixels), pixel_size_(tile_size / pixels)
{
    origin_e_ = static_cast<double>(id.ix) * tile_size;
    origin_n_ = static_cast<double>(id.iy + 1) * tile_size;
    const std::size_t n = static_cast<std::size_t>(pixels) * pixels;
    rgb_.assign(3 * n, 0);
    angle_.assign(n, kEmpty);
}

bool OrthoTile::set(int col, int row, const std::uint8_t rgb[3], std::uint16_t angle)
{
    const std::size_t i = index(col, row);
    if (angle >= angle_[i])
    {
        return false;
    }
    angle_[i]       = angle;
    rgb_[3 * i]     = rgb[0];
    rgb_[3 * i + 1] = rgb[1];
    rgb_[3 * i + 2] = rgb[2];
    return true;
}

std::size_t OrthoTile::filledPixels() const
{
    return static_cast<std::size_t>(std::count_if(angle_.begin(), angle_.end(), [](std::uint16_t a) { return a != kEmpty; }));
}

OrthoRenderer::OrthoRenderer(const OrthoConfig &config, const DsmFusion &dsm, const ProductFrame &frame)
    : dsm_(dsm), frame_(frame)
{
    const TileGridConfig &g = dsm.config().grid;
    tile_size_              = g.cell_size * g.tile_cells;
    if (!(config.pixel_size > 0.0))
    {
        throw MapProductsError("invalid orthomosaic pixel size");
    }
    tile_pixels_ = std::max(1, static_cast<int>(std::lround(tile_size_ / config.pixel_size)));
}

OrthoTile &OrthoRenderer::tileFor(const TileId &id)
{
    auto it = tiles_.find(id);
    if (it == tiles_.end())
    {
        std::unique_ptr<OrthoTile> t = loader_ ? loader_(id) : nullptr;
        if (!t)
        {
            t = std::make_unique<OrthoTile>(id, tile_size_, tile_pixels_);
        }
        it = tiles_.emplace(id, std::move(t)).first;
    }
    return *it->second;
}

std::unique_ptr<OrthoTile> OrthoRenderer::release(const TileId &id)
{
    const auto it = tiles_.find(id);
    if (it == tiles_.end())
    {
        return nullptr;
    }
    std::unique_ptr<OrthoTile> t = std::move(it->second);
    tiles_.erase(it);
    return t;
}

const OrthoTile *OrthoRenderer::tile(const TileId &id) const
{
    const auto it = tiles_.find(id);
    return it == tiles_.end() ? nullptr : it->second.get();
}

RenderStats OrthoRenderer::render(const RgbImageView &image, const geo::CameraModel &camera,
                                  const Eigen::Isometry3d &T_E_C, const Eigen::AlignedBox2d &extent)
{
    RenderStats stats;
    if (image.data == nullptr || image.width != camera.width || image.height != camera.height
        || image.stride < static_cast<std::size_t>(3 * image.width))
    {
        throw MapProductsError("invalid keyframe image");
    }
    if (extent.isEmpty())
    {
        return stats;
    }

    // Lattice of DSM cell corners covering the extent: node (a, b) lies at
    // easting a * cs and northing b * cs.
    const double       cs = dsm_.config().grid.cell_size;
    const double       a0 = std::floor(extent.min().x() / cs);  // whole numbers
    const double       b0 = std::floor(extent.min().y() / cs);
    const int          na = static_cast<int>(std::ceil(extent.max().x() / cs) - a0) + 1;
    const int          nb = static_cast<int>(std::ceil(extent.max().y() / cs) - b0) + 1;
    // A keyframe covers a few hundred metres; a larger area indicates
    // outliers in the extent and would exhaust memory.
    if (static_cast<double>(na) * nb > 64.0e6)
    {
        throw MapProductsError("render extent too large");
    }

    // Node heights: mean of the DSM cells around the corner.
    std::vector<double> height(static_cast<std::size_t>(na) * nb, std::nan(""));
    double              h_lo = 1e300, h_hi = -1e300;
    for (int b = 0; b < nb; ++b)
    {
        for (int a = 0; a < na; ++a)
        {
            const double e = (a0 + a) * cs, n = (b0 + b) * cs;
            double       sum = 0.0;
            int          count = 0;
            for (const auto &[de, dn] : { std::pair { -0.5, -0.5 }, { 0.5, -0.5 }, { -0.5, 0.5 }, { 0.5, 0.5 } })
            {
                if (const std::optional<double> h = dsm_.heightAt(e + de * cs, n + dn * cs))
                {
                    sum += *h;
                    ++count;
                }
            }
            if (count > 0)
            {
                const double h                               = sum / count;
                height[static_cast<std::size_t>(b) * na + a] = h;
                h_lo                                         = std::min(h_lo, h);
                h_hi                                         = std::max(h_hi, h);
            }
        }
    }
    if (h_lo > h_hi)
    {
        return stats;  // no DSM in the extent
    }

    // P to ECEF, interpolated over the extent and its height range.
    const Eigen::AlignedBox3d box(Eigen::Vector3d(a0 * cs, b0 * cs, h_lo),
                                  Eigen::Vector3d((a0 + na - 1) * cs, (b0 + nb - 1) * cs, h_hi));
    const InterpolatedTransform to_E([this](const Eigen::Vector3d &p) { return frame_.toEcef(p); }, box);
    const Eigen::Vector3d       mid = box.center();
    const Eigen::Vector3d       up  = (to_E(mid + Eigen::Vector3d(0, 0, 1)) - to_E(mid)).normalized();

    // Points far outside the field of view are not projected: the
    // distortion polynomial is meaningless there.
    double r2_max = 0.0;
    for (const Eigen::Vector2d &px : { Eigen::Vector2d(-0.5, -0.5), Eigen::Vector2d(camera.width - 0.5, -0.5),
                                       Eigen::Vector2d(-0.5, camera.height - 0.5),
                                       Eigen::Vector2d(camera.width - 0.5, camera.height - 0.5) })
    {
        if (const auto nrm = geo::undistortPixel(camera, px))
        {
            r2_max = std::max(r2_max, nrm->squaredNorm());
        }
    }
    r2_max *= 1.2;

    const Eigen::Isometry3d T_C_E = T_E_C.inverse();
    const Eigen::Vector3d   C     = T_E_C.translation();
    std::vector<Node>       nodes(height.size());
    for (int b = 0; b < nb; ++b)
    {
        for (int a = 0; a < na; ++a)
        {
            const std::size_t k = static_cast<std::size_t>(b) * na + a;
            if (std::isnan(height[k]))
            {
                continue;
            }
            const Eigen::Vector3d p_E = to_E({ (a0 + a) * cs, (b0 + b) * cs, height[k] });
            const Eigen::Vector3d p_C = T_C_E * p_E;
            if (p_C.z() <= 0.0 || (p_C.head<2>() / p_C.z()).squaredNorm() > r2_max)
            {
                continue;
            }
            const std::optional<Eigen::Vector2d> uv = geo::projectPoint(camera, p_C);
            if (!uv)
            {
                continue;
            }
            const double cos_angle = std::clamp((C - p_E).normalized().dot(up), -1.0, 1.0);
            nodes[k]               = { static_cast<float>(uv->x()), static_cast<float>(uv->y()),
                                       static_cast<float>(std::acos(cos_angle) * 180.0 / kPi), true };
            ++stats.nodes;
        }
    }

    // Pixels: interpolate the projection of the four surrounding nodes and
    // sample the image bilinearly.
    const double         ps      = tile_size_ / tile_pixels_;
    const std::int64_t   tx0     = static_cast<std::int64_t>(std::floor(extent.min().x() / tile_size_));
    const std::int64_t   tx1     = static_cast<std::int64_t>(std::floor(extent.max().x() / tile_size_));
    const std::int64_t   ty0     = static_cast<std::int64_t>(std::floor(extent.min().y() / tile_size_));
    const std::int64_t   ty1     = static_cast<std::int64_t>(std::floor(extent.max().y() / tile_size_));
    const double         u_max   = image.width - 1.0, v_max = image.height - 1.0;
    std::set<TileId>     changed;
    for (std::int64_t ty = ty0; ty <= ty1; ++ty)
    {
        for (std::int64_t tx = tx0; tx <= tx1; ++tx)
        {
            const TileId id { tx, ty };
            OrthoTile   *tile = nullptr;
            const double e0 = static_cast<double>(tx) * tile_size_, n0 = static_cast<double>(ty + 1) * tile_size_;
            const int    c0 = std::max(0, static_cast<int>(std::floor((extent.min().x() - e0) / ps)));
            const int    c1 = std::min(tile_pixels_ - 1, static_cast<int>(std::ceil((extent.max().x() - e0) / ps)));
            const int    r0 = std::max(0, static_cast<int>(std::floor((n0 - extent.max().y()) / ps)));
            const int    r1 = std::min(tile_pixels_ - 1, static_cast<int>(std::ceil((n0 - extent.min().y()) / ps)));
            for (int r = r0; r <= r1; ++r)
            {
                const double fb = (n0 - (r + 0.5) * ps) / cs - b0;
                const int    ib = std::clamp(static_cast<int>(std::floor(fb)), 0, nb - 2);
                const double tb = fb - ib;
                if (tb < 0.0 || tb > 1.0)
                {
                    continue;
                }
                for (int c = c0; c <= c1; ++c)
                {
                    const double fa = (e0 + (c + 0.5) * ps) / cs - a0;
                    const int    ia = std::clamp(static_cast<int>(std::floor(fa)), 0, na - 2);
                    const double ta = fa - ia;
                    if (ta < 0.0 || ta > 1.0)
                    {
                        continue;
                    }
                    const std::size_t k  = static_cast<std::size_t>(ib) * na + ia;
                    const Node       &q00 = nodes[k], &q10 = nodes[k + 1], &q01 = nodes[k + na], &q11 = nodes[k + na + 1];
                    if (!(q00.valid && q10.valid && q01.valid && q11.valid))
                    {
                        continue;
                    }
                    const double w00 = (1 - ta) * (1 - tb), w10 = ta * (1 - tb), w01 = (1 - ta) * tb, w11 = ta * tb;
                    const double u   = w00 * q00.u + w10 * q10.u + w01 * q01.u + w11 * q11.u;
                    const double v   = w00 * q00.v + w10 * q10.v + w01 * q01.v + w11 * q11.v;
                    if (!(u >= 0.0 && v >= 0.0 && u <= u_max && v <= v_max))
                    {
                        continue;
                    }
                    const double angle = w00 * q00.angle + w10 * q10.angle + w01 * q01.angle + w11 * q11.angle;
                    const auto   a16   = static_cast<std::uint16_t>(std::min(65534.0, std::round(angle * 100.0)));
                    if (tile == nullptr)
                    {
                        tile = &tileFor(id);
                    }
                    if (a16 >= tile->angle(c, r))
                    {
                        continue;
                    }
                    // Bilinear sample; pixel centres at integer coordinates.
                    const int     x0 = std::min(static_cast<int>(u), image.width - 2 < 0 ? 0 : image.width - 2);
                    const int     y0 = std::min(static_cast<int>(v), image.height - 2 < 0 ? 0 : image.height - 2);
                    const int     x1 = std::min(x0 + 1, image.width - 1), y1 = std::min(y0 + 1, image.height - 1);
                    const double  fx = u - x0, fy = v - y0;
                    const auto   *p00 = image.data + static_cast<std::size_t>(y0) * image.stride + 3 * x0;
                    const auto   *p10 = image.data + static_cast<std::size_t>(y0) * image.stride + 3 * x1;
                    const auto   *p01 = image.data + static_cast<std::size_t>(y1) * image.stride + 3 * x0;
                    const auto   *p11 = image.data + static_cast<std::size_t>(y1) * image.stride + 3 * x1;
                    std::uint8_t  rgb[3];
                    for (int ch = 0; ch < 3; ++ch)
                    {
                        const double val = (1 - fx) * (1 - fy) * p00[ch] + fx * (1 - fy) * p10[ch]
                                           + (1 - fx) * fy * p01[ch] + fx * fy * p11[ch];
                        rgb[ch] = static_cast<std::uint8_t>(std::lround(std::clamp(val, 0.0, 255.0)));
                    }
                    tile->set(c, r, rgb, a16);
                    ++stats.pixels;
                    changed.insert(id);
                }
            }
        }
    }
    stats.tiles.assign(changed.begin(), changed.end());
    return stats;
}

}  // namespace orthostream::map_products
