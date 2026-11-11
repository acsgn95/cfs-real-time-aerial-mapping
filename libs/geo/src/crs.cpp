#include "orthostream/geo/crs.hpp"

#include <cmath>
#include <filesystem>
#include <vector>

#include <proj.h>

namespace orthostream::geo
{

namespace
{

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

std::string contextError(PJ_CONTEXT *ctx)
{
    const char *text = proj_context_errno_string(ctx, proj_context_errno(ctx));
    return text != nullptr ? text : "unknown PROJ error";
}

}  // namespace

struct CrsTransformer::Impl
{
    PJ_CONTEXT *ctx = nullptr;
    PJ         *pj  = nullptr;

    ~Impl()
    {
        if (pj != nullptr)
        {
            proj_destroy(pj);
        }
        if (ctx != nullptr)
        {
            proj_context_destroy(ctx);
        }
    }

    Eigen::Vector3d transform(PJ_DIRECTION direction, const Eigen::Vector3d &p) const
    {
        proj_errno_reset(pj);
        const PJ_COORD out = proj_trans(pj, direction, proj_coord(p.x(), p.y(), p.z(), 0.0));
        const int      err = proj_errno(pj);
        if (err != 0 || !std::isfinite(out.xyz.x) || !std::isfinite(out.xyz.y) || !std::isfinite(out.xyz.z))
        {
            const char *text = proj_context_errno_string(ctx, err);
            throw GeoError(std::string("PROJ transformation failed: ") + (text != nullptr ? text : "non-finite result"));
        }
        return { out.xyz.x, out.xyz.y, out.xyz.z };
    }
};

CrsTransformer::CrsTransformer(const std::string              &source,
                               const std::string              &target,
                               const std::string              &proj_data_dir,
                               const std::vector<std::string> &grid_dirs) :
    impl_(std::make_unique<Impl>())
{
    // PROJ falls back to the data directory it was built with when the given
    // one lacks proj.db. That would make results depend on the build
    // machine, so the given directory must hold proj.db.
    std::error_code ec;
    if (!std::filesystem::is_regular_file(std::filesystem::path(proj_data_dir) / "proj.db", ec))
    {
        throw GeoError("no proj.db in PROJ data directory " + proj_data_dir);
    }

    impl_->ctx = proj_context_create();
    if (impl_->ctx == nullptr)
    {
        throw GeoError("cannot create a PROJ context");
    }
    proj_context_set_enable_network(impl_->ctx, 0);
    std::vector<const char *> paths { proj_data_dir.c_str() };
    for (const std::string &dir : grid_dirs)
    {
        paths.push_back(dir.c_str());
    }
    proj_context_set_search_paths(impl_->ctx, static_cast<int>(paths.size()), paths.data());

    PJ *src = proj_create(impl_->ctx, source.c_str());
    PJ *dst = proj_create(impl_->ctx, target.c_str());
    PJ *raw = nullptr;
    if (src != nullptr && dst != nullptr)
    {
        const char *options[] = { "ONLY_BEST=YES", nullptr };
        raw                   = proj_create_crs_to_crs_from_pj(impl_->ctx, src, dst, nullptr, options);
    }
    if (src != nullptr)
    {
        proj_destroy(src);
    }
    if (dst != nullptr)
    {
        proj_destroy(dst);
    }
    if (raw == nullptr)
    {
        throw GeoError("cannot create transformation " + source + " -> " + target + ": " + contextError(impl_->ctx));
    }
    impl_->pj = proj_normalize_for_visualization(impl_->ctx, raw);
    proj_destroy(raw);
    if (impl_->pj == nullptr)
    {
        throw GeoError("cannot normalise transformation " + source + " -> " + target + ": "
                       + contextError(impl_->ctx));
    }
}

CrsTransformer::~CrsTransformer()                                     = default;
CrsTransformer::CrsTransformer(CrsTransformer &&) noexcept            = default;
CrsTransformer &CrsTransformer::operator=(CrsTransformer &&) noexcept = default;

Eigen::Vector3d CrsTransformer::forward(const Eigen::Vector3d &p) const
{
    return impl_->transform(PJ_FWD, p);
}

Eigen::Vector3d CrsTransformer::inverse(const Eigen::Vector3d &p) const
{
    return impl_->transform(PJ_INV, p);
}

GeodeticConverter::GeodeticConverter(const std::string &proj_data_dir) :
    transformer_("EPSG:4978", "EPSG:4979", proj_data_dir)
{
}

Geodetic GeodeticConverter::ecefToGeodetic(const Eigen::Vector3d &p_E) const
{
    const Eigen::Vector3d lon_lat_h = transformer_.forward(p_E);
    return { lon_lat_h.y() * kDegToRad, lon_lat_h.x() * kDegToRad, lon_lat_h.z() };
}

Eigen::Vector3d GeodeticConverter::geodeticToEcef(const Geodetic &g) const
{
    return transformer_.inverse({ g.longitude_rad * kRadToDeg, g.latitude_rad * kRadToDeg, g.height_m });
}

}  // namespace orthostream::geo
