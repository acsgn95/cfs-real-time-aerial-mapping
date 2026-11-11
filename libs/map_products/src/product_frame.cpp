#include "orthostream/map_products/product_frame.hpp"

#include <algorithm>
#include <cmath>

namespace orthostream::map_products
{

ProductFrame::ProductFrame(const std::string &projected_crs, const std::string &proj_data_dir,
                           const std::vector<std::string> &grid_dirs)
    : to_projected_("EPSG:4978", projected_crs, proj_data_dir, grid_dirs), geodetic_(proj_data_dir)
{
}

Eigen::Vector3d ProductFrame::fromEcef(const Eigen::Vector3d &p_E) const
{
    // The full 3D position goes into PROJ: datum shifts to the projection's
    // datum depend slightly on the height.
    const Eigen::Vector3d en = to_projected_.forward(p_E);
    return { en.x(), en.y(), geodetic_.ecefToGeodetic(p_E).height_m };
}

Eigen::Vector3d ProductFrame::toEcef(const Eigen::Vector3d &p_P) const
{
    // PROJ's height on the projected side is not the WGS 84 ellipsoidal
    // height; a few fixed-point steps on the input remove the difference.
    Eigen::Vector3d input = p_P;
    Eigen::Vector3d p_E   = to_projected_.inverse(input);
    for (int i = 0; i < 5; ++i)
    {
        const Eigen::Vector3d residual = p_P - fromEcef(p_E);
        if (residual.cwiseAbs().maxCoeff() < 1e-7)
        {
            break;
        }
        input += residual;
        p_E = to_projected_.inverse(input);
    }
    return p_E;
}

InterpolatedTransform::InterpolatedTransform(const Function &exact, const Eigen::AlignedBox3d &box, int nodes)
    : nodes_(nodes)
{
    if (nodes < 2 || box.isEmpty())
    {
        throw MapProductsError("invalid interpolation grid");
    }
    Eigen::Vector3d lo = box.min(), hi = box.max();
    for (int a = 0; a < 3; ++a)
    {
        if (hi[a] - lo[a] < 1.0)
        {
            const double c = 0.5 * (lo[a] + hi[a]);
            lo[a]          = c - 0.5;
            hi[a]          = c + 0.5;
        }
    }
    origin_ = lo;
    step_   = (hi - lo) / static_cast<double>(nodes - 1);
    values_.resize(static_cast<std::size_t>(nodes) * nodes * nodes);
    for (int k = 0; k < nodes; ++k)
    {
        for (int j = 0; j < nodes; ++j)
        {
            for (int i = 0; i < nodes; ++i)
            {
                const Eigen::Vector3d p = origin_ + Eigen::Vector3d(i, j, k).cwiseProduct(step_);
                values_[(static_cast<std::size_t>(k) * nodes + j) * nodes + i] = exact(p);
            }
        }
    }
}

Eigen::Vector3d InterpolatedTransform::operator()(const Eigen::Vector3d &p) const
{
    const Eigen::Vector3d f = (p - origin_).cwiseQuotient(step_);
    int                   idx[3];
    double                t[3];
    for (int a = 0; a < 3; ++a)
    {
        idx[a] = std::clamp(static_cast<int>(std::floor(f[a])), 0, nodes_ - 2);
        t[a]   = f[a] - idx[a];  // outside [0, 1] when extrapolating
    }
    auto at = [&](int i, int j, int k) -> const Eigen::Vector3d &
    {
        return values_[(static_cast<std::size_t>(idx[2] + k) * nodes_ + (idx[1] + j)) * nodes_ + (idx[0] + i)];
    };
    const Eigen::Vector3d c00 = (1 - t[0]) * at(0, 0, 0) + t[0] * at(1, 0, 0);
    const Eigen::Vector3d c10 = (1 - t[0]) * at(0, 1, 0) + t[0] * at(1, 1, 0);
    const Eigen::Vector3d c01 = (1 - t[0]) * at(0, 0, 1) + t[0] * at(1, 0, 1);
    const Eigen::Vector3d c11 = (1 - t[0]) * at(0, 1, 1) + t[0] * at(1, 1, 1);
    const Eigen::Vector3d c0  = (1 - t[1]) * c00 + t[1] * c10;
    const Eigen::Vector3d c1  = (1 - t[1]) * c01 + t[1] * c11;
    return (1 - t[2]) * c0 + t[2] * c1;
}

}  // namespace orthostream::map_products
