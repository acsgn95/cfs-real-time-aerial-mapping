#include "vertical_datum.hpp"

#include <algorithm>
#include <cstdlib>

namespace orthostream::map_products::detail
{
namespace
{

int epsgCode(const std::string &crs)
{
    if (crs.rfind("EPSG:", 0) != 0)
    {
        throw MapProductsError("CRS must be given as EPSG:<code>: " + crs);
    }
    char      *end  = nullptr;
    const long code = std::strtol(crs.c_str() + 5, &end, 10);
    if (end == crs.c_str() + 5 || *end != '\0' || code <= 0 || code > 65535)
    {
        throw MapProductsError("invalid EPSG code: " + crs);
    }
    return static_cast<int>(code);
}

}  // namespace

OffsetGrid::OffsetGrid(double west, double north, double size)
    : west_(west), north_(north), step_(size / (kNodes - 1))
{
}

double OffsetGrid::at(double easting, double northing) const
{
    const double fi = std::clamp((easting - west_) / step_, 0.0, kNodes - 1.0);
    const double fj = std::clamp((north_ - northing) / step_, 0.0, kNodes - 1.0);
    const int    i  = std::min(static_cast<int>(fi), kNodes - 2);
    const int    j  = std::min(static_cast<int>(fj), kNodes - 2);
    const double ti = fi - i, tj = fj - j;
    auto         v  = [&](int a, int b) { return values_[static_cast<std::size_t>(b * kNodes + a)]; };
    return (1 - tj) * ((1 - ti) * v(i, j) + ti * v(i + 1, j)) + tj * ((1 - ti) * v(i, j + 1) + ti * v(i + 1, j + 1));
}

VerticalDatum::VerticalDatum(const ProductFrame &frame, const std::string &projected_crs,
                             const std::string &vertical_crs, const std::string &proj_data_dir,
                             const std::vector<std::string> &grid_dirs)
    : frame_(frame), crs_(projected_crs), projected_epsg_(epsgCode(projected_crs))
{
    if (!vertical_crs.empty())
    {
        vertical_epsg_ = epsgCode(vertical_crs);
        crs_           = projected_crs + "+" + std::to_string(vertical_epsg_);
        to_output_     = std::make_unique<geo::CrsTransformer>("EPSG:4978", crs_, proj_data_dir, grid_dirs);
    }
}

OffsetGrid VerticalDatum::offsets(double west, double north, double size, double height) const
{
    OffsetGrid grid(west, north, size);
    for (int j = 0; j < OffsetGrid::kNodes; ++j)
    {
        for (int i = 0; i < OffsetGrid::kNodes; ++i)
        {
            if (to_output_)
            {
                const Eigen::Vector3d p_E = frame_.toEcef({ west + i * grid.step(), north - j * grid.step(), height });
                grid.node(i, j)           = to_output_->forward(p_E).z() - height;
            }
            else
            {
                grid.node(i, j) = 0.0;
            }
        }
    }
    return grid;
}

}  // namespace orthostream::map_products::detail
