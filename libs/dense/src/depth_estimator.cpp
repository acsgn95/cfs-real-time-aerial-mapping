#include "orthostream/dense/depth_estimator.hpp"

#include <cmath>

namespace orthostream::dense
{

void validateBatch(const DepthBatch &batch, const DepthOutput &out)
{
    const geo::CameraModel &c = batch.camera;
    if (c.width <= 0 || c.height <= 0 || !(c.fx > 0.0) || !(c.fy > 0.0))
    {
        throw DenseError("invalid batch camera");
    }
    if (c.k1 != 0.0 || c.k2 != 0.0 || c.k3 != 0.0 || c.p1 != 0.0 || c.p2 != 0.0)
    {
        throw DenseError("batch camera must be undistorted (pinhole)");
    }
    if (batch.sources.empty())
    {
        throw DenseError("batch has no source views");
    }
    if (!(batch.depth_min > 0.0) || !(batch.depth_max > batch.depth_min))
    {
        throw DenseError("invalid depth range");
    }
    const std::size_t min_stride = static_cast<std::size_t>(c.width);
    auto              check_view = [&](const BatchView &v)
    {
        if (v.image.data == nullptr || v.image.stride < min_stride)
        {
            throw DenseError("invalid batch image");
        }
        if (!v.T_L_C.matrix().allFinite())
        {
            throw DenseError("invalid batch pose");
        }
    };
    check_view(batch.reference);
    for (const BatchView &v : batch.sources)
    {
        check_view(v);
    }
    if (out.depth == nullptr || out.normals == nullptr || out.confidence == nullptr)
    {
        throw DenseError("missing output buffer");
    }
}

std::size_t applyTerrainGate(const DepthBatch &batch, const HeightGrid &grid, double tolerance, const DepthOutput &out)
{
    if (grid.nx < 2 || grid.ny < 2 || !(grid.spacing > 0.0)
        || grid.heights.size() != static_cast<std::size_t>(grid.nx) * static_cast<std::size_t>(grid.ny)
        || !(tolerance > 0.0) || out.depth == nullptr || out.confidence == nullptr)
    {
        throw DenseError("invalid terrain gate input");
    }
    const geo::CameraModel  &c = batch.camera;
    const Eigen::Isometry3d &T = batch.reference.T_L_C;
    std::size_t              removed = 0;
    for (int v = 0; v < c.height; ++v)
    {
        for (int u = 0; u < c.width; ++u)
        {
            const std::size_t i = static_cast<std::size_t>(v) * c.width + u;
            const double      d = out.depth[i];
            if (!(d > 0.0))
            {
                continue;
            }
            const Eigen::Vector3d p  = T * Eigen::Vector3d(d * (u - c.cx) / c.fx, d * (v - c.cy) / c.fy, d);
            const double          fx = (p.x() - grid.x0) / grid.spacing, fy = (p.y() - grid.y0) / grid.spacing;
            bool                  keep = false;
            if (fx >= 0.0 && fy >= 0.0 && fx < grid.nx - 1 && fy < grid.ny - 1)
            {
                const int    a = static_cast<int>(fx), b = static_cast<int>(fy);
                const double tx = fx - a, ty = fy - b;
                auto         h  = [&](int x, int y)
                { return static_cast<double>(grid.heights[static_cast<std::size_t>(y) * grid.nx + x]); };
                const double z = (1 - tx) * (1 - ty) * h(a, b) + tx * (1 - ty) * h(a + 1, b)
                                 + (1 - tx) * ty * h(a, b + 1) + tx * ty * h(a + 1, b + 1);
                keep = std::isfinite(z) && std::abs(p.z() - z) <= tolerance;
            }
            if (!keep)
            {
                out.depth[i]      = 0.0f;
                out.confidence[i] = 0.0f;
                ++removed;
            }
        }
    }
    return removed;
}

}  // namespace orthostream::dense
