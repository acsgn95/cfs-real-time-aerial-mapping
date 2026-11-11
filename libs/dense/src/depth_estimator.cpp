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

}  // namespace orthostream::dense
