// Internal: conversion of depth map pixels to points of P, shared by DSM
// fusion and the point cloud.

#ifndef ORTHOSTREAM_MAP_PRODUCTS_DEPTH_POINTS_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_DEPTH_POINTS_HPP

#include <cstddef>

#include "orthostream/map_products/dsm.hpp"
#include "orthostream/map_products/product_frame.hpp"

namespace orthostream::map_products::detail
{

/// Calls fn(pixel index, point in the camera, point in P) for every pixel
/// with a depth and a confidence of at least min_confidence (and above 0).
/// ECEF to P is interpolated over the extent of the points. Returns the
/// number of points.
template <typename Fn>
std::size_t forEachDepthPoint(const DepthMapView &map, const geo::CameraModel &camera, const Eigen::Isometry3d &T_E_C,
                              const ProductFrame &frame, double min_confidence, Fn &&fn)
{
    if (map.depth == nullptr || map.confidence == nullptr || camera.width <= 0 || camera.height <= 0)
    {
        throw MapProductsError("invalid depth map");
    }
    const int    w = camera.width, h = camera.height;
    const double fx = camera.fx, fy = camera.fy, cx = camera.cx, cy = camera.cy;
    auto         valid = [&](std::size_t i)
    { return map.depth[i] > 0.0f && map.confidence[i] > 0.0f && map.confidence[i] >= min_confidence; };
    auto in_camera = [&](int u, int v, double d) { return Eigen::Vector3d(d * (u - cx) / fx, d * (v - cy) / fy, d); };

    Eigen::AlignedBox3d box;
    std::size_t         count = 0;
    for (int v = 0; v < h; ++v)
    {
        for (int u = 0; u < w; ++u)
        {
            const std::size_t i = static_cast<std::size_t>(v) * w + u;
            if (valid(i))
            {
                box.extend(T_E_C * in_camera(u, v, map.depth[i]));
                ++count;
            }
        }
    }
    if (count == 0)
    {
        return 0;
    }
    const InterpolatedTransform to_P([&frame](const Eigen::Vector3d &p) { return frame.fromEcef(p); }, box);
    for (int v = 0; v < h; ++v)
    {
        for (int u = 0; u < w; ++u)
        {
            const std::size_t i = static_cast<std::size_t>(v) * w + u;
            if (valid(i))
            {
                const Eigen::Vector3d p_C = in_camera(u, v, map.depth[i]);
                fn(i, p_C, to_P(T_E_C * p_C));
            }
        }
    }
    return count;
}

}  // namespace orthostream::map_products::detail

#endif  // ORTHOSTREAM_MAP_PRODUCTS_DEPTH_POINTS_HPP
