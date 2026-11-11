#include "orthostream/selection/footprint.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace orthostream::selection
{
namespace
{

using Polygon = std::vector<Eigen::Vector2d>;

double cross2(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return a.x() * b.y() - a.y() * b.x();
}

double signedArea(const Polygon &p)
{
    double a = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i)
    {
        a += cross2(p[i], p[(i + 1) % p.size()]);
    }
    return 0.5 * a;
}

Polygon horizontal(const Footprint &f)
{
    Polygon p;
    for (const Eigen::Vector3d &c : f.corners)
    {
        p.emplace_back(c.x(), c.y());
    }
    return p;
}

// Sutherland–Hodgman: clips a polygon by a convex, counter-clockwise one.
Polygon clip(const Polygon &subject, const Polygon &convex)
{
    Polygon out = subject;
    for (std::size_t e = 0; e < convex.size() && !out.empty(); ++e)
    {
        const Eigen::Vector2d a      = convex[e];
        const Eigen::Vector2d b      = convex[(e + 1) % convex.size()];
        auto                  inside = [&](const Eigen::Vector2d &p) { return cross2(b - a, p - a) >= 0.0; };
        const Polygon         in     = out;
        out.clear();
        for (std::size_t i = 0; i < in.size(); ++i)
        {
            const Eigen::Vector2d p = in[i];
            const Eigen::Vector2d q = in[(i + 1) % in.size()];
            const bool            pi = inside(p), qi = inside(q);
            if (pi)
            {
                out.push_back(p);
            }
            if (pi != qi)
            {
                const double t = cross2(b - a, a - p) / cross2(b - a, q - p);
                out.push_back(p + t * (q - p));
            }
        }
    }
    return out;
}

}  // namespace

std::optional<Footprint> computeFootprint(const geo::CameraModel  &camera,
                                          const Eigen::Isometry3d &T_L_C,
                                          const TerrainHeight     &terrain,
                                          const FootprintConfig   &config)
{
    if (camera.width <= 0 || camera.height <= 0 || config.iterations < 0 || !terrain)
    {
        throw SelectionError("invalid footprint input");
    }
    // Outer pixel edges in the OpenCV convention, then the principal point.
    const double                   w = camera.width, h = camera.height;
    const std::array<Eigen::Vector2d, 5> pixels = { Eigen::Vector2d(-0.5, -0.5), Eigen::Vector2d(w - 0.5, -0.5),
                                                    Eigen::Vector2d(w - 0.5, h - 0.5), Eigen::Vector2d(-0.5, h - 0.5),
                                                    Eigen::Vector2d(camera.cx, camera.cy) };
    std::array<Eigen::Vector3d, 5> rays;
    for (std::size_t i = 0; i < pixels.size(); ++i)
    {
        const std::optional<Eigen::Vector2d> n = geo::undistortPixel(camera, pixels[i]);
        if (!n)
        {
            return std::nullopt;
        }
        rays[i] = T_L_C.linear() * Eigen::Vector3d(n->x(), n->y(), 1.0);
        if (!(rays[i].z() < 0.0))
        {
            return std::nullopt;
        }
    }
    const Eigen::Vector3d C = T_L_C.translation();

    std::array<Eigen::Vector3d, 5> ground;
    std::array<double, 5>          heights {};
    auto intersect_and_sample = [&](double plane) -> bool
    {
        if (!(plane < C.z()))
        {
            return false;
        }
        for (std::size_t i = 0; i < rays.size(); ++i)
        {
            ground[i]                       = C + ((plane - C.z()) / rays[i].z()) * rays[i];
            const std::optional<double> hgt = terrain(ground[i].x(), ground[i].y());
            if (!hgt)
            {
                return false;
            }
            heights[i] = *hgt;
        }
        return true;
    };
    auto mean_height = [&]
    {
        double s = 0.0;
        for (double v : heights)
        {
            s += v;
        }
        return s / static_cast<double>(heights.size());
    };

    double plane = C.z() - config.nominal_height_agl;
    for (int it = 0; it < config.iterations; ++it)
    {
        if (!intersect_and_sample(plane))
        {
            return std::nullopt;
        }
        plane = mean_height();
    }
    if (!intersect_and_sample(plane))
    {
        return std::nullopt;
    }

    Footprint f;
    f.plane_height    = plane;
    const double mean = mean_height();
    double       var  = 0.0;
    for (std::size_t i = 0; i < ground.size(); ++i)
    {
        ground[i].z() = heights[i];
        var += (heights[i] - mean) * (heights[i] - mean);
    }
    f.height_spread = std::sqrt(var / static_cast<double>(heights.size()));
    f.corners       = { ground[0], ground[1], ground[2], ground[3] };
    f.centre        = ground[4];
    if (signedArea(horizontal(f)) < 0.0)
    {
        std::swap(f.corners[1], f.corners[3]);  // keep the top-left corner first
    }
    auto len = [&](int a, int b) { return (ground[a] - ground[b]).head<2>().norm(); };
    f.width_m  = 0.5 * (len(0, 1) + len(3, 2));
    f.height_m = 0.5 * (len(0, 3) + len(1, 2));
    return f;
}

double footprintArea(const Footprint &footprint)
{
    return std::abs(signedArea(horizontal(footprint)));
}

double overlapFraction(const Footprint &reference, const Footprint &other)
{
    const double area = footprintArea(reference);
    if (!(area > 0.0))
    {
        return 0.0;
    }
    const Polygon inter = clip(horizontal(other), horizontal(reference));
    if (inter.size() < 3)
    {
        return 0.0;
    }
    return std::clamp(std::abs(signedArea(inter)) / area, 0.0, 1.0);
}

double tiltFromNadir(const Eigen::Isometry3d &T_L_C)
{
    const Eigen::Vector3d axis = T_L_C.linear().col(2);
    return std::acos(std::clamp(-axis.z() / axis.norm(), -1.0, 1.0));
}

}  // namespace orthostream::selection
