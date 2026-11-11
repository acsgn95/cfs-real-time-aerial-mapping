#include "orthostream/geo/camera.hpp"

#include <cmath>

#include <Eigen/LU>

namespace orthostream::geo
{

CameraModel cameraFromMetashape(const MetashapeCalibration &calibration)
{
    CameraModel camera;
    camera.width  = calibration.width;
    camera.height = calibration.height;
    camera.fx     = calibration.f;
    camera.fy     = calibration.f;
    // Metashape: offset from the image centre, top-left pixel centre at 0.5.
    camera.cx = 0.5 * calibration.width + calibration.cx - 0.5;
    camera.cy = 0.5 * calibration.height + calibration.cy - 0.5;
    camera.k1 = calibration.k1;
    camera.k2 = calibration.k2;
    camera.k3 = calibration.k3;
    // Metashape's P1 multiplies the terms of OpenCV's p2 and vice versa.
    camera.p1 = calibration.p2;
    camera.p2 = calibration.p1;
    return camera;
}

std::array<double, 12> colmapFullOpenCvParams(const CameraModel &camera)
{
    // COLMAP places the centre of the top-left pixel at (0.5, 0.5).
    return { camera.fx,
             camera.fy,
             camera.cx + 0.5,
             camera.cy + 0.5,
             camera.k1,
             camera.k2,
             camera.p1,
             camera.p2,
             camera.k3,
             0.0,
             0.0,
             0.0 };
}

Eigen::Vector2d distortNormalized(const CameraModel &camera, const Eigen::Vector2d &undistorted)
{
    const double x      = undistorted.x();
    const double y      = undistorted.y();
    const double r2     = x * x + y * y;
    const double radial = 1.0 + r2 * (camera.k1 + r2 * (camera.k2 + r2 * camera.k3));
    return { x * radial + 2.0 * camera.p1 * x * y + camera.p2 * (r2 + 2.0 * x * x),
             y * radial + camera.p1 * (r2 + 2.0 * y * y) + 2.0 * camera.p2 * x * y };
}

namespace
{

// Jacobian of distortNormalized with respect to the undistorted coordinates.
Eigen::Matrix2d distortionJacobian(const CameraModel &camera, const Eigen::Vector2d &undistorted)
{
    const double x       = undistorted.x();
    const double y       = undistorted.y();
    const double r2      = x * x + y * y;
    const double radial  = 1.0 + r2 * (camera.k1 + r2 * (camera.k2 + r2 * camera.k3));
    const double dradial = camera.k1 + r2 * (2.0 * camera.k2 + 3.0 * r2 * camera.k3);  // d radial / d r2

    Eigen::Matrix2d J;
    J(0, 0) = radial + 2.0 * x * x * dradial + 2.0 * camera.p1 * y + 6.0 * camera.p2 * x;
    J(0, 1) = 2.0 * x * y * dradial + 2.0 * camera.p1 * x + 2.0 * camera.p2 * y;
    J(1, 0) = 2.0 * x * y * dradial + 2.0 * camera.p1 * x + 2.0 * camera.p2 * y;
    J(1, 1) = radial + 2.0 * y * y * dradial + 6.0 * camera.p1 * y + 2.0 * camera.p2 * x;
    return J;
}

}  // namespace

std::optional<Eigen::Vector2d> projectPoint(const CameraModel &camera, const Eigen::Vector3d &p_C)
{
    if (!(p_C.z() > 0.0))
    {
        return std::nullopt;
    }
    const Eigen::Vector2d d = distortNormalized(camera, p_C.head<2>() / p_C.z());
    return Eigen::Vector2d(camera.fx * d.x() + camera.cx, camera.fy * d.y() + camera.cy);
}

std::optional<Eigen::Vector2d> undistortPixel(const CameraModel &camera, const Eigen::Vector2d &pixel)
{
    if (!(camera.fx > 0.0) || !(camera.fy > 0.0))
    {
        return std::nullopt;
    }
    const Eigen::Vector2d target((pixel.x() - camera.cx) / camera.fx, (pixel.y() - camera.cy) / camera.fy);

    // Newton iteration on distortNormalized(x) = target, starting at the
    // distorted coordinates.
    constexpr int    kMaxIterations = 20;
    constexpr double kTolerance     = 1e-14;  // in normalised coordinates
    Eigen::Vector2d  x              = target;
    for (int i = 0; i < kMaxIterations; ++i)
    {
        const Eigen::Vector2d residual = target - distortNormalized(camera, x);
        if (residual.squaredNorm() < kTolerance * kTolerance)
        {
            return x;
        }
        const Eigen::Matrix2d J = distortionJacobian(camera, x);
        if (std::abs(J.determinant()) < 1e-12)
        {
            return std::nullopt;
        }
        x += J.inverse() * residual;
        if (!x.allFinite())
        {
            return std::nullopt;
        }
    }
    const Eigen::Vector2d residual = target - distortNormalized(camera, x);
    if (residual.norm() < 1e-10)
    {
        return x;
    }
    return std::nullopt;
}

}  // namespace orthostream::geo
