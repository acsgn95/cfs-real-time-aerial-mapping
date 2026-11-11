// Camera model and calibration conversions (ADR-008 section 2).
//
// Internal convention: OpenCV pinhole model with radial (k1, k2, k3) and
// tangential (p1, p2) distortion. Pixel coordinates have their origin at the
// centre of the top-left pixel. Camera axes: x right, y down, z forward.

#ifndef ORTHOSTREAM_GEO_CAMERA_HPP
#define ORTHOSTREAM_GEO_CAMERA_HPP

#include <array>
#include <optional>

#include <Eigen/Core>

namespace orthostream::geo
{

/// OpenCV pinhole camera with radial-tangential distortion.
struct CameraModel
{
    int    width  = 0;  ///< image width in pixels
    int    height = 0;  ///< image height in pixels
    double fx     = 0.0;
    double fy     = 0.0;
    double cx     = 0.0;  ///< principal point, OpenCV pixel convention
    double cy     = 0.0;
    double k1     = 0.0;
    double k2     = 0.0;
    double k3     = 0.0;
    double p1     = 0.0;
    double p2     = 0.0;
};

/// Frame camera calibration as stored by Agisoft Metashape: one focal length,
/// principal point as an offset from the image centre, centre of the top-left
/// pixel at (0.5, 0.5). The terms k4, b1 and b2 are not supported and must be
/// zero.
struct MetashapeCalibration
{
    int    width  = 0;
    int    height = 0;
    double f      = 0.0;
    double cx     = 0.0;
    double cy     = 0.0;
    double k1     = 0.0;
    double k2     = 0.0;
    double k3     = 0.0;
    double p1     = 0.0;
    double p2     = 0.0;
};

/// Converts a Metashape calibration to the internal model (ADR-008).
CameraModel cameraFromMetashape(const MetashapeCalibration &calibration);

/// Parameters of COLMAP's FULL_OPENCV camera model for the same camera:
/// fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6. COLMAP places the centre of
/// the top-left pixel at (0.5, 0.5).
std::array<double, 12> colmapFullOpenCvParams(const CameraModel &camera);

/// The same camera for an image resampled to width x height pixels. Pixel
/// edges stay aligned: u' = sx * (u + 0.5) - 0.5 with sx = width / camera.width
/// (and likewise for v), so cx and cy are not simply multiplied. Distortion
/// coefficients act on normalised coordinates and do not change.
CameraModel scaledCamera(const CameraModel &camera, int width, int height);

/// The camera without distortion: the model of an undistorted image with the
/// same intrinsics.
CameraModel pinholeCamera(const CameraModel &camera);

/// Applies the distortion to normalised image coordinates (x / z, y / z).
Eigen::Vector2d distortNormalized(const CameraModel &camera, const Eigen::Vector2d &undistorted);

/// Projects a point given in the camera frame to pixel coordinates. Returns
/// nothing for points at or behind the camera (z <= 0). The result may lie
/// outside the image.
std::optional<Eigen::Vector2d> projectPoint(const CameraModel &camera, const Eigen::Vector3d &p_C);

/// Inverts the distortion: returns the normalised undistorted coordinates of
/// a pixel, or nothing if the iteration does not converge.
std::optional<Eigen::Vector2d> undistortPixel(const CameraModel &camera, const Eigen::Vector2d &pixel);

}  // namespace orthostream::geo

#endif  // ORTHOSTREAM_GEO_CAMERA_HPP
