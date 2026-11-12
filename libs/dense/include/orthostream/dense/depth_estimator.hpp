// Depth estimation for one keyframe batch (ADR-005).
//
// A batch is a reference image and its source views, all at the dense
// working resolution, undistorted (pinhole camera) and posed in the batch's
// local working frame L_k (ADR-004 section 4). The result is a depth map, a
// normal map and a confidence map in the reference camera. All methods (GPU
// PatchMatch, the SGBM baseline) implement this interface, so they can be
// swapped by configuration.

#ifndef ORTHOSTREAM_DENSE_DEPTH_ESTIMATOR_HPP
#define ORTHOSTREAM_DENSE_DEPTH_ESTIMATOR_HPP

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>

#include "orthostream/geo/camera.hpp"

namespace orthostream::dense
{

class DenseError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

/// Raised when a method is not built in (for example PatchMatch without
/// ORTHOSTREAM_WITH_COLMAP).
class NotAvailableError : public DenseError
{
public:
    using DenseError::DenseError;
};

/// 8-bit grey image owned by the caller, of the batch camera's size.
struct GreyImageView
{
    const std::uint8_t *data   = nullptr;
    std::size_t         stride = 0;  ///< bytes per row
};

struct BatchView
{
    GreyImageView     image;
    Eigen::Isometry3d T_L_C = Eigen::Isometry3d::Identity();  ///< camera to working frame
};

struct DepthBatch
{
    /// Pinhole camera (no distortion) at the working resolution, shared by
    /// all views of the batch.
    geo::CameraModel       camera;
    BatchView              reference;
    std::vector<BatchView> sources;
    /// Depth range searched, in metres along the reference camera's z axis.
    double depth_min = 0.0;
    double depth_max = 0.0;
};

/// Output buffers owned by the caller, each for camera.width x camera.height
/// pixels in row-major order. Depth is the z coordinate in the reference
/// camera, 0 where no valid depth was found or the depth lies outside the
/// batch's depth range. Normals are unit vectors in the
/// reference camera frame, interleaved x, y, z. Confidence is in [0, 1].
struct DepthOutput
{
    float *depth      = nullptr;
    float *normals    = nullptr;  ///< 3 floats per pixel
    float *confidence = nullptr;
};

class DepthEstimator
{
public:
    virtual ~DepthEstimator() = default;

    /// Computes the depth of the reference view. Throws DenseError on invalid
    /// input or failure.
    virtual void estimate(const DepthBatch &batch, const DepthOutput &out) = 0;
};

/// Checks the parts of a batch that every method relies on; throws
/// DenseError otherwise.
void validateBatch(const DepthBatch &batch, const DepthOutput &out);

/// Terrain heights on a regular grid in the batch frame L_k (z up): node
/// (i, j) lies at x = x0 + i * spacing, y = y0 + j * spacing; NaN where the
/// terrain prior has no value. Row-major, j outer.
struct HeightGrid
{
    double             x0      = 0.0;
    double             y0      = 0.0;
    double             spacing = 0.0;
    int                nx      = 0;
    int                ny      = 0;
    std::vector<float> heights;
};

/// Terrain gate (ADR-005 section 5a): invalidates (depth and confidence 0)
/// the pixels whose point, in L_k, lies more than tolerance metres above or
/// below the bilinearly interpolated grid, or outside the grid. Returns the
/// number of pixels invalidated.
std::size_t applyTerrainGate(const DepthBatch &batch, const HeightGrid &grid, double tolerance, const DepthOutput &out);

}  // namespace orthostream::dense

#endif  // ORTHOSTREAM_DENSE_DEPTH_ESTIMATOR_HPP
