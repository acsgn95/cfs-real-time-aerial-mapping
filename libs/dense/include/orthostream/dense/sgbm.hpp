// Two-view semi-global matching on the CPU (OpenCV StereoSGBM), the
// default method of ADR-005 (section 1, since 2026-11-13) and the method of
// AerialMapper. Works without COLMAP and without a GPU.

#ifndef ORTHOSTREAM_DENSE_SGBM_HPP
#define ORTHOSTREAM_DENSE_SGBM_HPP

#include "orthostream/dense/depth_estimator.hpp"

namespace orthostream::dense
{

struct SgbmConfig
{
    /// Source view used: the one whose baseline over the mid depth of the
    /// batch is closest to this ratio (consecutive keyframes at the nominal
    /// point are about 0.2).
    double target_baseline_ratio = 0.2;
    int    block_size            = 5;    ///< odd, matching block side in pixels
    int    p1_factor             = 8;    ///< P1 = p1_factor * block_size^2 (small disparity changes)
    int    p2_factor             = 32;   ///< P2 = p2_factor * block_size^2 (large disparity changes)
    int    uniqueness_ratio      = 10;   ///< percent margin of the best over the second best cost
    int    disp12_max_diff       = 1;    ///< left-right consistency check, pixels
    int    speckle_window        = 100;  ///< pixels; 0 disables speckle filtering
    int    speckle_range         = 2;    ///< disparity range within a speckle
    int    max_disparities       = 512;  ///< upper limit of the searched disparity range
    bool   three_way             = true; ///< OpenCV MODE_SGBM_3WAY: faster, similar quality
};

/// Rectifies the reference view with one source view using the known poses,
/// matches them with StereoSGBM and converts the disparities to depth in the
/// reference camera. Epipolar lines along the image columns are handled by
/// transposing the rectified images. Confidence is 1 for matched pixels;
/// normals point towards the camera (SGBM gives no surface orientation).
class SgbmEstimator : public DepthEstimator
{
public:
    explicit SgbmEstimator(const SgbmConfig &config = {});

    void estimate(const DepthBatch &batch, const DepthOutput &out) override;

    /// Index of the source view that estimate() uses for a batch.
    std::size_t chooseSource(const DepthBatch &batch) const;

private:
    SgbmConfig config_;
};

}  // namespace orthostream::dense

#endif  // ORTHOSTREAM_DENSE_SGBM_HPP
