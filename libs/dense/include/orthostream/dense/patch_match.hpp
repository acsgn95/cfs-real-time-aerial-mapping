// GPU PatchMatch stereo through COLMAP, used as a library with known poses
// (ADR-005 section 1).

#ifndef ORTHOSTREAM_DENSE_PATCH_MATCH_HPP
#define ORTHOSTREAM_DENSE_PATCH_MATCH_HPP

#include "orthostream/dense/depth_estimator.hpp"

namespace orthostream::dense
{

/// Parameters passed to COLMAP's PatchMatch stereo; defaults are COLMAP's,
/// except where ADR-005 sets a provisional value.
struct PatchMatchConfig
{
    int    gpu_index                      = 0;
    // Defaults of ADR-005 section 5 (2.4 s per Dorfberg keyframe at 2048 px
    // on the RTX 3050 Laptop); COLMAP's own defaults are 5 iterations,
    // radius 5 and 15 samples.
    int    num_iterations                 = 3;
    int    window_radius                  = 3;
    /// NCC uses every n-th row and column of the window (1 or 2); 2 is
    /// about four times faster.
    int    window_step                    = 1;
    int    num_samples                    = 7;
    double ncc_sigma                      = 0.6;
    double min_triangulation_angle_deg    = 1.0;
    double incident_angle_sigma           = 0.9;
    bool   filter                         = true;
    double filter_min_ncc                 = 0.3;  // ADR-005 section 5 (COLMAP default 0.1)
    double filter_min_triangulation_angle = 3.0;
    int    filter_min_num_consistent      = 2;
    /// Geometric consistency needs depth maps of all source views; not
    /// implemented yet (ADR-005 section 2).
    bool geom_consistency = false;
};

/// True if this build contains PatchMatch (ORTHOSTREAM_WITH_COLMAP).
bool patchMatchAvailable();

/// Depth estimation with COLMAP's CUDA PatchMatch stereo. Confidence is the
/// share of source views that are photometrically consistent at a pixel.
///
/// Known limitation: COLMAP ends the process on CUDA errors (for example
/// when GPU memory runs out) instead of reporting them.
class PatchMatchEstimator : public DepthEstimator
{
public:
    /// Throws NotAvailableError if the build has no PatchMatch, and
    /// DenseError for an invalid configuration.
    explicit PatchMatchEstimator(const PatchMatchConfig &config);

    void estimate(const DepthBatch &batch, const DepthOutput &out) override;

private:
    PatchMatchConfig config_;
};

}  // namespace orthostream::dense

#endif  // ORTHOSTREAM_DENSE_PATCH_MATCH_HPP
