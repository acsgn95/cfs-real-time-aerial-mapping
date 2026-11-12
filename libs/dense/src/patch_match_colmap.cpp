// PatchMatch through COLMAP; compiled only with ORTHOSTREAM_WITH_COLMAP.

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <colmap/mvs/consistency_graph.h>
#include <colmap/mvs/image.h>
#include <colmap/mvs/patch_match.h>
#include <colmap/sensor/bitmap.h>
#include <glog/logging.h>

#include "orthostream/dense/patch_match.hpp"

namespace orthostream::dense
{

bool patchMatchAvailable()
{
    return true;
}

namespace
{

colmap::mvs::PatchMatchOptions toColmap(const PatchMatchConfig &c, const DepthBatch &batch)
{
    colmap::mvs::PatchMatchOptions o;
    o.max_image_size                 = -1;  // images arrive at the working resolution
    o.gpu_index                      = std::to_string(c.gpu_index);
    o.depth_min                      = batch.depth_min;
    o.depth_max                      = batch.depth_max;
    o.window_radius                  = c.window_radius;
    o.window_step                    = c.window_step;
    o.num_samples                    = c.num_samples;
    o.ncc_sigma                      = c.ncc_sigma;
    o.min_triangulation_angle        = c.min_triangulation_angle_deg;
    o.incident_angle_sigma           = c.incident_angle_sigma;
    o.num_iterations                 = c.num_iterations;
    o.geom_consistency               = false;
    o.filter                         = c.filter;
    o.filter_min_ncc                 = c.filter_min_ncc;
    o.filter_min_triangulation_angle = c.filter_min_triangulation_angle;
    o.filter_min_num_consistent      = c.filter_min_num_consistent;
    return o;
}

// COLMAP image of a batch view: K in COLMAP's pixel convention (centre of
// the top-left pixel at (0.5, 0.5)), and the world-to-camera transform of
// the working frame L_k, both row-major in single precision.
colmap::mvs::Image toColmapImage(const geo::CameraModel &cam, const BatchView &view)
{
    const float K[9] = { static_cast<float>(cam.fx),
                         0.0f,
                         static_cast<float>(cam.cx + 0.5),
                         0.0f,
                         static_cast<float>(cam.fy),
                         static_cast<float>(cam.cy + 0.5),
                         0.0f,
                         0.0f,
                         1.0f };

    const Eigen::Matrix3d R_C_L = view.T_L_C.linear().transpose();
    const Eigen::Vector3d t_C_L = -R_C_L * view.T_L_C.translation();
    float                 R[9];
    float                 T[3];
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            R[3 * r + c] = static_cast<float>(R_C_L(r, c));
        }
        T[r] = static_cast<float>(t_C_L(r));
    }

    colmap::mvs::Image image("", static_cast<size_t>(cam.width), static_cast<size_t>(cam.height), K, R, T);
    image.SetBitmap(colmap::Bitmap::ConvertFromRawBits(
        view.image.data, static_cast<int>(view.image.stride), cam.width, cam.height, /*rgb=*/false));
    return image;
}

}  // namespace

PatchMatchEstimator::PatchMatchEstimator(const PatchMatchConfig &config) : config_(config)
{
    if (config.geom_consistency)
    {
        throw DenseError("geometric consistency is not implemented yet");
    }
    if (config.gpu_index < 0 || config.num_iterations <= 0 || config.window_radius <= 0 || config.num_samples <= 0
        || (config.window_step != 1 && config.window_step != 2))
    {
        throw DenseError("invalid PatchMatch configuration");
    }
    // Libraries do not print (ADR-011). COLMAP reports progress through glog
    // at INFO level; only errors are kept. This setting is process-wide and
    // affects every glog user in the process (COLMAP, Ceres).
    FLAGS_minloglevel = google::GLOG_ERROR;
}

void PatchMatchEstimator::estimate(const DepthBatch &batch, const DepthOutput &out)
{
    validateBatch(batch, out);
    const geo::CameraModel &cam = batch.camera;

    std::vector<colmap::mvs::Image> images;
    images.reserve(batch.sources.size() + 1);
    images.push_back(toColmapImage(cam, batch.reference));
    colmap::mvs::PatchMatch::Problem problem;
    problem.ref_image_idx = 0;
    for (const BatchView &source : batch.sources)
    {
        problem.src_image_idxs.push_back(static_cast<int>(images.size()));
        images.push_back(toColmapImage(cam, source));
    }
    problem.images = &images;

    const colmap::mvs::PatchMatchOptions options = toColmap(config_, batch);
    if (!options.Check())
    {
        throw DenseError("COLMAP rejected the PatchMatch options");
    }

    colmap::mvs::PatchMatch patch_match(options, problem);
    try
    {
        patch_match.Run();
    }
    catch (const std::exception &e)
    {
        throw DenseError(std::string("PatchMatch failed: ") + e.what());
    }

    const colmap::mvs::DepthMap         depth   = patch_match.GetDepthMap();
    const colmap::mvs::NormalMap        normals = patch_match.GetNormalMap();
    const colmap::mvs::ConsistencyGraph graph   = patch_match.GetConsistencyGraph();

    const std::size_t w     = static_cast<std::size_t>(cam.width);
    const std::size_t h     = static_cast<std::size_t>(cam.height);
    const float      *d     = depth.GetPtr();
    const float      *n     = normals.GetPtr();  // planar: x plane, y plane, z plane
    const float       n_src = static_cast<float>(batch.sources.size());
    // COLMAP's perturbation step can leave depths outside the searched
    // range; they are not supported by the batch and count as invalid.
    const float lo = static_cast<float>(batch.depth_min), hi = static_cast<float>(batch.depth_max);
    for (std::size_t i = 0; i < w * h; ++i)
    {
        out.depth[i] = (d[i] >= lo && d[i] <= hi) ? d[i] : 0.0f;
    }
    for (std::size_t row = 0; row < h; ++row)
    {
        for (std::size_t col = 0; col < w; ++col)
        {
            const std::size_t i  = row * w + col;
            out.normals[3 * i]     = n[i];
            out.normals[3 * i + 1] = n[w * h + i];
            out.normals[3 * i + 2] = n[2 * w * h + i];

            int        num_consistent = 0;
            const int *consistent     = nullptr;
            graph.GetImageIdxs(static_cast<int>(row), static_cast<int>(col), &num_consistent, &consistent);
            out.confidence[i] = out.depth[i] > 0.0f ? static_cast<float>(num_consistent) / n_src : 0.0f;
        }
    }
}

}  // namespace orthostream::dense
