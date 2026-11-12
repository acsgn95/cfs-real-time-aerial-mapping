#include "orthostream/dense/sgbm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace orthostream::dense
{

SgbmEstimator::SgbmEstimator(const SgbmConfig &config) : config_(config)
{
    if (config.block_size < 1 || config.block_size % 2 == 0 || config.p1_factor <= 0
        || config.p2_factor <= config.p1_factor || config.max_disparities < 16 || !(config.target_baseline_ratio > 0.0))
    {
        throw DenseError("invalid SGBM configuration");
    }
}

std::size_t SgbmEstimator::chooseSource(const DepthBatch &batch) const
{
    const double          mid  = 0.5 * (batch.depth_min + batch.depth_max);
    const Eigen::Vector3d C    = batch.reference.T_L_C.translation();
    std::size_t           best = 0;
    double                gap  = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < batch.sources.size(); ++i)
    {
        const double ratio = (batch.sources[i].T_L_C.translation() - C).norm() / mid;
        const double g     = std::abs(ratio - config_.target_baseline_ratio);
        if (g < gap)
        {
            gap  = g;
            best = i;
        }
    }
    return best;
}

void SgbmEstimator::estimate(const DepthBatch &batch, const DepthOutput &out)
{
    validateBatch(batch, out);
    const geo::CameraModel &cam = batch.camera;
    const cv::Size          size(cam.width, cam.height);
    const std::size_t       n = static_cast<std::size_t>(cam.width) * cam.height;
    std::fill(out.depth, out.depth + n, 0.0f);
    std::fill(out.confidence, out.confidence + n, 0.0f);
    for (std::size_t i = 0; i < n; ++i)
    {
        out.normals[3 * i]     = 0.0f;
        out.normals[3 * i + 1] = 0.0f;
        out.normals[3 * i + 2] = -1.0f;
    }

    // Relative pose in OpenCV's convention: x_src = R x_ref + T.
    const BatchView        &src     = batch.sources[chooseSource(batch)];
    const Eigen::Isometry3d T_s_r   = src.T_L_C.inverse() * batch.reference.T_L_C;
    if (T_s_r.translation().norm() < 1e-6)
    {
        throw DenseError("SGBM source view has no baseline");
    }
    const cv::Matx33d K(cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1);
    cv::Matx33d       R;
    cv::Vec3d         T;
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            R(r, c) = T_s_r.linear()(r, c);
        }
        T[r] = T_s_r.translation()(r);
    }
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K, cv::noArray(), K, cv::noArray(), size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1,
                      size);

    // Epipolar lines along columns: transpose. Source on the left of the
    // reference: mirror, so that the reference is the left image of SGBM.
    const bool   vertical = std::abs(P2.at<double>(1, 3)) > std::abs(P2.at<double>(0, 3));
    const double offset   = vertical ? P2.at<double>(1, 3) : P2.at<double>(0, 3);
    const bool   mirror   = offset > 0.0;
    const double f_r      = P1.at<double>(0, 0);
    const double baseline = std::abs(offset) / f_r;
    const double cx_r = P1.at<double>(0, 2), cy_r = P1.at<double>(1, 2);

    cv::Mat map_rx, map_ry, map_sx, map_sy;
    cv::initUndistortRectifyMap(K, cv::noArray(), R1, P1, size, CV_16SC2, map_rx, map_ry);
    cv::initUndistortRectifyMap(K, cv::noArray(), R2, P2, size, CV_16SC2, map_sx, map_sy);
    const cv::Mat ref_img(size, CV_8UC1, const_cast<std::uint8_t *>(batch.reference.image.data),
                          batch.reference.image.stride);
    const cv::Mat src_img(size, CV_8UC1, const_cast<std::uint8_t *>(src.image.data), src.image.stride);
    cv::Mat       left, right;
    cv::remap(ref_img, left, map_rx, map_ry, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::remap(src_img, right, map_sx, map_sy, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    // Where the rectified source has no image, SGBM matches the black border
    // with itself; a mask of the valid source area rejects those matches.
    cv::Mat right_valid;
    cv::remap(cv::Mat(size, CV_8UC1, cv::Scalar(255)), right_valid, map_sx, map_sy, cv::INTER_NEAREST,
              cv::BORDER_CONSTANT, cv::Scalar(0));
    if (vertical)
    {
        cv::transpose(left, left);
        cv::transpose(right, right);
        cv::transpose(right_valid, right_valid);
    }
    if (mirror)
    {
        cv::flip(left, left, 1);
        cv::flip(right, right, 1);
        cv::flip(right_valid, right_valid, 1);
    }

    // Disparity range from the depth range: d = f B / Z.
    const int min_disp = std::max(0, static_cast<int>(std::floor(f_r * baseline / batch.depth_max)));
    const int span     = static_cast<int>(std::ceil(f_r * baseline / batch.depth_min)) - min_disp + 1;
    const int num_disp = std::clamp(((span + 15) / 16) * 16, 16, config_.max_disparities);
    const int bs       = config_.block_size;
    cv::Ptr<cv::StereoSGBM> sgbm =
        cv::StereoSGBM::create(min_disp, num_disp, bs, config_.p1_factor * bs * bs, config_.p2_factor * bs * bs,
                               config_.disp12_max_diff, 63, config_.uniqueness_ratio, config_.speckle_window,
                               config_.speckle_range,
                               config_.three_way ? cv::StereoSGBM::MODE_SGBM_3WAY : cv::StereoSGBM::MODE_SGBM);
    // SGBM leaves the first min_disp + num_disp columns of the left image
    // without disparity; padding both images on the left keeps them.
    const int pad = min_disp + num_disp;
    cv::copyMakeBorder(left, left, 0, 0, pad, 0, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::copyMakeBorder(right, right, 0, 0, pad, 0, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::Mat padded;  // CV_16S, 16 x disparity
    sgbm->compute(left, right, padded);
    const cv::Mat disparity = padded.colRange(pad, padded.cols);

    // Back to the reference camera: original pixel -> rectified pixel
    // (homography P1 R1 K^-1) -> disparity -> point -> depth.
    const cv::Matx33d P1_3(P1.at<double>(0, 0), P1.at<double>(0, 1), P1.at<double>(0, 2), P1.at<double>(1, 0),
                           P1.at<double>(1, 1), P1.at<double>(1, 2), P1.at<double>(2, 0), P1.at<double>(2, 1),
                           P1.at<double>(2, 2));
    const cv::Matx33d R1m(R1);
    const cv::Matx33d H   = P1_3 * R1m * K.inv();
    const cv::Matx33d R1t = R1m.t();
    const int         dw = disparity.cols, dh = disparity.rows;
    for (int v = 0; v < cam.height; ++v)
    {
        for (int u = 0; u < cam.width; ++u)
        {
            const cv::Vec3d p  = H * cv::Vec3d(u, v, 1.0);
            const double    xr = p[0] / p[2], yr = p[1] / p[2];
            double          xs = vertical ? yr : xr, ys = vertical ? xr : yr;
            if (mirror)
            {
                xs = dw - 1 - xs;
            }
            const int ix = static_cast<int>(std::lround(xs)), iy = static_cast<int>(std::lround(ys));
            if (ix < 0 || iy < 0 || ix >= dw || iy >= dh)
            {
                continue;
            }
            const short d16 = disparity.at<short>(iy, ix);
            if (d16 < min_disp * 16 || d16 <= 0)
            {
                continue;  // no match
            }
            const int jx = static_cast<int>(std::lround(xs - d16 / 16.0));  // matching column in the source
            if (jx < 0 || jx >= dw || right_valid.at<std::uint8_t>(iy, jx) == 0)
            {
                continue;  // matched outside the source image
            }
            const double    z_r = f_r * baseline / (d16 / 16.0);
            const cv::Vec3d X_rect((xr - cx_r) * z_r / f_r, (yr - cy_r) * z_r / f_r, z_r);
            const double    z   = (R1t * X_rect)[2];
            if (z >= batch.depth_min && z <= batch.depth_max)
            {
                const std::size_t i = static_cast<std::size_t>(v) * cam.width + u;
                out.depth[i]        = static_cast<float>(z);
                out.confidence[i]   = 1.0f;
            }
        }
    }
}

}  // namespace orthostream::dense
