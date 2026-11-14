// Bundle adjustment with telemetry priors (ADR-019 section 3), GTSAM.
//
// Variables: one pose X_i per image (prior: the recorded pose), optionally a
// shared boresight D (prior: identity, translation held at zero), and one
// point L_j per tie point. The camera of image i is X_i * D. Measurements
// are normalised image coordinates; their noise is pixel_sigma / fx.

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/expressions.h>

#include "orthostream/geo/triangulation.hpp"
#include "orthostream/refine/refine.hpp"

namespace orthostream::refine
{
namespace
{

constexpr double kDeg = M_PI / 180.0;

gtsam::Key poseKey(std::size_t i) { return gtsam::Symbol('x', i); }
gtsam::Key pointKey(std::size_t j) { return gtsam::Symbol('l', j); }
const gtsam::Key kBoresight = gtsam::Symbol('d', 0);

gtsam::Pose3 toPose3(const Eigen::Isometry3d &T)
{
    return gtsam::Pose3(gtsam::Rot3(T.linear()), gtsam::Point3(T.translation()));
}

Eigen::Isometry3d toIsometry(const gtsam::Pose3 &P)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear()          = P.rotation().matrix();
    T.translation()     = P.translation();
    return T;
}

double rotationAngle(const Eigen::Matrix3d &R)
{
    return std::acos(std::clamp((R.trace() - 1.0) / 2.0, -1.0, 1.0));
}

struct TiePoint
{
    Eigen::Vector3d X;
    Track           observations;
};

class Problem
{
public:
    Problem(const geo::CameraModel &camera, const std::vector<PosePrior> &priors, const RefineConfig &config,
            const BoresightPrior &boresight)
        : cam_(camera), priors_(priors), config_(config), boresight_(boresight)
    {
    }

    Eigen::Vector2d normalised(const Eigen::Vector2d &pixel) const
    {
        return { (pixel.x() - cam_.cx) / cam_.fx, (pixel.y() - cam_.cy) / cam_.fy };
    }

    /// Reprojection error in pixels; infinity behind the camera.
    double error(const gtsam::Pose3 &camera_pose, const Eigen::Vector3d &X, const Eigen::Vector2d &pixel) const
    {
        const gtsam::Point3 p = camera_pose.transformTo(gtsam::Point3(X));
        if (p.z() <= 0.0)
        {
            return std::numeric_limits<double>::infinity();
        }
        const Eigen::Vector2d uv(cam_.fx * p.x() / p.z() + cam_.cx, cam_.fy * p.y() / p.z() + cam_.cy);
        return (uv - pixel).norm();
    }

    gtsam::Pose3 cameraPose(const gtsam::Values &v, std::size_t i) const
    {
        const gtsam::Pose3 X = v.at<gtsam::Pose3>(poseKey(i));
        return config_.estimate_boresight ? X.compose(v.at<gtsam::Pose3>(kBoresight)) : X;
    }

    /// Solves for the given points, starting from initial (poses, boresight,
    /// points); returns the solution and the number of iterations.
    std::pair<gtsam::Values, int> solve(const std::vector<TiePoint> &points, const gtsam::Values &initial) const
    {
        namespace nm = gtsam::noiseModel;
        gtsam::NonlinearFactorGraph graph;
        for (std::size_t i = 0; i < priors_.size(); ++i)
        {
            const double sr = priors_[i].attitude_sigma_deg * kDeg;
            const double st = priors_[i].position_sigma_m;
            graph.addPrior(poseKey(i), toPose3(priors_[i].X_W_C),
                           nm::Diagonal::Sigmas((gtsam::Vector6() << sr, sr, sr, st, st, st).finished()));
        }
        if (config_.estimate_boresight)
        {
            const double sb = boresight_.sigma_deg * kDeg;
            graph.addPrior(kBoresight, gtsam::Pose3(gtsam::Rot3(boresight_.R), gtsam::Point3(0, 0, 0)),
                           nm::Diagonal::Sigmas((gtsam::Vector6() << sb, sb, sb, 1e-4, 1e-4, 1e-4).finished()));
        }
        const auto measurement_noise =
            nm::Robust::Create(nm::mEstimator::Huber::Create(config_.huber_threshold),
                               nm::Isotropic::Sigma(2, config_.pixel_sigma / cam_.fx));
        for (std::size_t j = 0; j < points.size(); ++j)
        {
            const gtsam::Point3_ L(pointKey(j));
            for (const Observation &o : points[j].observations)
            {
                const gtsam::Pose3_ X(poseKey(o.image));
                const gtsam::Pose3_ C = config_.estimate_boresight ? X * gtsam::Pose3_(kBoresight) : X;
                const gtsam::Point2 z(normalised(o.pixel));
                graph.addExpressionFactor(measurement_noise, z, gtsam::project(gtsam::transformTo(C, L)));
            }
        }
        gtsam::LevenbergMarquardtParams params;
        params.maxIterations    = config_.max_iterations;
        params.relativeErrorTol = config_.relative_error_tol;
        params.absoluteErrorTol = 1e-10;
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
        gtsam::Values                      result = optimizer.optimize();
        return { result, static_cast<int>(optimizer.iterations()) };
    }

private:
    const geo::CameraModel       &cam_;
    const std::vector<PosePrior> &priors_;
    const RefineConfig           &config_;
    const BoresightPrior         &boresight_;
};

/// Chooses at most max_points tie points: first, in order of track length,
/// every point that is the first in a cell of one of its images (an 8 x 8
/// grid per image), so that all images and all parts of them are covered;
/// then the longest of the remaining ones.
std::vector<TiePoint> choosePoints(std::vector<TiePoint> points, const geo::CameraModel &cam, std::size_t n,
                                   std::size_t max_points)
{
    std::stable_sort(points.begin(), points.end(), [](const TiePoint &a, const TiePoint &b) {
        return a.observations.size() > b.observations.size();
    });
    if (points.size() <= max_points)
    {
        return points;
    }
    constexpr int     kGrid = 8;
    std::vector<bool> occupied(n * kGrid * kGrid, false);
    std::vector<bool> taken(points.size(), false);
    std::vector<TiePoint> chosen;
    auto cell = [&](const Observation &o) {
        const int cx = std::clamp(static_cast<int>(o.pixel.x() / cam.width * kGrid), 0, kGrid - 1);
        const int cy = std::clamp(static_cast<int>(o.pixel.y() / cam.height * kGrid), 0, kGrid - 1);
        return (o.image * kGrid + static_cast<std::size_t>(cy)) * kGrid + static_cast<std::size_t>(cx);
    };
    for (std::size_t j = 0; j < points.size() && chosen.size() < max_points; ++j)
    {
        bool fills = false;
        for (const Observation &o : points[j].observations)
        {
            fills = fills || !occupied[cell(o)];
        }
        if (fills)
        {
            for (const Observation &o : points[j].observations)
            {
                occupied[cell(o)] = true;
            }
            taken[j] = true;
            chosen.push_back(points[j]);
        }
    }
    for (std::size_t j = 0; j < points.size() && chosen.size() < max_points; ++j)
    {
        if (!taken[j])
        {
            chosen.push_back(points[j]);
        }
    }
    return chosen;
}

}  // namespace

AdjustmentResult adjust(const geo::CameraModel &camera, const std::vector<Eigen::Isometry3d> &priors,
                        const std::vector<Track> &tracks, const RefineConfig &config)
{
    std::vector<PosePrior> p;
    const double st = config.precise_position ? config.precise_position_sigma : config.position_sigma;
    for (const Eigen::Isometry3d &T : priors)
    {
        p.push_back({ T, config.attitude_sigma_deg, st });
    }
    BoresightPrior b;
    b.sigma_deg = config.boresight_sigma_deg;
    return adjust(camera, p, tracks, config, b);
}

AdjustmentResult adjust(const geo::CameraModel &camera, const std::vector<PosePrior> &prior_specs,
                        const std::vector<Track> &tracks, const RefineConfig &config,
                        const BoresightPrior &boresight)
{
    if (camera.k1 != 0.0 || camera.k2 != 0.0 || camera.k3 != 0.0 || camera.p1 != 0.0 || camera.p2 != 0.0)
    {
        throw RefineError("refinement needs an undistorted camera");
    }
    if (!(camera.fx > 0.0) || !(camera.fy > 0.0) || prior_specs.empty())
    {
        throw RefineError("invalid camera or no images");
    }
    const std::size_t n = prior_specs.size();
    const Problem     problem(camera, prior_specs, config, boresight);
    // Camera poses at the priors, with the initial boresight applied.
    Eigen::Isometry3d D = Eigen::Isometry3d::Identity();
    if (config.estimate_boresight)
    {
        D.linear() = boresight.R;
    }
    std::vector<Eigen::Isometry3d> priors;
    for (const PosePrior &p : prior_specs)
    {
        priors.push_back(p.X_W_C * D);
    }

    // Tie points triangulated from the prior poses. Points behind a camera
    // or seen under too small an angle are left out.
    std::vector<TiePoint> points;
    const double          min_angle = config.min_triangulation_deg * kDeg;
    for (const Track &t : tracks)
    {
        if (t.size() < 2)
        {
            continue;
        }
        std::vector<geo::Ray>        rays;
        std::vector<Eigen::Vector3d> dirs;
        for (const Observation &o : t)
        {
            if (o.image >= n)
            {
                throw RefineError("observation of an unknown image");
            }
            const Eigen::Vector2d   x = problem.normalised(o.pixel);
            const Eigen::Vector3d d = (priors[o.image].linear() * Eigen::Vector3d(x.x(), x.y(), 1.0)).normalized();
            rays.push_back({ priors[o.image].translation(), d });
            dirs.push_back(d);
        }
        double largest = 0.0;
        for (std::size_t a = 0; a < dirs.size(); ++a)
        {
            for (std::size_t b = a + 1; b < dirs.size(); ++b)
            {
                largest = std::max(largest, std::acos(std::clamp(dirs[a].dot(dirs[b]), -1.0, 1.0)));
            }
        }
        if (largest < min_angle)
        {
            continue;
        }
        const auto X = geo::triangulateRays(rays);
        if (!X)
        {
            continue;
        }
        const bool in_front = std::all_of(t.begin(), t.end(), [&](const Observation &o) {
            return (priors[o.image].inverse() * *X).z() > 0.0;
        });
        if (in_front)
        {
            points.push_back({ *X, t });
        }
    }
    points = choosePoints(std::move(points), camera, n, config.max_points);
    if (points.empty())
    {
        throw RefineError("no tie points could be triangulated");
    }

    gtsam::Values initial;
    for (std::size_t i = 0; i < n; ++i)
    {
        initial.insert(poseKey(i), toPose3(prior_specs[i].X_W_C));
    }
    if (config.estimate_boresight)
    {
        initial.insert(kBoresight, gtsam::Pose3(gtsam::Rot3(boresight.R), gtsam::Point3(0, 0, 0)));
    }
    for (std::size_t j = 0; j < points.size(); ++j)
    {
        initial.insert(pointKey(j), gtsam::Point3(points[j].X));
    }

    auto rms = [&](const gtsam::Values &v, const std::vector<TiePoint> &pts, bool from_values) {
        double      sum = 0.0;
        std::size_t m   = 0;
        for (std::size_t j = 0; j < pts.size(); ++j)
        {
            const Eigen::Vector3d X = from_values ? Eigen::Vector3d(v.at<gtsam::Point3>(pointKey(j))) : pts[j].X;
            for (const Observation &o : pts[j].observations)
            {
                const double e = problem.error(problem.cameraPose(v, o.image), X, o.pixel);
                if (std::isfinite(e))
                {
                    sum += e * e;
                    ++m;
                }
            }
        }
        return m > 0 ? std::sqrt(sum / static_cast<double>(m)) : 0.0;
    };

    AdjustmentResult result;
    result.rms_before_px = rms(initial, points, false);

    // First solution with the Huber loss, then outliers removed and solved
    // again from the first solution.
    auto [first, it1] = problem.solve(points, initial);
    std::vector<TiePoint> kept;
    gtsam::Values         second_initial;
    for (std::size_t i = 0; i < n; ++i)
    {
        second_initial.insert(poseKey(i), first.at<gtsam::Pose3>(poseKey(i)));
    }
    if (config.estimate_boresight)
    {
        second_initial.insert(kBoresight, first.at<gtsam::Pose3>(kBoresight));
    }
    for (std::size_t j = 0; j < points.size(); ++j)
    {
        const Eigen::Vector3d X(first.at<gtsam::Point3>(pointKey(j)));
        TiePoint              p { X, {} };
        for (const Observation &o : points[j].observations)
        {
            if (problem.error(problem.cameraPose(first, o.image), X, o.pixel) <= config.outlier_threshold_px)
            {
                p.observations.push_back(o);
            }
            else
            {
                ++result.rejected;
            }
        }
        if (p.observations.size() >= 2)
        {
            second_initial.insert(pointKey(kept.size()), gtsam::Point3(X));
            kept.push_back(std::move(p));
        }
        else
        {
            result.rejected += p.observations.size();
        }
    }
    if (kept.empty())
    {
        throw RefineError("all tie points were rejected as outliers");
    }
    auto [second, it2] = problem.solve(kept, second_initial);
    result.iterations = it1 + it2;

    result.points       = kept.size();
    result.rms_after_px = rms(second, kept, true);
    result.observations_per_image.assign(n, 0);
    for (const TiePoint &p : kept)
    {
        result.observations += p.observations.size();
        for (const Observation &o : p.observations)
        {
            ++result.observations_per_image[o.image];
        }
    }
    if (config.estimate_boresight)
    {
        result.boresight = second.at<gtsam::Pose3>(kBoresight).rotation().matrix();
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        const Eigen::Isometry3d X = toIsometry(second.at<gtsam::Pose3>(poseKey(i)));
        const Eigen::Isometry3d T = toIsometry(problem.cameraPose(second, i));
        result.T_W_C.push_back(T);
        result.X_W_C.push_back(X);
        const double rot = rotationAngle(prior_specs[i].X_W_C.linear().transpose() * X.linear()) / kDeg;
        const double pos = (X.translation() - prior_specs[i].X_W_C.translation()).norm();
        result.rotation_correction_deg.push_back(rot);
        result.position_correction_m.push_back(pos);
        result.max_rotation_correction_deg = std::max(result.max_rotation_correction_deg, rot);
        result.max_position_correction_m   = std::max(result.max_position_correction_m, pos);
    }
    return result;
}

}  // namespace orthostream::refine
