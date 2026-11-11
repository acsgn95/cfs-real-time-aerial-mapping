#include "orthostream/geo/triangulation.hpp"

#include <Eigen/Eigenvalues>

namespace orthostream::geo
{

std::optional<Eigen::Vector3d> triangulateRays(const std::vector<Ray> &rays)
{
    if (rays.size() < 2)
    {
        return std::nullopt;
    }

    // Minimise sum_i |(I - d_i d_i^T)(p - o_i)|^2: the normal equations are
    // sum_i P_i p = sum_i P_i o_i with the projector P_i = I - d_i d_i^T.
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    for (const Ray &ray : rays)
    {
        const double norm = ray.direction.norm();
        if (!(norm > 0.0))
        {
            return std::nullopt;
        }
        const Eigen::Vector3d d = ray.direction / norm;
        const Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - d * d.transpose();
        A += P;
        b += P * ray.origin;
    }

    // A is singular when all rays are parallel; its smallest eigenvalue
    // measures how well the intersection is defined.
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(A);
    if (eigen.eigenvalues()(0) < 1e-12 * static_cast<double>(rays.size()))
    {
        return std::nullopt;
    }
    return Eigen::Vector3d(A.ldlt().solve(b));
}

}  // namespace orthostream::geo
