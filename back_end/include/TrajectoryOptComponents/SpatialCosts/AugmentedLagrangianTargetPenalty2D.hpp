#pragma once

#include <Eigen/Eigen>

namespace traj_opt_components
{
inline double accumulateAugmentedLagrangianTargetPenalty2D(const Eigen::Vector2d &error,
                                                           const Eigen::Vector2d &lambda,
                                                           const Eigen::Vector2d &rho,
                                                           Eigen::Vector2d &grad_error)
{
    grad_error.array() += rho.array() * error.array() + lambda.array();
    const Eigen::Array2d shifted = error.array() + lambda.array() / rho.array();
    return 0.5 * (rho.array() * shifted.square()).sum();
}
} // namespace traj_opt_components
