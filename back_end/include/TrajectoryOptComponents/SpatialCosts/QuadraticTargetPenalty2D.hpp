#pragma once

#include <Eigen/Eigen>

namespace traj_opt_components
{
inline double accumulateQuadraticTargetPenalty2D(const Eigen::Vector2d &position,
                                                 const Eigen::Vector2d &target,
                                                 const double weight,
                                                 Eigen::Vector2d &grad_position)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    const Eigen::Vector2d error = position - target;
    grad_position += weight * 2.0 * error;
    return weight * error.squaredNorm();
}
} // namespace traj_opt_components
