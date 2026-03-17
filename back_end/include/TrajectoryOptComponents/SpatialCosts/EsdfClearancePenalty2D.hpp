#pragma once

#include "TrajectoryOptComponents/PenaltyUtils.hpp"

#include <Eigen/Eigen>

namespace traj_opt_components
{
inline double accumulateEsdfClearancePenalty2D(const double sdf_value,
                                               const Eigen::Vector2d &grad_sdf,
                                               const double safe_distance,
                                               const double smooth_eps,
                                               const double weight,
                                               Eigen::Vector2d &grad_position)
{
    const double violation = safe_distance - sdf_value;
    double penalty = 0.0;
    double penalty_grad = 0.0;
    if (!smoothedL1(violation, smooth_eps, penalty, penalty_grad))
    {
        return 0.0;
    }

    grad_position -= weight * penalty_grad * grad_sdf;
    return weight * penalty;
}
} // namespace traj_opt_components
