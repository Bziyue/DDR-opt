#pragma once

#include <Eigen/Eigen>

namespace traj_opt_components
{
inline double accumulateMeanTimeWindowPenalty(const Eigen::VectorXd &piece_time,
                                              const int segment_index,
                                              const double lower_ratio,
                                              const double upper_ratio,
                                              const double weight,
                                              Eigen::VectorXd &grad_time)
{
    if (weight <= 0.0 || segment_index < 0 || segment_index >= piece_time.size())
    {
        return 0.0;
    }

    const double mean_time = piece_time.mean();
    const double current_time = piece_time(segment_index);
    double cost = 0.0;

    if (current_time < mean_time * lower_ratio)
    {
        const double violation = current_time - mean_time * lower_ratio;
        cost += weight * violation * violation;
        grad_time.array() += weight * 2.0 * violation * (-lower_ratio / piece_time.size());
        grad_time(segment_index) += weight * 2.0 * violation;
    }

    if (current_time > mean_time * upper_ratio)
    {
        const double violation = current_time - mean_time * upper_ratio;
        cost += weight * violation * violation;
        grad_time.array() += weight * 2.0 * violation * (-upper_ratio / piece_time.size());
        grad_time(segment_index) += weight * 2.0 * violation;
    }

    return cost;
}
} // namespace traj_opt_components
