#pragma once

#include <vector>

#include <Eigen/Eigen>

namespace traj_opt_components
{
struct LinearTimeCost
{
    double weight = 0.0;

    double operator()(const std::vector<double> &times, Eigen::VectorXd &grad) const
    {
        double cost = 0.0;
        for (size_t i = 0; i < times.size(); ++i)
        {
            cost += weight * times[i];
            grad(i) += weight;
        }
        return cost;
    }
};
} // namespace traj_opt_components
