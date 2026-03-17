#pragma once

#include <cmath>

#include <Eigen/Eigen>

namespace traj_opt_components
{
struct TimeMapUtils
{
    static inline void forwardMapTauToT(const Eigen::VectorXd &tau,
                                        Eigen::VectorXd &T)
    {
        const long size_tau = tau.size();
        T.resize(size_tau);
        for (long i = 0; i < size_tau; ++i)
        {
            T(i) = tau(i) > 0.0
                       ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                       : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
        }
    }

    template <typename VectorType>
    static inline void backwardMapTToTau(const Eigen::VectorXd &T,
                                         VectorType &tau)
    {
        const long size_t = T.size();
        tau.resize(size_t);
        for (long i = 0; i < size_t; ++i)
        {
            tau(i) = T(i) > 1.0
                         ? (std::sqrt(2.0 * T(i) - 1.0) - 1.0)
                         : (1.0 - std::sqrt(2.0 / T(i) - 1.0));
        }
    }

    template <typename VectorType>
    static inline void propagateGradientTToTau(const Eigen::VectorXd &tau,
                                               const Eigen::VectorXd &gradT,
                                               VectorType &gradTau)
    {
        const long size_tau = tau.size();
        gradTau.resize(size_tau);
        for (long i = 0; i < size_tau; ++i)
        {
            if (tau(i) > 0.0)
            {
                gradTau(i) = gradT(i) * (tau(i) + 1.0);
            }
            else
            {
                const double den_sqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                gradTau(i) = gradT(i) * (1.0 - tau(i)) / (den_sqrt * den_sqrt);
            }
        }
    }
};
} // namespace traj_opt_components
