#pragma once

#include "traj_opt/spline/SplineOptimizer.hpp"

namespace traj_opt_adapters
{
class DdrFinalStateAuxiliaryStateMap
{
public:
    using SplineType = SplineTrajectory::QuinticSpline2D;
    using WaypointsType = typename SplineType::MatrixType;
    using BoundaryConditions = SplineTrajectory::BoundaryConditions<2>;
    using Gradients = typename SplineType::Gradients;

    int getDimension() const
    {
        return 1;
    }

    Eigen::VectorXd getInitialValue(const std::vector<double> & /*ref_times*/,
                                    const WaypointsType &ref_waypoints,
                                    double /*ref_start_time*/,
                                    const BoundaryConditions & /*ref_bc*/) const
    {
        Eigen::VectorXd z(1);
        z(0) = ref_waypoints.row(ref_waypoints.rows() - 1)(1);
        return z;
    }

    void apply(const Eigen::VectorXd &z,
               std::vector<double> & /*times*/,
               WaypointsType &waypoints,
               double & /*start_time*/,
               BoundaryConditions & /*bc*/) const
    {
        waypoints.row(waypoints.rows() - 1)(1) = z(0);
    }

    double backward(const Eigen::VectorXd & /*z*/,
                    const SplineType & /*spline*/,
                    const std::vector<double> & /*times*/,
                    const WaypointsType & /*waypoints*/,
                    double /*start_time*/,
                    const BoundaryConditions & /*bc*/,
                    Gradients &grads,
                    Eigen::VectorXd &grad_z) const
    {
        grad_z.resize(1);
        grad_z(0) = grads.end.p(1);
        grads.end.p(1) = 0.0;
        return 0.0;
    }
};
} // namespace traj_opt_adapters
