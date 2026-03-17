#pragma once

#include "TrajectoryOptAdapters/DdrSplineTrajectoryAdapter.hpp"

namespace traj_opt_adapters
{
template <typename PlannerT>
struct DdrPathPenaltyCostAdapter
{
    using SplineType = typename DdrSplineTrajectoryAdapter::SplineType;
    using BoundaryConditions = typename DdrSplineTrajectoryAdapter::BoundaryConditions;
    using Gradients = typename SplineType::Gradients;
    using WaypointsType = typename SplineType::MatrixType;

    PlannerT *planner = nullptr;

    void reset(PlannerT *planner_ptr)
    {
        planner = planner_ptr;
    }

    double operator()(const SplineType &spline,
                      const std::vector<double> &times,
                      const WaypointsType &waypoints,
                      double start_time,
                      const BoundaryConditions &bc,
                      Gradients &grads) const
    {
        if (planner == nullptr)
        {
            return 0.0;
        }
        return planner->evaluatePathTrajectoryCost(spline, times, waypoints, start_time, bc, grads);
    }
};
} // namespace traj_opt_adapters
