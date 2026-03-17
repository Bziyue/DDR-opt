#pragma once

#include "traj_opt/spline/SplineTrajectory.hpp"

#include <vector>

namespace traj_opt_adapters
{
class DdrSplineTrajectoryAdapter
{
public:
    using SplineType = SplineTrajectory::QuinticSpline2D;
    using BoundaryConditions = SplineTrajectory::BoundaryConditions<2>;
    using VectorType = Eigen::Vector2d;
    using MatrixType = typename SplineType::MatrixType;
    using TrajectoryType = typename SplineType::TrajectoryType;

    DdrSplineTrajectoryAdapter() = default;

    static BoundaryConditions buildBoundaryConditions(const Eigen::MatrixXd &init_state,
                                                      const Eigen::MatrixXd &final_state)
    {
        BoundaryConditions bc;
        bc.start_velocity = init_state.col(1);
        bc.start_acceleration = init_state.col(2);
        bc.end_velocity = final_state.col(1);
        bc.end_acceleration = final_state.col(2);
        return bc;
    }

    static MatrixType buildWaypoints(const Eigen::MatrixXd &init_state,
                                     const Eigen::MatrixXd &final_state,
                                     const Eigen::MatrixXd &inner_points)
    {
        MatrixType waypoints(inner_points.cols() + 2, 2);
        waypoints.row(0) = init_state.col(0).transpose();
        for (Eigen::Index i = 0; i < inner_points.cols(); ++i)
        {
            waypoints.row(i + 1) = inner_points.col(i).transpose();
        }
        waypoints.row(waypoints.rows() - 1) = final_state.col(0).transpose();
        return waypoints;
    }

    static std::vector<double> buildSegmentTimes(const Eigen::VectorXd &piece_times)
    {
        return std::vector<double>(piece_times.data(), piece_times.data() + piece_times.size());
    }

    void update(const std::vector<double> &segment_times,
                const MatrixType &waypoints,
                double start_time,
                const BoundaryConditions &bc)
    {
        spline_.update(segment_times, waypoints, start_time, bc);
    }

    void setTraj(const Eigen::Vector3d &start_state,
                 const Eigen::MatrixXd &init_state,
                 const Eigen::MatrixXd &final_state,
                 const Eigen::MatrixXd &inner_points,
                 const Eigen::VectorXd &piece_times)
    {
        start_state_ = start_state;
        update(buildSegmentTimes(piece_times),
               buildWaypoints(init_state, final_state, inner_points),
               0.0,
               buildBoundaryConditions(init_state, final_state));
    }

    double getTotalDuration() const
    {
        return spline_.getTrajectory().getDuration();
    }

    int getPieceNum() const
    {
        return spline_.getTrajectory().getNumSegments();
    }

    Eigen::VectorXd getDurations() const
    {
        const auto &breakpoints = spline_.getTrajectory().getBreakpoints();
        Eigen::VectorXd durations(std::max<int>(0, breakpoints.size() - 1));
        for (Eigen::Index i = 0; i < durations.size(); ++i)
        {
            durations(i) = breakpoints[i + 1] - breakpoints[i];
        }
        return durations;
    }

    VectorType getPos(double t) const
    {
        return spline_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Pos);
    }

    VectorType getVel(double t) const
    {
        return spline_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Vel);
    }

    VectorType getAcc(double t) const
    {
        return spline_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Acc);
    }

    VectorType getJer(double t) const
    {
        return spline_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Jerk);
    }

    const SplineType &getSpline() const
    {
        return spline_;
    }

    const TrajectoryType &getTrajectory() const
    {
        return spline_.getTrajectory();
    }

    const Eigen::Vector3d &getStartState() const
    {
        return start_state_;
    }

private:
    SplineType spline_;
    Eigen::Vector3d start_state_ = Eigen::Vector3d::Zero();
};
} // namespace traj_opt_adapters
