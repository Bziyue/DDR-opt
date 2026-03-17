#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include "ros/publisher.h"
#include "tf/transform_datatypes.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

#include "plan_env/sdf_map.h"
#include "front_end/traj_representation.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "gcopter/lbfgs.hpp"
#include "traj_opt/spline/SplineOptimizer.hpp"
#include "TrajectoryOptAdapters/DdrFinalStateAuxiliaryStateMap.hpp"
#include "TrajectoryOptAdapters/DdrFormalPenaltyCostAdapter.hpp"
#include "TrajectoryOptAdapters/DdrPathPenaltyCostAdapter.hpp"
#include "TrajectoryOptAdapters/DdrSplineTrajectoryAdapter.hpp"

namespace traj_opt_adapters
{
template <typename PlannerT>
struct DdrFormalPenaltyCostAdapter;

template <typename PlannerT>
struct DdrPathPenaltyCostAdapter;
} // namespace traj_opt_adapters


struct Config
{
    // kinematic constraints
    double max_vel_;
    double min_vel_;
    double max_acc_;
    double max_omega_;
    double max_domega_;
    double max_centripetal_acc_;

    bool if_directly_constrain_v_omega_;
    
    Config(const ros::NodeHandle &nh_)
    {
        nh_.param<double>(ros::this_node::getName()+ "/max_vel",max_vel_,5);
        nh_.param<double>(ros::this_node::getName()+ "/min_vel",min_vel_,-5);
        nh_.param<double>(ros::this_node::getName()+ "/max_acc",max_acc_,5);
        nh_.param<double>(ros::this_node::getName()+ "/max_domega",max_domega_,50);
        nh_.param<double>(ros::this_node::getName()+ "/max_centripetal_acc",max_centripetal_acc_,10000);
        nh_.param<double>(ros::this_node::getName()+ "/max_omega",max_omega_,1);
        nh_.param<bool>(ros::this_node::getName()+ "/if_directly_constrain_v_omega", if_directly_constrain_v_omega_, false);
    }
};

struct PenaltyWeights{
    double time_weight;
    double time_weight_backup_for_replan;
    double acc_weight;
    double domega_weight;
    double collision_weight;
    double moment_weight;
    double mean_time_weight;
    double cen_acc_weight;
};

// For trajectory pre-processing
struct PathpenaltyWeights{
    double time_weight;
    double bigpath_sdf_weight;
    double mean_time_weight;
    double moment_weight;
    double acc_weight;
    double domega_weight;
};

// For trajectory pre-processing
struct PathLbfgsParams{
    lbfgs::lbfgs_parameter_t path_lbfgs_params;
    double normal_past;
    double shot_path_past;
    double shot_path_horizon;
};


class MSPlanner
{
private:
    template <typename PlannerT>
    friend struct traj_opt_adapters::DdrFormalPenaltyCostAdapter;

    template <typename PlannerT>
    friend struct traj_opt_adapters::DdrPathPenaltyCostAdapter;

    Config config_;
    ros::NodeHandle nh_;
    std::shared_ptr<SDFmap> map_;
    
    ros::Publisher initTrajPathPub_;
    ros::Publisher preprocessedTrajPathPub_;
    ros::Publisher CollisionpointPub;
    ros::Publisher optimizedTrajPathPub_;

    ros::Publisher initTrajPointPub_;
    ros::Publisher preprocessedTrajPointPub_;
    ros::Publisher innerinitpositionsPoint;

    ros::Publisher recordTextPub;

    // optimizer parameters
    double mean_time_lowBound_;
    double mean_time_uppBound_;
    double smoothEps;// for smoothL1
    PenaltyWeights penaltyWt;
    Eigen::Vector2d energyWeights;
    lbfgs::lbfgs_parameter_t lbfgs_params_;
    
    double finalMinSafeDis;
    int finalSafeDisCheckNum;
    int safeReplanMaxTime;

    Eigen::Vector3d iniStateXYTheta;
    Eigen::Vector3d finStateXYTheta;

    Eigen::Vector3d final_initStateXYTheta_;
    Eigen::Vector3d final_finStateXYTheta_;

    Eigen::VectorXd pieceTime;
    Eigen::MatrixXd Innerpoints;
    Eigen::MatrixXd iniState;
    Eigen::MatrixXd finState;
    // trajectory segments number
    int TrajNum;
    // if the traj is cutted
    bool ifCutTraj_;

    std::vector<Eigen::Vector3d> inner_init_positions;

    Eigen::MatrixXd finalInnerpoints;
    Eigen::VectorXd finalpieceTime;

    std::vector<Eigen::Vector3d> statelist;

    PathLbfgsParams path_lbfgs_params_;
    PathpenaltyWeights PathpenaltyWt;
 
    using TrajectoryAdapter = traj_opt_adapters::DdrSplineTrajectoryAdapter;
    using SplineType = typename TrajectoryAdapter::SplineType;
    using SplineBoundaryConditions = typename TrajectoryAdapter::BoundaryConditions;
    using SplineOptimizer = SplineTrajectory::SplineOptimizer<
        2,
        SplineType,
        SplineTrajectory::QuadInvTimeMap,
        SplineTrajectory::IdentitySpatialMap<2>,
        traj_opt_adapters::DdrFinalStateAuxiliaryStateMap>;

    SplineOptimizer spline_optimizer_;
    SplineOptimizer::Workspace spline_workspace_;
    traj_opt_adapters::DdrFinalStateAuxiliaryStateMap final_state_aux_map_;
    traj_opt_adapters::DdrFormalPenaltyCostAdapter<MSPlanner> formal_cost_adapter_;
    traj_opt_adapters::DdrPathPenaltyCostAdapter<MSPlanner> path_cost_adapter_;

    SplineBoundaryConditions spline_boundary_conditions_;
    typename TrajectoryAdapter::MatrixType spline_waypoints_;


    // sampling parameters
    int sparseResolution_;
    int sparseResolution_6_;
    double timeResolution_;
    int mintrajNum_;

    int iter_num_;
    Eigen::Vector2d FinalIntegralXYError;
    // for ALM
    Eigen::Vector2d FinalIntegralXYError_;
    // for debug, record the collision points
    std::vector<Eigen::Vector2d> collision_point;
    std::vector<Eigen::Vector2d> collision_point_;

    // unchanged auxiliary parameters in the loop
    int SamNumEachPart;
    // Simpson integration coefficients for each sampling point
    Eigen::VectorXd IntegralChainCoeff;

    // checkpoints for collision check
    std::vector<Eigen::Vector2d> check_point;
    double safeDis_, safeDis;

    // Whether to perform visualization
    bool ifprint = false;

    // Augmented Lagrangian
    Eigen::VectorXd init_EqualLambda_, init_EqualRho_, EqualRhoMax_, EqualGamma_;
    Eigen::VectorXd EqualLambda, EqualRho;
    Eigen::VectorXd EqualTolerance_;

    Eigen::VectorXd Cut_init_EqualLambda_, Cut_init_EqualRho_, Cut_EqualRhoMax_, Cut_EqualGamma_;
    Eigen::VectorXd Cut_EqualLambda, Cut_EqualRho;
    Eigen::VectorXd Cut_EqualTolerance_;

    bool hrz_limited_;
    double hrz_laser_range_dgr_;

    // Trajectory prediction resolution for get_the_predicted_state
    double trajPredictResolution_;

    bool if_visual_optimization_ = false;

 public:

    // Results
    TrajectoryAdapter final_traj_;
    // Results before collision check
    TrajectoryAdapter optimizer_traj_;
    // Results before trajectory pre-processing
    TrajectoryAdapter init_final_traj_;

    Eigen::Vector3d ICR_;
    bool if_standard_diff_;

    MSPlanner(const Config &conf, ros::NodeHandle &nh, std::shared_ptr<SDFmap> map);

    // Main function of the optimizer
    bool spline_plan(const FlatTrajData &flat_traj);
    // Obtain the initial state for planning
    bool get_state(const FlatTrajData &flat_traj);
    // Optimization
    bool optimizer();
    // Result check: whether a collision occurred
    bool check_final_collision(const TrajectoryAdapter &final_traj, const Eigen::Vector3d &start_state_XYTheta);

    static inline int earlyExit(void *instance,
                                const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g,
                                const double fx,
                                const double step,
                                const int k,
                                const int ls);

    static double costFunctionCallback(void *ptr,
                                       const Eigen::VectorXd &x,
                                       Eigen::VectorXd &g);

    static double costFunctionCallbackPath(void *ptr,
                                           const Eigen::VectorXd &x,
                                           Eigen::VectorXd &g);

    void trajectoryPathPub(const TrajectoryAdapter &final_traj, const Eigen::Vector3d &start_state_XYTheta, const ros::Publisher &publisher);
    void trajectoryPointPub(const TrajectoryAdapter &final_traj, const Eigen::Vector3d &start_state_XYTheta, const ros::Publisher &publisher, const Eigen::Vector3d &color);

    double evaluateFormalTrajectoryCost(const SplineType &spline,
                                        const std::vector<double> &times,
                                        const typename SplineType::MatrixType &waypoints,
                                        double start_time,
                                        const SplineBoundaryConditions &bc,
                                        typename SplineType::Gradients &grads);

    double evaluatePathTrajectoryCost(const SplineType &spline,
                                      const std::vector<double> &times,
                                      const typename SplineType::MatrixType &waypoints,
                                      double start_time,
                                      const SplineBoundaryConditions &bc,
                                      typename SplineType::Gradients &grads);

    double accumulateWeightedEnergyCost(const SplineType &spline,
                                        Eigen::MatrixX2d &partial_grad_by_coeffs,
                                        Eigen::VectorXd &partial_grad_by_times) const;

    void updateSplineReferenceState();
    void decodeDecisionVariables(const Eigen::VectorXd &x,
                                Eigen::MatrixXd &inner_points,
                                Eigen::VectorXd &piece_times,
                                Eigen::MatrixXd &final_state) const;
    void updateTrajectoryFromDecision(const Eigen::VectorXd &x,
                                      TrajectoryAdapter &traj,
                                      Eigen::MatrixXd &inner_points,
                                      Eigen::VectorXd &piece_times,
                                      Eigen::MatrixXd &final_state);

    void Collision_point_Pub();

    Eigen::MatrixXd get_current_iniState(){
        return iniState;
    }
    Eigen::MatrixXd get_current_finState(){
        return finState;
    }
    Eigen::MatrixXd get_current_Innerpoints(){
        return finalInnerpoints;
    }
    Eigen::VectorXd get_current_finalpieceTime(){
        return finalpieceTime;
    }

    Eigen::Vector3d get_current_iniStateXYTheta(){
        return iniStateXYTheta;
    }

    void get_the_predicted_state(const double& time, Eigen::Vector3d& XYTheta, Eigen::Vector3d& VAJ, Eigen::Vector3d& OAJ);

    std::vector<Eigen::Vector3d> get_the_predicted_state_and_path(const double &start_time, const double &time, 
                                                                  const Eigen::Vector3d &start_XYTheta, 
                                                                  Eigen::Vector3d &XYTheta, bool &if_forward);

    inline double normlize_angle(double angle);

    template<typename T>
    bool readParam(const std::string &path, T &param){
        if(!nh_.hasParam(path)){
            ROS_ERROR("ERROR!! cannot read param %s", path.c_str());
            return false;
        }
        nh_.getParam(path, param);
        return true;
    }

    template<typename T>
    bool readParam(const std::string &path, T &param, const T &Default){
        if(!nh_.hasParam(path)){
            param = Default;
            ROS_ERROR("ERROR!! cannot read param %s", path.c_str());
            return false;
        }
        nh_.getParam(path, param);
        return true;
    }

    void pub_inner_init_positions(const std::vector<Eigen::Vector3d> &inner_init_positions);
};



#endif
