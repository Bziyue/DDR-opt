#ifndef _PLAN_MANAGER_HPP_
#define _PLAN_MANAGER_HPP_

#include "plan_env/sdf_map.h"
#include "visualizer/visualizer.hpp"
#include "front_end/jps_planner/jps_planner.h"
#include "geometry_msgs/PoseStamped.h"
#include "back_end/optimizer.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "carstatemsgs/CarState.h"
#include "carstatemsgs/Polynome.h"
#include "std_msgs/Bool.h"

#include <thread>
#include <nav_msgs/Odometry.h>

enum StateMachine{
  INIT,
  IDLE,
  PLANNING,
  REPLAN,
  GOINGTOGOAL,
  EMERGENCY_STOP,
};

class PlanManager
{
  private:
    ros::NodeHandle nh_;

    std::shared_ptr<SDFmap> sdfmap_;
    std::shared_ptr<Visualizer> visualizer_;
    std::shared_ptr<MSPlanner> msplanner_;
    std::shared_ptr<JPS::JPSPlanner> jps_planner_;


    ros::Subscriber goal_sub_;
    ros::Subscriber current_state_sub_;
    ros::Timer main_thread_timer_;
    ros::Publisher cmd_pub_;
    ros::Publisher mpc_polynome_pub_;
    ros::Publisher emergency_stop_pub_;

    ros::Publisher record_pub_;

    ros::Time current_time_;
    Eigen::Vector3d current_state_XYTheta_;
    Eigen::Vector3d current_state_VAJ_;
    Eigen::Vector3d current_state_OAJ_;

    double plan_start_time_;
    Eigen::Vector3d plan_start_state_XYTheta;
    Eigen::Vector3d plan_start_state_VAJ;
    Eigen::Vector3d plan_start_state_OAJ;

    Eigen::Vector3d goal_state_;

    ros::Time Traj_start_time_;
    double Traj_total_time_;

    ros::Time loop_start_time_;

    bool have_geometry_;
    bool have_goal_;

    bool if_fix_final_;
    Eigen::Vector3d final_state_;

    double replan_time_;
    
    double max_replan_time_;

    double predicted_traj_start_time_;

    StateMachine state_machine_ = StateMachine::INIT;

  public:
    PlanManager(ros::NodeHandle nh){
      nh_ = nh;
      
      sdfmap_ = std::make_shared<SDFmap>(nh);
      visualizer_ = std::make_shared<Visualizer>(nh);
      msplanner_ = std::make_shared<MSPlanner>(Config(ros::NodeHandle("~")), nh_, sdfmap_);
      jps_planner_ = std::make_shared<JPS::JPSPlanner>(sdfmap_, nh_);

      goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,&PlanManager::goal_callback,this);
      // current_state_sub_ = nh_.subscribe<carstatemsgs::CarState>("/simulation/PosePub",1,&PlanManager::GeometryCallback,this);
      current_state_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",1,&PlanManager::GeometryCallback,this);
      main_thread_timer_ = nh_.createTimer(ros::Duration(0.001),&PlanManager::MainThread, this);
      cmd_pub_ = nh_.advertise<carstatemsgs::CarState>("/simulation/PoseSub",1);
      emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/planner/emergency_stop",1);

      record_pub_ = nh_.advertise<visualization_msgs::Marker>("/planner/calculator_time",1);

      mpc_polynome_pub_ = nh_.advertise<carstatemsgs::Polynome>("traj", 1);

      have_geometry_ = false;
      have_goal_ = false;

      nh_.param<bool>("if_fix_final", if_fix_final_, false);
      if(if_fix_final_){
        nh_.param<double>("final_x", final_state_(0), 0.0);
        nh_.param<double>("final_y", final_state_(1), 0.0);
        nh_.param<double>("final_yaw", final_state_(2), 0.0);
      }

      nh_.param<double>("replan_time",replan_time_,10000.0);
      nh_.param<double>("max_replan_time", max_replan_time_, 1.0);

      state_machine_ = StateMachine::IDLE;

      loop_start_time_ = ros::Time::now();

    }

    ~PlanManager(){ 
      sdfmap_->~SDFmap();
      visualizer_->~Visualizer();
    }

    void printStateMachine(){
      if(state_machine_ == INIT) ROS_INFO("state_machine_ == INIT");
      if(state_machine_ == IDLE) ROS_INFO("state_machine_ == IDLE");
      if(state_machine_ == PLANNING) ROS_INFO("state_machine_ == PLANNING");
      if(state_machine_ == REPLAN) ROS_INFO("state_machine_ == REPLAN");
    }

    // void GeometryCallback(const carstatemsgs::CarState::ConstPtr &msg){
    //   have_geometry_ = true;
    //   current_state_XYTheta_ << msg->x, msg->y, msg->yaw;
    //   current_state_VAJ_ << msg->v, msg->a, msg->js;
    //   current_state_OAJ_ << msg->omega, msg->alpha, msg->jyaw;
    //   current_time_ = msg->Header.stamp;
    // }

    void GeometryCallback(const nav_msgs::Odometry::ConstPtr &msg){
      have_geometry_ = true;
      current_state_XYTheta_ << msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation);
      current_state_VAJ_ << 0.0, 0.0, 0.0;
      current_state_OAJ_ << 0.0, 0.0, 0.0;
      current_time_ = msg->header.stamp;
    }


    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
      // Ignore the given goal at runtime, commenting out this check may cause unexpected bugs
      // Especially when there is no re-planning
      if(state_machine_ != StateMachine::IDLE){
        ROS_ERROR("Haven't reached the goal yet!!");
        return;
      }
      ROS_INFO("\n\n\n\n\n\n\n\n");
      ROS_INFO("---------------------------------------------------------------");
      ROS_INFO("---------------------------------------------------------------");

      ROS_INFO("get goal!");
      state_machine_ = StateMachine::IDLE;
      have_goal_ = true;
      goal_state_<<msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation);
      if(if_fix_final_) goal_state_ = final_state_;
      ROS_INFO_STREAM("goal state: " << goal_state_.transpose());

      ROS_INFO("---------------------------------------------------------------");
      ROS_INFO("---------------------------------------------------------------");
      ROS_INFO("\n\n\n\n\n\n\n\n");
    
    }

    void MainThread(const ros::TimerEvent& event){
      
      if(!have_geometry_ || !have_goal_) return;

      // collision check
      if(have_geometry_){
        if(sdfmap_->getDistanceReal(Eigen::Vector2d(current_state_XYTheta_.x(), current_state_XYTheta_.y())) < 0.0){
          std_msgs::Bool emergency_stop;
          emergency_stop.data = true;
          emergency_stop_pub_.publish(emergency_stop);
          state_machine_ = EMERGENCY_STOP;
          ROS_INFO_STREAM("current_state_XYTheta_: " << current_state_XYTheta_.transpose());
          ROS_INFO_STREAM("Dis: " << sdfmap_->getDistanceReal(Eigen::Vector2d(current_state_XYTheta_.x(), current_state_XYTheta_.y())));
          ROS_ERROR("EMERGENCY_STOP!!! too close to obstacle!!!");
          return;
        }
      }
      
      if(state_machine_ == StateMachine::IDLE || 
          ((state_machine_ == StateMachine::PLANNING||state_machine_ == StateMachine::REPLAN) 
            && (ros::Time::now() - loop_start_time_).toSec() > replan_time_)){
        loop_start_time_ = ros::Time::now();
        double current = loop_start_time_.toSec();
        // start new plan
        if(state_machine_ == StateMachine::IDLE){
          state_machine_ = StateMachine::PLANNING;
          plan_start_time_ = -1;
          predicted_traj_start_time_ = -1;
          plan_start_state_XYTheta = current_state_XYTheta_;
          plan_start_state_VAJ = current_state_VAJ_;
          plan_start_state_OAJ = current_state_OAJ_;
        } 
        // Use predicted distance for replanning in planning state
        else if(state_machine_ == StateMachine::PLANNING || state_machine_ == StateMachine::REPLAN){
          
          if(((current_state_XYTheta_ - goal_state_).head(2).squaredNorm() + fmod(fabs((plan_start_state_XYTheta - goal_state_)[2]), 2.0 * M_PI)*0.02 < 1.0) ||
             msplanner_->final_traj_.getTotalDuration() < max_replan_time_){
            state_machine_ = StateMachine::GOINGTOGOAL;
            return;
          }

          state_machine_ = StateMachine::REPLAN;

          predicted_traj_start_time_ = current + max_replan_time_ - plan_start_time_;
          msplanner_->get_the_predicted_state(predicted_traj_start_time_, plan_start_state_XYTheta, plan_start_state_VAJ, plan_start_state_OAJ);

        } 
        
        ROS_INFO("\033[32;40m \n\n\n\n\n-------------------------------------start new plan------------------------------------------ \033[0m");
        
        visualizer_->finalnodePub(plan_start_state_XYTheta, goal_state_);
        ROS_INFO("init_state_: %.10f  %.10f  %.10f", plan_start_state_XYTheta(0), plan_start_state_XYTheta(1), plan_start_state_XYTheta(2));
        ROS_INFO("goal_state_: %.10f  %.10f  %.10f", goal_state_(0), goal_state_(1), goal_state_(2));
        std::cout<<"<arg name=\"start_x_\" value=\""<< plan_start_state_XYTheta(0) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"start_y_\" value=\""<< plan_start_state_XYTheta(1) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"start_yaw_\" value=\""<< plan_start_state_XYTheta(2) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"final_x_\" value=\""<< goal_state_(0) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"final_y_\" value=\""<< goal_state_(1) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"final_yaw_\" value=\""<< goal_state_(2) <<"\"/>"<<std::endl;

        std::cout<<"plan_start_state_VAJ: "<<plan_start_state_VAJ.transpose()<<std::endl;
        std::cout<<"plan_start_state_OAJ: "<<plan_start_state_OAJ.transpose()<<std::endl;

        ROS_INFO("<arg name=\"start_x_\" value=\"%f\"/>", plan_start_state_XYTheta(0));
        ROS_INFO("<arg name=\"start_y_\" value=\"%f\"/>", plan_start_state_XYTheta(1));
        ROS_INFO("<arg name=\"start_yaw_\" value=\"%f\"/>", plan_start_state_XYTheta(2));
        ROS_INFO("<arg name=\"final_x_\" value=\"%f\"/>", goal_state_(0));
        ROS_INFO("<arg name=\"final_y_\" value=\"%f\"/>", goal_state_(1));
        ROS_INFO("<arg name=\"final_yaw_\" value=\"%f\"/>", goal_state_(2));

        ROS_INFO_STREAM("plan_start_state_VAJ: " << plan_start_state_VAJ.transpose());
        ROS_INFO_STREAM("plan_start_state_OAJ: " << plan_start_state_OAJ.transpose());

        // front end
        ros::Time astar_start_time = ros::Time::now();
        if(!findJPSRoad()){
          state_machine_ = EMERGENCY_STOP;
          ROS_ERROR("EMERGENCY_STOP!!! can not find astar road !!!");
          return;
        }
        ROS_INFO("\033[41;37m all of front end time:%f \033[0m", (ros::Time::now()-astar_start_time).toSec());

        // optimizer
        bool result = msplanner_->spline_plan(jps_planner_->flat_traj_);
        if(!result){
          return;
        }

        ROS_INFO("\033[43;32m all of plan time:%f \033[0m", (ros::Time::now().toSec()-current));

        // visualization
        msplanner_->trajectoryPathPub(msplanner_->final_traj_, plan_start_state_XYTheta, visualizer_->trajPathPub_);
        msplanner_->trajectoryPointPub(msplanner_->final_traj_, plan_start_state_XYTheta, visualizer_->trajPointMarker_, Eigen::Vector3d(239, 41, 41));
        
        // for replan
        if(plan_start_time_ < 0){
          Traj_start_time_ = ros::Time::now();
          plan_start_time_ = Traj_start_time_.toSec();
        }
        else{
          plan_start_time_ = current + max_replan_time_;
          Traj_start_time_ = ros::Time(plan_start_time_);
        }
        

        MPCPathPub(plan_start_time_);

        Traj_total_time_ = msplanner_->final_traj_.getTotalDuration();
      }

      if((ros::Time::now() - Traj_start_time_).toSec() >= Traj_total_time_){
        state_machine_ = StateMachine::IDLE;
        have_goal_ = false;
      }

    }

    bool findJPSRoad(){

      ros::Time current = ros::Time::now();
      Eigen::Vector3d start_state;
      std::vector<Eigen::Vector3d> start_path;
      std::vector<Eigen::Vector3d> start_path_both_end;
      bool if_forward = true;
      if(plan_start_time_ > 0){
        start_path = msplanner_->get_the_predicted_state_and_path(predicted_traj_start_time_, predicted_traj_start_time_ + jps_planner_->jps_truncation_time_, plan_start_state_XYTheta, start_state, if_forward);
        u_int start_path_size = start_path.size();
        u_int start_path_i = 0;
        for(; start_path_i < start_path_size; start_path_i++){
          if(!jps_planner_->JPS_check_if_collision(start_path[start_path_i].head(2)))
            break;
        }
        if(start_path_i == 0){
          start_state = plan_start_state_XYTheta;
          start_path_both_end.push_back(start_path.front());
          start_path_both_end.push_back(start_state);
        }
        else if(start_path_i < start_path_size){
          start_path = std::vector<Eigen::Vector3d>(start_path.begin(), start_path.begin() + start_path_i);
          start_state = start_path.back();
          start_path_both_end.push_back(start_path.front());
          start_path_both_end.push_back(start_state);
        }
        else{
          start_path_both_end.push_back(start_path.front());
          start_path_both_end.push_back(start_state);
        }
      }
      else{
        start_state = plan_start_state_XYTheta;
      }

      jps_planner_->plan(start_state, goal_state_);
      
      jps_planner_->getKinoNodeWithStartPath(start_path, if_forward, plan_start_state_VAJ, plan_start_state_OAJ);


      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "jps_planner";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 11;
      marker.pose.position.y = 8;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 0.5;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      double search_time = (ros::Time::now()-current).toSec() * 1000.0;
      std::ostringstream out;
      out << std::fixed <<"JPS: \n"<< std::setprecision(2) << search_time<<" ms";
      marker.text = out.str();
      record_pub_.publish(marker);


      ROS_INFO("\033[40;36m jps_planner_ search time:%lf  \033[0m", (ros::Time::now()-current).toSec());

      return true;
    }

    void MPCPathPub(const double& traj_start_time){
      Eigen::MatrixXd initstate = msplanner_->get_current_iniState();
      Eigen::MatrixXd finState = msplanner_->get_current_finState();
      Eigen::MatrixXd finalInnerpoints = msplanner_->get_current_Innerpoints();
      Eigen::VectorXd finalpieceTime = msplanner_->get_current_finalpieceTime();
      Eigen::Vector3d iniStateXYTheta = msplanner_->get_current_iniStateXYTheta();

      carstatemsgs::Polynome polynome;
      polynome.header.frame_id = "world";
      polynome.header.stamp = ros::Time::now();
      polynome.init_p.x = initstate.col(0).x();
      polynome.init_p.y = initstate.col(0).y();
      polynome.init_v.x = initstate.col(1).x();
      polynome.init_v.y = initstate.col(1).y();
      polynome.init_a.x = initstate.col(2).x();
      polynome.init_a.y = initstate.col(2).y();
      polynome.tail_p.x = finState.col(0).x();
      polynome.tail_p.y = finState.col(0).y();
      polynome.tail_v.x = finState.col(1).x();
      polynome.tail_v.y = finState.col(1).y();
      polynome.tail_a.x = finState.col(2).x();
      polynome.tail_a.y = finState.col(2).y();

      if(plan_start_time_ < 0) polynome.traj_start_time = ros::Time::now();
      else polynome.traj_start_time = ros::Time(plan_start_time_);

      for(u_int i=0; i<finalInnerpoints.cols(); i++){
        geometry_msgs::Vector3 point;
        point.x = finalInnerpoints.col(i).x();
        point.y = finalInnerpoints.col(i).y();
        point.z = 0.0;
        polynome.innerpoints.push_back(point);
      }
      for(u_int i=0; i<finalpieceTime.size(); i++){
        polynome.t_pts.push_back(finalpieceTime[i]);
      }
      polynome.start_position.x = iniStateXYTheta.x();
      polynome.start_position.y = iniStateXYTheta.y();
      polynome.start_position.z = iniStateXYTheta.z();

      if(!msplanner_->if_standard_diff_){
        polynome.ICR.x = msplanner_->ICR_.x();
        polynome.ICR.y = msplanner_->ICR_.y();
        polynome.ICR.z = msplanner_->ICR_.z();
      }
      
      mpc_polynome_pub_.publish(polynome);


      
    }

};


#endif
