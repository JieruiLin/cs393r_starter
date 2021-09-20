//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <cmath>
#include <iostream>
#include <numeric>
#include "car.h"
#include <vector>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace



namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

float Navigation::TOC(float dt, float vel_current, float arc_length, float dist_traveled) 
{
    // Car Parameters
    int const vel_max {1};
    int const acl_max {4};
    int const dcl_max {4};
    
    // ============================================================================
    // if car is STOPPED, ACCELERATING, or DECELERATING
    if (vel_current < vel_max)
    {
        float vel_new = vel_current + (acl_max * dt);          // new velocity if you still need to get to max vel
        //float dist_if_commanded = 0.5*(vel_current + vel_new)*dt;  // hypotethical distance traveled if commanded new velocity
        float dist_left = arc_length - dist_traveled;          // distance left on the free path length
        float dist_to_dcl = pow(vel_new,2)/(2*dcl_max);        // distance needed to decelerate based on new velocity
        // std::cout << "> vel_new: " << vel_new
        //           << "; dist_traveled: " << dist_traveled
        //           << "; dist_left: " << dist_left
        //           << "; dist_to_dcl: " << dist_to_dcl 
        //           << "; dist_if_commanded: " << dist_if_commanded 
        //           << std::endl;
        
        // If distance needed to stop is greater than the 
        // distance left on the curvature arc
        if (dist_to_dcl > dist_left) 
        {
            // set dist needed to decelerate at input velocity
            float dist_to_dcl_current = pow(vel_current,2) / (2*dcl_max);

            // STOP: if the distance needed to stop at the current velocity
            // is greater than the distance needed to get to the end of the arc
            if (dist_to_dcl_current > arc_length)
            {
                return 0;
            }
            // DECELERATE: 
            else
            {
                return vel_current - (dcl_max*dt);
            }
        }

        // ACCELERATE: If the distance needed to stop is less
        // than or equal to the distance left to travel
        else if (dist_to_dcl <= dist_left)
        {
            return vel_current + (acl_max*dt);
        }
        else
            return 0;
    }

    // ===========================================================================
    // if car is at MAX VELOCITY

    else if (vel_current >= vel_max)
    {
        vel_current = vel_max;
        //float dist_if_commanded = vel_max*dt;            // hypotethical distance traveled at constant velocity based on time
        float dist_left = arc_length - dist_traveled;    // distance left on path 
        float dist_to_dcl = pow(vel_max,2)/(2*dcl_max);  // distance needed to decelerate
        // std::cout << "> dist_traveled: " << dist_traveled
        //           << "; dist_left: " << dist_left
        //           << "; dist_to_dcl: " << dist_to_dcl 
        //           << "; dist_if_commanded: " << dist_if_commanded 
        //           << std::endl;
        
        // if distance needed to decelerate is greater 
        // than the arc length, STOP
        if (dist_to_dcl > arc_length)
            return 0;
        
        // if the distance needed to decelerate is greater
        // than the distance left to travel on free path length, intiate deceleration
        else if (dist_to_dcl > dist_left)
            return vel_current - (dcl_max*dt);

        // if the distance is less than or equal to the 
        // distance left, continue at max velocity
        else if (dist_to_dcl <= dist_left)
            return vel_max;
        else
            return 0;
    }
    else
    {
        return 0;
    }
}

Odometry Navigation::LatencyCompensation(float observation_duration_, float actuation_duration_, float dt, float x, float y, float theta, float xdot, float ydot, float omega){

    float previous_observation_time_ = -2.0;
    float system_delay_ = observation_duration_ + actuation_duration_; // predefined durations
    
    Odometry odom_location_;

    odom_location_.x = odom_start_loc_.x(); // TODO : not needed?
    odom_location_.y = odom_start_loc_.y(); // TODO : not needed?
    odom_location_.theta = theta; 

    previous_observation_time_ = ros::Time::now().toSec() - observation_duration_; // May need to add function for ros::Time::now().toSec()
    
    float cutoff_time = previous_observation_time_ - actuation_duration_;
    float record_cutoff_time = ros::Time::now().toSec() - actuation_duration_;

    odom_location_.vx = xdot;
    odom_location_.vy = ydot;
    odom_location_.omega = omega;

    record_motion_.push_back(std::vector<double> {double(xdot), double(ydot), double(omega), ros::Time::now().toSec()});

    Odometry prediction = odom_location_;
    
    if(previous_observation_time_ < 0)
        return odom_location_;
    if(system_delay_ == 0)
        return odom_location_;

    bool input = false;

    for(auto &one_record : record_motion_)
    {
        if(one_record[3] <= cutoff_time){
            one_record.pop_back();
            continue;
        }

        if(one_record[3] >= record_cutoff_time and not input){
            prediction.vx = one_record[0];
            prediction.vy = one_record[1];
            prediction.omega = one_record[2];
            input = true;
        }

        else{
            prediction.x += one_record[0]*dt;
            prediction.y += one_record[1]*dt;
            prediction.omega += one_record[2]*dt;
        }
    }
    return prediction;
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  double t_start = ros::WallTime::now().toSec();
  float dt = 0.05; // Time Step: 20Hz converted to sec
  float dist_traveled = abs(odom_loc_.x() - odom_start_loc_.x());
  
  // MESSAGE DATA
  std::cout << "========"
            << "\n[Time Stamp] " << t_start << "s"
            << "\n[Time Step] " << dt << "s"
            << "\n[Odom Start Location] x: " << odom_start_loc_.x() << "m; y: " << odom_start_loc_.y() << "m"
            << "\n[Odom Start Angle] " << odom_start_angle_ << " rad"
            << "\n[Odom Location] x: " << odom_loc_.x() << "m; y: " << odom_loc_.y() << "m"
            << "\n[Dist Traveled] " << dist_traveled << "m"
            << "\n[Odom Angle] " << odom_angle_ << " rad"
            << "\n[Robot Location] x: " << robot_loc_.x() << "m; y: " << robot_loc_.y() << "m"
            << "\n[Robot Angle] " << robot_angle_ << " rad"
            << "\n[Robot Velocity] dx: " << robot_vel_.x() << "m/s; dy: " << robot_vel_.y() << "m/s"
            << std::endl;

  float arc_length = 4; // Travel 4 m

  float vel_command = TOC(dt, robot_vel_.x(), arc_length, dist_traveled);
  std::cout << "\n vel_command: " << vel_command << std::endl;

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"
  
  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = 0;
  drive_msg_.velocity = vel_command;
  
  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

  //std::cout << "Point: " << point_cloud_.at(0) << std::endl;
}

}  // namespace navigation
