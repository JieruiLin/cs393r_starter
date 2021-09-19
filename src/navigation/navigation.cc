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
#include <chrono>
#include <math.h> 

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

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

// float Navigation::TOC(float dt, float vel_current, float arc_length, float dist_traveled) 
// {
//     // Car Parameters
//     int const vel_max {1};
//     int const acl_max {4};
//     int const dcl_max {4};
    
//     // ============================================================================
//     // if car is STOPPED, ACCELERATING, or DECELERATING
//     if (vel_current < vel_max)
//     {
//         float vel_new = vel_current + (acl_max * dt);          // new velocity if you still need to get to max vel
//         //float dist_if_commanded = 0.5*(vel_current + vel_new)*dt;  // hypotethical distance traveled if commanded new velocity
//         float dist_left = arc_length - dist_traveled;          // distance left on the free path length
//         float dist_to_dcl = pow(vel_new,2)/(2*dcl_max);        // distance needed to decelerate based on new velocity
//         // std::cout << "> vel_new: " << vel_new
//         //           << "; dist_traveled: " << dist_traveled
//         //           << "; dist_left: " << dist_left
//         //           << "; dist_to_dcl: " << dist_to_dcl 
//         //           << "; dist_if_commanded: " << dist_if_commanded 
//         //           << std::endl;
        
//         // If distance needed to stop is greater than the 
//         // distance left on the curvature arc
//         if (dist_to_dcl > dist_left) 
//         {
//             // set dist needed to decelerate at input velocity
//             float dist_to_dcl_current = pow(vel_current,2) / (2*dcl_max);

//             // STOP: if the distance needed to stop at the current velocity
//             // is greater than the distance needed to get to the end of the arc
//             if (dist_to_dcl_current > arc_length)
//             {
//                 return 0;
//             }
//             // DECELERATE: 
//             else
//             {
//                 return vel_current - (dcl_max*dt);
//             }
//         }

//         // ACCELERATE: If the distance needed to stop is less
//         // than or equal to the distance left to travel
//         else if (dist_to_dcl <= dist_left)
//         {
//             return vel_current + (acl_max*dt);
//         }
//         else
//             return 0;
//     }

//     // ===========================================================================
//     // if car is at MAX VELOCITY

//     else if (vel_current >= vel_max)
//     {
//         vel_current = vel_max;
//         //float dist_if_commanded = vel_max*dt;            // hypotethical distance traveled at constant velocity based on time
//         float dist_left = arc_length - dist_traveled;    // distance left on path 
//         float dist_to_dcl = pow(vel_max,2)/(2*dcl_max);  // distance needed to decelerate
//         // std::cout << "> dist_traveled: " << dist_traveled
//         //           << "; dist_left: " << dist_left
//         //           << "; dist_to_dcl: " << dist_to_dcl 
//         //           << "; dist_if_commanded: " << dist_if_commanded 
//         //           << std::endl;
        
//         // if distance needed to decelerate is greater 
//         // than the arc length, STOP
//         if (dist_to_dcl > arc_length)
//             return 0;
        
//         // if the distance needed to decelerate is greater
//         // than the distance left to travel on free path length, intiate deceleration
//         else if (dist_to_dcl > dist_left)
//             return vel_current - (dcl_max*dt);

//         // if the distance is less than or equal to the 
//         // distance left, continue at max velocity
//         else if (dist_to_dcl <= dist_left)
//             return vel_max;
//         else
//             return 0;
//     }
//     else
//     {
//         return 0;
//     }
// }

double arc_radius(double p1x, double p1y, double p2x, double p2y)
{
  double a = p2y - p1y;
  //std::cout << a << std::endl;
  double b = p2x - p1x;
  //std::cout << b << std::endl;
  if (a == 0)
  {
    //std::cout << "a: " << a << std::endl;
    return a;
  }
  else if (b == 0)
  {
    //std::cout << "b: " << b << std::endl;
    return b;
  }
  else
  {
  double F = sqrt(pow(b,2)+pow(a,2));
  //std::cout << "F: " << F << std::endl;
  double rho = asin(b/F) - asin(a/F);
  //std::cout << "rho: " << rho << std::endl;
  double r = -(a/(sin(rho)-1));
  //std::cout << "r: " << r << std::endl;
  return r;
  }
}

double arc_angle(double p1x, double p1y, double p2x, double p2y)
{
  double b = p2x - p1x;
  //std::cout << "b2: " << b << std::endl;
  double r = arc_radius(p1x, p1y, p2x, p2y);
  //std::cout << "r2: " << r << std::endl;
  double angle = asin(b/r);
  //std::cout << "angle: " << angle << std::endl;
  return angle;
}

double arc_length(double p1x, double p1y, double p2x, double p2y)
{
  double r = arc_radius(p1x, p1y, p2x, p2y);
  double phi = arc_angle(p1x, p1y, p2x, p2y);
  //std::cout << "phi: " << phi << std::endl;
  return r*phi;
}


void Navigation::Run() {

  //auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  //auto sec_since_epoch = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
  //std::cout << "millisec_since_epoch: " << millisec_since_epoch
  //          << "\nsec_since_epoch: " << sec_since_epoch
  //          << std::endl;

  // This function gets called 20 times a second to form the control loop.

  ros::Time t_start = ros::Time::now();
  double dt = 0.05; // Time Step: 20Hz converted to sec
  double dist_traveled = abs(odom_loc_.norm() - odom_start_loc_.norm());
  
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


  double p1x = 0;
  double p1y = 0;
  float p2x = 4.00;
  float p2y = 1.00;

  double arc_l = arc_length(p1x, p1y, p2x, p2y);
  double r = arc_radius(p1x, p1y, p2x, p2y);

  double vel_command =  car_.TOC(dt, robot_vel_.norm(), arc_l, dist_traveled);
  std::cout << "arc_length: " << arc_l 
            << "\n vel_command: " << vel_command << std::endl;

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"
  
  // Eventually, you will have to set the control values to issue drive commands:

  Vector2f p2;
  p2 << 4, 1;

  visualization::DrawCross(p2,0.5,0x3449eb,global_viz_msg_);

  drive_msg_.curvature = 1/r;
  drive_msg_.velocity = vel_command;
  
  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

  // double end_time = std::chrono::system_clock::now();
  // double time_elapsed = start_time - end_time;
  // std::cout << time_elapsed << std::endl;
}

}  // namespace navigation
