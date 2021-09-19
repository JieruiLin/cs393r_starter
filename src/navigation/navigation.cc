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

// Robot Parameters
const float curvature_max_ = 1/1.0;

// Fake Robot Parameters (TODO)
const float car_width_ 	= 0.2;		// width
const float car_length_ = 0.5;		// length
const float safety_margin_ 	= 0.1;		// padding
const float wheelbase_ 	= 0.3;	// wheelbase
const Vector2f p_min(0, car_width_/2+padding_); //coordinate of the closest point
const Vector2f p_middle((wheelbase_+car_length_)/2 + padding_,  car_width_/2+padding_); //coordinate of the intersection of left and front
const Vector2f p_max((wheelbase_+car_length_)/2 + padding_, -car_width_/2-padding_); //coordinate of the intersection of right and front

// weight in scoring function
free_path_length_weight_ = 1.0
dist_to_goal_weight_ = 1.0

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

Eigen::Vector2f Navigation::BaseLink2Odom(Eigen::Vector2f p) {return odom_loc_ + R_odom2base_ * p;}
Eigen::Vector2f Navigation::Odom2BaseLink(Eigen::Vector2f p) {return R_odom2base_.transpose()*(p - odom_loc_);}

// save point cloud observation to obstacle list
void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  // Transform point cloud observation from local to global
  for (auto &pt_loc_local : cloud){
    Obstacle_list_.push_back(Obstacle {BaseLink2Odom(pt_loc_local), time})
  }                                    
}

// visualize some obstacles
void Navigation::VisObstacles(){
	for (const auto &obs : ObstacleList_)
	{
		visualization::DrawCross(Odom2BaseLink(obs.loc), 0.05, 0x000000, local_viz_msg_);
	}
}

// sample paths with fixed radius interval
void Navigation::samplePaths(float num) {

  Paths_.clear();
  float curve_increment = 2*curvature_max_/num;
  for (int i = 0; i<num; i++){
    float curvature = -curvature_max_ + i*curve_increment;
    // put initialized path option to Paths list
    Paths_.push_back(PathOption {curvature, // curvature
                                          0,		// clearance
                                          0,		// free path length
                                          0,		// distance to goal
                                          0,		// cost
                                          {0,0},	// obstruction point
                                          {0,0},	// closest point
                                          {0,0}});	// end point of wheel base
}

// trim path to not turn away from goal
void Navigation::trimPath(PathOption &path, Vector2f goal)
{
	Vector2f P_rc = {0, 1/path.curvature};	// rotation center

	// works when angle is less than pi/2, need to check if it works otherwise
	float angle = atan((P_rc.y()-goal.y())/goal.x());

	// take the smaller as free path length
	if (abs(angle/path.curvature) < path.free_path_length)
	{
		path.free_path_length = abs(angle/path.curvature);
		// the obstruction point is the intersection of (p_goal, p_rc) and the curvature
		path.obstruction = P_center + 1/path.curvature * (goal - P_center)/(goal - P_center).norm();
	} 
}

// calculate free path length
void Navigation::predictCollisions(PathOption& path){
	float radius = 1/path.curvature;
	Vector2f turning_center(0,radius); // rotation center

	// Initialize obstruction point (without considering obstacles)
	float fpl = abs(M_PI*radius);
	Vector2f p_obstruction(0, 2*radius);
	Vector2f p_end(0, 2*radius)

    // turning radius of different points on the car
	float r_min = (Sign(radius)*turning_center - p_min).norm();	// smallest rotation radius
	float r_middle = (Sign(radius)*turning_center - p_middle).norm();	// front left point rotation radius
	float r_max = (Sign(radius)*turning_center - p_max).norm();	// largest rotation radius

  	// Iterate through points in point cloud
	for (const auto &obs : ObstacleList_){
		// tramsform from global to local
		Vector2f obs_loc = Odom2BaseLink(obs.loc);
		// distance to obstacle from turning center
		float r_obstacle = (turning_center - obs_loc).norm();
		// Check if point will obstruct car
		if (obs_radius > r_min && obs_radius < r_max) {
			Vector2f p_current;
		
			// Inner side collision
			if (obs_radius < r_middle) {
				float beta = acos((Sign(radius)*radius-car_width_/2-padding_)/obs_radius);
			}else if (obs_radius > rdif) {
				// Front side collision
				float beta = asin(((wheelbase_+car_length_)/2+padding_)/obs_radius);
			}

			// calculate alpha + beta
			float sum_alpha_beta = acos(obs_loc[0]/obs_radius);
			float alpha = sum_alpha_beta - beta
			// free path length is the arc length traversed by wheelbase
			float fpl_current = Sign(radius)*alpha*radius;

			// save the smallest fpl
			if (fpl_current < fpl){
				fpl = fpl_current;
				p_obstruction = obs_loc;
				p_end = {cos(alpha)*radius, sin(alpha)*radius}
			}
    	}
  	}
	path.obstruction = p_obstruction;
	path.free_path_length = fpl;
	path.end_point = p_end;
}

// TODO
void Navigation::calculateClearance(PathOption &path){
}

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
// select best path based on scoring function

PathOption Navigation::getBestPath(Vector2f goal_loc)
{
	// Number to tune
	int num_paths = 20;

	// Sample paths
	samplePaths(num_paths);

	PathOption BestPath;
	float min_cost = 1000;

	vector<double> free_path_length_array;
	vector<double> dist_to_goal_array;

	// save the range of each variable for normalize in the scoring function
	float max_free_path_length = 0;
	float min_dist_to_goal = 0;

	for (auto &path : Paths_)
	{
		predictCollisions(path);
		trimPath(path, goal_loc);

		path.dist_to_goal = (path.end_point - goal_loc).norm();

		max_free_path_length = std::max(path.free_path_length, max_free_path_length);
		min_dist_to_goal = std::min(path.dist_to_goal, min_dist_to_goal);

		free_path_length_vec.push_back(path.free_path_length);
		dist_to_goal_vec.push_back(path.dist_to_goal);
	}

	for (int i = 0; i < num_paths; i++)
	{
		float free_path_length = free_path_length_array.at(i);
		float dist_to_goal = dist_to_goal_array.at(i);

		// the longer fpl, the better
		float free_path_length_cost = -(free_path_length/max_free_path_length) * free_path_length_weight_;
		// the smaller dist_to_goal, the better
		float dist_to_goal_cost =  (dist_to_goal/min_dist_to_goal) * dist_to_goal_weight_;

		float cost = free_path_length_cost + clearance_padded_cost + dist_to_goal_cost;
		if (cost < min_cost) {
			min_cost = cost;
			BestPath = PossiblePaths_.at(i);
			BestPath.cost = cost;
		}
	}
	return BestPath;
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
  p2 << (odom_start_loc_.x()+4) - odom_loc_.x(), (odom_start_loc_.y()+1) - odom_loc_.y();

  visualization::DrawCross(p2,0.5,0x3449eb,local_viz_msg_);

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
