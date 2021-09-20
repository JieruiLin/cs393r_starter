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

// Robot Parameters
const float curvature_max_ = 1/1.0;

// Fake Robot Parameters (TODO)
const float car_width_ 	= 0.2794;		    // width (11 inches)
const float car_length_ = 0.5334;		    // length (21 inches)
const float safety_margin_ 	= 0.1524;		// margin of safety (6inches)
const float wheelbase_ 	= 0.3302;	      // wheelbase (13 inches)
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
		if (r_obstacle > r_min && r_obstacle < r_max) {
			Vector2f p_current;
		
			// Inner side collision
			if (r_obstacle < r_middle) {
				float beta = acos((Sign(radius)*radius-car_width_/2-padding_)/r_obstacle);
			}else if (r_obstacle > rdif) {
				// Front side collision
				float beta = asin(((wheelbase_+car_length_)/2+padding_)/r_obstacle);
			}

			// calculate alpha + beta
			float sum_alpha_beta = acos(obs_loc[0]/r_obstacle);
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

// clearance is defined as the minimum distance from any point on the free path length to
void Navigation::calculateClearance(PathOption &path){
	float radius = 1/path.curvature;
	float alpha = path.free_path_length * path.curvature
	float min_clearance = 1000

	Vector2f turning_center(0,radius);
	Vector2f closest_point(0,0);
	for (const auto &obs : ObstacleList_){
		Vector2f obs_point = Odom2BaseLink(obs.loc);
		float r_obstacle = (turning_center - obs_loc).norm();
		// check if obstacle is inside the cone defined by RC, start location and final location
		float angle_obstacle = acos(obs_point.x()/r_obstacle);
		if (angle < alpha){
			float clearance = abs((turning_center-obs_point).norm() - abs(radius));
			if (clearance < min_clearance){
				min_clearance = clearance;
				closest_point = obs_point;
			}
		}
	}
	path.clearance = min_clearance;
	path.closest_point = closest_points;
}

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
	vector<double> clearance_array;

	// save the range of each variable for normalize in the scoring function
	float max_free_path_length = 0;
	float min_dist_to_goal = 0;
	float max_clearance = 0

	for (auto &path : Paths_)
	{
		predictCollisions(path);
		trimPath(path, goal_loc);
		calculateClearance(path);

		path.dist_to_goal = (path.end_point - goal_loc).norm();

		max_free_path_length = std::max(path.free_path_length, max_free_path_length);
		min_dist_to_goal = std::min(path.dist_to_goal, min_dist_to_goal);
		max_clearance = td::max(path.clearance, max_clearance);

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
	// This function gets called 20 times a second to form the control loop.
	
	// Clear previous visualizations.
	visualization::ClearVisualizationMsg(local_viz_msg_);
	visualization::ClearVisualizationMsg(global_viz_msg_);

	// If odometry has not been initialized, we can't do anything.
	if (!odom_initialized_) return;

	// get best path to take
	PathOption BestPath = getBestPath(goal_vector_);

	viz_pub_.publish(local_viz_msg_);
	viz_pub_.publish(global_viz_msg_);

  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = 0.0;
  drive_msg_.velocity = 1.0;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
