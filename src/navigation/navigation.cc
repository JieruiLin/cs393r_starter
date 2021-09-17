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

//jerry
// Fake Robot Parameters
const float car_width_ 	= 0.27;		// width
const float car_length_ = 0.5;		// length
const float padding_ 	= 0.1;		// padding
const float wheelbase_ 	= 0.324;	// wheelbase
const Vector2f p_min(0, car_width_/2+padding_);
const Vector2f p_middle((wheelbase_+car_length_)/2 + padding_,  car_width_/2+padding_);
const Vector2f p_max((wheelbase_+car_length_)/2 + padding_, -car_width_/2-padding_);


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

// jerry
void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  // Transform point cloud observation from local to global
  for (auto &pt_loc_local : cloud)
  {
    Obstacle_list_.push_back(Obstacle {BaseLink2Odom(pt_loc_local), time})
  }                                    
}

// jerry
void Navigation::samplePaths(float num) {
  SamplePaths_.clear();
  float curve_increment = 2*curvature_max_/num;
  for (int i = 0; i<num; i++){
    float curvature = -curvature_max_ + i*curve_increment;
    // Enforce max radius of 1km (any bigger and the angles get so small the math is bad)
		// if (std::abs(curvature) < 0.001) curvature = 0.001;
    PossiblePaths_.push_back(PathOption {curvature, // curvature
                                          0,			// clearance
                                          0,			// free path length
                                          0,			// distance to goal
                                          0,			// cost
                                          {0,0},		// obstruction location
                                          {0,0},		// closest point location
                                          {0,0}});	// end point of the movement
  }
}

// jerry: trim path length to not turn away from goal
void Navigation::trimPathLength(PathOption &path, Vector2f goal)
{
	// NOTE: Defined in the Base Link frame
	Vector2f P_center = {0, 1/path.curvature};	// Center of rotation

	float angle = getAngleBetween(goal, {0,0}, P_center);
	if (goal[0] < 0) angle = 2*M_PI - angle; 	// negative x means angle > 180°

	// If trimmed path length is less than free path length, substitute in the trimmed value
	if (abs(angle/path.curvature) < path.free_path_length)
	{
		path.free_path_length = abs(angle/path.curvature);
		path.obstruction = P_center + 1/path.curvature * (goal - P_center)/(goal - P_center).norm();
	} 
}

// jerry: calculate free path length
void Navigation::predictCollisions(PathOption& path){
  float radius = 1/path.curvature;
  Vector2f turning_center(0,radius); // rotation center
  float obs_radius = (turning_center - obs_loc).norm();

  // turning radius of different points on the car
	float rmin = (Sign(radius)*turning_center - p_min).norm();	// radius from center of rotation to innermost point on the rear axle
	float rdif = (Sign(radius)*turning_center - p_middle).norm();	// radius from center of rotation to the turning corner of the robot
	float rmax = (Sign(radius)*turning_center - p_max).norm();	// radius from center of rotation to the outermost point on the robot

  // Iterate through points in point cloud
	for (const auto &obs : ObstacleList_){
    // tramsform from global to local
    Vector2f obs_loc = Odom2BaseLink(obs.loc);
    // distance to obstacle from turning center
    float r_obstacle = (turning_center - obs_loc).norm();
    // Check if point will obstruct car
		if (obs_radius > rmin && obs_radius < rmax) {
      Vector2f p_current;
			
			// Inner side collision
			if (obs_radius < rdif) {
        // cos(beta) = A_0_y/obs_radius
				float beta = acos(( Sign(radius)*radius-car_width_/2-padding_)/obs_radius );
				//float x = obs_radius*sin(phi);
				//p_current = {x, Sign(radius)*(car_width_/2+padding_)};
			
			// Front collision
			}else if (obs_radius > rdif) {
        // sin(beta) = ((b +l)/2 + m)/obs_radius
				float beta = asin(((wheelbase_+car_length_)/2+padding_)/obs_radius);
				//float y = radius - Sign(radius)*obs_radius*cos(phi);
				//p_current = {(wheelbase_+car_length_)/2+padding_, y};
			}

      // calculate alpha + beta
      float sum_alpha_beta = acos(obs_loc[0]/obs_radius);
			float alpha = sum_alpha_beta - beta
			// Solve for free path length of the current point
			float fpl_current = Sign(radius)*alpha*radius;

			// If this is the first loop or this is the smallest fpl so far, record this value
			if (fpl_current < fpl_min){
				fpl_min = fpl_current;
				p_obstruction = obs_loc;
			}
    }
  }
  // Save results to path struct
	path.obstruction = p_obstruction;
	path.free_path_length = fpl_min;
}

// jerry
// New base_link frame clearance calc
void Navigation::calculateClearance(PathOption &path){
	// Warning: These can be negative
	float radius = 1/path.curvature;
	float theta = path.free_path_length/radius;

	// Look 3 car lengths ahead
	float look_ahead_dist = 3*car_length_;

	// Get start and end points
	Vector2f center(0,radius);
	Vector2f start_point(0,0);
	Vector2f end_point(0,0);
	end_point.x() = radius*sin(theta) + look_ahead_dist*cos(theta);
	end_point.y() = radius*(1-cos(theta)) + look_ahead_dist*sin(theta);

	// Flip points for isBetween turning direction dependency
	Vector2f point_1 = (radius > 0 ? start_point : end_point);
	Vector2f point_2 = (radius > 0 ? end_point : start_point);

	// Initialize clearance at its maximum allowed value
	float min_clearance = clearance_limit_;

	// Get angle between start and end
	Vector2f closest_point(0,0);
	for (const auto &obs : ObstacleList_)
	{
		Vector2f obs_point = Odom2BaseLink(obs.loc);

		if (isBetween(center, point_1, point_2, obs_point))
		{
			float radius_to_point = (center-obs_point).norm();
			float clearance = abs(radius_to_point - abs(radius));
			if (clearance < min_clearance)
			{
				min_clearance = clearance;
				closest_point = obs_point;
			}
		}
	}
	path.clearance = min_clearance;
	path.closest_point = closest_point;
}

// jerry
PathOption Navigation::getGreedyPath(Vector2f goal_loc)
{
	int num_paths = 20;

	// Clear out possible paths and reinitialize
	samplePaths(num_paths);

	// Initialize output and cost
	PathOption BestPath;
	float min_cost = 1e10;

	// Vectors to store results
	vector<double> free_path_length_vec;
	//vector<double> clearance_padded_vec;
	vector<double> distance_to_goal_vec;

	// Get best parameter from all possible paths for normalization
	float max_free_path_length = 1e-5;
	float max_clearance_padded = 1e-5;
	float min_distance_to_goal = 1e5;

	for (auto &path : PossiblePaths_)
	{
		// Update FLP, Clearance, Closest Point, Obstruction, End Point
		predictCollisions(path);
		trimPathLength(path, goal_loc);
		calculateClearance(path);
		float clearance_padded = path.clearance - (car_width_/2+padding_*2);
		if (clearance_padded < 0) clearance_padded = 1e-5;

		path.distance_to_goal = (path.obstruction - goal_loc).norm(); // approximately

		max_free_path_length = std::max(path.free_path_length, max_free_path_length);
		max_clearance_padded = std::max(clearance_padded, max_clearance_padded);
		min_distance_to_goal = std::min(path.distance_to_goal, min_distance_to_goal);

		free_path_length_vec.push_back(path.free_path_length);
		clearance_padded_vec.push_back(clearance_padded);
		distance_to_goal_vec.push_back(path.distance_to_goal);
	}

	// Iterate through paths to find the best one
	for (int i = 0; i < num_paths; i++)
	{
		float free_path_length = free_path_length_vec.at(i);
		float clearance_padded = clearance_padded_vec.at(i);
		float distance_to_goal = distance_to_goal_vec.at(i);

		// Decrease cost with larger free path length
		float free_path_length_cost = -(free_path_length/max_free_path_length) * free_path_length_weight_;
		// Increase cost with more 1/clearance (normalized and padded)
		float clearance_padded_cost	=  (max_clearance_padded/clearance_padded) * clearance_weight_;
		// Increase cost with larger distance to goal
		float distance_to_goal_cost =  (distance_to_goal/min_distance_to_goal) * distance_to_goal_weight_;

		float cost = free_path_length_cost + clearance_padded_cost + distance_to_goal_cost;
		if (cost < min_cost) {
			min_cost = cost;
			BestPath = PossiblePaths_.at(i);
			BestPath.cost = cost;
		}
	}
	return BestPath;
}

void Navigation::showObstacles()
{
	int i = 0;
	int cutoff_count = ObstacleList_.size()/1000;
	for (const auto &obs : ObstacleList_)
	{
		if (i != cutoff_count) {i++; continue;} // ensure that no more than 1000 obstacles are displayed
		visualization::DrawCross(Odom2BaseLink(obs.loc), 0.05, 0x000000, local_viz_msg_);
		i = 0;
	}
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  if  (init_){
		while (init_ and ros::ok()){
			ros::spinOnce();
			ros::Rate(10).sleep();
		}
		goal_vector_ = Vector2f(4,0);
		time_prev_ = ros::Time::now();
	}

  showObstacles();
	PathOption BestPath = getGreedyPath(goal_vector_);
	moveAlongPath(BestPath);
	printPathDetails(BestPath);
	plotPathDetails(BestPath);

  // If we have reached our goal we can stop (not relevant for dynamic goal)
	float dist_to_goal = (odom_loc_-goal_vector_).norm();
	float current_speed = robot_vel_.norm();
	if (current_speed > 2.0) current_speed = 0; // disregards initial infinite velocity
	float stopping_dist = 0.1+-0.5*current_speed*current_speed/min_accel_;
	if (dist_to_goal <= stopping_dist){
		navigation_success_ = true;
		ROS_WARN("Navigation success!");
	}

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
