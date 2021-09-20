
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
#include <vector>


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
const float safety_margin_ 	= 0.1;		// margin of safety
const float wheelbase_ 	= 0.3;	// wheelbase
const Vector2f p_min(0, car_width_/2+safety_margin_); //coordinate of the closest point
const Vector2f p_middle((wheelbase_+car_length_)/2 + safety_margin_,  car_width_/2+safety_margin_); //coordinate of the intersection of left and front
const Vector2f p_max((wheelbase_+car_length_)/2 + safety_margin_, -car_width_/2-safety_margin_); //coordinate of the intersection of right and front
const Vector2f P_center(0,0);

// weight in scoring function
float free_path_length_weight_ = 1.0;
float dist_to_goal_weight_ = 1.0;



namespace navigation {

// Provided Method
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

// Provided Method
void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

// Provided Method
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

  // Added Rotation Matrix
  R_odom2base_ << cos(odom_angle_), -sin(odom_angle_),
                  sin(odom_angle_), cos(odom_angle_);
}

// Conversions from Base Links to Odometry
Eigen::Vector2f Navigation::BaseLink2Odom(Eigen::Vector2f p) 
{
  return odom_loc_ + R_odom2base_ * p;
}

Eigen::Vector2f Navigation::Odom2BaseLink(Eigen::Vector2f p) 
{
  return R_odom2base_.transpose()*(p - odom_loc_);
}

// save point cloud observation to obstacle list
void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  // Transform point cloud observation from local to global
  for (auto &pt_loc_local : cloud){
    ObstacleList_.push_back(Obstacle {BaseLink2Odom(pt_loc_local), time});
    //visualization::DrawCross(pt_loc_local,1,0xeb4034,local_viz_msg_);
  }                                
}


// visualize some obstacles
void Navigation::VisObstacles(){
	for (const auto &obs : ObstacleList_)
	{
		visualization::DrawCross(Odom2BaseLink(obs.loc), 0.5, 0x3449eb, local_viz_msg_);
	}
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

// sample paths with fixed radius interval
void Navigation::samplePaths(float num) {

  Paths_.clear();
  float curve_increment = 2*curvature_max_/num;
  for (int i = 0; i<num; i++){
    float curvature = -curvature_max_ + i*curve_increment;
    // put initialized path option to Paths list
    Paths_.push_back(PathOption {curvature, // curvature
                                          0,		          // clearance
                                          0,		          // free path length
                                          0,		          // distance to goal
                                          0,		          // cost
                                          0,              // alpha
                                          0,              // beta
                                          1/curvature,    // radius
                                          {0,0},	        // obstruction point
                                          {0,0},	        // closest point
                                          {0,0}});	      // end point of wheel base
}
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
void Navigation::predictCollisions(PathOption& path)
{
  //std::cout << "======= predictCollisions =======" << std::endl;
  
  // Initializers
  float alpha {M_PI};
  float fpl_current {0};

  // Obstruction point (without considering obstacles)
  float fpl = abs(M_PI*path.radius);
  float best_alpha {0};
  float best_beta {0};
  Vector2f p_obstruction(0, 2*path.radius);
  Vector2f p_end(0, 2*path.radius);

  // If straight line:
  if (path.curvature == 0)
  {
    //std::cout << "Entered Straight Line Calcs" << std::endl;

    for (auto &obs:ObstacleList_)
    {
      if (obs.loc.x() >= p_middle.x() && obs.loc.y() < p_middle.y() && obs.loc.y() > p_max.y())
      {
        Vector2f obs_loc = obs.loc;
        fpl_current = obs.loc.x() - p_middle.x();
        
        // save the smallest fpl
        if (fpl_current < fpl)
        {
          fpl = fpl_current;
          p_obstruction = obs_loc;
          p_end << fpl_current, 0;
        }
      }
    }

    path.obstruction = p_obstruction;
    path.free_path_length = std::min(fpl, path.dist_to_goal);
    path.end_point = p_end;
    path.alpha = 0;
    path.radius = INFINITY;
  }
  else 
  { 
    //std::cout << "Entered FPL Arc Calcs" << std::endl;
	float radius = 1/path.curvature;
  
  // If turing right
  if (radius < 0) 
  {
    for (auto &obs:ObstacleList_)
    {
      obs.loc.y() = -obs.loc.y();
      radius = -radius;
    }
  }

	Vector2f turning_center(0,radius); // rotation center

  // turning radius of different points on the car
	float r_min = (turning_center - p_min).norm();	// smallest rotation radius
	float r_middle = (turning_center - p_middle).norm();	// front left point rotation radius
	float r_max = (turning_center - p_max).norm();	// largest rotation radius

  // Iterate through points in point cloud
	for (const auto &obs : ObstacleList_)
  {
		// tramsform from global to local
		Vector2f obs_loc = Odom2BaseLink(obs.loc);

		// distance to obstacle from turning center
		float r_obstacle = (turning_center - obs_loc).norm();  

		// Check if point will obstruct car
		if (r_obstacle > r_min && r_obstacle < r_max) 
    {
			Vector2f p_current;
		  float beta {0};

			// Inner side collision
			if (r_obstacle < r_middle) 
      {
				beta = acos((radius-car_width_/2-safety_margin_)/r_obstacle);
			}
      else if (r_obstacle > r_middle) 
      {
				// Front side collision
				beta = asin(((wheelbase_+car_length_)/2+safety_margin_)/r_obstacle);
			}

			// calculate alpha + beta
      if (obs.loc.y() < turning_center.y())
			{
        float sum_alpha_beta = acos(obs_loc.x()/r_obstacle);
        alpha = sum_alpha_beta - beta;
      }
			else 
      {
        float sum_alpha_beta = acos(obs_loc.x()/r_obstacle) + M_PI/2;
        alpha = sum_alpha_beta - beta;
      }
			// free path length is the arc length traversed by wheelbase
			float fpl_current = alpha*radius;

			// save the smallest fpl
			if (fpl_current < fpl)
      {
				fpl = fpl_current;
        best_alpha = alpha;
        best_beta = beta;
				p_obstruction = obs_loc;
				p_end = {cos(alpha)*radius, sin(alpha)*radius};
			}
    	}

  	}

  radius = 1/path.curvature;
  
  if (radius < 0) 
  {
    for (auto &obs:ObstacleList_)
    {
      obs.loc.y() = -obs.loc.y();
    }
  }

	path.obstruction = p_obstruction;
	path.free_path_length = fpl;
	path.end_point = p_end;
  path.alpha = best_alpha;
  path.beta = best_beta;
  path.radius = radius;

}
}

// clearance is defined as the minimum distance from any point on the free path length to
void Navigation::calculateClearance(PathOption &path){
	float radius = 1/path.curvature;
	float alpha = path.free_path_length * path.curvature;
	float min_clearance = 1000;

	Vector2f turning_center(0,radius);
	Vector2f closest_point(0,0);
	for (const auto &obs : ObstacleList_){
		Vector2f obs_point = Odom2BaseLink(obs.loc);
		float r_obstacle = (turning_center - obs.loc).norm();
		// check if obstacle is inside the cone defined by RC, start location and final location
		float angle_obstacle = acos(obs_point.x()/r_obstacle);
		if (angle_obstacle < alpha){
			float clearance = abs((turning_center-obs_point).norm() - abs(radius));
			if (clearance < min_clearance){
				min_clearance = clearance;
				closest_point = obs_point;
			}
		}
	}
	path.clearance = min_clearance;
	path.closest_point = closest_point;
}

// Frank - ArcRadius Calcs
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

// Frank - ArcAngle Calcs
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

// Frank - ArcLength Calcs
double arc_length(double p1x, double p1y, double p2x, double p2y)
{
  double r = arc_radius(p1x, p1y, p2x, p2y);
  double phi = arc_angle(p1x, p1y, p2x, p2y);
  //std::cout << "phi: " << phi << std::endl;
  return r*phi;
// select best path based on scoring function
}


// Finding the Best Path
// TODO : define the return of this method
PathOption Navigation::getBestPath(Vector2f goal_loc)
{
  PathOption BestPath;
  //std::cout << "******** getBestPath *********" << std::endl;
	// Number to tune
	int num_paths = 20;

	// Sample paths
	samplePaths(num_paths);

	std::vector<double> free_path_length_vec;
	std::vector<double> dist_to_goal_vec;
	std::vector<double> clearance_vec;

	// save the range of each variable for normalize in the scoring function
	float max_free_path_length = 0;
	float min_dist_to_goal = 100;
	float max_clearance = 0;
  int i = {0};


	for (auto &path : Paths_)
	{

    // TODO : Not accurate, should fix to global
    path.dist_to_goal = goal_loc.norm() - abs(odom_loc_.norm() - odom_start_loc_.norm());

    //std::cout << "\n\npath radius: " << path.radius << std::endl;
    //std::cout << "path alpha: " << path.alpha << std::endl;
    //std::cout << "free path length before predictCollisions: " << path.free_path_length << std::endl;
    //std::cout << "path dist to goal: " << path.dist_to_goal << std::endl;

		predictCollisions(path);

    //std::cout << "[Iter: " << i << "]"
    //          << "\ninitial free path length: " << path.free_path_length
    //          << "\nbest_alpha: " << path.alpha 
    //          << "\nbest_beta: " << path.beta << std::endl;

		trimPath(path, goal_loc);

    // dist to goal for all paths
    path.dist_to_goal = (goal_loc - path.obstruction).norm();

    //std::cout << "path dist to goal: " << path.dist_to_goal << std::endl;
    //std::cout << "trimmed free path length: " << path.free_path_length << std::endl;

		max_free_path_length = std::max(path.free_path_length, max_free_path_length);
    //std::cout << "max_free_path_length: " << max_free_path_length << std::endl;
		min_dist_to_goal = std::min(path.dist_to_goal, min_dist_to_goal);
    //std::cout << "min_dist_to_goal: " << min_dist_to_goal << std::endl;
		max_clearance = std::max(path.clearance, max_clearance);
    //std::cout << "max_clearance: " << max_clearance << std::endl;

    // TODO : Variables HERE TWICE?
		max_free_path_length = std::max(path.free_path_length, max_free_path_length);
		min_dist_to_goal = std::min(path.dist_to_goal, min_dist_to_goal);

		free_path_length_vec.push_back(path.free_path_length);
		dist_to_goal_vec.push_back(path.dist_to_goal);
    i++;
	}


	for (int i = 0; i < num_paths; i++)
	{
    // the longer fpl, the better
		float free_path_length_cost = -(free_path_length_vec.at(i)/max_free_path_length) * free_path_length_weight_;
    //std::cout << "fpl_cost: " << free_path_length_cost << std::endl;

		// the smaller dist_to_goal, the better
		float dist_to_goal_cost =  (dist_to_goal_vec.at(i)/min_dist_to_goal) * dist_to_goal_weight_;
    //std::cout << "dist_to_goal_cost: " << dist_to_goal << std::endl;

    // TODO : add clearance_padded_cost
		// float cost = free_path_length_cost + clearance_padded_cost + dist_to_goal_cost;
    float cost = free_path_length_cost + dist_to_goal_cost;
    //std::cout << "cost: " << cost << std::endl;


    // TODO : define a min_cost, BestPath, PossiblePaths_
    float min_cost {1000};

		if (cost < min_cost) 
    {
			min_cost = cost;
			BestPath = Paths_.at(i);
			BestPath.cost = cost;
		}
	}
	return BestPath;
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

  //auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  //auto sec_since_epoch = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
  //std::cout << "millisec_since_epoch: " << millisec_since_epoch
  //          << "\nsec_since_epoch: " << sec_since_epoch
  //          << std::endl;

  // This function gets called 20 times a second to form the control loop.

  double dt = 0.05; // Time Step: 20Hz converted to sec
  double dist_traveled = abs(odom_loc_.norm() - odom_start_loc_.norm());
  
  // MESSAGE DATA
  std::cout << "========"
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

  //double p1x = 0;
  //double p1y = 0;
  //float p2x = 4.00;
  //float p2y = -0.2;

  //double arc_l = arc_length(p1x, p1y, p2x, p2y);
  //double r = arc_radius(p1x, p1y, p2x, p2y);
  
  // samplePaths(5);

  VisObstacles();

  Vector2f goal;
  goal << 4.00, 0.0;

  PathOption BestPath = getBestPath(goal);
  

  double vel_command =  car_.TOC(dt, robot_vel_.norm(), BestPath.free_path_length, dist_traveled);
  std::cout << "============================="
            << "\nBestPath FPL: " << BestPath.free_path_length 
            << "\n vel_command: " << vel_command << std::endl;

  // ======================================================================================================

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"
  
  // Eventually, you will have to set the control values to issue drive commands:

  // Draw Cross
  Vector2f p2;
  p2 << (odom_start_loc_.x()+4) - odom_loc_.x(), (odom_start_loc_.y()+1) - odom_loc_.y();

  visualization::DrawCross(goal,0.5,0x3449eb,local_viz_msg_);

  drive_msg_.curvature = BestPath.curvature;
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