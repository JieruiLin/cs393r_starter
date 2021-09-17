#ifndef _CAR_H_
#define _CAR_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// Ackermann Car Class
class Car 
{
private:
    float length;    
    float width;     
    float wheelbase; 
    float buffer;
    float length_axle_to_bumper; 
    float length_axle_to_door;
    int const vel_max {1}; // m/s, max velocity
    int const acl_max {4}; // m/s^2, maximum acceleration
    int const dcl_max {4}; // m/s^2, maxium deacceleration

    // Car Corner (x, y) coordinates from centroid of car (rear axle) - with buffer added in
    Eigen::Vector2f driv_frnt_xy;
    Eigen::Vector2f pass_frnt_xy;
    Eigen::Vector2f driv_rear_xy;
    Eigen::Vector2f pass_rear_xy;

    float turning_radius;
    float turning_radius_prime;
    float curvature;
    

public:
    Car(float length, float width, float wheelbase, float buffer);
    Car(float length, float width, float wheelbase);
    Car();

    float get_turning_radius (const float &steering_angle);
    float get_turning_radius_prime (const float &steering_angle);
    float get_curvature(const float &steering_angle);

    float get_curv_driv_frnt (const float &steering_angle);
    float get_curv_pass_frnt (const float &steering_angle);
    float get_curv_driv_rear (const float &steering_angle);
    float get_curv_pass_rear (const float &steering_angle);

    float TOC(float dt, float vel_current, float arc_length, float dist_traveled);
};

#endif // _CAR_H_