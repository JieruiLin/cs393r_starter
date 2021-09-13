#include "car.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// Constructor
Car::Car(float length, float width, float wheelbase, float buffer)
    : length {length}, width {width}, wheelbase {wheelbase}, buffer {buffer}
{
    length_axle_to_bumper = (length - wheelbase)/2;
    length_axle_to_door = width/2;
    driv_frnt_xy << (wheelbase+length_axle_to_bumper+buffer), (length_axle_to_door+buffer);
    pass_frnt_xy << (wheelbase+length_axle_to_bumper+buffer), -(length_axle_to_door+buffer);
    driv_rear_xy << -(wheelbase+length_axle_to_bumper+buffer), (length_axle_to_door+buffer);
    pass_rear_xy << -(wheelbase+length_axle_to_bumper+buffer), -(length_axle_to_door+buffer);
}

Car::Car(float length, float width, float wheelbase)
    : Car(length, width, wheelbase, 152.4)
{
    // default is a buffer of 6 inches (152.4 mm)
}
// Default Constructor
Car::Car()
    : Car(0,0,0,0) 
{

}

// Getters
float Car::get_turning_radius(const float &steering_angle)
{
    turning_radius = wheelbase/tanf(steering_angle); //ensure angle is in radians
    return turning_radius;
}

float Car::get_turning_radius_prime(const float &steering_angle)
{
    turning_radius_prime = sqrt(pow(get_turning_radius(steering_angle),2) + pow(wheelbase, 2));
    return turning_radius_prime;
}

float Car::get_curvature(const float &steering_angle)
{
    curvature = 1/get_turning_radius(steering_angle);
    return curvature;
}

float Car::get_curv_driv_frnt (const float &steering_angle) 
{
    float r_driv_frnt = sqrt(pow(driv_frnt_xy.x(),2) + pow(get_turning_radius(steering_angle) - abs(driv_frnt_xy.y()),2));
    return get_curvature(r_driv_frnt);
}

float Car::get_curv_pass_frnt (const float &steering_angle) 
{
    float r_pass_frnt = sqrt(pow(pass_frnt_xy.x(),2) + pow(get_turning_radius(steering_angle) + abs(pass_frnt_xy.y()),2));
    return get_curvature(r_pass_frnt);
}

float Car::get_curv_driv_rear (const float &steering_angle) 
{
    float r_driv_rear = sqrt(pow(driv_rear_xy.x(),2) + pow(get_turning_radius(steering_angle) - abs(driv_rear_xy.y()),2));
    return get_curvature(r_driv_rear);
}

float Car::get_curv_pass_rear (const float &steering_angle) 
{
    float r_pass_rear = sqrt(pow(pass_rear_xy.x(),2) + pow(get_turning_radius(steering_angle) + abs(pass_rear_xy.y()),2));
    return get_curvature(r_pass_rear);
}


