#include "car.h"
#include <iostream>
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

// debug
float Car::TOC(float dt, float vel_current, float arc_length)// float dist_traveled) 
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
        float dist_left = arc_length; //- dist_traveled;          // distance left on the free path length
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
        float dist_left = arc_length; //- dist_traveled;    // distance left on path 
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

