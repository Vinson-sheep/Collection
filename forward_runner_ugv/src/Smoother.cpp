/// @file Smoother.cpp
/// @author Vinson Sheep (775014077@qq.com)
/// @brief 
/// @version 0.1
/// @date 2021-04-28
/// 
/// @copyright Copyright (c) 2021
/// 

#include "forward_runner_ugv/Smoother.h"

namespace Naive_Planner
{

Smoother::Smoother(){}

Smoother::~Smoother(){}

/// @attention recommend to use
Smoother::Smoother(ros::NodeHandle &nh, const string topic): 
    _control_linear_vel(0.0),
    _control_angular_vel(0.0),
    _pub(nh.advertise<geometry_msgs::Twist>(topic.c_str(),10))
{}

void Smoother::set_init_lin_vel(const double init_lin_vel){
    _control_linear_vel = init_lin_vel;
}

void Smoother::set_init_ang_vel(const double init_ang_vel){
    _control_angular_vel = init_ang_vel;
}

const double Smoother::make_simple_profile(const double input, const double output,
                                         const double slot){
    if (input > output){
        return min(input, output + slot/2);
    }
    else if (input < output){
        return max(input, output - slot/2);
    }
    return output;
}

const double  Smoother::constrain(const double input, const double low, const double high){
    if (input > high)
        return high;
    else if (input < low)
        return low;
    return input;
}


/// @note It would correct the velocity beyond limitaion and smooth it according target velocity
/// while control message would be publish and velocity infomation would be print in terminal.
/// @attention This function is no spin and sleep. User should use spin function and sleep in the 
/// exteral while loop.
/// @details description
void  Smoother::set(const double target_linear_vel, const double target_angular_vel){

    // create twist msg
    geometry_msgs::Twist twist;

    if (target_linear_vel < 10e-4 && abs(target_angular_vel) < 10e-4){  // [STOP] mission
        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0;
        _control_linear_vel = _control_angular_vel = 0;
    }
    else {
        // correct abnormal velocity
        double target_linear_vel_new = constrain(target_linear_vel, -MAX_LIN_VEL, MAX_LIN_VEL);
        double target_angular_vel_new = constrain(target_angular_vel, -MAX_ANG_VEL, MAX_ANG_VEL);
        
        // set linear velocidy
        twist.linear.x = _control_linear_vel = make_simple_profile(target_linear_vel_new, 
                        _control_linear_vel, LIN_STEP_SIZE);
        twist.linear.y = twist.linear.z = 0;
        // set angular velocity
        twist.angular.x = twist.angular.y = 0;
        twist.angular.z = _control_angular_vel = make_simple_profile(target_angular_vel_new, 
                        _control_angular_vel, ANG_STEP_SIZE);
    }
    // print
    // ROS_INFO("burger: linear %.2f angular %.2f", twist.linear.x, twist.angular.z);
    // publish
    _pub.publish(twist);
}

}