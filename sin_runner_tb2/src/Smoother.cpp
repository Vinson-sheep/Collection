/// @file Smoother.cpp
/// @author Vinson Sheep (775014077@qq.com)
/// @brief 
/// @version 2.0
/// @date 2021-04-28
/// 
/// @copyright Copyright (c) 2021
/// 

#include "sin_runner_tb2/Smoother.h"

namespace TB2
{

Smoother::Smoother(){}
Smoother::~Smoother(){}

Smoother::Smoother(const string name):
    cur_linear_vel_(0),
    cur_angular_vel_(0)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~" + name);

    string pub_topic;

    private_nh.param("max_lin_vel", MAX_LIN_VEL_, 0.70);
    private_nh.param("max_ang_vel", MAX_ANG_VEL_, 3.14);
    private_nh.param("lin_step_size", LIN_STEP_SIZE_, 0.05);
    private_nh.param("ang_step_size", ANG_STEP_SIZE_, 0.2); 
    private_nh.param("pub_topic", pub_topic, string("cmd_vel")); 
    
    vel_pub_ = nh.advertise<geometry_msgs::Twist>(pub_topic, 2);
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
void  Smoother::pub(const double target_linear_vel, const double target_angular_vel){

    // create twist msg
    geometry_msgs::Twist twist;

    if (target_linear_vel < 10e-4 && abs(target_angular_vel) < 10e-4){  // [STOP] mission
        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0;
    }
    else {
        // correct abnormal velocity
        double target_linear_vel_new = constrain(target_linear_vel, -MAX_LIN_VEL_, MAX_LIN_VEL_);
        double target_angular_vel_new = constrain(target_angular_vel, -MAX_ANG_VEL_, MAX_ANG_VEL_);
        
        // set linear velocidy
        twist.linear.x = cur_linear_vel_ = make_simple_profile(target_linear_vel_new, 
                        cur_linear_vel_, LIN_STEP_SIZE_);
        twist.linear.y = twist.linear.z = 0;
        // set angular velocity
        twist.angular.x = twist.angular.y = 0;
        twist.angular.z = cur_angular_vel_ = make_simple_profile(target_angular_vel_new, 
                        cur_angular_vel_, ANG_STEP_SIZE_);
    }
    // print
    // ROS_INFO("burger: linear %.2f angular %.2f", twist.linear.x, twist.angular.z);
    // publish
    vel_pub_.publish(twist);
}

}
