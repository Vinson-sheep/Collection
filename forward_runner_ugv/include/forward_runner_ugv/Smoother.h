/// @file Smoother.h
/// @author Vinson Sheep (775014077@qq.com)
/// @brief 
/// 
/// @version 1.0
/// @date 2021-04-28
/// 
/// @copyright Copyright (c) 2021
/// 

#ifndef _SMOOTHER_H_
#define _SMOOTHER_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include <algorithm>	// min, max
#include <string>	// string
#include <memory> // shared_ptr

namespace Naive_Planner
{

const double MAX_LIN_VEL = 0.21;		///< Maximum linear velocity
const double MAX_ANG_VEL = 0.84;		///< Maximum angular velocity
const double LIN_STEP_SIZE = 0.01;	///< Linear step size, in terms of linear accelerator
const double ANG_STEP_SIZE = 0.1;	///< Angular step size, in terms of angular accelerator

using std::min;
using std::max;
using std::string;

/// @brief Velocity Smoother for Turtlebot3 burger
/// Smoother can correct abnormal velocity input, and send reasonable velocity messages to topic
/// you desire.
///
/// @note All of Api is provided in public part of class header.
/// @details USER GUIDE - a demo of using
/// 
/// ......

class Smoother
{
private:

	/// @brief real control linear velocity 
	/// The velocity which have sent to burger in last time period or in the past. 0.0 as default.
	/// @see Smoother Constructor
	double _control_linear_vel;

	/// @brief real control angular velocity 
	/// The velocity which have sent to burger in last time period or in the past. 0.0 as default
	/// @see Smoother Constructor
	double _control_angular_vel;

	/// @brief Control topic publisher
	/// Initialize by Constructor.
	/// @see Smoother
	ros::Publisher _pub;

	/// @brief smooth input data according output and slot
	/// While output is much greater or lower than input, smoothing operation woule be token. Input
	/// data is being smoothed to get closer to ouput according to slot.
	/// @param input data being smoothing
	/// @param output  data to which input get close
	/// @param slot step size of smoothing
	/// @return const double smoothed data
	const double make_simple_profile(const double input, const double output, const double slot);
	
	/// @brief velocity constrain function
	/// Correct abnormal values.
	/// @param input data
	/// @param low minimum
	/// @param high maximum
	/// @return const double data between minimum and maximum
	const double constrain(const double input, const double low, const double high);

	///////////////////
	// possible API
	///////////////////

	/// @brief Set the initial linear velocity
	/// A proper value should be provided while robot is moving. 0.0 as default.
	/// @param init_lin_vel initial linear velocity
	void set_init_lin_vel(const double init_lin_vel = 0.0);

	/// @brief Set the initial angular velocity
	/// A proper value should be provided while robot is moving. 0.0 as default.
	/// @param init_ang_vel initial angular velocity
	void set_init_ang_vel(const double init_ang_vel = 0.0);

public:
	/// @brief Construct a new Smoother object
	/// @details 
	Smoother();

	/// @brief Destroy the Smoother object
	/// @details Do nothing.
	~Smoother();

	/// @brief Construct a new Smoother object
	/// @details To initialize all of private attributes.
	/// @param nh All of the topics and parameters of ros can double by namespace according to nh.
	/// @param topic is the topic to which you want to publish infomations
	Smoother(ros::NodeHandle &nh, const string topic = "cmd_vel");

	/// @brief controling message publisher function
	/// @details Publish velocity messages after smoothing.
	/// @details dsadsa
	/// @param target_linear_vel target linear velocity
	/// @param target_angular_vel target angular velocity
	void set(const double target_linear_vel = 0.0, const double target_angular_vel = 0.0);

	/// @brief shared pointer for Smoother *
	/// 
	/// We recommand to use pointer rather than a class instance.
	typedef std::shared_ptr<Smoother> Ptr; 
};


}


#endif