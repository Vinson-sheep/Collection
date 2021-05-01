/// @file TrackUnit.h
/// @author Vinson Sheep (775014077@qq.com)
/// @brief The definition of TrackUnit
/// @version 0.1
/// @date 2021-04-28
/// 
/// @copyright Copyright (c) 2021
/// 


#ifndef _TRACKUNIT_H_
#define _TRACKUNIT_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include <memory> // shared_ptr
#include <string> // string
#include "tracking_burger/Smoother.h"
#include <vector> // vector
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"	// buffer
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //transform
#include "math.h" // sqrt, pow
#include "float.h" // DBL_MAX

/// @brief Namespace for Kongdijiqun
///
namespace Global_Planning{

using std::string;

/// @brief Burger Tracking Control Unit.
///
/// @details 
/// By this class, turtlebot3 can trace the path smoothly without any consideration of controling.
/// Initualization is expected to be executed before using. Don't worry. Operation is extreme
/// simple and you can grasp by a minute.
///
/// demo
///
///		......
///
/// 	Global_Planning::TrackUnit::Ptr tu_ptr(new Global_Planning::TrackUnit(nh, "cmd_vel"));
///     tu_ptr->set_frame_id_burger("base_footprint");
///     tu_ptr->set_frame_id_odom("odom");
///     tu_ptr->set_fw_lk_distance(0.5);
///
///     ros::Rate r(1);
///     while(ros::ok()){
///         tu_ptr->pub_control(path);
///         ros::spinOnce();
///         r.sleep();
///     }
/// 	......
///
///   
class TrackUnit
{

private:

	/// @name attributes needed to initialize
	/// @{

	/// @brief the topic to which you publish
	/// @details You should initialize before publishing msg. "cmd_vel" as default	
	/// @see set_pub_topic
	string _pub_topic;
	/// @brief the frame id correspond to turtlebot3 burger
	/// @details You should initialize before publishing msg. "turtlebot3" as default
	/// @see set_frame_id_burge
	string _frame_id_burger;
	/// @brief the frame id correspond to odom
	/// @details You should initialize before publishing msg. "odom" as default
	/// @see set_frame_id_odom
	string _frame_id_odom;
	/// @brief forward looking distance /m
	/// @details You should initialize before publishing msg. 1.0 meter as default
	/// @see set_fw_lk_distance
	double _fw_lk_dist;
	/// @brief distance error 
	/// @details If the mechine have run into the radius (distance error) of the end point of the
	/// path, it would stop and return feedback.
	/// @see set_dist_error
	double _dist_error;

	/// @brief Smoother pointer for burger
	/// @details Smooth unreasonable velocity and publish control topic
	Smoother::Ptr _sm_prt;

	/// @}


	/// @name tf relative
	/// @{

	/// @brief buffer pointer 
	/// @details Restore tf information.
	std::shared_ptr<tf2_ros::Buffer> _buffer_ptr;
	/// @brief transform frame listener
	/// @details
	std::shared_ptr<tf2_ros::TransformListener> _listener_ptr;

	/// @}


	/// @name remaining function
	/// @{

	/// @brief Get the closest point index of the path 
	/// @details This function is for special using.
	/// @param poses the poses which have been transfromed to burger frame
	/// @return const int the index of path waypoint 
	const int get_closest_point_id(const std::vector<geometry_msgs::PoseStamped> &poses);
	/// @brief Get the tracing point index of the path
	/// @details This function is for special using.
	/// @param poses the poses which have been transfromed to burger frame
	/// @return const int the index of path waypoint 
	const int get_track_point_id(const std::vector<geometry_msgs::PoseStamped> &poses);
	/// @brief transform path to poseStamped relative to burger
	/// @details This function is for special using.
	/// @param path is the waypoints mechine is expected to trace
	/// @return const std::vector<geometry_msgs::PoseStamped> 
	const std::vector<geometry_msgs::PoseStamped> transform(const nav_msgs::Path &path);

	/// @}


	/// @name debug
	/// @{

	void print_poses(const std::vector<geometry_msgs::PoseStamped> &poses);

	/// @}

public:

	/// @name construction function
	/// @{

	/// @brief Construct a new Track Unit object
	/// 
	TrackUnit();

	/// @brief Destroy the Track Unit object
	/// @details No thing to do.
	~TrackUnit();

	/// @brief Construct a new Track Unit object
	/// @details To initialize pub_topic, _frame_id_burger, _frame_id_odom, forward looking distance
	/// @param nh All of the topics and parameters of ros can float by namespace according to nh.
	/// @param topic is the topic to which you want to publish infomations
	TrackUnit(ros::NodeHandle &nh, const string topic = "cmd_vel");

	/// @}

	/// @name initialization function
	/// @{

	/// @brief Set the frame id for burger object
	/// @details You should set frame id rightly. It's "base_footprint" most of the time.
	/// "base_footprint" as default.
	/// @param burger is the frame id of turbot3
	void set_frame_id_burger(const string burger = "base_footprint");
	/// @brief Set the frame id odom object
	/// @details You should set frame id rightly. It's "map" most of the time.
	/// @param odom is the frame id to which points are refered originally
	void set_frame_id_odom(const string odom = "map");
	/// @brief Set forward looking distance
	/// @details
	/// Forward looking distance refers to the length from furthest traking point to current postion.
	/// A higher value should be passed when the mechine is expected to trace path smoothly. However,
	/// a great value would make mechine ingore some shape point of the path.
	/// @param new_dist forward looking distance (meter) - 1.0 as default
	void set_fw_lk_distance(const double new_dist = 1.0);
	/// @brief radius of end point
	/// @details If the mechine have run into the radius (distance error) of the end point of the
	/// path, it would stop and return feedback.
	/// @param temp meter of radius
	void set_dist_error(const double temp = 0.1);
	
	/// @}

	/// @name core function
	/// @{

	/// @brief publish control command to burger
	/// @details One invoking means one publishing without spinOnce. It can be used in while-loop 
	/// or callback function. 
	/// @param path is the waypoints mechine is expected to trace
	void pub_control(const nav_msgs::Path &path);

	/// @}

	/// @brief shared pointer for TrackUnit * 
	/// @details We recommand to use pointer rather than a class instance.
	typedef std::shared_ptr<TrackUnit> Ptr; 
};

}

#endif