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
#include <vector>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件


/// @brief namespace for global plannning mission
/// 
namespace Global_Planning{

using std::string;

/// @brief Burger Tracking Control Unit
/// By this class, turtlebot3 can trace the path smoothly without any consideration of controling.
/// Initualization is expected to be executed before using. Don't worry. Operation is extreme
/// simplely and you can grasp by a minute.
///
/// @note All of Api is provided in public part of class header.
/// @details USER GUIDE - a demo of using
///
/// 	.....
/// 	TrackUnit::Ptr tu_ptr = new TrackUnit(nh);
/// 	tu_ptr->set_pub_topic("/turbot3/cmd_vel");
/// 	tu_ptr->set_frame_id_burger("vicon/turtlebot3/turtlebot3");
/// 	tu_ptr->set_frame_id_odom("world");
/// 	tu_ptr->set_fw_lk_distance(0.5);
///		while(ros::ok()){
///			tu_ptr->pub_control(path, cur_pos);
///		} 
/// 	......
///
class TrackUnit
{

private:

	/// @brief the topic to which you publish
	/// You should initialize before publishing msg. "cmd_vel" as default	
	/// @see set_pub_topic
	string _pub_topic;
	/// @brief the frame id correspond to turtlebot3 burger
	/// You should initialize before publishing msg. "turtlebot3" as default
	/// @see set_frame_id_burge
	string _frame_id_burger;
	/// @brief the frame id correspond to odom
	/// You should initialize before publishing msg. "odom" as default
	/// @see set_frame_id_odom
	string _frame_id_odom;

	/// @brief forward looking distance /m
	/// You should initialize before publishing msg. 1.0 meter as default
	/// @see set_fw_lk_distance
	double _fw_lk_dist;

	/// @brief Get the closest point index of the path
	/// This function is for special using.
	/// @param path is the waypoints mechine is expected to trace
	/// @param cur_pos is current position (meter)
	/// @return int is the index of path waypoint

	const int get_closest_point_id(const std::vector<geometry_msgs::PoseStamped> &poses);

	/// @brief Get the track point index of the path
	/// This function is for special using.
	/// @param path is the waypoints mechine is expected to trace
	/// @param cur_pos is current position (meter)
	/// @return int is the index of path waypoint

	const int get_track_point_id(const std::vector<geometry_msgs::PoseStamped> &poses);

	const std::vector<geometry_msgs::PoseStamped> transform(const nav_msgs::Path &path);

	void print_path(const std::vector<geometry_msgs::PoseStamped> &poses);

	/// @brief Smoother for burger
	/// Smooth unreasonable velocity and publish control topic
	Smoother::Ptr _sm_prt;

public:

	TrackUnit();

	/// @brief Destroy the Track Unit object
	/// No thing to do.
	~TrackUnit();

	/// @brief Construct a new Track Unit object
	/// To initialize pub_topic, _frame_id_burger, _frame_id_odom, forward looking distance
	/// @param nh All of the topics and parameters of ros can float by namespace according to nh.
	/// @param topic is the topic to which you want to publish infomations
	TrackUnit(ros::NodeHandle &nh, const string topic = "cmd_vel");

	/// @brief Set the frame id for burger object
	/// You should set frame id rightly. It's "base_footprint" most of the time.
	/// "base_footprint" as default.
	/// @param burger is the frame id of turbot3
	void set_frame_id_burger(const string burger = "base_footprint");

	/// @brief Set the frame id odom object
	/// You should set frame id rightly. It's "map" most of the time.
	/// @param odom is the frame id to which points are refered originally
	void set_frame_id_odom(const string odom = "map");

	/// @brief Set forward looking distance
	/// Forward looking distance refers to the length from furthest traking point to current postion.
	/// A higher value should be passed when the mechine is expected to trace path smoothly. However,
	/// a great value would make mechine ingore some shape point of the path.
	/// @param new_dist forward looking distance (meter) - 1.0 as default
	void set_fw_lk_distance(const double new_dist = 1.0);

	/// @brief publish control command to burger
	/// One invoking means one publishing without spinOnce. It can be used in while-loop or callback
	/// function. 
	/// @param path is the waypoints mechine is expected to trace
	/// @param is_new_path is set true if your path is the same as the last path.
	void pub_control(const nav_msgs::Path &path);

	/// @brief shared pointer for TrackUnit * 
	/// We recommand to use pointer rather than a class instance.
	typedef std::shared_ptr<TrackUnit> Ptr; 
};

}

#endif