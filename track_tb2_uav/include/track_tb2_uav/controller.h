#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>
#include <string>
#include "tf2_ros/buffer.h" // buffer
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/PositionTarget.h"
#include "track_tb2_uav/Mission.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // getIdentity
#include <math.h>	// abs

namespace UAV
{

class controller
{
private:
	controller();

	enum Mission{
		IDLE,
		TAKEOFF,
		TRACK,
		LANDING
	};

	Mission mission_;

	std::string uav_name_, ugv_name_;

	ros::Subscriber uav_pose_sub_;
	ros::Subscriber ugv_pose_sub_;
	
	ros::Publisher vision_pub_;
	ros::Publisher goal_pub_;

	ros::ServiceServer mission_server_;

	geometry_msgs::PoseStamped uav_pose_;
	geometry_msgs::PoseStamped ugv_pose_;

	mavros_msgs::PositionTarget goal_;

	ros::Timer vision_timer_;
	ros::Timer mainloop_timer_;

	double last_x, last_y;

	void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg_p){
		uav_pose_ = *msg_p;
	}

	void ugv_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg_p){
		ugv_pose_ = *msg_p;
	}

	void vision_timer_cb(const ros::TimerEvent &event){
		uav_pose_.header.stamp = ros::Time::now();
		vision_pub_.publish(uav_pose_);
	}

	void mainloop_timer_cb(const ros::TimerEvent &event);

	bool mission_cb(track_tb2_uav::Mission::Request& req, track_tb2_uav::Mission::Response& resp);


public:

	controller(const std::string name, tf2_ros::Buffer *tf);

	~controller();
};

}


#endif



