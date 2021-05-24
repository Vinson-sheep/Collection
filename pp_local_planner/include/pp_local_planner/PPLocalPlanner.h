#ifndef PP_LOCAL_PLANNER_H
#define PP_LOCAL_PLANNER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <pp_local_planner/Smoother.h>
#include <tf2/utils.h> // getYaw
#include <vector>
#include <angles/angles.h>

namespace TB2
{

class PPLocalPlanner
{
private:
	PPLocalPlanner();

	double angular_tolerance_, dist_tolerance_;
	double max_fw_looking_dist_, min_fw_looking_dist_,
		fw_looking_factor_;

	int track_index_;
	
	bool initialized_;
	bool first_arrive_;
	bool need_rotate_;

	bool isGoalReachedWithRotate();
	bool isGoalReachedWithDist();

	bool path_ok_;
	std::vector<geometry_msgs::PoseStamped> plan_;
	double prune_radius_;
	bool prunePath();

	double move_factor_;
	double rotate_factor_;

	bool stoped();

	tf2_ros::Buffer* tf_;
	string world_frame_id_, base_frame_id_;

	std::string twist_topic_;
	geometry_msgs::Twist twist_;
	ros::Subscriber twist_sub_;
	bool twist_ok_;
	void twist_cb(const geometry_msgs::Twist::ConstPtr& msg_p);

	const int get_closest_point_id();
	const int get_track_point_id();

	Smoother::Ptr sm_ptr_;

public:
	
	~PPLocalPlanner();

	PPLocalPlanner(const std::string name, tf2_ros::Buffer* tf);

	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
	bool isGoalReached(){
		return isGoalReachedWithRotate() && stoped();
	}
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
	void initialize(const std::string name, tf2_ros::Buffer* tf);

	typedef std::shared_ptr<PPLocalPlanner> Ptr;
};

}


#endif

