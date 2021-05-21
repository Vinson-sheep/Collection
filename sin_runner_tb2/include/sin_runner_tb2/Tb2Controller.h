#ifndef TB2CONTROLLER_H
#define TB2CONTROLLER_H

#include "ros/ros.h"
#include "tf2_ros/buffer.h" // buffer
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <std_srvs/SetBool.h>
#include "sin_runner_tb2/Smoother.h"


namespace TB2{


class Tb2Controller
{
private:

	Tb2Controller();
	Tb2Controller(const Tb2Controller &rhs);
	Tb2Controller& operator=(const Tb2Controller &rhs);

	enum STATE{
		ROTATE,
		TRACKING,
		PAUSE
	};

	// state record
	STATE last_state_, state_;
	
	// path initializing
	double abs_y_;
	double a_x_;
	double b_x_;

	// path
	std::vector<geometry_msgs::Pose2D> path_1_, path_2_;
	char path_selected_;

	geometry_msgs::Twist twist_;
	bool twist_ok_;

	// service
	ros::ServiceServer control_server;

	// vicon twist
	ros::Subscriber twist_sub;

	int goal_index_;

	tf2_ros::Buffer* tf_;
	Smoother::Ptr sm_ptr_;

	// callback
	bool control_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
	void twist_cb(const geometry_msgs::Twist::ConstPtr& msg_p);

	// tool

	const int get_closest_point_id(const std::vector<geometry_msgs::Pose2D> &poses);
	const int get_track_point_id(const std::vector<geometry_msgs::Pose2D> &poses);
	const std::vector<geometry_msgs::Pose2D> transform(const geometry_msgs::Pose2D &path);
	// return true if stoped
	bool stoped();

	bool initialize_path();


public:
	
	Tb2Controller(const std::string name, tf2_ros::Buffer* tf);

	~Tb2Controller();

	typedef std::shared_ptr<Tb2Controller> Ptr;
};


}

#endif
