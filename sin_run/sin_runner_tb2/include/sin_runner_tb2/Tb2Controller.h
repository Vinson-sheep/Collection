#ifndef TB2CONTROLLER_H
#define TB2CONTROLLER_H

#include "ros/ros.h"
#include "tf2_ros/buffer.h" // buffer
#include "geometry_msgs/Twist.h"
#include <vector>
#include <std_srvs/SetBool.h>
#include <string>
#include "pp_local_planner/PPLocalPlanner.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // getIdentity
#include "geometry_msgs/PoseStamped.h"
#include <math.h> // sin
#include "nav_msgs/Path.h"


namespace TB2{


class Tb2Controller
{
private:

	Tb2Controller();
	Tb2Controller(const Tb2Controller &rhs);
	Tb2Controller& operator=(const Tb2Controller &rhs);

	enum STATE{
		RUN,
		PAUSE
	};

	STATE state_;
	
	// path initializing
	double abs_y_;
	double a_x_;
	double b_x_;

	// path
	std::vector<geometry_msgs::PoseStamped> path_1_, path_2_;
	char path_selected_;

	PPLocalPlanner::Ptr plp_p_;

	// service
	ros::ServiceServer control_server_;

	// publisher
	ros::Publisher vel_pub_;
	string vel_topic_;
	ros::Publisher path_pub_;

	string world_frame_id_;

	tf2_ros::Buffer* tf_;

	ros::Timer mainloop_timer_;

	// mainloop
	void mainloop_cb(const ros::TimerEvent &event);

	// callback
	bool control_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);

	bool initialize_path();

	void publishZeroVelocity();

public:
	
	Tb2Controller(const std::string name, tf2_ros::Buffer* tf);

	~Tb2Controller();

	typedef std::shared_ptr<Tb2Controller> Ptr;
};


}

#endif
