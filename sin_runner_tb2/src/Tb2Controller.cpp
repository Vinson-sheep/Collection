#include "sin_runner_tb2/Tb2Controller.h"

namespace TB2
{

Tb2Controller::Tb2Controller(){}
Tb2Controller::~Tb2Controller(){}

Tb2Controller::Tb2Controller(const std::string name, tf2_ros::Buffer* tf){

}

bool Tb2Controller::stoped(){

}

bool Tb2Controller::initialize_path(){

}

const int Tb2Controller::get_closest_point_id(const std::vector<geometry_msgs::Pose2D> &poses){

}
const int Tb2Controller::get_track_point_id(const std::vector<geometry_msgs::Pose2D> &poses){
    
}
const std::vector<geometry_msgs::Pose2D> Tb2Controller::transform(const geometry_msgs::Pose2D &path){
    
}

bool Tb2Controller::control_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp){
    
}
void Tb2Controller::twist_cb(const geometry_msgs::Twist::ConstPtr& msg_p){
    
}


}


