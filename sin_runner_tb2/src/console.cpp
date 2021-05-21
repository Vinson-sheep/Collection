#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <string>

std::string topic_name;
std::string vehicle_name;

void doMsg(const geometry_msgs::Pose2D::ConstPtr& msg_p){
    ROS_INFO("%s: (%.2f, %.2f, %.2f)", vehicle_name.c_str(),
        msg_p->x,
        msg_p->y,
        msg_p->theta
         );
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "console"); 
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;    

    ROS_INFO("staion console.");

    private_nh.param("topic_name", topic_name, std::string("tb2/pose"));
    private_nh.param("vehicle_name", vehicle_name, std::string("tb2"));

    ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose2D>(topic_name,10,doMsg);

    ros::spin();

    return 0;
}