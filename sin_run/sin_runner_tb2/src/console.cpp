#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <tf2/utils.h> // getYaw

std::string topic_name;
std::string vehicle_name;

void doMsg(const nav_msgs::Odometry::ConstPtr& msg_p){
    double x,y,yaw;
    x = msg_p->pose.pose.position.x;
    y = msg_p->pose.pose.position.y;
    yaw = tf2::getYaw(msg_p->pose.pose.orientation);
    ROS_INFO("%s: (%.2f, %.2f, %.2f)", vehicle_name.c_str(), x, y, yaw);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "console"); 
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;    

    ROS_INFO("staion console.");

    private_nh.param("topic_name", topic_name, std::string("tb2/odom"));
    private_nh.param("vehicle_name", vehicle_name, std::string("tb2"));

    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(topic_name,10,doMsg);

    ros::spin();

    return 0;
}