#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <tf2/utils.h> // getYaw

std::string topic_name;
std::string vehicle_name;
bool sim_mode;

void doMsg1(const nav_msgs::Odometry::ConstPtr& msg_p){
    double x,y,yaw;
    x = msg_p->pose.pose.position.x;
    y = msg_p->pose.pose.position.y;
    yaw = tf2::getYaw(msg_p->pose.pose.orientation);
    ROS_INFO("%s: (%.2f, %.2f, %.2f)", vehicle_name.c_str(), x, y, yaw);
}

void doMsg2(const geometry_msgs::PoseStamped::ConstPtr& msg_p){
    double x,y,yaw;
    x = msg_p->pose.position.x;
    y = msg_p->pose.position.y;
    yaw = tf2::getYaw(msg_p->pose.orientation);
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
    private_nh.param("sim_mode", sim_mode, true);
    
    ros::Subscriber sub;

    if(sim_mode){
        sub = nh.subscribe<nav_msgs::Odometry>(topic_name,10,doMsg1);    
    }
    else{
        sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_name,10,doMsg2);    
    }

    ros::spin();

    return 0;
}