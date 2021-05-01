#include "ros/ros.h"
#include "tracking_burger/TrackUnit.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "test_node"); 
    ros::NodeHandle nh;

    nav_msgs::Path path;

    geometry_msgs::PoseStamped pose1, pose2, pose3, pose4, pose5;
    
    pose1.header.frame_id = "odom";
    pose1.pose.position.x = 0.25;
    pose1.pose.position.y = 0;

    pose2.header.frame_id = "odom";
    pose2.pose.position.x = 0.50;
    pose2.pose.position.y = 0;

    pose3.header.frame_id = "odom";
    pose3.pose.position.x = 0.60;
    pose3.pose.position.y = 0.25;

    pose4.header.frame_id = "odom";
    pose4.pose.position.x = 0.7;
    pose4.pose.position.y = 0.40;

    pose5.header.frame_id = "odom";
    pose5.pose.position.x = 0.8;
    pose5.pose.position.y = 0.50;

    path.poses.push_back(pose1);
    path.poses.push_back(pose2);
    path.poses.push_back(pose3);
    path.poses.push_back(pose4);
    path.poses.push_back(pose5);
    
    Global_Planning::TrackUnit::Ptr tu_ptr(new Global_Planning::TrackUnit(nh, "cmd_vel"));
    tu_ptr->set_frame_id_burger("base_footprint");
    tu_ptr->set_frame_id_odom("odom");
    tu_ptr->set_fw_lk_distance(0.5);

    ros::Rate r(1);

    while(ros::ok()){
        tu_ptr->pub_control(path);
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}