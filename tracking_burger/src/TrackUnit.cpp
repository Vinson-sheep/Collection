/// @file TrackUnit.cpp
/// @author Vinson Sheep (775014077@qq.com)
/// @brief 
/// @version 0.1
/// @date 2021-04-28
/// 
/// @copyright Copyright (c) 2021
/// 

#include "tracking_burger/TrackUnit.h"


namespace Global_Planning
{

TrackUnit::TrackUnit(){}
TrackUnit::~TrackUnit(){}


TrackUnit::TrackUnit(ros::NodeHandle &nh, const string topic):
    _pub_topic("cmd_vel"),
    _frame_id_burger("base_footprint"),
    _frame_id_odom("map"),
    _fw_lk_dist(1.0),
    _sm_prt(new Smoother(nh, topic))
{}

void TrackUnit::set_frame_id_burger(const string burger){
    _frame_id_burger = burger;
}

void TrackUnit::set_frame_id_odom(const string odom){
    _frame_id_odom = odom;
}

void TrackUnit::set_fw_lk_distance(const double new_dist){
    _fw_lk_dist = new_dist;
}



/// @details
///         1. 将path中所有点转变为相对于turtlebot3的点
///         2. 计算最近可追踪的点id
///         3. 计算目标线速度和角速度
///         4. 发布速度信息给Smoother
void TrackUnit::pub_control(const nav_msgs::Path &path){
    // 1. 将path中所有点转变为相对于turtlebot3的点
    std::vector<geometry_msgs::PoseStamped> poses = transform(path);
    // 2. 计算最近可追踪的点id
    int index = get_track_point_id(poses);
    // 3. 计算目标线速度和角速度
    ROS_INFO("Calculate targer linear and angular velocity.");
    // 4. 发布速度信息给Smoother
    ROS_INFO("Publish control message to Smoother.");
}


const std::vector<geometry_msgs::PoseStamped> TrackUnit::transform(const nav_msgs::Path &path){
    // 初始化tf相关的工具类
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener listener(buffer);

    for(i=0; i<path.poses.size(), i++){
        geometry_msgs::PointStamped point_tur = point;
        point_tur = buffer.transform(point_tur, TURTLEBOT_FRAME_ID.c_str());
    }


    return result;
}

// TODO
const int TrackUnit::get_closest_point_id(const std::vector<geometry_msgs::PoseStamped> &poses){

    return 0;
}

// TODO
const int TrackUnit::get_track_point_id(const std::vector<geometry_msgs::PoseStamped> &poses){

    return 0;
}

void TrackUnit::print_path(const std::vector<geometry_msgs::PoseStamped> &poses){
    for (int i = 0; i < poses.size(); i++){
        std::cout << i << ": " << poses[i].pose.position.x << " " << poses[i].pose.position.y << " ";
    }
    std::cout << std::endl;
}

}
