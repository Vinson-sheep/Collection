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
    _frame_id_odom("odom"),
    _fw_lk_dist(1.0),
    _dist_error(0.1),
    _sm_prt(new Smoother(nh, topic)),
    _buffer_ptr(new tf2_ros::Buffer()),
    _listener_ptr(new tf2_ros::TransformListener(*_buffer_ptr))
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
void TrackUnit::set_dist_error(const double temp){
    _dist_error = 0.1;
}


/// @details
///         1. 将path中所有点转变为相对于turtlebot3的点
///         2. 计算最近可追踪的点id
///         3. 计算目标线速度和角速度
///         4. 发布速度信息给Smoother
void TrackUnit::pub_control(const nav_msgs::Path &path){
    // 1. 将path中所有点转变为相对于turtlebot3的点
    std::vector<geometry_msgs::PoseStamped> poses = transform(path);
    if (poses.size() == 0){
        return;
    }

    if (sqrt(pow(poses[poses.size()-1].pose.position.x,2) + pow(poses[poses.size()-1].pose.position.y,2)) < _dist_error){
        _sm_prt->set(0, 0);
        ROS_INFO("Have reached the end of the path.");
        return;
    }
    // 2. 计算最近可追踪的点id
    int index = get_track_point_id(poses);
    // 4. 发布速度信息给Smoother
    _sm_prt->set(sqrt(pow(poses[index].pose.position.x,2) + pow(poses[index].pose.position.y,2)), 
                atan2(poses[index].pose.position.y, poses[index].pose.position.x));
    ROS_INFO("距离点%f m.", sqrt(pow(poses[index].pose.position.x,2) + pow(poses[index].pose.position.y,2)));
}


const std::vector<geometry_msgs::PoseStamped> TrackUnit::transform(const nav_msgs::Path &path){
    // 初始化tf相关的工具类

    geometry_msgs::PoseStamped temp;

    std::vector<geometry_msgs::PoseStamped> result;
    
    std::cout << _buffer_ptr->canTransform(_frame_id_burger, _frame_id_odom, ros::Time(0), ros::Duration(0.1));

    // wait for transform    
    if (!_buffer_ptr->canTransform(_frame_id_burger, _frame_id_odom, ros::Time(0), ros::Duration(0.1))){
        ROS_ERROR("Can't transfrom from %s to %s.", _frame_id_burger.c_str(), _frame_id_odom.c_str());
        return std::vector<geometry_msgs::PoseStamped>();
    }
    
    for(int i=0; i<path.poses.size(); i++){
        temp = path.poses[i];
        temp.header.stamp = ros::Time(0);
        temp = _buffer_ptr->transform(temp, _frame_id_burger.c_str());
        result.push_back(temp);
    } 

    return result;
}

// TODO
// if size = 0, return -1
const int TrackUnit::get_closest_point_id(const std::vector<geometry_msgs::PoseStamped> &poses){
    double min_dist = DBL_MAX;
    int index = -1;

    for (int i=0; i<poses.size(); i++){
        if(sqrt(pow(poses[i].pose.position.x, 2) + pow(poses[i].pose.position.y, 2)) < min_dist){
            min_dist = sqrt(pow(poses[i].pose.position.x, 2) + pow(poses[i].pose.position.y, 2));
            index = i;
        }
    }

    return index;
}

// TODO
const int TrackUnit::get_track_point_id(const std::vector<geometry_msgs::PoseStamped> &poses){
    int index = get_closest_point_id(poses);

    double dist_sum = 0;
    double next_dist = sqrt(pow((poses[index+1].pose.position.x - poses[index].pose.position.x), 2)
        + pow((poses[index+1].pose.position.y - poses[index].pose.position.y), 2));

    while(index < poses.size()-1 && dist_sum + next_dist < _fw_lk_dist){
        std::cout << next_dist << std::endl;
        index++;
        dist_sum += next_dist;
        next_dist = sqrt(pow((poses[index+1].pose.position.x - poses[index].pose.position.x), 2) 
        + pow((poses[index+1].pose.position.y - poses[index].pose.position.y), 2));
    }

    return index;
}

void TrackUnit::print_poses(const std::vector<geometry_msgs::PoseStamped> &poses){
    for (int i = 0; i < poses.size(); i++){
        std::cout << i << ": " << poses[i].pose.position.x << " " << poses[i].pose.position.y << " ";
    }
    std::cout << std::endl;
}

}
