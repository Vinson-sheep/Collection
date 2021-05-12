/// @file NaiveController.h
/// @author Vinson Sheep (775014077@qq.com)
/// @brief 写一个针对turtlebot3 burger的傻瓜控制器
/// @version 0.1
/// @date 2021-05-11
/// 
/// @copyright Copyright (c) 2021
/// 

#ifndef NAIVECONTROLLER_H_
#define NAIVECONTROLLER_H_

#include "ros/ros.h"
#include "tf2_ros/buffer.h" // buffer
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include "forward_runner_ugv/Command.h"
#include "geometry_msgs/PoseStamped.h"
#include "forward_runner_ugv/Smoother.h"
#include <string>
#include <memory.h> // shared_ptr
#include "tf2/LinearMath/Quaternion.h"
#include "math.h" // abs, atan
#include <tf2/utils.h> // getYaw
#include <nav_msgs/Odometry.h>

namespace Naive_Planner
{

/// @brief 针对turtlebot3 burger的傻瓜控制器
/// @details 该控制器不能够实现避障，利用单线程完成所有任务，能够中断之前的目标。当一个新目标到达的时候，
/// 会暂停当前任务，该任务可以是原地旋转或者直线移动，然后转入新的任务循环中。
class NaiveController
{
public:
	/// @brief 状态机 
	/// 
	enum State
	{
		ROTATE, ///< 原地旋转
		MOVE	///< 直线移动
	};
	
	/// @brief 构造函数
	NaiveController(tf2_ros::Buffer* tf);

	/// @brief 析构函数
	~NaiveController();

private:

	/// @name 禁止复制和默认构造
	/// @{
	NaiveController();
	NaiveController(const NaiveController &rhs);
	NaiveController& operator=(const NaiveController &rhs);
	/// @}


	/// @name 参数
	/// @{
	string world_frame_id_;
	string ugv_frame_id_;
	string ugv_name_;	///< 小车的名字，用于打印数据
	bool sim_mode_;	///< 是否仿真
	string vicon_pose_name_;	///< vicon发送的pose话题名称，用于打印位姿
	/// @}

	ros::Subscriber odom_sub_;
	ros::Subscriber vicon_pose_sub_;

	/// @brief 是否有新的目标 
	/// 
	bool new_goal_;
	/// @brief 目前是否有目标点 
	/// @details 刚开始启动burger的时候是没有目标点的，此时为false
	bool have_goal_;
	/// @brief odom数据是否准备好了 
	/// 
	bool odom_ready_;

	/// @brief 用于存储最新的odom 
	/// 
	nav_msgs::Odometry odom_;
	/// @brief 用于存储最新的vicon_pose 
	/// 
	geometry_msgs::PoseStamped vicon_pose_;

	tf2_ros::Buffer* tf_;

	/// @brief 记录控制器当前状态
	/// 
	State state_;

	/// @brief 停止动作的计数器 
	/// @details 当需要的停止的时候就会计数三次，表示发送三次零速度
	int stop_count_ = 0;


	ros::NodeHandle private_nh_;
	ros::NodeHandle nh_;

	/// @brief 主循环计时器 
	/// 
	ros::Timer mainloop_timer_;

	/// @brief 目标订阅器
	/// 
	ros::Subscriber goal_sub_;

	/// @brief 全局目标点 
	/// @details 将新的目标点转换为该标准形式进行存储
	geometry_msgs::PoseStamped global_goal_;

	/// @brief 局部目标点
	/// @details 将global_goal转变为相对于小车的位姿再进行存储
	geometry_msgs::PoseStamped local_goal_;
	
	/// @brief 速度平滑器 
	/// @details 无视了小车加速度，简单实现了速度的平滑控制，使得小车的速度不会变化太大
	Smoother::Ptr sm_ptr_;

	/// @brief 主循环回调函数
	/// @details 负责转换global_goal，并根据状态发出不同的速度信号
	/// @param event 略
	void mainloop_cb(const ros::TimerEvent &event);

	/// @brief 命令订阅器回调函数
	/// @details 讲原始命令存储到global_pose_
	/// @param command 来自终端的命令
	void goal_cb(const forward_runner_ugv::Command::ConstPtr &command);

	/// @brief odom数据回调 
	/// 
	/// @param odom 略
	void odom_cb(const nav_msgs::Odometry::ConstPtr &odom);
	/// @brief vicon传来的pose的数据回调 
	/// 
	/// @param pose 略
	void vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose);

};

}

#endif


