/*
速度平滑器，针对差速控制器做优化。

假设传入一个很大的初速度和角速度，turtlebot3有可能只执行转向，或者不动。这是因为turtlebot3有速度限制，并且不能立即达到速度要求。
速度平滑器限定了传输给小车的实际控制速度最大值，并且对速度进行增量平滑处理，使得任意目标速度都能有效执行。
*/

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include <algorithm>

const float MAX_LIN_VEL = 0.21;	// 最大线速度
const float MAX_ANG_VEL = 0.84;	// 最大角速度
const float LIN_STEP_SIZE = 0.01;	// 线速度步长
const float ANG_STEP_SIZE = 0.1;	// 角速度步长
const float RATE = 10;	// 发布频率

extern std::string CONTROL_TOPIC;
using std::min;
using std::max;


/*
速度平滑器类
*/
class Controller
{
private:
	float control_linear_vel = 0.0;	// 实际控制线速度
	float control_angular_vel = 0.0;	// 实际控制角速度

public:
	Controller();	// 构造函数
	virtual ~Controller();	// 析构函数
	// 平滑处理函数
	const float make_simple_profile(const float input, const float output, const float slot);
	// 速度限制函数
	const float constrain(const float input, const float low, const float high);
	// 控制消息发布函数
	void set(const float target_linear_vel, const float target_angular_vel);
};

#endif


