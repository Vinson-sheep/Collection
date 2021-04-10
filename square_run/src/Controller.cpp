#include "square_run/Controller.h"

Controller::Controller()
{
}

Controller::~Controller()
{
}

/*
平滑处理函数

输入：
    input: 目标速度
    output: 现在的控制速度
    slot: 步长
输出：
    新的控制速度
*/
const float Controller::make_simple_profile(const float input, const float output, const float slot){
    if (input > output){
        return min(input, output + slot/2);
    }
    else if (input < output){
        return max(input, output - slot/2);
    }
    return output;
}

/*
速度限制函数

输入：
    input: 目标速度
    low: 速度下限
    high: 速度上限
输出：
    修正后的目标速度
*/
const float  Controller::constrain(const float input, const float low, const float high){
    if (input > high)
        return high;
    else if (input < low)
        return low;
    return input;
}


/*
控制消息发布函数

输入：
    target_linear_vel： 目标线速度
    target_angular_vel：目标角速度
输出：
    无
*/
void  Controller::set(const float target_linear_vel, const float target_angular_vel){
    // 初始化变量
    ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(CONTROL_TOPIC.c_str(),1000);
    static ros::Rate rate(RATE);
    geometry_msgs::Twist twist;
    // 给twist赋值
    if (target_linear_vel < 10e-4 && abs(target_angular_vel) < 10e-4){  // 表示[STOP]任务
        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0;
        control_linear_vel = control_angular_vel = 0;
    }
    else {  // 如果[ROTATE]或者[MOVE]任务
        float target_linear_vel_new = constrain(target_linear_vel, -MAX_LIN_VEL, MAX_LIN_VEL);  // 对线速度进行修正
        float target_angular_vel_new = constrain(target_angular_vel, -MAX_ANG_VEL, MAX_ANG_VEL);    // 对角速度进行修正
        
        // 对线速度进行赋值
        twist.linear.x = control_linear_vel = make_simple_profile(target_linear_vel_new, control_linear_vel, LIN_STEP_SIZE);
        twist.linear.y = twist.linear.z = 0;
        // 对角速度进行赋值
        twist.angular.x = twist.angular.y = 0;
        twist.angular.z = control_angular_vel = make_simple_profile(target_angular_vel_new, control_angular_vel, ANG_STEP_SIZE);
        // 打印赋值结果
        ROS_INFO("linear %.2f angular %.2f", twist.linear.x, twist.angular.z);
    }
    // 发布控制消息
    pub.publish(twist);
    ros::spinOnce();
    rate.sleep();
}