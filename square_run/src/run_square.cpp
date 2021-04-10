#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include <sstream>
#include <algorithm>
#include <string>
#include "square_run/Controller.h"

using std::string;

// 定义全局变量
string TURTLEBOT_FRAME_ID;
string ODOM_FRAME_ID;
string CONTROL_TOPIC;

// 定义全局常量
const double ALPHA = 1; // 线速度因子
const double BATA = 2;  // 角速度因子
const double ANGULAR_ERROR = 0.1;   // 角度误差限，单位rad
const double LINEAR_ERROR = 0.1;    // 距离误差限，单位m
enum mission {ROTATE, MOVE, STOP};  // 任务类型

/*
将来自odom/world的点转变为相对于turtlebot3的点
并且将计算得到的线速度和角速度传给速度平滑器

如果没有速度平滑器，那么传入的速度信息得不到有效执行
*/
bool execuate(const geometry_msgs::PointStamped &point, mission m){
    // 初始化tf相关的工具类
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener listener(buffer);
    // 初始化速度平滑器
    static Controller ctr;
    // 转换点坐标，计算速度信息，传递给速度平滑器
    try{
        // 转换点坐标
        geometry_msgs::PointStamped point_tur = point; // 由于point不可变，所以新建变量point_tur，表示相对于turtlebot的点坐标
        point_tur.header.stamp = ros::Time(0);
        point_tur = buffer.transform(point_tur, TURTLEBOT_FRAME_ID.c_str());
        ROS_INFO("坐标点相对于 turtle 的坐标为:(%.2f,%.2f,%.2f)",point_tur.point.x,point_tur.point.y,point_tur.point.z); // 打印转换后的点坐标

        if (m == ROTATE){   // 如果是[旋转]任务
            ROS_INFO("ROTATE");
            ctr.set(0, BATA * atan2(point_tur.point.y,point_tur.point.x)); // 线速度为0，角速度动态变化
            if (abs(atan2(point_tur.point.y,point_tur.point.x)) < ANGULAR_ERROR){   // 如果角度小于角度误差限，说明已经转向完成
                return true;
            }

        }
        else if (m == MOVE){    // 如果是[移动]任务
            ROS_INFO("MOVE");
            ctr.set(ALPHA*sqrt(pow(point_tur.point.x,2) + pow(point_tur.point.y,2)), BATA * atan2(point_tur.point.y,point_tur.point.x));    // 线速度和角速度动态变化
            if (sqrt(pow(point_tur.point.x,2) + pow(point_tur.point.y,2)) < LINEAR_ERROR){  // 如果距离小于距离误差限，说明已经到达目的地
                return true;
            }
        }
        else if (m == STOP){    // 如果是[停止]任务
            ROS_INFO("STOP");
            ctr.set(0, 0);
        }

    }
    catch(const std::exception& e){
        // 程序执行初期可能还没有存储tf数据，因此转换失败
        ros::Rate(10).sleep();
        ROS_INFO("程序异常:%s",e.what());
    }

    return false;
}


/*
获取参数，执行循环
*/
int main(int argc, char **argv){
    // 初始化操作
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "turbot_ctrl_unit");
    ros::NodeHandle nh("~");

    // 获取参数
    int num;
    nh.getParam("pos_num", num);
    nh.getParam("turtlebot_frame_id", TURTLEBOT_FRAME_ID);
    nh.getParam("odom_frame_id", ODOM_FRAME_ID);
    nh.getParam("control_topic", CONTROL_TOPIC);
    // 获取点坐标
    geometry_msgs::PointStamped arr[num];
    for (int i = 0; i < num; i++)
    {
        arr[i].header.stamp = ros::Time::now();
        arr[i].header.frame_id = ODOM_FRAME_ID;

        std::stringstream s1, s2;
        s1 << "pos" << i << "_x";
        s2 << "pos" << i << "_y";
        nh.getParam(s1.str().c_str(), arr[i].point.x);
        nh.getParam(s2.str().c_str(), arr[i].point.y);
        arr[i].point.z = 0;

        ROS_INFO("数据为 frame_id: %s; x: %.2f, y: %.2f, z: %.2f",
                arr[i].header.frame_id.c_str(),
                arr[i].point.x,
                arr[i].point.y,
                arr[i].point.z
            );
    }
    // 走方阵的主循环
    // 旋转->暂停->走直线->暂停->......
    while(ros::ok()){
        for (int i = 0; i < num && ros::ok(); i++){
            while(ros::ok() && !execuate(arr[i], ROTATE));
            for (int j=0; j<5 ;j++){
                arr[i].header.frame_id = ODOM_FRAME_ID;
                execuate(arr[i], STOP);
            }
            while(ros::ok() && !execuate(arr[i], MOVE));
            for (int j=0; j<5 ;j++){
                execuate(arr[i], STOP);
            }
        }
    }

    return 0;
}