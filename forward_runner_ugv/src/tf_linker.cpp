// 该node实现了tf添加

// vehicle_frame_id_1   --- world_frame_id
// vehicle_frame_id_2   --- odom_frame_id
// output_frame_id
// 
// 最终生成了 ouput_frame_id -> odom_frame_id
// outpur_frame_id === world_frame_id


#include "ros/ros.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "tf_linker");
    ros::NodeHandle private_nh;
    ros::Rate r(10);
    // 获取参数
    string vehicle_frame_id_1, vehicle_frame_id_2, world_frame_id, 
        odom_frame_id， output_frame_id;
    private_nh.param("vehicle_frame_id_1", vehicle_frame_id_1, std::string("vicon/ugv1/ugv1"));
    private_nh.param("vehicle_frame_id_2", vehicle_frame_id_2, std::string("ugv1/base_footprint"));
    private_nh.param("world_frame_id", world_frame_id, std::string("world"));
    private_nh.param("odom_frame_id", odom_frame_id, std::string("ugv1/odom"));
    private_nh.param("output_frame_id", output_frame_id, std::string("ugv1/map"));
    // 初始化buffer
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener(buffer);
    tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::TransformStamped tfs;

    while(ros::ok()){

// TODO
        tf2::toMsg(tf2::Transform::getIdentity(), pose.pose);
        // 第一次转换
        pose.header.frame_id = world_frame_id;
        pose.header.time = ros::Time(0);
        try
        {
            pose = buffer.transform(pose, vehicle_frame_id_1.c_str());
        }
        catch(const std::exception& e)
        {
            ROS_INFO("程序异常:%s",e.what());
            r.sleep();
            ros::spinOnce();
            continue;
        }
        // 第二次转换
        pose.header.frame_id = vehicle_frame_id_2;
        pose.header.time = ros::Time(0);
        try
        {
            pose = buffer.transform(pose, odom_frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            ROS_INFO("程序异常:%s",e.what());
            r.sleep();
            ros::spinOnce();
            continue;
        }
        // 发送tf
        tfs.header.frame_id = odom_frame_id;
        tfs.header.time = ros::Time::now();
        tfs.child_frame_id = output_frame_id;
        tfs.transform.translation.x = pose.pose.position.x;
        tfs.transform.translation.y = pose.pose.position.y;
        tfs.transform.translation.z = pose.pose.position.z;
        tfs.transform.rotation = pose.pose.orientation;
        broadcaster.sendTransform(tfs);

        // sleep
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}