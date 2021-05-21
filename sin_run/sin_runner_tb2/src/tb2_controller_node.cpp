#include "ros/ros.h"
#include "sin_runner_tb2/Tb2Controller.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "controller_node");

    ros::NodeHandle nh;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener(buffer);

    TB2::Tb2Controller tc("controller", &buffer);

    ros::spin();
    return 0;
}