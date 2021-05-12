#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "forward_runner_ugv/NaiveController.h"


int main( int argc, char** argv )
{   
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener(buffer);
    
    Naive_Planner::NaiveController controller(&buffer);
    
    ros::spin();
    return 0;
}