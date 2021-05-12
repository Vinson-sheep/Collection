#include "ros/ros.h"
#include <iostream>
#include "forward_runner_ugv/Command.h"

using namespace std;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"terminal");
    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<forward_runner_ugv::Command>("/ugv1/control_cmd", 2);
    
    forward_runner_ugv::Command cmd;

    while(ros::ok()){
        cout << "Please input a new goal (x(meter) y(meter) yaw(degree)):" << endl;
        cin >> cmd.x >> cmd.y >> cmd.yaw;
        goal_pub.publish(cmd);
        ros::spinOnce();
    }

    return 0;
}