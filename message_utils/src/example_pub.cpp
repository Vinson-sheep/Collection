#include "ros/ros.h"
#include "message_utils/message_utils.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"talker");
    ros::NodeHandle nh;
    ros::Rate r(1);

    while(ros::ok()){

        MP::message_pub(message_utils::Message::NORMAL, "uav1", "hello, world");
        MP::message_pub(message_utils::Message::WARN, "uav2", "hello, world");
        MP::message_pub(message_utils::Message::ERROR, "uav3", "hello, world");
        
        r.sleep();
        ros::spinOnce();
    }


    return 0;
}