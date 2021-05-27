/// @file message_utils.cpp
/// @author Vinson Sheep (775014077@qq.com)
/// @brief message utilities for ROS
/// @version 1.0
/// @date 2021-05-27
/// 
/// @copyright Copyright (c) 2021
/// 

#include "message_utils/message_utils.h"

using namespace std;

namespace MP
{

void message_pub(const char message_type, const std::string source_node, const std::string content){
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<message_utils::Message>("message",10);
    static message_utils::Message msg;

    msg.header.frame_id = source_node;
    msg.header.stamp = ros::Time::now();
    msg.message_type = message_type;
    msg.source_node = source_node;
    msg.content = content;
    
    pub.publish(msg);
}

void message_print(const message_utils::Message& message)
{
    if(message.message_type ==message_utils::Message::NORMAL)
    {
        cout << "[NORMAL]" << "["<< message.source_node << "]:" << message.content <<endl;
    }
    else if(message.message_type == message_utils::Message::WARN)
    {
        cout << "[WARN]" << "["<< message.source_node << "]:" <<message.content <<endl;
    }
    else if(message.message_type == message_utils::Message::ERROR)
    {
        cout << "[ERROR]" << "["<< message.source_node << "]:" << message.content <<endl;
    }
}

}
