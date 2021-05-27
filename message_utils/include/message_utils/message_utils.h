/// @file message_utils.h
/// @author Vinson Sheep (775014077@qq.com)
/// @brief message utilities for ROS
/// @version 1.0
/// @date 2021-05-27
/// 
/// @copyright Copyright (c) 2021
/// 
#include "ros/ros.h"
#include <string>
#include <message_utils/Message.h>

namespace MP
{

void message_pub(const char message_type, const std::string source_node, const std::string content);

void message_print(const message_utils::Message& message);

}


