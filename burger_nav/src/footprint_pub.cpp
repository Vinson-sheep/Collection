#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
#include <vector>
#include "geometry_msgs/Point32.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"footprint_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Polygon>("/costmap_node/costmap/footprint",10);

    //5.组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    std::vector<geometry_msgs::Point32> msg;

    geometry_msgs::Point32 point1;
    geometry_msgs::Point32 point2;
    geometry_msgs::Point32 point3;
    geometry_msgs::Point32 point4;

    point1.x = -0.105;
    point1.y = -0.105;
    point1.z = 0;

    point2.x = -0.105;
    point2.y = 0.105;
    point2.z = 0;

    point3.x = 0.041;
    point3.y = 0.105;
    point3.z = 0;

    point4.x = 0.041;
    point4.y = -0.105;
    point4.z = 0;

    msg.push_back(point1);
    msg.push_back(point2);
    msg.push_back(point3);
    msg.push_back(point4);

    //逻辑(一秒10次)
    ros::Rate r(10);

    geometry_msgs::Polygon p;
    p.points = msg;

    //节点不死
    while (ros::ok())
    {
        //发布消息
        pub.publish(p);
        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        //暂无应用
        ros::spinOnce();
    }

    return 0;
}