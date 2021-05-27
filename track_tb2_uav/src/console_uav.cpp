#include "ros/ros.h"
#include "track_tb2_uav/Mission.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"uav_console");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<track_tb2_uav::Mission>("uav1/mission");

    ros::service::waitForService("uav1/mission");
    
    track_tb2_uav::Mission m;

    int n;
    bool flag;

    while(ros::ok()){
        std::cout << "mission: 1) takeoff 2) track 3) land" << std::endl;
        std::cin >> n;
        
        if (n==1 || n==2 || n==3){
            m.request.mission = n;
            flag = client.call(m);
            if (flag)
            {
                ROS_INFO("%s", m.response.message.c_str());
            }
            else
            {
                ROS_ERROR("请求处理失败....");
                return 1;
            }
        }
        else{
            ROS_ERROR("error input!");
        }

    }

    return 0;
}