#include "ros/ros.h"
#include "std_srvs/SetBool.h"


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "station"); 
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("tb2/start");
    ros::service::waitForService("tb2/start");

    int n;
    std_srvs::SetBool msg;
    bool flag;

    while(ros::ok()){
        std::cout << "请输入指令： 1） 开始 2）停止" << std::endl; 
        std::cin >> n;

        switch(n){
            case 1:
                msg.request.data = true;
                flag = client.call(msg);
                if (msg.response.success){
                    std::cout << msg.response.message;
                }
                break;
            case 2:
                msg.request.data = false;
                flag = client.call(msg);
                if (msg.response.success){
                    std::cout << msg.response.message;
                }
                break;
            default:
                std::cout << "错误输入！" << std::endl;
        }
    }

    return 0;
}