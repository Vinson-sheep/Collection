#include "forward_runner_ugv/NaiveController.h"

namespace Naive_Planner
{

NaiveController::NaiveController(){}

NaiveController::~NaiveController(){}

NaiveController::NaiveController(tf2_ros::Buffer* tf):
    new_goal_(false),
    have_goal_(false),
    tf_(tf),
    state_(ROTATE),
    private_nh_("~"),
    sm_ptr_(new Smoother(nh_, "cmd_vel"))
{
    private_nh_.param("world_frame_id", world_frame_id_, std::string("ugv1/odom"));
    private_nh_.param("ugv_frame_id", ugv_frame_id_, std::string("ugv1/base_footprint"));
    private_nh_.param("ugv_name", ugv_name_, std::string("ugv1")); 
    
    mainloop_timer_ = nh_.createTimer(ros::Duration(0.25), &NaiveController::mainloop_cb, this);
    goal_sub_ = nh_.subscribe<forward_runner_ugv::Command>("control_cmd", 2, &NaiveController::goal_cb, this);

    ROS_INFO("%s is waiting a goal.", ugv_name_.c_str());

}

void NaiveController::mainloop_cb(const ros::TimerEvent &event){
    // ROS_INFO("main loop is running");
    // 如果有新目标，则发送0速度，然后转到ROTATE
    if (new_goal_){
        sm_ptr_->set(0, 0);
        stop_count_++;
        if (stop_count_ > 3){
            state_ = ROTATE;
            new_goal_ = false;
            stop_count_ = 0;
        }
        return;
    }
    // 如果有目标，则将global_goal_转为local_goal
    if (have_goal_){
        try
        {
            // ROS_INFO("%s %s", global_goal_.header.frame_id.c_str(), ugv_frame_id_.c_str());
            local_goal_ = tf_->transform(global_goal_, ugv_frame_id_.c_str());
            // // 打印转换后的结果
            // ROS_INFO("%s, x=%f, y=%f, ox=%f, oy=%f, oz=%f, ow=%f", 
            // local_goal_.header.frame_id.c_str(),
            // local_goal_.pose.position.x,
            // local_goal_.pose.position.y,
            // local_goal_.pose.orientation.x,
            // local_goal_.pose.orientation.y,
            // local_goal_.pose.orientation.z,
            // local_goal_.pose.orientation.w);

        }
        catch(const std::exception& e)
        {
            ROS_INFO("程序异常:%s",e.what());
            return;
        }
    }
    else{
        return;
    }

    // 准备数据
    double dist = sqrt(pow(local_goal_.pose.position.x, 2) + pow(local_goal_.pose.position.y, 2));
    double direct = atan2(local_goal_.pose.position.y,local_goal_.pose.position.x);
    double yaw = tf2::getYaw(local_goal_.pose.orientation);

    ROS_INFO("DIST %f, YAW %f, direct %f", dist, yaw, direct);

    switch(state_){
        case ROTATE:
            // 如果已经到达目标，则调整角度
            if (dist < 0.05){
                sm_ptr_->set(0, 0.5*yaw);
            }
            // 如果未到达
            else{
                // 如果方向已经调整好，则发送0速度，转移到MOVE
                if (abs(direct) < 0.1){
                    sm_ptr_->set(0, 0);
                    state_ = MOVE;
                }
                // 否则，原地调整角度
                else{
                    sm_ptr_->set(0, direct);
                }
            }
            break;
        case MOVE:
            // 如果已经到达目标，则发送0速度，转移到ROTATE
            if (dist < 0.05){
                sm_ptr_->set(0, 0);
                state_ = ROTATE;
            }
            // 否则，发送正常速度
            else{
                sm_ptr_->set(2.0*dist, direct);
            }
            break;
    }
}
void NaiveController::goal_cb(const forward_runner_ugv::Command::ConstPtr &command){
    // 打印目标点
    ROS_INFO("%s 接受目标 (%f, %f, %f).", ugv_name_.c_str(), command->x, command->y, command->yaw);
    // 更新状态
    new_goal_ = true;
    have_goal_ = true;
    // 刷新global_goal_
    global_goal_.header.frame_id = world_frame_id_;
    global_goal_.header.stamp = ros::Time(0);
    global_goal_.pose.position.x = command->x;
    global_goal_.pose.position.y = command->y;
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,command->yaw * M_PI/180);
    global_goal_.pose.orientation.x = qtn.getX();
    global_goal_.pose.orientation.y = qtn.getY();
    global_goal_.pose.orientation.z = qtn.getZ();
    global_goal_.pose.orientation.w = qtn.getW();

    //打印转换后的结果
    // ROS_INFO("%s, x=%f, y=%f, ox=%f, oy=%f, oz=%f, ow=%f", 
    //     global_goal_.header.frame_id.c_str(),
    //     global_goal_.pose.position.x,
    //     global_goal_.pose.position.y,
    //     global_goal_.pose.orientation.x,
    //     global_goal_.pose.orientation.y,
    //     global_goal_.pose.orientation.z,
    //     global_goal_.pose.orientation.w);
}

}


