#include "sin_runner_tb2/Tb2Controller.h"

namespace TB2
{

Tb2Controller::Tb2Controller(){}
Tb2Controller::~Tb2Controller(){}

Tb2Controller::Tb2Controller(const std::string name, tf2_ros::Buffer* tf):
    state_(PAUSE),
    tf_(tf),
    plp_p_(new PPLocalPlanner("local_planner", tf))
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"+name);

    private_nh.param("abs_y", abs_y_, 2.5);
    private_nh.param("a_x", a_x_, -2.5);
    private_nh.param("b_x", b_x_, 2.5);

    initialize_path();

    control_server_ = private_nh.advertiseService("Trigger", &Tb2Controller::control_cb, this);;

    private_nh.param("vel_topic", vel_topic_, std::string("cmd_vel_mux/input/teleop"));
    private_nh.param("world_frame_id", world_frame_id_, std::string("world"));    

    vel_pub_ = nh.advertise<geometry_msgs::Twist>(vel_topic_.c_str(),10);

    mainloop_timer_ = nh.createTimer(ros::Duration(0.1), &Tb2Controller::mainloop_cb, this);

}

void Tb2Controller::mainloop_cb(const ros::TimerEvent &event){
    geometry_msgs::Twist cmd;

    switch(state_){

        case PAUSE:
            publishZeroVelocity();
            break;

        case RUN:
            if(plp_p_->isGoalReached()){
                // switch path
                if (path_selected_ == 1){
                    plp_p_->setPlan(path_2_);
                    path_selected_ = 2;
                }
                else{
                    plp_p_->setPlan(path_1_);
                    path_selected_ = 1;
                }
            }
            if (plp_p_->computeVelocityCommands(cmd)){
                vel_pub_.publish(cmd);
            }
            else{
                 ROS_ERROR("No proper local velocity. Stop!");
                 publishZeroVelocity();
            }
            break;

        default :
            ROS_ERROR("Imposible state of control.");
            break;
    }
}

bool Tb2Controller::control_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp){
    if (req.data){ // start
        if(state_ == RUN){
            resp.success = false;
            resp.message = "TB2 has been running.";
        }
        else{
            state_ = RUN;
            resp.success = true;
            resp.message = "TB2 is running now.";
        }
    }
    else{
        if(state_ == PAUSE){
            resp.success = false;
            resp.message = "TB2 has been stoped.";
        }
        else{
            state_ = PAUSE;
            resp.success = true;
            resp.message = "TB2 is stoping.";
        }
    }
}

bool Tb2Controller::initialize_path(){
    ROS_INFO("initialize_path.");
    // temp
    geometry_msgs::PoseStamped temp;
    tf2::toMsg(tf2::Transform::getIdentity(), temp.pose);
    temp.header.frame_id = world_frame_id_;
    // init
     if (a_x_ > b_x_){
        double c = a_x_;
        a_x_ = b_x_;
        b_x_ = c;
    }
    abs_y_ = abs(abs_y_);
    path_1_.clear();
    path_2_.clear();
    // push_back
    for(double x=a_x_; x<=b_x_; x+=0.1){
        temp.pose.position.x = x;
        temp.pose.position.y = abs_y_*sin(x);
        path_1_.push_back(temp);
    }
    // goal push
    temp.pose.position.x = b_x_;
    temp.pose.position.y = abs_y_*sin(b_x_);
    path_1_.push_back(temp);
    // path_2_
    path_2_ = path_1_;
    reverse(path_2_.begin(), path_2_.end());

    path_selected_ = 1;
    return true;
}

void Tb2Controller::publishZeroVelocity(){
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.angular.z = 0;
    vel_pub_.publish(twist);
}


}




