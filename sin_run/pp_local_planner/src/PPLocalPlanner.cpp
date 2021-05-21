#include "pp_local_planner/PPLocalPlanner.h"

namespace TB2
{

PPLocalPlanner::PPLocalPlanner():
    initialized_(false)
{}

PPLocalPlanner::~PPLocalPlanner(){}

PPLocalPlanner::PPLocalPlanner(const std::string name, tf2_ros::Buffer* tf):
    track_index_(0),
    initialized_(false),
    first_arrive_(false),
    need_rotate_(true),
    path_ok_(false),
    twist_ok_(0),
    tf_(tf)
{
    initialize(name ,tf);
}


void PPLocalPlanner::initialize(const std::string name, tf2_ros::Buffer* tf){
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"+name);

    private_nh.param("angular_tolerance", angular_tolerance_, 0.05);
    private_nh.param("dist_tolerance", dist_tolerance_, 0.1);
    private_nh.param("max_fw_looking_dist", max_fw_looking_dist_, 1.0);
    private_nh.param("min_fw_looking_dist", min_fw_looking_dist_, 0.1);
    private_nh.param("fw_looking_factor_", fw_looking_factor_, 4.0);
    
    private_nh.param("prune_radius", prune_radius_, 1.0);
    private_nh.param("world_frame_id", world_frame_id_, std::string("tb2/odom"));
    private_nh.param("base_frame_id", base_frame_id_, std::string("tb2/base_footprint"));    
    private_nh.param("twist_topic", twist_topic_, std::string("mobile_base/commands/velocity"));
    
    twist_sub_ = nh.subscribe<geometry_msgs::Twist>(twist_topic_.c_str(), 2, &PPLocalPlanner::twist_cb, this);

    twist_.linear.x = twist_.linear.y = twist_.angular.z = 0;

    initialized_ = true;
}

// TODO
bool PPLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    ROS_INFO("compute cmd_vel");
    return true;
}

bool PPLocalPlanner::isGoalReachedWithRotate(){
    geometry_msgs::PoseStamped goal = plan_[plan_.size() - 1];
    goal = tf_->transform(goal, base_frame_id_.c_str());
    double dist = sqrt(pow(goal.pose.position.x, 2) + pow(goal.pose.position.y, 2));
    double yaw = tf2::getYaw(goal.pose.orientation);
    yaw = min(yaw, 6.24 - yaw);

    if(dist <= dist_tolerance_ && abs(yaw) <= angular_tolerance_){
        return true;
    }

    return false;
}

bool PPLocalPlanner::isGoalReachedWithDist(){
    geometry_msgs::PoseStamped goal = plan_[plan_.size() - 1];
    goal = tf_->transform(goal, base_frame_id_.c_str());
    double dist = sqrt(pow(goal.pose.position.x, 2) + pow(goal.pose.position.y, 2));
    double yaw = tf2::getYaw(goal.pose.orientation);
    yaw = min(yaw, 6.24 - yaw);

    if(dist <= dist_tolerance_ && abs(yaw) > angular_tolerance_){
        return true;
    }

    return false;
}

// TODO
bool PPLocalPlanner::prunePath(){
    return true;
}

bool PPLocalPlanner::stoped(){
    if (twist_ok_ && twist_.linear.x == 0 && twist_.angular.z == 0){
        return true;
    }
    return false;
}

void PPLocalPlanner::twist_cb(const geometry_msgs::Twist::ConstPtr& msg_p){
    twist_ok = true;
    twist_ = *msg_p;
}


// TODO
const int PPLocalPlanner::get_closest_point_id(){
    double min_dist = DBL_MAX;
    int index = -1;


    for (int i=0; i<poses.size(); i++){
        if(sqrt(pow(poses[i].pose.position.x, 2) + pow(poses[i].pose.position.y, 2)) < min_dist){
            min_dist = sqrt(pow(poses[i].pose.position.x, 2) + pow(poses[i].pose.position.y, 2));
            index = i;
        }
    }

    return index;
}


// TODO
const int PPLocalPlanner::get_track_point_id(){
    int index = get_closest_point_id(poses);

    double dist_sum = 0;
    double next_dist = sqrt(pow((poses[index+1].pose.position.x - poses[index].pose.position.x), 2)
        + pow((poses[index+1].pose.position.y - poses[index].pose.position.y), 2));

    while(index < poses.size()-1 && dist_sum + next_dist < _fw_lk_dist){
        std::cout << next_dist << std::endl;
        index++;
        dist_sum += next_dist;
        next_dist = sqrt(pow((poses[index+1].pose.position.x - poses[index].pose.position.x), 2) 
        + pow((poses[index+1].pose.position.y - poses[index].pose.position.y), 2));
    }

    return index;
}

bool PPLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
    if (plan.size() < 1 || plan[0].header.frame_id != world_frame_id_){
        return false;
    }
    plan_ = plan;
    return true;
}

}
