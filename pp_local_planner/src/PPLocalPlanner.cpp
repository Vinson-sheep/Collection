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
    twist_ok_(false),
    tf_(tf),
    sm_ptr_(new Smoother("smoother"))
{
    initialize(name ,tf);
}


void PPLocalPlanner::initialize(const std::string name, tf2_ros::Buffer* tf){
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"+name);

    private_nh.param("angular_tolerance", angular_tolerance_, 0.05);
    private_nh.param("dist_tolerance", dist_tolerance_, 0.1);
    private_nh.param("max_fw_looking_dist", max_fw_looking_dist_, 0.5);
    private_nh.param("min_fw_looking_dist", min_fw_looking_dist_, 0.5);
    private_nh.param("fw_looking_factor", fw_looking_factor_, 4.0);
    
    private_nh.param("prune_radius", prune_radius_, 1.0);
    private_nh.param("world_frame_id", world_frame_id_, std::string("tb2/odom"));
    private_nh.param("base_frame_id", base_frame_id_, std::string("tb2/base_footprint"));    
    private_nh.param("twist_topic", twist_topic_, std::string("mobile_base/commands/velocity"));
    
    private_nh.param("move_factor", move_factor_, 2.0);
    private_nh.param("rotate_factor", rotate_factor_, 2.0);
    

    twist_sub_ = nh.subscribe<geometry_msgs::Twist>(twist_topic_.c_str(), 2, &PPLocalPlanner::twist_cb, this);

    twist_.linear.x = twist_.angular.z = 0;

    initialized_ = true;
}

bool PPLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    // check
    if (plan_.size() < 1){
        cmd_vel.linear.x = cmd_vel.angular.z = 0;
        return false;
    }
    // reach with rotate
    if(isGoalReachedWithRotate()){
        cmd_vel.linear.x = cmd_vel.angular.z = 0;
        return true;
    }
    // reach with distance but not rotate completedly
    if(isGoalReachedWithDist()){
        if(first_arrive_ && !stoped()){
            cmd_vel.linear.x = cmd_vel.angular.z = 0;
            return true;
        }
        else{
            first_arrive_ = false;
            // angular
            geometry_msgs::PoseStamped goal = plan_[plan_.size() - 1];
            goal = tf_->transform(goal, base_frame_id_.c_str());
            double yaw_diff = tf2::getYaw(goal.pose.orientation);
            
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = rotate_factor_*yaw_diff;

            sm_ptr_->smooth(0, twist_.angular.z, cmd_vel.linear.x, cmd_vel.angular.z);

            return true;
        }
    }
    // if not in the range
    int cur_track_goal_id = get_track_point_id(); 
    if (cur_track_goal_id == -1){
        cmd_vel.linear.x = cmd_vel.angular.z = 0;
        return true;;
    }
    // ROS_INFO("%d", cur_track_goal_id);
    geometry_msgs::PoseStamped cur_track_goal = plan_[cur_track_goal_id];
    // ROS_INFO("%f, %f", cur_track_goal.pose.position.x, cur_track_goal.pose.position.y);
    cur_track_goal = tf_->transform(cur_track_goal, base_frame_id_.c_str());
    // ROS_INFO("%f, %f", cur_track_goal.pose.position.x, cur_track_goal.pose.position.y);
    if (need_rotate_){ // rotate at first
        double direct_diff = atan2(cur_track_goal.pose.position.y,cur_track_goal.pose.position.x);
        // direct_diff = min(direct_diff, 6.28-direct_diff);
        if (abs(direct_diff) < angular_tolerance_){
            if (stoped()){
                need_rotate_ = false;
            }
            cmd_vel.linear.x = cmd_vel.angular.z = 0;
            return true;
        }
        else{
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = rotate_factor_*direct_diff;
            return true;
        }
    }

    double dist_diff = sqrt(pow(cur_track_goal.pose.position.x, 2) + pow(cur_track_goal.pose.position.y, 2));
    double direct_diff = atan2(cur_track_goal.pose.position.y,cur_track_goal.pose.position.x);
    // smooth
    cmd_vel.linear.x = move_factor_*dist_diff;
    cmd_vel.angular.z = rotate_factor_*direct_diff;

    sm_ptr_->smooth(sqrt(pow(twist_.linear.x, 2) + pow(twist_.linear.y, 2)),
                    twist_.angular.z,
                    cmd_vel.linear.x,
                    cmd_vel.angular.z);

    return true;
    
}

bool PPLocalPlanner::isGoalReachedWithRotate(){
    geometry_msgs::PoseStamped goal = plan_[plan_.size() - 1];
    goal = tf_->transform(goal, base_frame_id_.c_str());
    double dist = sqrt(pow(goal.pose.position.x, 2) + pow(goal.pose.position.y, 2));
    double yaw = tf2::getYaw(goal.pose.orientation);
    // yaw = min(yaw, 6.24 - yaw);

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
    // yaw = min(yaw, 6.24 - yaw);

    if(dist <= dist_tolerance_ && abs(yaw) > angular_tolerance_){
        return true;
    }

    return false;
}

bool PPLocalPlanner::prunePath(){
    // check
    if (plan_.size() <= 1){
        return true;
    }
    // prune
    geometry_msgs::PoseStamped temp;
    double dist;
    while (plan_.size() > 1){
        temp = tf_->transform(plan_[0], base_frame_id_.c_str());    
        dist = sqrt(pow(temp.pose.position.x, 2) + pow(temp.pose.position.y, 2));
        if(dist > prune_radius_){
            plan_.erase(plan_.begin());
        }
        else{
            break;
        }
    }


    return true;
}

bool PPLocalPlanner::stoped(){
    if (twist_ok_ && twist_.linear.x == 0 && twist_.angular.z == 0){
        return true;
    }
    return false;
}

void PPLocalPlanner::twist_cb(const geometry_msgs::Twist::ConstPtr& msg_p){
    twist_ok_ = true;
    twist_ = *msg_p;
}

const int PPLocalPlanner::get_closest_point_id(){
    double min_dist = DBL_MAX;
    int index = -1;
    // 30 points at bests
    int n = std::min(double(plan_.size()), 50.0);

    geometry_msgs::PoseStamped temp;
    double dist;
    

    for (int i=0; i<n; i++){
        // transform into local pose
        temp = tf_->transform(plan_[i], base_frame_id_.c_str());    
        dist = sqrt(pow(temp.pose.position.x, 2) + pow(temp.pose.position.y, 2));
        // update min_dist
        if(dist < min_dist){
            min_dist = dist;
            index = i;
        }
    }

    return index;
}


const int PPLocalPlanner::get_track_point_id(){\
    // check
    if (plan_.size() == 1){
        return 0;
    }
    else if (plan_.size() == 0){
        return -1;
    }
    // get closest pose
    int index = get_closest_point_id();

    // ROS_INFO("index1 %d", index);

    if (!(index+1 < plan_.size())){
        return index;
    }
    // calculate forward looking distance
    double fw_lk_dist;
    if (!twist_ok_){
        fw_lk_dist = min_fw_looking_dist_;
    }
    else{
        double vel_x = sqrt(pow(twist_.linear.x, 2) + pow(twist_.linear.y, 2));
        fw_lk_dist = vel_x * fw_looking_factor_;
        fw_lk_dist = min(fw_lk_dist, max_fw_looking_dist_);
        fw_lk_dist = max(fw_lk_dist, min_fw_looking_dist_);
    }
    // ROS_INFO("fw_lk_dist %f", fw_lk_dist);
    // find tracking pose
    double dist_sum = 0;
    
    double next_dist = sqrt(pow((plan_[index+1].pose.position.x - plan_[index].pose.position.x), 2)
        + pow((plan_[index+1].pose.position.y - plan_[index].pose.position.y), 2));

    // ROS_INFO("next_dist %f", next_dist);

    while(dist_sum + next_dist < fw_lk_dist){
        // std::cout << next_dist << std::endl;
        index++;
        dist_sum += next_dist;
        if (!(index+1 < plan_.size())){
            next_dist = sqrt(pow((plan_[index+1].pose.position.x - plan_[index].pose.position.x), 2)
            + pow((plan_[index+1].pose.position.y - plan_[index].pose.position.y), 2));
        }
        else{
            break;
        }
    }

    // ROS_INFO("index2 %d", index);

    return index;
}

bool PPLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
    if (plan.size() < 1 || plan[0].header.frame_id != world_frame_id_){
        return false;
    }
    first_arrive_ = true;
    need_rotate_ = true;
    plan_ = plan;
    path_ok_ = true;
    return true;
}

}
