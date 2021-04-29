#ifndef _ASTAR_2D_H_
#define	_ASTAR_2D_H_

#include "ros/ros.h"
#include "burger_nav/Occupancy_2d_grid.h"
#include <Eigen/Eigen>

class Astar_2d
{
private:
	
public:
	enum
	{
		REACH_END = 1,
		NO_PATH = 2
	};


	Astar_2d();
	~Astar_2d();
	
	// 重置
	void reset();
	// // 初始化
	void init(ros::NodeHandle& nh);
	// // 检查安全性
	// bool check_safety(Eigen::Vector3d &cur_pos, double safe_distance);
	// 搜索
	int search(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt);
	// 返回路径 - 活的
	const std::vector<Eigen::Vector3d> getPath();
	// 返回ros消息格式的路径
	const nav_msgs::Path get_ros_path();
	// 返回访问过的节点
	// // std::vector<NodePtr> getVisitedNodes();

	typedef std::shared_ptr<Astar> Ptr;
};

#endif
