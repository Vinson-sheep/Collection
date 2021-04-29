/*
*	author: Vinson Sheep
*/

#ifndef _OCCUPANCY_2D_GRID_H_
#define _OCCUPANCY_2D_GRID_H_

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Eigen>

/*
*	Occupancy_2d_grid类
*	
*	负责将costmap_node中产生的costmap信息进行存储，并且为这个地图配上各种转换接口，供给上层类使用
*
*	注意：这样的实现方式是非常不高效的，如果能够在costmap_2d中直接调用数据的话就不需要这样做了
*/

class Occupancy_2d_grid
{
private:
	// 存放
	std::shared_ptr<const nav_msgs::OccupancyGrid> data;
	// 参数
	int width, height;	// 地图宽高
	double resolution, resolution_inv;	// 像素和像素的倒数
	Eigen::Vector3d origin;	// 初始位置

public:	
	Occupancy_2d_grid();
	~Occupancy_2d_grid();

	//初始化
	void init(ros::NodeHandle& nh);
	// // 地图更新函数 - 输入新的栅格信息共享指针
	void map_update(std::shared_ptr<const nav_msgs::OccupancyGrid> new_data);
	// // 判断当前点是否在地图内
	bool isInMap(const Eigen::Vector3d pos);
	// // 由位置计算索引 - 转换成功会返回true，否则为false
	bool posToIndex(const Eigen::Vector3d pos, Eigen::Vector3i &id);
	// // 由索引计算位置 - 转换成功会返回true，否则为false
	bool indexToPos(const Eigen::Vector3i id, Eigen::Vector3d &pos);
	// // 根据位置返回占据状态 - 返回对应网格的值
	int getOccupancy(const Eigen::Vector3d pos);
	// // 根据索引返回占据状态 - 返回对应网格的值
	int getOccupancy(const Eigen::Vector3i id);
	// // 检查安全
	// bool check_safety(Eigen::Vector3d& pos, double check_distance/*, Eigen::Vector3d& map_point*/);
	// // 定义该类的指针
	typedef std::shared_ptr<Occupancy_2d_grid> Ptr;
};

#endif
