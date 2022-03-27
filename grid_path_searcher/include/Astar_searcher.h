#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"

class AstarPathFinder
{	
	private:

	protected:
		uint8_t * data;
		GridNodePtr *** GridNodeMap;//地图指针
		Eigen::Vector3i goalIdx;// 网格地图坐标形式的目标点
		// 定义的地图的长宽高
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution;// resolution表示栅格地图精度,inv_resolution=1/resolution
		// gl_x表示地图边界,l->low(下边界),u->up(上边界)
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;

		GridNodePtr terminatePtr;// 终点指针
		std::multimap<double, GridNodePtr> openSet;// openSet容器,用于存放规划中确定下来的路径点

		double getHeu(GridNodePtr node1, GridNodePtr node2);// 返回两点之间的距离(曼哈顿距离/欧式距离/对角线距离)

		// 返回当前指针指向坐标的所有相邻节点,以及当前节点与相邻节点的cost
		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		


		// 判断节点是否是障碍物
    	bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;
		// 判断节点是不是为空
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;
		
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);//栅格地图坐标转实际坐标
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);//实际坐标转栅格地图坐标

	public:
		AstarPathFinder(){};
		~AstarPathFinder(){};
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);// A*路径搜索
		void resetGrid(GridNodePtr ptr);// 将所有点的属性设置为未访问过的状态下
		void resetUsedGrids();// 通过循环遍历重置每一个点

		// 初始化地图
		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		void setObs(const double coord_x, const double coord_y, const double coord_z);// 设置障碍物

		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		std::vector<Eigen::Vector3d> getPath();// 获得A*搜索到的完整路径
		std::vector<Eigen::Vector3d> getVisitedNodes();// 获得访问过的所有节点
};

#endif