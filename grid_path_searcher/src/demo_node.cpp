#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "Astar_searcher.h"
#include "JPS_searcher.h"
#include "backward.hpp"

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;//精度，精度倒数，点云间隙
double _x_size, _y_size, _z_size;    //地图x y z 的尺寸

// useful global variables
bool _has_map   = false;

Vector3d _start_pt;//Vector3d 是一个类，有xyz三个坐标     这是起始点指针
Vector3d _map_lower, _map_upper;//地图上下界
int _max_x_id, _max_y_id, _max_z_id;//地图x y z 最大id

// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher  _grid_path_vis_pub, _visited_nodes_vis_pub, _grid_map_vis_pub;

AstarPathFinder * _astar_path_finder     = new AstarPathFinder();
JPSPathFinder   * _jps_path_finder       = new JPSPathFinder();

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);

void visGridPath( vector<Vector3d> nodes, bool is_use_jps );// 实体化搜索到的路径
void visVisitedNode( vector<Vector3d> nodes );// 实体化显示所有访问过的节点
void pathFinding(const Vector3d start_pt, const Vector3d target_pt);// 路径规划函数

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )// 如果终点信息的z值小于0,或者没有地图信息,则直接返回
        return;

    Vector3d target_pt;
    //获取交互式界面给出的终点坐标
    // 将终点信息传递给终点指针
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");// 终端打印已经接收到终点信息
    //输入起点、终点，调用 pathFind 函数
    pathFinding(_start_pt, target_pt); // 路径规划
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(_has_map ) return;// 如果已经有地图信息,则返回

    pcl::PointCloud<pcl::PointXYZ> cloud;//pcl::PointCloud<pcl::PointXYZ>就是表示 PCL 中用于存储 3D 点集合的基类
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;//sensor_msgs::PointCloud2是一类点云数据结构

    pcl::fromROSMsg(pointcloud_map, cloud);//数据格式转换
    // 将点云格式为sensor_msgs/PointCloud2 格式转为 pcl/PointCloud(在RVIZ中显示的点云的数据格式sensor_msgs::PointCloud2)
    
    if( (int)cloud.points.size() == 0 ) return; // 如果还没有点云数据,则返回

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        

        // set obstalces into grid map for path planning
        // 将障碍物信息设置进入栅格化地图，为后续路径规划做准备
        _astar_path_finder->setObs(pt.x, pt.y, pt.z);
        _jps_path_finder->setObs(pt.x, pt.y, pt.z);

        // for visualize only   // 可视化地图部分
        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt); // 坐标压入堆栈
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);// pcl::toROSMsg (pcl::PointCloud<pcl::PointXYZ>,sensor_msgs::PointCloud2);

    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);//发布地图信息

    _has_map = true;
}

void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
    //Call A* to search for a path
    //使用 A*进行路径搜索
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

    //Retrieve the path
    //获取规划的路径
    auto grid_path     = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    //Visualize the result
    //可视化结果
    visGridPath (grid_path, false);
    visVisitedNode(visited_nodes);// 显示所有访问过的节点(函数没问题,但是显示不出来,没找到问题)

    //Reset map for next call
    //为下次规划重置地图
    _astar_path_finder->resetUsedGrids();

    //进行 JPS 路径规划编写时，将_use_jps 的值置为 1 即可
    //_use_jps = 0 -> Do not use JPS
    //_use_jps = 1 -> Use JPS
    //you just need to change the #define value of _use_jps
#define _use_jps 0
#if _use_jps
    {
        //Call JPS to search for a path
        //使用 JPS 进行路径搜索
        _jps_path_finder -> JPSGraphSearch(start_pt, target_pt);

        //Retrieve the path
        auto grid_path     = _jps_path_finder->getPath();
        auto visited_nodes = _jps_path_finder->getVisitedNodes();

        //Visualize the result
        visGridPath   (grid_path, _use_jps);
        visVisitedNode(visited_nodes);

        //Reset map for next call
        _jps_path_finder->resetUsedGrids();
    }
#endif
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");//ROS节点初始化
    ros::NodeHandle nh("~");//创建节点句柄
    //订阅到地图信息的回调函数
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    //订阅到终点信息的回调函数
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );

    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
  //param是从参数服务器中获取第一个参数,将值保存到第二个参数中,如果没获取到第一个参数的值,则使用默认值(第三个参数)
    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    //定义了结构体 AstarPathFinder 变量_astar_path_finder，该结构体存储、实现了 Astar 路径规划所需的所有信息和功能
    _astar_path_finder  = new AstarPathFinder();
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    //定义了结构体 JPSPathFinder 变量_jps_path_finder，该结构体存储、实现了 JPS 路径规划所需的所有信息和功能
    _jps_path_finder    = new JPSPathFinder();
    _jps_path_finder    -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete _astar_path_finder;
    delete _jps_path_finder;
    return 0;
}
// 实体化搜索到的路径
void visGridPath( vector<Vector3d> nodes, bool is_use_jps )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    
    if(is_use_jps)// is_use_jps是用于判断用的是不是jps算法
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    //路径的颜色
    if(is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;// 将所有路径中的点压入堆栈
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);// 发布显示所有路径节点
}

void visVisitedNode( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}