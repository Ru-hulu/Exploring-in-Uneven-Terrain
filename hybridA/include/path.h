/**
 * @file path.h
 * @brief Path类：提供在ROS RVIZ可视化路径的函数。主要实现了三个函数：
 *  1) 加入segment（线段）: addSegment()
 *  2) 加入Node(节点): addNode()
 *  3) 加入Vehicle（车体）: addVehicle()
 *  这三个函数通过updatePath进行整合
 */
#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <cstring>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "node3d.h"
#include "constants.h"
#include "helper.h"
namespace HybridAStar 
{

class Path 
{
 public:
  Path(bool smoothed = false) 
  {
    //ROS发布的topic 名称(没有经过平滑处理)
    std::string pathNodesTopic = "/pathNodes";
    std::string pathVehicleTopic = "/pathVehicle";   
    //ROS发布的topic 名称(经平滑处理)
    if (smoothed) 
    {
      pathNodesTopic = "/sPathNodes";
      pathVehicleTopic = "/sPathVehicle";
      this->smoothed = smoothed;
    }
    pubPathNodes = n.advertise<visualization_msgs::MarkerArray>(pathNodesTopic, 1);
    pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>(pathVehicleTopic, 1);
    path.header.frame_id = "map";
  }
  // 对每个3D节点信息，分别进行addSegment、addNode、addVehicle
  //// 对每个3D节点信息，分别进行：增加线性addSegment(栅格转世界)、节点addNode(可视化)、和车辆标记符addVehicle(可视化)
  void updatePath(std::vector<Node3D> nodePath);
  void addSegment(const Node3D& node);
  void addNode(const Node3D& node, int i);
  void addVehicle(const Node3D& node, int i);
  void clear();//清除路径 
  void publishPathNodes() { pubPathNodes.publish(pathNodes); }//将路径的节点进行发布
  void publishPathVehicles() { pubPathVehicles.publish(pathVehicles); }//在路径上发布车
  ros::Publisher pubPathNodes;//ROS路径节点发布器

 private:
  ros::NodeHandle n;//ROS的节点句柄
  ros::Publisher pubPathVehicles;//用于在路径上发布车子位置
  nav_msgs::Path path;//路径数据结构，用于可视化 
  visualization_msgs::MarkerArray pathNodes;//节点数据结构，用于可视化
  visualization_msgs::MarkerArray pathVehicles;//车子数据结构，用于可视化
  bool smoothed = false;
};
}
#endif // PATH_H
