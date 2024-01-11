#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "algorithm.h"
#include "node3d.h"
#include "dubins.h"
// #include "path.h"
// #include "smoother.h"
// #include "dynamicvoronoi.h"

namespace HybridAStar 
{
class Planner 
{
 public:
  Planner();//默认构造器
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);//通过订阅者监听的回调函数设置地图
  bool plan(float x1,float y1,float yaw1,float &x2,float &y2,float &yaw2);//核心规划函数
  void tracePath(const Node3D* node);
  void GenerateRefCmd(std::vector<Node3D> nodePath);
  void Clear_data();
  std::vector<double> ref_v;
  std::vector<double> ref_w;
  std::vector<double> ref_x;
  std::vector<double> ref_y;
  std::vector<double> ref_yaw;
  std::vector<Node3D> Pathmotion_primitive;
  
 private:
  CollisionDetection configurationSpace;//碰撞检测类实体，用以检测某个配置是否会发生碰撞  
  nav_msgs::OccupancyGrid::Ptr grid;
  // Path path;//生成混合A*路径的实体对象
  // Smoother smoother;//路径平滑实体
  // Path smoothedPath = Path(true);//用于发布给控制器的平滑路径
  // DynamicVoronoi voronoiDiagram; //Voroni Diagram
};
}
#endif // PLANNER_H
