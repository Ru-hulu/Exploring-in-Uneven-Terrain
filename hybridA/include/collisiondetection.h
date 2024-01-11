#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>
#include "constants.h"
#include "node2d.h"
#include "node3d.h"
#include "dubins.h"
#include"ros/ros.h"

namespace HybridAStar {
namespace {
  /**
   * @brief Get the Configuration object of a node: node
   *        查询给定节点的configure (构型值)，分两种情况：
   *        - 若为Node2D类，theta分量恒为99
   *        - 若为Node3D类，theta分量为node的真实值
   * @param node: 节点指针
   * @param x 节点node的X分量
   * @param y 节点node的X分量
   * @param t 节点node的theta分量
   */
void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99; //2D的getConfiguration为多态函数，2D网格时统一将t=99
}

void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}

class CollisionDetection 
{
 public:
  /// Constructor
  CollisionDetection(); //构造函数建立了碰撞查找表
  //输入的节点有两种情况。
  //第一种是2d节点，发生在计算路径启发函数H的时候。此时只需要考虑点本身是否可通行，而不需要考虑车的体积。
  //第二种是3d节点，发生在生成通行路径的时候。此时需要考虑车体是否可通行。
  template<typename T> 
  bool isTraversable(const T* node) 
  {
    double time_b = ros::Time::now().toSec();
    //可通行性检验
    float x;
    float y;
    float t;
    getConfiguration(node, x, y, t);
    if (t == 99)  return !grid->data[node->getIdx()];//传入的是2D点
    bool tra = configurationMe1(x, y, t);//传入的是3D点,单位是grid
    //如果true，表明为自由网格 有障碍则返回false
    double time_a = ros::Time::now().toSec();
    time_check += (time_a-time_b);
    return tra;
  }
  bool configurationMe1(float x, float y, float t);
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}
  double time_check=0;
 private:
  nav_msgs::OccupancyGrid::Ptr grid;//就是传入的栅格地图
  std::vector<std::vector<std::pair<double,double>>> collision_map_check;//当车体和世界坐标原点重合，但是车体有yaw角度的时候，需要检查碰撞的坐标列表。单位是cm
   //碰撞查找表
};
}
#endif // COLLISIONDETECTION_H
