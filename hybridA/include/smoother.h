/**
 * @file smoother.h
 * @brief 用于将路径进行平滑处理的类
 *  
 */

#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "dynamicvoronoi.h"
#include "node3d.h"
#include "vector2d.h"
#include "helper.h"
#include "constants.h"
namespace HybridAStar 
{
class Smoother {
 public:
  Smoother() {}

   //   核心函数，将由节点组成的路径采用梯度下降方法进行平滑
   //   不同的迭代阶段采用计算下面的代价值作为指标
   //   obstacleCost：障碍物代价
   //   curvatureCost：曲率代价
   //   smoothnessCost：平滑代价
  void smoothPath(DynamicVoronoi& voronoi);
  void tracePath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());

  /// returns the path of the smoother object：返回平滑后的路径
  std::vector<Node3D> getPath() {return path;}
  Vector2D obstacleTerm(Vector2D xi);//障碍物项，用于约束路径远离障碍物
  Vector2D curvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);//曲率项，用于保证可转弯性及通行性
  //平滑项，用于将节点等距分布并尽量保持同一个方向
  Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);
  bool isOnGrid(Vector2D vec) 
  {
    if (vec.getX() >= 0 && vec.getX() < width &&
        vec.getY() >= 0 && vec.getY() < height) {
      return true;
    }
    return false;
  }

 private:
  /// maximum possible curvature of the non-holonomic vehicle
  float kappaMax = 1.f / (Constants::r *10* 1.1);//乘以10是因为要将m转换为grid为单位
  /// maximum distance to obstacles that is penalized
  float obsDMax = Constants::minRoadWidth;
  /// maximum distance for obstacles to influence the voronoi field
  float vorObsDMax = Constants::minRoadWidth;

  //权重系数
  float alpha = 0.1;
  float wObstacle = 0;//利用voronoi图对靠近障碍物的轨迹点优化
  float wVoronoi = 0;
  float wCurvature = 0;
  float wSmoothness = 0.2;

  // 描述地图中拓扑结构的voronoi diagram
  DynamicVoronoi voronoi;

  int width;
  int height;

  //待平滑的路径
  std::vector<Node3D> path;
};
}
#endif // SMOOTHER_H
