/**
 * @file dynamicvoronoi.h
 * @brief Voronoi Diagram的构造
 * @date 2019-11-18
 * 注：原始代码：http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
 *     ROS版本：https://github.com/frontw/dynamicvoronoi
 * 
 * 这份代码直接采用ROS版本的代码，仅有的修改为加上命名空间HybridAStar
 * 关联的文件主要有：
 *  - c++ head files:   bucketedqueue.h  dynamicvoronoi.h  point.h
 *  - c++ source files: bucketedqueue.cpp  dynamicvoronoi.cpp
 *  参考文献：
 *      B. Lau, C. Sprunk and W. Burgard, Improved Updating of Euclidean Distance Maps and Voronoi Diagrams, 
 *  IEEE Intl. Conf. on Intelligent Robots and Systems (IROS), Taipei, Taiwan, 2010.
 */
#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>

#include "bucketedqueue.h"

namespace HybridAStar {
//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
class DynamicVoronoi {

 public:

  DynamicVoronoi();
  ~DynamicVoronoi();

  //! Initialization with an empty map 从空白地图开始构造
  void initializeEmpty(int _sizeX, int _sizeY);
  //! Initialization with a given binary map (false==free, true==occupied) 从Grid Map初始化
  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist = true);//根据环境变化更新距离地图和Voronoi Diagram
  //! prune the Voronoi diagram

  //! returns the obstacle distance at the specified location
  float getDistance(int x, int y);//返回(x,y)位置处的最近障碍的距离
  bool isOccupied(int x, int y);//检查(x,y)是否为占据状态
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename = "result.ppm");//将当前的距离地图和voronoi diagram写进ppm文件里

  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() {return sizeX;}//返回地图横向size
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() {return sizeY;}//返回地图纵向size

  // was private, changed to public for obstX, obstY
 public:
  struct dataCell {
    float dist;//距离，单位是grid
    char voronoi;
    char queueing;
    int obstX;//最近障碍物坐标单位是grid
    int obstY;//最近障碍物坐标单位是grid
    bool needsRaise;
    int sqdist;//距离平方，单位是grid
  };

//状态，枚举型
  typedef enum {voronoiKeep = -4, freeQueued = -3, voronoiRetry = -2, voronoiPrune = -1, free = 0, occupied = 1} State;
  typedef enum {fwNotQueued = 1, fwQueued = 2, fwProcessed = 3, bwQueued = 4, bwProcessed = 1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX / 2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;



  // methods
  void setObstacle(int x, int y);//在(x,y)处设置障碍
  // void removeObstacle(int x, int y);//移除(x,y)处障碍
  //更新并刷新以可视化
  void commitAndColorize(bool updateRealDist = true);
  // inline void reviveVoroNeighbors(int& x, int& y);

//检查是否为占据状态
  inline bool isOccupied(int& x, int& y, dataCell& c);

  BucketPrioQueue open;
  std::vector<INTPOINT> removeList;
  std::vector<INTPOINT> addList;
  std::vector<INTPOINT> lastObstacles;

  // maps
  int sizeY;
  int sizeX;
  dataCell** data;//包含距离信息，是否为voronoi边等等。
  bool** gridMap;//二值
  int padding;
  double doubleThreshold;
  double sqrt2;
};
}

#endif

