#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>
#include<ros/ros.h>
#include "bucketedqueue.h"
#include <nav_msgs/OccupancyGrid.h>
//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
class DynamicVoronoi {

 public:
    DynamicVoronoi();
    ~DynamicVoronoi();
    void FreeAll();
    void initializeEmpty(int _sizeX, int _sizeY);
    void initializeMap(nav_msgs::OccupancyGrid* gridMap,int fro_obs_flag); //系统初始化, 99未知，fro_obs_flag = 100代表构建障碍物的vo图， fro_obs_flag = 20代表构建frontier的vo图
    void update(bool updateRealDist = true);//根据环境变化更新距离地图和Voronoi Diagram
    float getDistance(int x, int y);//返回(x,y)位置处的最近障碍的距离
    bool isOccupied(int x, int y);//检查(x,y)是否为占据状态
    void visualize(const char* filename = "result.ppm");//将当前的距离地图和voronoi diagram写进ppm文件里
    void getsdf(nav_msgs::OccupancyGrid* _gridMap);
    unsigned int getSizeX() {return sizeX;}//返回地图横向size
    unsigned int getSizeY() {return sizeY;}//返回地图纵向size
    bool getClosestObs(float x_w, float y_w,  float &x_o,float &y_o);//输入世界坐标中的一个位置，输出世界坐标中最近的障碍物位置
 public:
  struct dataCell {
    float dist;//距离，单位是grid
    char voronoi;
    char queueing;
    bool needsRaise;
    int obstX;//最近障碍物坐标单位是grid
    int obstY;//最近障碍物坐标单位是grid
    int sqdist;//距离平方，单位是grid
  };

//状态，枚举型
  typedef enum {voronoiKeep = -4, freeQueued = -3, voronoiRetry = -2, voronoiPrune = -1, free = 0, occupied = 1} State;
  typedef enum {fwNotQueued = 1, fwQueued = 2, fwProcessed = 3, bwQueued = 4, bwProcessed = 1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX / 2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;

  void setObstacle(int x, int y);//在(x,y)处设置障碍
  //更新并刷新以可视化
  void commitAndColorize(bool updateRealDist = true);
//检查是否为占据状态
  inline bool isOccupied(int& x, int& y, dataCell& c);
  inline int xy2indx(int x, int y);
  BucketPrioQueue open;
  std::vector<INTPOINT> removeList;
  std::vector<INTPOINT> addList;
  std::vector<INTPOINT> lastObstacles;
  int sizeY;
  int sizeX;
  float org_x; 
  float org_y;
  float rs; 
  // dataCell** data;//包含距离信息，是否为voronoi边等等。
  dataCell* data;//包含距离信息，是否为voronoi边等等。
  int padding;
  double doubleThreshold;
  double sqrt2;
};
