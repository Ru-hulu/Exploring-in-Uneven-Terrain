/**
 * @file dynamicvoronoi.cpp
 * 
 * @brief Voronoi Diagram的实现
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
#include "dynamicvoronoi.hpp"
#include <math.h>
#include <iostream>


DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
}

DynamicVoronoi::~DynamicVoronoi() 
{
  if (data) delete[] data;
  data = NULL;
}
void DynamicVoronoi::FreeAll()
{
  if (data) delete[] data;
  data = NULL;
} 


bool DynamicVoronoi::getClosestObs(float x_w, float y_w,  float &x_o,float &y_o)
{
  int g_x = int((x_w - org_x)/rs);
  int g_y = int((y_w - org_y)/rs);
  dataCell c = data[xy2indx(g_x,g_y)];
  if(c.obstX==invalidObstData||c.obstY==invalidObstData) return false;
  x_o = float(c.obstX)*rs + org_x;
  y_o = float(c.obstY)*rs + org_y;
  return true;
}


void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY) 
{
  sizeX = _sizeX;
  sizeY = _sizeY;
  if (data)  
  delete[] data;
  
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;
  // if(sizeY==105)  data = new dataCell[10];
  // delete[] data;
  data = new dataCell[sizeX*sizeY];
  for (int x=0; x<sizeX*sizeY; x++) data[x] = c;
}

//系统初始化, 99未知，fro_obs_flag = 100代表构建障碍物的vo图， fro_obs_flag = 20代表构建frontier的vo图
void DynamicVoronoi::initializeMap(nav_msgs::OccupancyGrid* gridMap,int fro_obs_flag) 
{
  org_x = gridMap->info.origin.position.x;
  org_y = gridMap->info.origin.position.y;
  rs = gridMap->info.resolution;
  sizeX = gridMap->info.width;
  sizeY = gridMap->info.height;
  initializeEmpty(gridMap->info.width, gridMap->info.height);
  for (int x=0; x<sizeX; x++) 
  {
    for (int y=0; y<sizeY; y++) 
    {
      if (gridMap->data[xy2indx(x,y)]==fro_obs_flag) 
      {
        dataCell c = data[xy2indx(x,y)];
        if (!isOccupied(x,y,c)) 
        {
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) 
          {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;

              if (gridMap->data[xy2indx(nx,ny)]!=fro_obs_flag) 
              {
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded) 
          {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[xy2indx(x,y)] = c;
          } 
          else setObstacle(x,y);
        }
      }
    }
  }
}

inline int DynamicVoronoi::xy2indx(int x, int y)
{
  return (x+y*sizeX);
}
void DynamicVoronoi::setObstacle(int x, int y) 
{
  dataCell c = data[xy2indx(x,y)];
  if(isOccupied(x,y,c)) return;
  
  addList.push_back(INTPOINT(x,y));
  c.obstX = x;
  c.obstY = y;
  data[xy2indx(x,y)] = c;
}

//这里的voronoi建立的ESDF以格子为单位
void DynamicVoronoi::update(bool updateRealDist) 
{
  commitAndColorize(updateRealDist);
  while (!open.empty()) 
  {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[xy2indx(x,y)];
    if(c.queueing==fwProcessed) continue; 
    if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[xy2indx(c.obstX,c.obstY)])) 
    {
      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;
      for (int dx=-1; dx<=1; dx++) 
      {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) 
        {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[xy2indx(nx,ny)];
          if(!nc.needsRaise) 
          {
            int distx = nx-c.obstX;//单位是grid
            int disty = ny-c.obstY;//单位是grid
            int newSqDistance = distx*distx + disty*disty;		//单位是grid
            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) 
            { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[xy2indx(nc.obstX,nc.obstY)])==false) overwrite = true;
            }
            if (overwrite) 
            {
              open.push(newSqDistance, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              if (updateRealDist) 
              {
                nc.dist = sqrt((double) newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } 
            data[xy2indx(nx,ny)] = nc;//这个位置处，最近的obs相关信息可以通过这个获取
          }
        }
      }
    }
    data[xy2indx(x,y)] = c;
  }
}

float DynamicVoronoi::getDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[xy2indx(x,y)].dist; 
  else return -INFINITY;
}

void DynamicVoronoi::commitAndColorize(bool updateRealDist) 
{
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[xy2indx(x,y)];

    if(c.queueing != fwQueued)
    {
      if (updateRealDist) 
      c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[xy2indx(x,y)] = c;
      open.push(0, INTPOINT(x,y));
    }
  }
  removeList.clear();
  addList.clear();
}


bool DynamicVoronoi::isOccupied(int x, int y) {
  dataCell c = data[xy2indx(x,y)];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y);
}

void DynamicVoronoi::getsdf(nav_msgs::OccupancyGrid* _gridMap)
{
  _gridMap->data.resize(sizeX*sizeY);
  _gridMap->header.frame_id ="map";
  _gridMap->info.width = sizeX;
  _gridMap->info.height = sizeY;
  _gridMap->info.resolution = rs;
  _gridMap->info.origin.position.x = org_x;
  _gridMap->info.origin.position.y = org_y;
  _gridMap->info.origin.position.z = 0.5;
  for(int y = sizeY-1; y >=0; y--)
  {      
    for(int x = 0; x<sizeX; x++)
    {	
      unsigned char c = 0;
      if (data[xy2indx(x,y)].sqdist==0) 
      _gridMap->data[xy2indx(x,y)]  = 100;
      else 
      {
        float f = 80+(data[xy2indx(x,y)].dist*5);
        if (f>255) f=255;
        if (f<0) f=0;
        _gridMap->data[xy2indx(x,y)]  = int(f/255.0*100);
      }
    }
  }

}
//用于生成可视化的ESDF
void DynamicVoronoi::visualize(const char *filename) 
{
  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  for(int y = sizeY-1; y >=0; y--)
  {      
    for(int x = 0; x<sizeX; x++)
    {	
    unsigned char c = 0;
    if (data[xy2indx(x,y)].sqdist==0) 
    {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
    } 
    else 
    {
      float f = 80+(data[xy2indx(x,y)].dist*5);
      std::cout<<f<<" ";
      if (f>255) f=255;
      if (f<0) f=0;
      c = (unsigned char)f;
      fputc( c, F );
      fputc( c, F );
      fputc( c, F );
    }
    }
    std::cout<<std::endl;
  }
  fclose(F);
}
