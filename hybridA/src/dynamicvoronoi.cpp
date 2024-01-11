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
#include "dynamicvoronoi.h"

#include <math.h>
#include <iostream>

using namespace HybridAStar;

DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
}

DynamicVoronoi::~DynamicVoronoi() 
{
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  if (gridMap) {
    for (int x=0; x<sizeX; x++) delete[] gridMap[x];
    delete[] gridMap;
  }
}

//系统初始化
void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY) 
{
  sizeX = _sizeX;
  sizeY = _sizeY;
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY];
  
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c;
}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) 
{
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY);
  for (int x=0; x<sizeX; x++) 
  {
    for (int y=0; y<sizeY; y++) 
    {
      if (gridMap[x][y]) 
      {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) 
        {
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;

              if (!gridMap[nx][ny]) {
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
            data[x][y] = c;
          } else setObstacle(x,y);
        }
      }
    }
  }
}


void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) return;
  
  addList.push_back(INTPOINT(x,y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

// void DynamicVoronoi::removeObstacle(int x, int y) {
//   dataCell c = data[x][y];
//   if(isOccupied(x,y,c) == false) return;

//   removeList.push_back(INTPOINT(x,y));
//   c.obstX = invalidObstData;
//   c.obstY  = invalidObstData;    
//   c.queueing = bwQueued;
//   data[x][y] = c;
// }
 
//这里的voronoi建立的ESDF以格子为单位
void DynamicVoronoi::update(bool updateRealDist) 
{
  commitAndColorize(updateRealDist);
  while (!open.empty()) 
  {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];
    if(c.queueing==fwProcessed) continue; 
    if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) 
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
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) 
          {
            int distx = nx-c.obstX;//单位是grid
            int disty = ny-c.obstY;//单位是grid
            int newSqDistance = distx*distx + disty*disty;		//单位是grid
            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) 
            { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
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
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}

float DynamicVoronoi::getDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist; 
  else return -INFINITY;
}

void DynamicVoronoi::commitAndColorize(bool updateRealDist) 
{
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing != fwQueued){
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, INTPOINT(x,y));
    }
  }
  removeList.clear();
  addList.clear();
}


bool DynamicVoronoi::isOccupied(int x, int y) {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y);
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
    if (data[x][y].sqdist==0) 
    {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
    } 
    else 
    {
      float f = 80+(data[x][y].dist*5);
      if (f>255) f=255;
      if (f<0) f=0;
      c = (unsigned char)f;
      fputc( c, F );
      fputc( c, F );
      fputc( c, F );
    }
    }
  }
  fclose(F);
}
