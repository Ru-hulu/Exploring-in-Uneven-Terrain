#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include<nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <vector>
#include <queue>
#define infinity 1.0e10
using namespace std;


struct Node2d{
  float cost;
  int index;
};

namespace astar_planner 
{
    class AstarPlanner
    {
      public:
          AstarPlanner(std::vector<float>*height_map, int map_width, int map_height,float start_x, float start_y,float resolution);
          void worldToMap(double wx, double wy, unsigned int &m_x,unsigned int &m_y);
          int getIndex(int m_x,int m_y);
          void indexToCells(int current_cell,unsigned int &tmp1,unsigned int &tmp2);
          void mapToWorld(unsigned int tmp1,unsigned int tmp2, double &x, double &y);
          bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>* plan);
          int width;
          int height;
          float gdstart_x;
          float gdstart_y;
          float rz;
          int map_size;
          std::vector<int> his_search;
          vector<float> OGM;
          double getHeuristic(int cell_index, int goal_index);
          vector<int> get_neighbors(int current_cell);
          double getMoveCost(int firstIndex, int secondIndex);
          bool isInBounds(int x, int y);    
          float search_elevation_th_gap = 0.25;
    };
};
