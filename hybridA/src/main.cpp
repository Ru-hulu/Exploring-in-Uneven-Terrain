#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"
#include"node3d.h"


//功能：
//输入：自定义srv获得地图信息和当前位置、目标位置
//输出：参考坐标、参考指令通过另外一个srv发出给Controller。
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "a_star");
  HybridAStar::Planner hy;
  // hy.plan(); 
  ros::spin();
  return 0;
}

//思考：如果起点在格子中心，6个扩展都在同一grid,是否还有解？

