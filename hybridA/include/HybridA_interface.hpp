#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "constants.h"
#include "planner.h"
#include"node3d.h"
#include"nav_msgs/Path.h"

void HybridA_Solve_Handle(nav_msgs::OccupancyGrid::Ptr map_test,std::vector<std::vector<float>> depa_destina,nav_msgs::Path::Ptr Path_ref);