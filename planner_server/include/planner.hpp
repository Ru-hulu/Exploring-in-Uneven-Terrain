#include<ros/ros.h>
#include<iostream>
#include<string>
#include<fstream>
#include<ctime>
#include<cmath>
#include<ros/time.h>
#include <thread>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include<sensor_msgs/Image.h>
#include<gazebo_msgs/GetModelState.h>
#include<gazebo_msgs/SetModelState.h>
#include<tf/tf.h>
#include"lkh_interface.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Quaternion.h>
#include<tf/tf.h>
#include<stdlib.h>
#include<nav_msgs/OccupancyGrid.h>
#include<srvbg/getlcplan.h>
#include<srvbg/mpcref.h>
#include <octomap/octomap.h>
#include "spectral_clustering.hpp"
#include "astar_planner.h"
#include"non_uniform_bspline.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<geometry_msgs/Twist.h>
struct elevation_map
{
  std::vector<float> data;
  int width;
  int height;
  float org_x;
  float org_y;
  float resolution;
};

class Local_planner
{
  public:
  ros::NodeHandle nh;
  // ros::ServiceServer plansrv;//传入地图和frontier
  ros::ServiceClient getmap;//传入地图和frontier
  ros::ServiceClient states_client;
  ros::ServiceClient set_state_client;
  ros::Publisher cmd_vel_pub;
  ros::ServiceClient car_mpc_client;
  ros::Publisher gridmapPub;
  ros::Publisher show_frontier_cluster;
  // bool fresh_flag = false;//重新进行局部规划工作
  nav_msgs::Path::Ptr Path_ref;
  elevation_map lcpmap2d;
  // nav_msgs::OccupancyGrid lcpmap2d;// 20表示frontier, 0 表示可以通行，100表示障碍物， 99表示未知。
  Local_planner(const ros::NodeHandle &nh_);
  // bool LCPCallback(srvbg::getlcplan::Request &req,srvbg::getlcplan::Response &res);//进行局部规划的入口函数。拿到地图、frontier二维信息后直接返回true.  
  void ShowCluster(std::vector<int> *real_label);
  bool LPS_Planner();//HybridA求解得到参考控制序列。
  void EscapeFromStack(Eigen::Vector2f v_sum);//当机器人初始状态不安全时，调用此函数
  bool GetMapService(bool local_flag);
  bool FomulateTSP();
  void Optimize_Bspline();
  void Solve_ATSP(std::vector<std::pair<float,float>>* vp_l,Eigen::MatrixXf Weight_matrix);
  bool inline CheckCollision3(int grid2d_x,int grid2d_y,int grid_w,int grid_h);
  bool inline CheckCollision2(int grid2d_x,int grid2d_y,int grid_w,int grid_h);
  bool inline CheckCollision1(int grid2d_x,int grid2d_y,int grid_w,int grid_h);
  bool inline CheckCollision_self(int grid2d_x,int grid2d_y,int grid_w,int grid_h);
  int inline global2grid(float x, float y,float org_x,float org_y,float gridw,float gridh,float gridres);
  int inline rotdir(float now_yaw, float tar_yaw);
  void saveOccupancyGridAsJpg(const nav_msgs::OccupancyGrid& occupancyGrid, const std::string& filename);
  void clearfrontiers();
  float now_w_x;  float now_w_y;  float now_w_yaw;
  private:
    nav_msgs::OccupancyGrid gdmap_show;    
    std::vector<octomap::point3d> submapfrontier_v;
    std::vector<octomap::point3d> submapfrontierclustercenter_v;
    std::vector<std::pair<int,int>> checklist1;
    std::vector<std::pair<int,int>> checklist2;
    std::vector<std::pair<int,int>> checklist3;
    std::vector<int> atsp_vp_sequence_index;//访问vp的顺序
    std::vector<std::pair<float,float>> atsp_vp_sequence;//访问vp的顺序,绝对坐标
    std::vector<std::pair<float, float>> vp_reach_list;//所有的观测点中，能够到达的,绝对坐标
    std::vector<std::vector<geometry_msgs::PoseStamped>> atsp_vp_path_All;//astar求解的vp彼此之间全连接路径(不包括当前位置到vp的路径)    
    std::vector<std::vector<geometry_msgs::PoseStamped>> atsp_vp_travel_path;//按tsp求解顺序访问所有的观测点，ASTAR轨迹
    std::shared_ptr<NonUniformBspline> bspline_ptr;
    float res_map = 0.25;
};
