#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include<Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include<nav_msgs/Path.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<nlopt.hpp>
#include<nav_msgs/GetMap.h>
#include<tf/tf.h>
#include"dynamicvoronoi.hpp"
#include <iostream>
#include <fstream>
// An implementation of non-uniform B-spline with different dimensions
// It also represents uniform B-spline which is a special case of non-uniform
struct Hybrid_data
{
  DynamicVoronoi* obs_voro;
  DynamicVoronoi* fro_voro;
  Eigen::MatrixXd* contr_list;
  bool opt_front_flag;
  nav_msgs::OccupancyGrid* temp_use_pointer_lcpmap2d;
};
class NonUniformBspline 
{
  public:
    int matrix_ct =0;
    bool use_frontier;
    bool use_cut;
    ros::NodeHandle nh;
    ros::Publisher showspline;
    ros::Publisher showcontrp;  
    ros::Publisher showspeci;  
    ros::Publisher showspline_opt;
    ros::Publisher showcontrp_smf;
    ros::Publisher showcontrp_smo;
    ros::Publisher sdfmap_pub;
    // ros::ServiceClient map_cl;
    nav_msgs::OccupancyGrid* lcpmap2d_;
    Eigen::MatrixXd control_points_;//控制点序列,世界坐标
    Eigen::MatrixXd refine_contr_list;//优化后的控制点序列
    Eigen::MatrixXd bsline_list;//平滑优化后的bspline曲线上的坐标点
    int non_zero_line;
    Eigen::VectorXd u_;          //节点序列t0 t1 t2 ... t_(m-1)
    DynamicVoronoi obs_voro;
    DynamicVoronoi fro_voro;
    int             p_, n_, m_;//p_是次数3，m是节点序列长度-1 n是控制点数量-1 
    int dim_;
    double          interval_;//时间间隔，即节点序列间隔.
    double min_cost=-1; 
    NonUniformBspline(const ros::NodeHandle &nh_);
    ~NonUniformBspline();
    inline bool check_close_enough(int ii);
    inline Eigen::Vector2d SmoothGradTerm(const Eigen::Vector2d &p_s2,const Eigen::Vector2d &p_s1,const Eigen::Vector2d &p_n,const Eigen::Vector2d &p_a1,const Eigen::Vector2d &p_a2);
    inline Eigen::Vector2d CurvatureGradTerm(const Eigen::Vector2d &p_s1,const Eigen::Vector2d &p_n,const Eigen::Vector2d &p_a1,double& cost);
    inline Eigen::Vector2d ObsGradTerm(const Eigen::Vector2d &x,double &cost);
    inline Eigen::Vector2d FroGradTerm(const Eigen::Vector2d &x,double &cost);
    inline Eigen::Vector2d conjugate(const Eigen::Vector2d &a,const Eigen::Vector2d &b);
           Eigen::VectorXd   evaluateDeBoor(const double& u);   // use u \in [up, u_mp] 德布尔算法计算样条曲线
    inline bool CheckSafeCorrection(const Eigen::Vector2d &p_n);
    void InitControl_list(std::vector<std::vector<geometry_msgs::PoseStamped>>* allcontrol_point);  //传入的是世界坐标系
    void InitialSpline(const int& order, const double& interval);    //初始化Bspline信息
    void InitialPara();//参数在example.yaml文件中
    void Freemaps();
    void ClearData();
    void Optimize_ControlP();    
    void CutControlPoint();    
    void Optimize_ControlP1();
    bool Optimize_nlopt(bool fron_opt);
    void PublishBspline(bool fron_opt); 
    void Publishspecific();
    void saveplanning();    
    bool PrepareObsVoronoi(nav_msgs::OccupancyGrid* lcpmap2d);
    void GetCmd(float now_w_x,float now_w_y,float now_w_yaw);
    static double costFunction(const std::vector<double> &x, std::vector<double>& grad ,void* func_data); 
    static double  costFunction_smooth_fro_only(const std::vector<double> &x, std::vector<double>& grad ,void* func_data); 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    float opt_time= 0.15;//优化时间
    float relax_x = 0.1;
    int opt_iter= 100;//优化变量
    float smo_weight= 0.25;//优化权重
    float vor_weight= 0.25;//优化权重
    float cur_weight= 0.25;//优化权重
    float fro_weight= 0.25;//优化权重    
    float obs_optthr= 0.25;//优化thr 
    float gra_weight_sum = 0.25;  
    float observe_optdis = 0;
    float obst_safe_dis = 0.5;
    float car_control_dis_step=0.05;//假设车辆以恒定速度运行,控制频率0.1s,速度恒定为1m/s,那么这个数值就是0.1
    float car_cont_time_step = 0.1;
    int dec_cct=0;
    int decrease_cct = 100;
    int min_deci_len = 100;//最短长度
    int now_deci_len = 400;//当前长度
    int decrease_step = 3;
    int numFiles = 0;
};
#endif