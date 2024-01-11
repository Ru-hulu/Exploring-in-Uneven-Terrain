#include<ros/ros.h>
#include<unistd.h>
#include<iostream>
#include<fstream>
#include<stdlib.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<utility>
#include<cmath>
#include<algorithm>
#include "Eigen/Core"
#include"qpOASES.hpp"
#include<vector>
#include<srvbg/mpcref.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

// https://codeleading.com/article/77125797422/ qp安装与使用
class MPC_Control
{
    public:
        ros::ServiceServer mpc_server;
        ros::ServiceClient gz_client;
        ros::Publisher cmd_v_w_pub;
        ros::Publisher showSUBS;
        ros::Publisher robpos;
        MPC_Control(ros::NodeHandle &n);
        bool MPC_Top_Handle(srvbg::mpcref::Request& req,srvbg::mpcref::Response& res);//输入参考序列,输出控制指令.
        bool MPC_Test_Handle();
        void Param_Init();
        void TEST();
        void ShowSubspace();
        void BreakPointRotate(double target_yaw);
        std::pair<double,double> MPC_Solve();
        int inline rotdir(float tar_yaw, float now_yaw);
        ros::NodeHandle nh;
        ros::ServiceServer RoutServer;
        ros::ServiceClient  SetCarvw;
        ros::ServiceClient GetCarstate;        
        ros::Publisher CarPub;
        int path_len;
        Eigen::Matrix<float,3,1> now_carstate{0.0,0.0,0.0};//当前车的状态

    private:
        int Np=5;//预测时长
        int Nc=4;//控制时域
        int Nx=3;//状态量个数
        int Nu=2;//控制量个数
        float delta_mpc_t=0.1;
        float max_v=3;
        float min_v=-3;
        float max_w=2;
        float min_w=-2;
        float temp_x=0;
        float temp_w=0;
        int path_track_counter;//当前已经追踪到了轨迹中哪一个点上。
        std::vector<double> ref_x;
        std::vector<double> ref_y;
        std::vector<double> ref_yaw;
        std::vector<double> ref_v;
        std::vector<double> ref_w;
        Eigen::Matrix<float, 3,1>x_bo{0.0,0.0,0.0};
        Eigen::MatrixXf U_max;
        Eigen::MatrixXf U_min;
        // Eigen::MatrixXf Ur;//(Nc*Nu+1,1)
        Eigen::MatrixXf A_qua;
};