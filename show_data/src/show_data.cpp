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
#include<array>
struct Color {
    int r;
    int g;
    int b;
};

Color getColor(double z, double z_min, double z_max) {
    // 根据高度z计算色调（H）的值
    double hue = (1 - (z - z_min) / (z_max - z_min)) * 240;

    // 将色调（H）转换为RGB颜色
    double h = hue / 360;
    double s = 1;
    double v = 1;

    int hi = static_cast<int>(std::floor(h * 6)) % 6;
    double f = h * 6 - std::floor(h * 6);
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);

    int r, g, b;
    switch (hi) {
        case 0:
            r = static_cast<int>(v * 255);
            g = static_cast<int>(t * 255);
            b = static_cast<int>(p * 255);
            break;
        case 1:
            r = static_cast<int>(q * 255);
            g = static_cast<int>(v * 255);
            b = static_cast<int>(p * 255);
            break;
        case 2:
            r = static_cast<int>(p * 255);
            g = static_cast<int>(v * 255);
            b = static_cast<int>(t * 255);
            break;
        case 3:
            r = static_cast<int>(p * 255);
            g = static_cast<int>(q * 255);
            b = static_cast<int>(v * 255);
            break;
        case 4:
            r = static_cast<int>(t * 255);
            g = static_cast<int>(p * 255);
            b = static_cast<int>(v * 255);
            break;
        case 5:
            r = static_cast<int>(v * 255);
            g = static_cast<int>(p * 255);
            b = static_cast<int>(q * 255);
            break;
    }

    return {r, g, b};
}

void PublishPlane(visualization_msgs::MarkerArray &PlaneMarker)
{
    PlaneMarker.markers.clear();
    std::ifstream file("/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/mapdata.txt");
    std::string line;
    visualization_msgs::Marker marker;    //Ｍａｒｋｅｒ是一个结构体模板
    //设置frame ID 和 timstamp ，具体信息可以参考TF 教程
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    //为marker设置命名空间和ＩＤ，创建不同的ＩＤ
    //任何带有相同的命名空间和ＩＤ被发送，都会覆盖先前的那个marker
    marker.ns = "array_plane";
    //设置marker的类型。初始化是CUBE,　接着循环SPHERE,ARROW,CYLINDER
    marker.type = visualization_msgs::Marker::ARROW;
    // marker.type = visualization_msgs::Marker::CUBE;
    //设置marker的操作：ADD, DELETE, and new in ROS indigo:3(DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    //设置marker的位姿。这是6个自由度，相对于header中特定的坐标系和时间戳
    //设定marker的大小----- 1x1x1 表示边长为１m
    marker.scale.x = 0.3;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
    float pa=0;  float pb=0;  float pc=0; float pd=0;
    float p1a=0;  float p1b=0;  float p1c=0;
    float ttt;
    //向量起点A，就是Pose的position.
    //计算将向量[1,0,0]旋转到向量AB方向的四元数。
    //target_v = AB/|AB| //单位向量
    //r = [1,0,0] X target_v
    //r = r/|r| //单位向量
    //Theta = arccos(target_v*[1,0,0]) //旋转角
    //quaternion = (r*sin(Theta/2),cos(Theta/2))
    float cr1=0;  float cr2=0;
    int attc=0;
    float min_z = 999;
    float max_z = -999;

    while (std::getline(file, line)) 
    {
        attc++;
        std::istringstream iss(line);
        std::string number;
        std::vector<float> floats;
        while (iss >> number)floats.push_back(std::stof(number));
        marker.id = attc;

        if(floats[3]==-1&&floats[4]==-1&&floats[5]==-1&&floats[6]==-1)
        {
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position.x = floats[0];
        marker.pose.position.y = floats[1];
        marker.pose.position.z = floats[2];
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        }
        else
        {
    //设置marker的位姿。这是6个自由度，相对于header中特定的坐标系和时间戳
    //设定marker的大小----- 1x1x1 表示边长为１m
    marker.scale.x = 0.02;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    // marker.type = visualization_msgs::Marker::ARROW;
    marker.type = visualization_msgs::Marker::CUBE;

        pa = floats[0];
        pb = floats[1];
        pc = floats[2];
        pd = floats[3];
        float tem_d = (pa*floats[4]+pb*floats[5]+pc*floats[6]+pd)/sqrt(pa*pa+pb*pb+pc*pc);

        marker.pose.position.x = floats[4]-tem_d*pa;
        marker.pose.position.y = floats[5]-tem_d*pb;
        marker.pose.position.z = floats[6]-tem_d*pc;//Voxel中心在小方面上的投影坐标
        // marker.pose.position.z = (pa*floats[4]+pb*floats[5]+pd)/(0-pc);
                    min_z  = std::min(min_z,floats[6]-tem_d*pc);
                    max_z  = std::max(max_z,floats[6]-tem_d*pc);
        ttt = sqrt(pa*pa+pb*pb+pc*pc);
        pa/=ttt;    pb/=ttt;    pc/=ttt;//target_v
        p1a = 0;    p1b = 0-pc; p1c = pb;//[1 0 0 ]cross pa pb pc
        ttt = sqrt(p1a*p1a+p1b*p1b+p1c*p1c);
        p1a =0; p1b = p1b/ttt; p1c = p1c / ttt;//normalize r
        ttt = pa*1+pb*0+pc*0;
        ttt = std::acos(ttt);//Theta
        geometry_msgs::Quaternion q4;
        q4.x = p1a*sin(ttt/2);    q4.y = p1b*sin(ttt/2);    q4.z = p1c*sin(ttt/2);
        q4.w = cos(ttt/2);
        marker.pose.orientation.x = q4.x;
        marker.pose.orientation.y = q4.y;
        marker.pose.orientation.z = q4.z;
        marker.pose.orientation.w = q4.w;
        }
        PlaneMarker.markers.push_back(marker);
    }
    for(int i =0;i<PlaneMarker.markers.size();i++)
    {
        if(PlaneMarker.markers[i].color.r>0.9)continue;
        Color thisc =  getColor(PlaneMarker.markers[i].pose.position.z, min_z, max_z);
        PlaneMarker.markers[i].color.r = thisc.r/255.0;
        PlaneMarker.markers[i].color.g = thisc.g/255.0;
        PlaneMarker.markers[i].color.b = thisc.b/255.0;
    }
        file.close();
        return;
}
int main(int argc, char** argv)
{
    ros::init(argc,argv,"show_data_node");
    ros::NodeHandle nh;
    // std::vector<float>test_kernel;
    // test_kernel.resize(60*60,0);

    // std::vector<float>test_kernel_obs;
    // test_kernel_obs = test_kernel;

    // double times =     ros::Time::now().toSec();
    // for(int i =0 ;i<test_kernel.size();i++)test_kernel[i]=float(i) / 10.0;
    // int sw_index =0;
    // std::array<float,9>kee={0,1,2,3,4,5,6,7,8};
    // std::array<float,9>da={0,1,2,3,4,5,6,7,8};
    // for(int i =0 ;i<test_kernel.size();i++)
    // {
    //     sw_index = i-60-1;
    //     if(sw_index>=3600||sw_index<0)continue;
    //     da[0]=test_kernel[sw_index];
    //     sw_index = i-60;
    //     if(sw_index>=3600||sw_index<0)continue;
    //     da[1]=test_kernel[sw_index];
    //     sw_index = i-60+1;
    //     if(sw_index>=3600||sw_index<0)continue;
    //     da[2]=test_kernel[sw_index];
    //     sw_index = i-1;
    //     if(sw_index>=3600||sw_index<0)continue;
    //     da[3]=test_kernel[sw_index];
    //     sw_index = i;
    //     if(sw_index>=3600||sw_index<0)continue;
    //     da[4]=test_kernel[sw_index];

    //     sw_index = i+1;
    //     if(sw_index>=3600||sw_index<0)continue;
    //     da[5]=test_kernel[sw_index];

    //     sw_index = i+60-1;
    //     if(sw_index>=3600||sw_index<0)continue;
    //     da[6]=test_kernel[sw_index];

    //     sw_index = i+60;
    //     if(sw_index>=3600||sw_index<0)continue;
    //     da[7]=test_kernel[sw_index];

    //     sw_index = i+60+1;
    //     if(sw_index>=3600||sw_index<0)continue;
    //     da[8]=test_kernel[sw_index];
    //     test_kernel_obs[i] = da[0]*kee[0]+da[1]*kee[1]+da[2]*kee[2]+da[3]*kee[3]+da[4]*kee[4]+da[5]*kee[5]
    //     +da[6]*kee[6]+da[7]*kee[7]+da[8]*kee[8];
    //     // std::cout<<"i"<<i<<std::endl;
    // }    
    // double times1 =     ros::Time::now().toSec();
    // std::cout<<""<<times1-times<<std::endl;    


    ros::Publisher showMap;
    showMap = nh.advertise<visualization_msgs::MarkerArray>("PlaneMap",5);
    ros::Rate rate(1);
    visualization_msgs::MarkerArray PlaneMarker;
    PublishPlane(PlaneMarker);
    while(ros::ok())
    {
        showMap.publish(PlaneMarker);
        rate.sleep();
    }
    return 1;
}
//问题2：mpc求解序列不完全。
