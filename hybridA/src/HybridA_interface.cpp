


// void Map_initial_test(nav_msgs::OccupancyGrid::Ptr map_test)
// {
//   map_test->info.resolution = 0.25;
//   map_test->info.origin.position.x = 0;
//   map_test->info.origin.position.y = 0;
//   map_test->info.origin.position.z = 0;
//   map_test->info.width= 80;
//   map_test->info.height= 80;
//   map_test->data.resize(80*80,0);
//   int wd = map_test->info.width;
//   for(int i =20;i<60;i++)
//   map_test->data[40+i*wd]=100;
// }

//功能：
//输入：自定义srv获得地图信息和当前位置、目标位置
//输出：参考坐标、参考指令通过另外一个srv发出给Controller。
// void HybridA_Solve_Handle(ros::NodeHandle nh) 
// {
//   ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
//   HybridAStar::Planner hy;
//   nav_msgs::OccupancyGrid::Ptr map_test(new nav_msgs::OccupancyGrid);
//   Map_initial_test(map_test);

//     double t1 =  ros::Time::now().toSec();
//     hy.setMap(map_test);
//     hy.plan();
//     hy.ref_v.clear();
//     hy.ref_w.clear();
//     hy.ref_x.clear();
//     hy.ref_y.clear();
//     hy.ref_yaw.clear();
//     hy.Pathmotion_primitive.clear();
//     double t2 =  ros::Time::now().toSec();
//     std::cout<<"Time cost is "<<t2-t1<<std::endl;
// }
//思考：如果起点在格子中心，6个扩展都在同一grid,是否还有解？

// int main(int argc,char** argv)
// {
//   ros::init(argc,argv,"hybrida");
//   ros::NodeHandle nh;
//   HybridA_Solve_Handle(nh);
//   while(1);
//   return 1;
// }
#include"HybridA_interface.hpp"
//输入:地图信息 一组起点-终点 轨迹指针
//输出:形成的可通行路径
void HybridA_Solve_Handle(nav_msgs::OccupancyGrid::Ptr map_test,std::vector<std::vector<float>> depa_destina,nav_msgs::Path::Ptr Path_ref) 
{
    HybridAStar::Planner hy;
    hy.setMap(map_test);
    float x1,x2,y1,y2,yaw1,yaw2;
    bool flag_solve=false;
    std::vector<std::vector<float>>::iterator des_iterator;
    Path_ref->header.frame_id="map";
    Path_ref->header.stamp = ros::Time::now();
    int start_goal_pair = 1;
    for(auto i=depa_destina.begin();i!=depa_destina.end();i++)
    {
      double t1 =  ros::Time::now().toSec();
      if(start_goal_pair==1)
      { //第一次按照输入的起点终点来
        x1 = (*i)[0];      y1 = (*i)[1];      yaw1 = (*i)[2];
        x2 = (*i)[3];      y2 = (*i)[4];      yaw2 = (*i)[5];
      }
      else
      { //第2次开始按照上一次规划的终点作为这次的起点
        x1 = x2;      y1 = y2;      yaw1 = yaw2;
        x2 = (*i)[3];      y2 = (*i)[4];      yaw2 = (*i)[5];
      }
     std::cout<<"Solve "<<start_goal_pair<<"Begin"<<std::endl;
     flag_solve = hy.plan(x1,y1,yaw1,x2,y2,yaw2);
     std::cout<<"Solve "<<start_goal_pair<<"End"<<std::endl;
     start_goal_pair++;
      if(!flag_solve){des_iterator = i;}
      while(!flag_solve)
      {
        des_iterator++;
        if(des_iterator!=depa_destina.end())
        {
          x2 = (*des_iterator)[3];      y2 = (*des_iterator)[4];      yaw2 = (*des_iterator)[5];
        }
        else 
        {hy.Clear_data(); return;}
        std::cout<<"Solve "<<start_goal_pair<<"Begin"<<std::endl;
        flag_solve = hy.plan(x1,y1,yaw1,x2,y2,yaw2);
        std::cout<<"Solve "<<start_goal_pair<<"End"<<std::endl;
        if(flag_solve){i = des_iterator;}
        start_goal_pair++;
      }
      //能够运行到这里说明有解
      for(int len = 0;len<hy.ref_v.size();len++)
      {
        geometry_msgs::PoseStamped this_ref;
        this_ref.header.frame_id = "map";
        
        // this_ref.pose.position.x = hy.ref_x[len];
        // this_ref.pose.position.y = hy.ref_y[len];
        // this_ref.pose.position.z = hy.ref_yaw[len];
        // this_ref.pose.orientation.x = hy.ref_v[len];
        // this_ref.pose.orientation.y = hy.ref_w[len];        

        tf::Quaternion tqua  =  tf::createQuaternionFromYaw(hy.ref_yaw[len]);
        this_ref.pose.position.x = hy.ref_x[len];
        this_ref.pose.position.y = hy.ref_y[len];
        this_ref.pose.position.z = 0;
        this_ref.pose.orientation.w = tqua.getW();
        this_ref.pose.orientation.x = tqua.getX(); 
        this_ref.pose.orientation.y = tqua.getY();         
        this_ref.pose.orientation.z = tqua.getZ(); 
        Path_ref->poses.push_back(this_ref);
      }
      hy.Clear_data();
      double t2 =  ros::Time::now().toSec();
//      std::cout<<"Time cost is "<<t2-t1<<std::endl;
    }
}
