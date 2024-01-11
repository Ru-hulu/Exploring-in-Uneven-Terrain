#include"planner.hpp"

int inline Local_planner::rotdir(float tar_yaw, float now_yaw)
{
  float del_yaw = tar_yaw-now_yaw;
  if(std::abs(del_yaw)<0.1) return 0;
  if(del_yaw>0)
  {
    if(del_yaw<M_PI)return 1;
    else return -1;
  }
  else
  {
    if(del_yaw>-M_PI)return -1;
    else return 1;
  }
}

int inline Local_planner::global2grid(float x, float y,float org_x,float org_y,float gridw,float gridh,float gridres)
{
  int grid_x = (int)((x-org_x)/gridres);
  int grid_y = (int)((y-org_y)/gridres);
  return grid_x+grid_y*gridw;
}

//碰撞返回false,不碰撞返回true
bool inline Local_planner::CheckCollision3(int grid2d_x,int grid2d_y,int grid_w,int grid_h)
{
  for(auto che_it = checklist3.begin();che_it!=checklist3.end();che_it++)
  {
    int shif_x = grid2d_x+che_it->first;//grid_map位置移动
    int shif_y = grid2d_y+che_it->second;//grid_map位置移动
    if(shif_x>0&&shif_x<(grid_w-1)&&shif_y>0&&shif_y<(grid_h-1))
    {
      if(lcpmap2d.data[shif_x+shif_y*grid_w]<=-40)//未知或者障碍物
      return false;
    }
    else return false;
  }
    return true;
}

//碰撞返回false,不碰撞返回true
bool inline Local_planner::CheckCollision2(int grid2d_x,int grid2d_y,int grid_w,int grid_h)
{
  for(auto che_it = checklist2.begin();che_it!=checklist2.end();che_it++)
  {
    int shif_x = grid2d_x+che_it->first;//grid_map位置移动
    int shif_y = grid2d_y+che_it->second;//grid_map位置移动
    if(shif_x>0&&shif_x<(grid_w-1)&&shif_y>0&&shif_y<(grid_h-1))
    {
      if(lcpmap2d.data[shif_x+shif_y*grid_w]<=-40)//未知或者障碍物
      return false;
    }
    else return false;
  }
    return true;
}


bool inline Local_planner::CheckCollision1(int grid2d_x,int grid2d_y,int grid_w,int grid_h)
{
  for(auto che_it = checklist1.begin();che_it!=checklist1.end();che_it++)
  {
    int shif_x = grid2d_x+che_it->first;//grid_map位置移动
    int shif_y = grid2d_y+che_it->second;//grid_map位置移动
    if(shif_x>0&&shif_x<(grid_w-1)&&shif_y>0&&shif_y<(grid_h-1))
    {
      if(lcpmap2d.data[shif_x+shif_y*grid_w]<=-40)//未知或者障碍物
      return false;
    }
    else return false;
  }
    return true;
}
//碰撞返回false,不碰撞返回true
bool inline Local_planner::CheckCollision_self(int grid2d_x,int grid2d_y,int grid_w,int grid_h)
{
    if(grid2d_x>0&&grid2d_x<(grid_w-1)&&grid2d_y>0&&grid2d_y<(grid_h-1))
    {
      if(lcpmap2d.data[grid2d_x+grid2d_y*grid_w]<=-40)//未知或者障碍物
      return false;
      else return true;
    }
    return false;
}


//构建TSP求解问题，得到的控制点路径全部都在atsp_vp_travel_path中
void Local_planner::EscapeFromStack(Eigen::Vector2f v_sum)//传入的是建议速度方向
{
      float posx=0;
      float posy=0;
      float posz=0;
      //机器人的位置信息
      float pos_yaw = 0;//机器人的yaw
      float safe_xita=0;
      if(v_sum(0)==0) safe_xita = M_PI_2;
      else  safe_xita = std::atan2(v_sum(1),v_sum(0));      
      tf::Quaternion qw1; 
    gazebo_msgs::GetModelState rq;
    geometry_msgs::Twist cmd_twist;
    // gazebo_msgs::SetModelState Ss;
    rq.request.model_name="scout/";
    rq.request.relative_entity_name="map";
    // Ss.request.model_state.model_name= "robot";
    // Ss.request.model_state.reference_frame= "robot";
    ros::Rate tz(10);
    while(ros::ok())
    {
      while(!states_client.call(rq))
      std::cout<<"Waiting for gzbo";
      posx=rq.response.pose.position.x;
      posy=rq.response.pose.position.y;
      posz=rq.response.pose.position.z;
      qw1={rq.response.pose.orientation.x,rq.response.pose.orientation.y,rq.response.pose.orientation.z,rq.response.pose.orientation.w}; 
      pos_yaw = tf::getYaw(qw1);//机器人的yaw
      now_w_x = posx;    now_w_y = posy;    now_w_yaw = pos_yaw;
      int fla = rotdir(safe_xita,pos_yaw);
      if(fla==0)break;
      cmd_twist.angular.z = 1.0 * float(fla);      
      cmd_vel_pub.publish(cmd_twist);
      // Ss.request.model_state.twist.angular.z = 12.0 * float(fla);
      // set_state_client.call(Ss);
      std::cout<<"safe vector is "<<v_sum<<std::endl;  
      std::cout<<"escape rotate !!!! rotate speed is "<<cmd_twist.angular.z<<" safe_xita  "<< safe_xita <<" pos_yaw "<<pos_yaw<<std::endl;
      tz.sleep();
    }//目前，机器人已经转向到了合适的角度。
      cmd_twist.angular.z = 0;      
      cmd_twist.linear.x = 0.5;      
        // Ss.request.model_state.twist.angular.z = 0;
        // Ss.request.model_state.twist.linear.x = 3.0;
    int safestep = 5;        
    while(safestep>0)
    {
      cmd_vel_pub.publish(cmd_twist);
      // set_state_client.call(Ss);
      safestep--;
      tz.sleep();
    }//这里控制机器人向安全方向前进一段距离
      cmd_twist.angular.z = 0;      
      cmd_twist.angular.x = 0.0;      
      cmd_vel_pub.publish(cmd_twist);
    // Ss.request.model_state.twist.angular.z = 0;
    // Ss.request.model_state.twist.linear.x = 0.0;
    // set_state_client.call(Ss);
    //对机器人的状态信息再一次进行更新。
    while(!states_client.call(rq))
    std::cout<<"Waiting for gzbo";
    now_w_x=rq.response.pose.position.x;
    now_w_y=rq.response.pose.position.y;
    qw1={rq.response.pose.orientation.x,rq.response.pose.orientation.y,rq.response.pose.orientation.z,rq.response.pose.orientation.w}; 
    now_w_yaw = tf::getYaw(qw1);//机器人的yaw
}

bool Local_planner::FomulateTSP()
{
  //获得二维的栅格地图数据  
  float rs = lcpmap2d.resolution;
  int tempw = lcpmap2d.width;
  int temph = lcpmap2d.height;
  float sub2d_startx =lcpmap2d.org_x;
  float sub2d_starty =lcpmap2d.org_y;//grid_map的原点位置

  std::vector<std::pair<float,float>> vp_list;//下面这个循环得到的是观测点list,有1个聚类可能拥有几个观测点,绝对坐标
  std::pair<float,float> vp; vp.first=-1;vp.second=-1;
  for(int i=0;i<submapfrontierclustercenter_v.size();i++)
  {
    float tempx=submapfrontierclustercenter_v[i].x();
    float tempy=submapfrontierclustercenter_v[i].y();//每个聚类中心的绝对位置
	  //grid2dmap.data[int((tempx-sub2d_startx)/rs)+int((tempy-sub2d_starty)/rs)*tempw] = 80 ;		
    float delt_xita = 0.39; float xita_up = 6.28;//16个索引角度
    //保证聚类中心，一个角度一定要有一个观测点
    for(float xita =0; xita<xita_up; xita += delt_xita)
    {
      for(float grid_sample_r=0.5;grid_sample_r<=2.0;grid_sample_r = grid_sample_r+0.25)//采样半径步长 14次采样
      {
        float rx = tempx + grid_sample_r*std::cos(xita);         
        float ry = tempy + grid_sample_r*std::sin(xita);//采样点绝对位置        
        int grid_x =  int( (rx-sub2d_startx)/rs);
        int grid_y =  int( (ry-sub2d_starty)/rs);//采样点在grid_map位置
        // grid2dmap.data[grid_x+grid_y*tempw]=50;
        //检测grid_x grid_y的碰撞检测是否达标。
        if(!CheckCollision_self(grid_x,grid_y,tempw,temph))break;//一个角度如果出现了障碍物，直接放弃(raycast)
        if(std::abs(lcpmap2d.data[grid_x+grid_y*tempw]-submapfrontierclustercenter_v[i].z())>=0.55)break;
        if(!CheckCollision2(grid_x,grid_y,tempw,temph))continue;
        else 
        {
          // grid2dmap.data[grid_x+grid_y*tempw]=0;
          vp.first=rx;vp.second=ry;
          bool sp_flag=false;//如果当前角度得到的观测点距离其他角度的观测点太近，则就不纳入
          for(auto sw=vp_list.begin();sw!=vp_list.end();sw++)
          {
             if(std::abs(sw->first-rx)+std::abs(sw->second-ry)<1.0)
             {
              sp_flag=true;break;
             }           
          }
          if(!sp_flag)vp_list.push_back(vp); //得到一个聚类中心合理的采样点
          break;//运行到这里，要么这个角度刚刚存入了采样点，要么这个角度得到的第一个采样点和其他采样点距离很近。直接放弃这个角度
        }
        //如果出现碰撞，则continue;
      }
    }
  }
    gazebo_msgs::GetModelState rq;
    rq.request.model_name="scout/";
    rq.request.relative_entity_name="map";
    while(!states_client.call(rq))
    std::cout<<"Waiting for gzbo";
    float posx=rq.response.pose.position.x;
    float posy=rq.response.pose.position.y;
    float posz=rq.response.pose.position.z;//机器人的位置信息
    tf::Quaternion qw1={rq.response.pose.orientation.x,rq.response.pose.orientation.y,rq.response.pose.orientation.z,rq.response.pose.orientation.w}; 
    float pos_yaw = tf::getYaw(qw1);//机器人的yaw
    now_w_x = posx;    now_w_y = posy;    now_w_yaw = pos_yaw;
    astar_planner::AstarPlanner Ap(&(lcpmap2d.data),lcpmap2d.width,lcpmap2d.height,lcpmap2d.org_x,lcpmap2d.org_y,lcpmap2d.resolution);//初始化Astar算法
    // saveOccupancyGridAsJpg(lcpmap2d, "/home/r/Mysoftware/TARE/src/planner_server/src/path_pre.jpg");
    vp_reach_list.clear();
    std::vector<float> vp_reach_dis;//从当前位置出发，到达所有观测点的距离
    std::vector<std::vector<geometry_msgs::PoseStamped>> Now2Goal;//存放当前位置到所有的vp的路径


    Eigen::Vector2f v_temp;
    Eigen::Vector2f v_sum; 
    //为了防止起点是不安全的,这样就没有解了
  	while(!CheckCollision1( int((now_w_x-sub2d_startx)/rs), int((now_w_y-sub2d_starty)/rs),  tempw,  temph))
    {
      v_temp.setZero();
      v_sum.setZero();
      int sw_x = int((now_w_x-sub2d_startx)/rs);      int sw_y = int((now_w_y-sub2d_starty)/rs);
      for(int swfx = -3;swfx<=3;swfx=swfx+1)
      for(int swfy = -3;swfy<=3;swfy=swfy+1)
      {
        if(swfx==0&&swfy==0)continue;
        //拿到这个范围内的所有障碍物，加权得到障碍物指向当前车体的向量，作为车辆的速度。
        if(!CheckCollision_self(swfx+sw_x,swfy+sw_y,tempw,temph))
        {
          v_temp(0) = swfx;v_temp(1) = swfy;
          v_sum = v_sum + v_temp/v_temp.norm()/v_temp.norm()*(-1);//方向和权重，距离小，权重大
          if(std::isnan(v_sum(0))||std::isnan(v_sum(1))){std::cout<<swfx<<"  "<<swfy<<std::endl;}
        } 
      }
      v_sum = v_sum.normalized();//指向安全方向的向量
      std::cout<<"Start illegal, ESCAPE TO SAFE !!!"<<std::endl;
      EscapeFromStack(v_sum);
    } 
    std::vector<int> useless_debug;      useless_debug.push_back(int((now_w_x-sub2d_startx)/rs)+int((now_w_y-sub2d_starty)/rs)*tempw);
    std::vector<geometry_msgs::PoseStamped> unreach_goal;
    std::vector<geometry_msgs::PoseStamped> aplan;

    //为了防止起点是不安全的,这样就没有解了
    for(auto sam_vp = vp_list.begin();sam_vp!=vp_list.end();sam_vp++)
    {//Astar 测试从当前位置是否可到达vp 测试区
      float te_x = sam_vp->first; float te_y = sam_vp->second;
      te_x = float(int(te_x/res_map))*res_map;      te_y = float(int(te_y/res_map))*res_map;//这里将观测点进行规整化,统一一下,对后续的轨迹平滑有好处.
      geometry_msgs::PoseStamped astart; astart.pose.position.x = now_w_x; astart.pose.position.y = now_w_y;
      geometry_msgs::PoseStamped agoal; agoal.pose.position.x = te_x; agoal.pose.position.y = te_y;
  	  if(lcpmap2d.data[int((sam_vp->first-sub2d_startx)/rs)+int((sam_vp->second-sub2d_starty)/rs)*tempw]<=-50)//代表障碍物或未知
      while(1)std::cout<<"Goal illegal"<<std::endl;
      useless_debug.push_back(int((sam_vp->first-sub2d_startx)/rs)+int((sam_vp->second-sub2d_starty)/rs)*tempw);
      aplan.clear();
      bool thisas = Ap.makePlan(astart,agoal,&aplan);
      unreach_goal.push_back(agoal);
      if(thisas)
      {
        float x_t = astart.pose.position.x - agoal.pose.position.x;
        float y_t = astart.pose.position.y - agoal.pose.position.y;
        // float xi_ = std::atan2(y_t,x_t);
        // float coe  = now_w_yaw - xi_;
        // if(coe<0-M_PI) coe = 2*M_PI + coe;
        // if(coe>M_PI) coe = 2*M_PI - coe;
        // if(coe<0) coe = 0-coe;
        // coe/=M_PI;
        // coe = 1-coe;
        // coe=1;
        // std::cout<<"yaw is "<< pos_yaw<<"xi is "<<xi_<<"coe is "<<coe<<std::endl;
        // vp_reach_list.push_back(*sam_vp);//从当前位置可以到达的观测点
        vp_reach_list.push_back(std::pair<float,float>(te_x,te_y));//从当前位置可以到达的观测点
        vp_reach_dis.push_back(1.0 * float(aplan.size()));//从当前位置可以到达的观测点的距离
        // vp_reach_dis.push_back(coe * float(aplan.size()));//从当前位置可以到达的观测点的距离
        Now2Goal.push_back(aplan);//从当前位置可以到达的观测点的路径
      }
    }//Astar 测试从当前位置是否可到达vp 测试区
    vp_list.clear();
    atsp_vp_sequence.clear();
                                                                                  for(int i =0;i<vp_reach_list.size();i++)
                                                                                  {
                                                                                    int temp_show_x = int((vp_reach_list[i].first-gdmap_show.info.origin.position.x)/rs);
                                                                                    int temp_show_y = int((vp_reach_list[i].second-gdmap_show.info.origin.position.y)/rs);
                                                                                    gdmap_show.data[temp_show_x+temp_show_y*gdmap_show.info.width] = 80;
                                                                                  }
                                                                                  gdmap_show.info.origin.position.z = 1.5;
                                                                                  gridmapPub.publish(gdmap_show);  
    if(vp_reach_dis.size()==1)
    {
      atsp_vp_sequence.push_back(vp_reach_list[0]);
      atsp_vp_travel_path.clear();//按tsp求解顺序访问所有的观测点，ASTAR轨迹
      atsp_vp_travel_path.push_back(Now2Goal[0]);//这是由于只有一个可达的观测点，所有就选择这个观测点的路径
      std::cout<<"此时只有一个可达观测点！！观测点位置为 "<<vp_reach_list[0].first<<" "<<vp_reach_list[0].second<<std::endl;
      atsp_vp_sequence_index.clear();//访问vp的顺序
      atsp_vp_path_All.clear();//astar求解的vp彼此之间全连接路径(不包括当前位置到vp的路径)    
      vp_reach_list.clear();
      vp_reach_dis.clear();
      return true;
    }
    if(vp_reach_dis.size()==0)
    {
      // gdmap_show.data.clear();
      lcpmap2d.data.clear();
      std::cout<<"ATSP No path found， Recall Map!!!!!!"<<std::endl;
      std::cout<<"ATSP No path found， Recall Map!!!!!!"<<std::endl;
      std::cout<<"ATSP No path found， Recall Map!!!!!!"<<std::endl;
      //DO SOME DATA CLEAR HERE!!!!!!
      //DO SOME DATA CLEAR HERE!!!!!!
      submapfrontier_v.clear();
      submapfrontierclustercenter_v.clear();
      atsp_vp_sequence_index.clear();//访问vp的顺序
      atsp_vp_sequence.clear();//访问vp的顺序,绝对坐标
      atsp_vp_path_All.clear();//astar求解的vp彼此之间全连接路径(不包括当前位置到vp的路径)    
      atsp_vp_travel_path.clear();//按tsp求解顺序访问所有的观测点，ASTAR轨迹
      //DO SOME DATA CLEAR HERE!!!!!!
      //DO SOME DATA CLEAR HERE!!!!!!
      return false;
    }

    Eigen::MatrixXf ATSP_weight(vp_reach_list.size()+1,vp_reach_list.size()+1);
    ATSP_weight.setZero();
    atsp_vp_path_All.clear();
    for(int i=1;i<=vp_reach_list.size();i++){ATSP_weight(0,i) = vp_reach_dis[i-1];ATSP_weight(i,i)=999;}//先对第一列进行填充
    for(int i=1;i<=vp_reach_list.size();i++)
    for(int j=i+1;j<=vp_reach_list.size();j++)//对陈矩阵
    {
      geometry_msgs::PoseStamped astart; astart.pose.position.x = vp_reach_list[i-1].first; astart.pose.position.y = vp_reach_list[i-1].second;
      geometry_msgs::PoseStamped agoal; agoal.pose.position.x = vp_reach_list[j-1].first; agoal.pose.position.y = vp_reach_list[j-1].second;
      std::vector<geometry_msgs::PoseStamped> aplan;
      bool thisas = Ap.makePlan(astart,agoal,&aplan);
      atsp_vp_path_All.push_back(aplan);
      ATSP_weight(i,j) = aplan.size();
      ATSP_weight(j,i) = aplan.size();
      aplan.clear();
    }       
    // gdmap_show = lcpmap2d;   
    Solve_ATSP(&vp_reach_list,ATSP_weight);
    std::cout<<"TSP finish "<<std::endl;
    std::vector<geometry_msgs::PoseStamped> this_path = Now2Goal[atsp_vp_sequence_index[0]];//第一个要访问的位置，是从当前位置出发的，所以    
    for(auto it=this_path.begin();it!=this_path.end();it++)
    {
      float tempx = (it->pose.position.x-sub2d_startx);      float tempy = (it->pose.position.y-sub2d_starty);
      // gdmap_show.data[int(tempx/rs)+int(tempy/rs)*tempw] = 50;//代表路径        
      // PosePub.publish(*it);    
    }
    atsp_vp_travel_path.clear();
    float last_x = this_path[this_path.size()-1].pose.position.x;
    float last_y = this_path[this_path.size()-1].pose.position.y;
    // this_path.pop_back();//把尾部pop
    atsp_vp_travel_path.push_back(this_path); 
    Now2Goal.clear();     
    int ind_1;    int ind_2; int s_l = atsp_vp_sequence_index.size();
    for(int j=0;j<s_l;j++)
    {
      if(j+1==s_l)break;
      this_path.clear();
      ind_1 = std::min(atsp_vp_sequence_index[j+1],atsp_vp_sequence_index[j]);
      ind_2 = std::max(atsp_vp_sequence_index[j+1],atsp_vp_sequence_index[j]);
      int fly_index = 0;
      if(ind_1==0) fly_index=0;
      else fly_index = (s_l-1+s_l-ind_1) * ind_1 /2;
      fly_index += ind_2-ind_1-1;
      // std::cout<<"Fly index is "<<fly_index<<std::endl;
      this_path = atsp_vp_path_All[fly_index];
      float first_x = this_path[0].pose.position.x;
      float first_y = this_path[0].pose.position.y;
      float tlast_x = this_path[this_path.size()-1].pose.position.x; 
      float tlast_y = this_path[this_path.size()-1].pose.position.y;
      if((first_x-last_x)*(first_x-last_x)+(first_y-last_y)*(first_y-last_y)>
        (tlast_x-last_x)*(tlast_x-last_x)+(tlast_y-last_y)*(tlast_y-last_y))
      std::reverse(this_path.begin(),this_path.end());//这条轨迹的首尾可能是反的 //这里的轨迹可能是a->b也可能是b->a!!!
      last_x = this_path[this_path.size()-1].pose.position.x;//拿到这条轨迹的尾部节点
      last_y = this_path[this_path.size()-1].pose.position.y;//拿到这条轨迹的尾部节点
      if (!this_path.empty())this_path.erase(this_path.begin());//因为这条轨迹的头节点和上一条的尾部节点是重合的
      if(this_path.size()==0)continue;
      atsp_vp_travel_path.push_back(this_path);
      for(auto it=this_path.begin();it!=this_path.end();it++)
      {
        float it_x = it->pose.position.x;
        float it_y = it->pose.position.y;
        // gdmap_show.data[int((it_x-sub2d_startx)/rs)+int((it_y-sub2d_starty)/rs)*tempw] = 50;//代表路径		              
      }
    }
    // std::cout<<"Fly index finish "<<std::endl;
    for(int j=0;j<vp_reach_list.size();j++)
    // gdmap_show.data[int((vp_reach_list[j].first-sub2d_startx)/rs)+int((vp_reach_list[j].second-sub2d_starty)/rs)*tempw] = 80 ; //代表访问vp   
    atsp_vp_path_All.clear();
    atsp_vp_sequence_index.clear();
    submapfrontierclustercenter_v.clear();
    std::cout<<"now pos is "<< now_w_x<<" "<<now_w_y<<std::endl;
    std::cout<<"in atsp_vp_travel_path, start pos is "<< atsp_vp_travel_path[0][0].pose.position.x<<" "<<atsp_vp_travel_path[0][0].pose.position.y<<std::endl;
    return true;
}

//输入:所有的可达观测点
//输出：排序后的观测点atsp_vp_sequence、观测点的序列id  atsp_vp_sequence_index
void Local_planner::Solve_ATSP(std::vector<std::pair<float,float>>* vp_l,Eigen::MatrixXf Weight_matrix)
{
  std::ofstream out("/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/src/planeoct_server/src/vp6.atsp");
  std::string buf="NAME:  vp6\n";  out << buf;
  buf="TYPE: ATSP\n";  out << buf;
  buf="COMMENT: 6 vp problem (Repetto)\n";  out << buf;
  buf = "DIMENSION:  "; 
  buf +=std::to_string(vp_l->size()+1);//这里已经能够灵活的处理问题的维度了
  buf +="\n";
  out << buf;
  buf="EDGE_WEIGHT_TYPE: EXPLICIT\n";
  out << buf;
  buf="EDGE_WEIGHT_FORMAT: FULL_MATRIX\n";
  out << buf;
  buf="EDGE_WEIGHT_SECTION\n";
  out << buf;
  for(int i=0;i<=vp_l->size();i++)
  {
    std::string row_data="";
    for(int j=0;j<=vp_l->size();j++)
    {
      row_data+=std::to_string(int(Weight_matrix(i,j)));
      row_data+=" ";
    }
      row_data+="\n";
      out << row_data;
  }
  buf="EOF";
  out << buf;
  out.close();
  std::string fn="/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/src/planeoct_server/src/pr30.par";
  int da = Solve_TSP(fn.c_str());
  // std_srvs::SetBool srvmsg;srvmsg.request.data=true;
  // buf = "DIMENSION:  "; 
  // buf +=std::to_string(vp_l->size()+1);
  // std::cout<<buf<<std::endl;
  // ATSP_client.call(srvmsg); 
  std::cout<<"tsp bag ok "<<std::endl;
  std::ifstream read_result;
  read_result.open("/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/src/planeoct_server/src/output.txt");
  std::string read_line;
  atsp_vp_sequence.clear();
  atsp_vp_sequence_index.clear();
  int cct=0;
  while(cct!=6)//这个6和问题的维度无关。
  {
    getline(read_result,read_line);
    cct++;
  }
  while(getline(read_result,read_line))
  {
//    std::cout<<"read line "<<read_line<<std::endl;    
    int idx = std::stoi(read_line); 
    if(idx==1)continue;//是自己当前位置
    if(idx==-1) break;
    atsp_vp_sequence_index.push_back(idx-2);
    std::pair<float,float> vp = vp_l->at(idx-2);
    atsp_vp_sequence.push_back(vp);    
    cct++;
  }
  read_result.close();
}

//1月8日补充聚类中心点的优化
//若中心点的位置和成员不在同一个高度,则需要对位置进行微调
//raycast过程中需要检测的遮挡不仅仅是障碍物，还包括高度遮挡
//对于frontier的优化函数进行调整,如果frontier和路径点的高度差异不大,则为吸引力,否则为排斥力.
bool Local_planner::LPS_Planner()//HybridA求解得到参考控制序列。
{
    std::cout<<"局部规划开始"<<std::endl;
    double thist0 = ros::Time::now().toSec();
    std::vector<int> real_label;//长度可能不足6
    
    std::cout<<"The size of submapfrontier_v is "<<submapfrontier_v.size();
    if(submapfrontier_v.size()==0)while(1);
    int actual_class_num = Spectral_Clustering(&(submapfrontier_v),&real_label);
    octomap::point3d temp_center;
    std::vector<int> clas_ct;//每个类的数量,暂时认为最多就6类
    submapfrontierclustercenter_v.clear();//聚类的中心
    for(int i =0;i<actual_class_num;i++)
    {
      clas_ct.push_back(0);
      submapfrontierclustercenter_v.push_back(octomap::point3d(0,0,0));
    }
    std::cout<<"submapfrontierclustercenter_v "<<submapfrontierclustercenter_v.size()<<std::endl;
    std::cout<<"actual_class_num "<<actual_class_num<<std::endl;

    for(int i = 0;i<real_label.size();i++)
    {
      int lb = int(real_label[i]);
      clas_ct[lb] = clas_ct[lb]+1;
      submapfrontierclustercenter_v[lb] = submapfrontierclustercenter_v[lb]+submapfrontier_v[i];
    }
    ShowCluster(&real_label);
    
    for(int i = 0;i<actual_class_num;i++)
    {
        submapfrontierclustercenter_v[i] /= float(clas_ct[i]);
        int cluster_center_index_elev = global2grid(submapfrontierclustercenter_v[i].x(),submapfrontierclustercenter_v[i].y(),
        lcpmap2d.org_x,lcpmap2d.org_y,lcpmap2d.width,lcpmap2d.height,lcpmap2d.resolution);
        int temp_index = 0;
        int now_x=cluster_center_index_elev%(lcpmap2d.width);
        int now_y=cluster_center_index_elev/(lcpmap2d.width);
        if(std::abs(lcpmap2d.data[cluster_center_index_elev]-submapfrontierclustercenter_v[i].z())>=0.3)
        {
            int ssw_x = 0;            int ssw_y = 0;
            for(int temp_x =-3;temp_x<=3;temp_x++)
            for(int temp_y =-3;temp_y<=3;temp_y++)
            {
              ssw_x = now_x+temp_x;
              ssw_y = now_y+temp_y;
              if(ssw_x>=0 && ssw_x<lcpmap2d.width && ssw_y<lcpmap2d.height&& ssw_y>=0)
              {
                temp_index = ssw_y*lcpmap2d.width+ssw_x;
                if(std::abs(lcpmap2d.data[temp_index]-submapfrontierclustercenter_v[i].z())<0.3)
                {
                  float target_z = lcpmap2d.data[temp_index];
                  submapfrontierclustercenter_v[i] = octomap::point3d(float(ssw_x)*lcpmap2d.resolution + lcpmap2d.org_x,
                                                                      float(ssw_y)*lcpmap2d.resolution + lcpmap2d.org_y,
                                                                      target_z);
                }
              }
            }
        }
    }
    //拿到了每一个聚类的中心位置

    //这里对聚类中心的位置进行一个调整.



    double thist1 = ros::Time::now().toSec();
    bool formulate_ok = FomulateTSP();//这里对TSP问题进行构建，如果无解需要重新请求地图
    double thist2 = ros::Time::now().toSec();
    std::cout<<"局部规划时间开销："<<thist2-thist0 <<" 聚类 "<<thist1-thist0<<"astar "<<thist2-thist1<<std::endl;
    std::cout<<"局部结束"<<std::endl;
    // if(formulate_ok) gridmapPub.publish(gdmap_show);  //构建成功，有路径解
    return formulate_ok;
}
void Local_planner::ShowCluster(std::vector<int> *real_label)
{
  visualization_msgs::MarkerArray fronNodes;//节点数据结构，用于可视化
  visualization_msgs::Marker bbox_marker;
  fronNodes.markers.clear();
  bbox_marker.header.stamp = ros::Time::now();
  bbox_marker.header.frame_id = "map";
  bbox_marker.ns = "fron_cluns";
  bbox_marker.action = visualization_msgs::Marker::ADD;
  bbox_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  bbox_marker.color.r = 0.0f;
  bbox_marker.color.g = 1.0f;
  bbox_marker.color.b = 0.0f;
  bbox_marker.color.a = 1;
  bbox_marker.scale.x = 0.25;
  bbox_marker.scale.y = 0.25;
  bbox_marker.scale.z = 0.25;
  int ct =0;
  for(int i=0;i<submapfrontier_v.size();i++)
  {
    bbox_marker.id = ct;
    bbox_marker.pose.position.x=submapfrontier_v[i].x();
    bbox_marker.pose.position.y=submapfrontier_v[i].y();
    bbox_marker.pose.position.z=submapfrontier_v[i].z()+0.75;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.text = std::to_string((*real_label)[i]);
    fronNodes.markers.push_back(bbox_marker);
    ct++;
  }
  std::cout<<"frontier number is "<<ct<<std::endl;
  show_frontier_cluster.publish(fronNodes);
}
 
//lcpmap2d elevation地图已经准备好了
bool Local_planner::GetMapService(bool local_flag)//如果没有得到frontier信息，直接返回false,代表地图无效。
{   //这样写，service就不会被阻塞了
    if(local_flag)std::cout<<"Ask for local map！！！"<<std::endl;
    else std::cout<<"Ask for Global map！！！"<<std::endl;
    srvbg::getlcplan temp_v;
    temp_v.request.local_map = local_flag;
    while(!getmap.call(temp_v)) std::cout<<"Waitting for gridmap and frontier."<<std::endl;
    std::cout<<"temp_v.response.frontier_x.size() "<<temp_v.response.frontier_x.size()<<std::endl;
    if(temp_v.response.frontier_x.size()==0)//这里应该要找到最近的一个有frontier的子空间
    while(1) std::cout<<"frontier size is zero!!"<<std::endl;    
                                                                      gdmap_show.info.height = temp_v.response.height;
                                                                      gdmap_show.info.width = temp_v.response.width;
                                                                      gdmap_show.info.resolution = temp_v.response.res;
                                                                      gdmap_show.info.origin.position.x = temp_v.response.org_x;
                                                                      gdmap_show.info.origin.position.y = temp_v.response.org_y;
                                                                      gdmap_show.data.clear();    
                                                submapfrontier_v.clear();
                                                lcpmap2d.data.clear();
                                                lcpmap2d.height = temp_v.response.height;
                                                lcpmap2d.width = temp_v.response.width;
                                                lcpmap2d.resolution = temp_v.response.res;
                                                lcpmap2d.org_x = temp_v.response.org_x;
                                                lcpmap2d.org_y = temp_v.response.org_y;
                                                int temp_size = temp_v.response.map2d_data.size();
                                                lcpmap2d.data.resize(temp_size,-100);//这个-100,非标注数据
                                                                      gdmap_show.data.resize(temp_size,100);//这个给100，标准数据
                                                float temp_min_z = temp_v.response.min_z;
                                                float temp_max_z = temp_v.response.max_z;
                                                std::cout<<"z range "<<temp_max_z<<" "<<temp_min_z<<std::endl;
                                                int show_he = 0;
                                                float temp_now_h = 0;
                                                for (int i=0;i<temp_size;i++)
                                                {
                                                  temp_now_h = temp_v.response.map2d_data[i];
                                                  lcpmap2d.data[i]=temp_now_h;//得到地图高度信息                                              
                                                  if(temp_now_h<=-40) show_he = 100;
                                                  else show_he = int(60.0*(temp_now_h-temp_min_z)/(temp_max_z-temp_min_z));
                                                  gdmap_show.data[i]=show_he;

                                                  if(show_he> 100||show_he< 0) std::cout<<show_he<<"  "<<temp_now_h<<std::endl;
                                                }
    for(int i=0;i<int(temp_v.response.frontier_x.size());i++)
    {
        float x,y,z;
        x =temp_v.response.frontier_x[i];        y =temp_v.response.frontier_y[i];        z =temp_v.response.frontier_z[i];
        octomap::point3d thisp(x,y,z);
        submapfrontier_v.push_back(thisp);//得到frontier信息
    }
    gridmapPub.publish(gdmap_show);  
    return true;
}

Local_planner::Local_planner(const ros::NodeHandle &nh_):nh(nh_)
{
	NonUniformBspline spline(nh_);
	bspline_ptr = make_shared<NonUniformBspline>(spline);
	bspline_ptr->InitialPara();//读取优化参数
  // plansrv = nh.advertiseService("getlcp",&Local_planner::LCPCallback,this);
  getmap = nh.serviceClient<srvbg::getlcplan>("gridmap_server");//地图服务器
  states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",5);
  gridmapPub = nh.advertise<nav_msgs::OccupancyGrid>("elevation_map",5);
  show_frontier_cluster = nh.advertise<visualization_msgs::MarkerArray>("frontier_cluster",5); 
  car_mpc_client = nh.serviceClient<srvbg::mpcref>("mpc_solver");
  checklist3.clear();
    for(int xxx = -3;xxx<=3;xxx++)
    for(int yyy = -3;yyy<=3;yyy++)
    {
        std::pair<int,int> tp;
        tp.first=xxx;
        tp.second = yyy;
        checklist3.push_back(tp);
    }
    
    checklist2.clear();
    for(int xxx = -2;xxx<=2;xxx++)
    for(int yyy = -2;yyy<=2;yyy++)
    {
        std::pair<int,int> tp;
        tp.first=xxx;
        tp.second = yyy;
        checklist2.push_back(tp);
    }
    checklist1.clear();
    for(int xxx = -1;xxx<=1;xxx++)
    for(int yyy = -1;yyy<=1;yyy++)
    {
        std::pair<int,int> tp;
        tp.first=xxx;
        tp.second = yyy;
        checklist1.push_back(tp);
    }

}

void Local_planner::Optimize_Bspline()
{
  bool opt_succ = true;
  std::cout<<"曲线拟合开始"<<std::endl;
  bspline_ptr->InitControl_list(&atsp_vp_travel_path);//控制点初始化  
  double t1 =ros::Time::now().toSec();
                                        nav_msgs::OccupancyGrid lcpmap2d_test;
                                        lcpmap2d_test.info.height = lcpmap2d.height;
                                        lcpmap2d_test.info.width = lcpmap2d.width;
                                        lcpmap2d_test.info.resolution = lcpmap2d.resolution;
                                        lcpmap2d_test.info.origin.position.x = lcpmap2d.org_x;
                                        lcpmap2d_test.info.origin.position.y = lcpmap2d.org_y;
                                        lcpmap2d_test.data.resize(lcpmap2d.data.size(),0);

                                        int size_lcp = int(lcpmap2d_test.info.height)*int(lcpmap2d_test.info.width);
                                        float kkzs_orgx = float(lcpmap2d_test.info.origin.position.x);                                          
                                        float kkzs_orgy = float(lcpmap2d_test.info.origin.position.y);
                                        float kkzs_rs = lcpmap2d_test.info.resolution;
                                        int  kkzs_x = 0 ;
                                        int  kkzs_y = 0 ;

                                        for(int kkzs = 0;kkzs<size_lcp;kkzs++)
                                        {
                                          if((lcpmap2d.data[kkzs]<=-40)&&(lcpmap2d.data[kkzs]>=-60))
                                          lcpmap2d_test.data[kkzs] = 100;  //只有-50是障碍物，-100是未知区域
                                        }
                                        for(int kkzs = 0 ;kkzs< submapfrontier_v.size();kkzs++)
                                        {
                                          kkzs_x =int( (submapfrontier_v[kkzs].x()-kkzs_orgx)/kkzs_rs   );
                                          kkzs_y =int( (submapfrontier_v[kkzs].y()-kkzs_orgy)/kkzs_rs   );
                                          int kkzs_index  = kkzs_y*int(lcpmap2d.width) + kkzs_x;
                                          lcpmap2d_test.data[kkzs_index] = 20;
                                        }
  std::cout<<"V图准备前"<<std::endl;
  bspline_ptr->PrepareObsVoronoi(&lcpmap2d_test);//给两张地图指针。
  std::cout<<"V图准备结束"<<std::endl;
  double t2 =ros::Time::now().toSec();

  
  opt_succ = bspline_ptr->Optimize_nlopt(false);//smooth 结果存储在refine_contr_list
  bspline_ptr->PublishBspline(false);//smooth结果可视化
  if(bspline_ptr->use_frontier)
  {
  opt_succ = bspline_ptr->Optimize_nlopt(true);//frontier 结果存储在refine_contr_list
  if(!opt_succ)bspline_ptr->Optimize_nlopt(false);//只有使用了frontier，并且失败的情况下，才会进行二次优化
  bspline_ptr->PublishBspline(true);//frontier优化结果可视化
  }
  if(bspline_ptr->use_cut)bspline_ptr->CutControlPoint();//cut的结果在这里可视化




  bspline_ptr->Freemaps();//优化成功正常释放内存并且发送优化结果
  //这里得到优化以后的控制点数据
  std::cout<<"优化成功正常释放内存并且发送优化结果"<<std::endl;
  double t3 =ros::Time::now().toSec();
  bspline_ptr->InitialSpline(3,0.1);//Bspline参数初始化三阶，节点向量步长0.1  
  bspline_ptr->saveplanning();
  bspline_ptr->GetCmd(now_w_x,now_w_y,now_w_yaw);
  double t4 =ros::Time::now().toSec();
  std::cout<<"总时间 "<<t4-t1<<" voronoi图时间 "<<t2-t1<<" 优化时间 "<<t3-t2<<" 拟合时间 "<<t4-t3<<std::endl;
  std::cout<<"曲线拟合结束"<<std::endl;
  srvbg::mpcref this_ref_traj;

  int len  = bspline_ptr->non_zero_line;
  if(len == 0)while(1)std::cout<<"The size of bspline is zero, fail!!!!!!"<<std::endl;
  for(int i =0;i<len;i++)
  {
    this_ref_traj.request.ref_x.push_back(bspline_ptr->bsline_list(i,0));
    this_ref_traj.request.ref_y.push_back(bspline_ptr->bsline_list(i,1));
    this_ref_traj.request.ref_yaw.push_back(bspline_ptr->bsline_list(i,2));
    this_ref_traj.request.ref_v.push_back(bspline_ptr->bsline_list(i,3));
    this_ref_traj.request.ref_w.push_back(bspline_ptr->bsline_list(i,4));
  }
 std::cout<<"Before call mpc client "<<std::endl;
 car_mpc_client.call(this_ref_traj);  
 std::cout<<"After call mpc client "<<std::endl;
}

void Local_planner::clearfrontiers()
{
  submapfrontier_v.clear();  
}


void Local_planner::saveOccupancyGridAsJpg(const nav_msgs::OccupancyGrid& occupancyGrid, const std::string& filename)
{
  // 创建一个空的OpenCV图像
  cv::Mat image(occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC1);
  for(int i=0;i<int(occupancyGrid.info.height)*int(occupancyGrid.info.width);i++)
  image.at<uchar>(i) = 0;
  // 遍历OccupancyGrid的数据，并将其转换为灰度值
  for (int i = occupancyGrid.data.size()-1; i >=0 ; i--)
  {
    if (occupancyGrid.data[i] == 99||occupancyGrid.data[i] == 100)
      image.at<uchar>(i) = 0;
    // 如果值为100，则将其设置为黑色
    else if (occupancyGrid.data[i] == 0||occupancyGrid.data[i] == 20)
      image.at<uchar>(i) = 255;
    // 如果值为0，则将其设置为白色
    else if (occupancyGrid.data[i] == 100)
      image.at<uchar>(i) = 0;
    else if (occupancyGrid.data[i] == 50)
      image.at<uchar>(i) = 128;
    // 如果值为-1，则将其设置为灰色
  }

  // 将图像保存为jpg文件
  cv::imwrite(filename, image);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_server");
    ros::NodeHandle nh;
    Local_planner p(nh);
    bool local_planner_hava_solution = true;
    while(ros::ok())//这整个过程可以控制在20ms以内
    {
      float t_planning_start  = ros::Time::now().toSec();
      if(!p.GetMapService(local_planner_hava_solution))continue;//请求地图服务器得到地图信息
      if(p.LPS_Planner()) 
      {
        local_planner_hava_solution = true; 
        p.Optimize_Bspline();
        p.clearfrontiers();
      }//ATSP未必有解，如果无解，需要重新得到地图
      else {local_planner_hava_solution = false;}
      float t_planning_end  = ros::Time::now().toSec();
      std::cout<<"Planning time "<<t_planning_end-t_planning_start<<std::endl;
      ros::spinOnce();
    }
    return 1;
}
