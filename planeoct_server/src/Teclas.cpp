//自定义点云的情况下,回调函数的执行时间如下.
// 深度转换时间0.055
// 滤波时间0.038
// oct索引时间0.026
// 平面拟合时间0.007
// 总时间0.127
#include "Teclas.hpp"
struct elevation_map
{
  std::vector<float> data;
  float width;
  float height;
  float org_x;
  float org_y;
  float resolution;
};
void Teclas::InitailMapDateDebug()
{
  // if(1)return;
    std::ifstream file("/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/debug_map.txt");
    std::string line;
    float posx=0;
    float posy=0;
    float posz=0;

  gazebo_msgs::SetModelState rq;
  rq.request.model_state.model_name="scout/";
  rq.request.model_state.reference_frame="map";

    while (std::getline(file, line)) 
    {
        std::istringstream iss(line);
        std::string number;
        std::vector<float> floats;
        while (iss >> number)floats.push_back(std::stof(number));
        if(floats.size()==3)
        {
          rq.request.model_state.pose.position.x = floats[0];
          rq.request.model_state.pose.position.y = floats[1];
          rq.request.model_state.pose.position.z = floats[2];
          while(!initial_debug_client.call(rq));
          continue;
        }
        else
        {
          octomap::point3d thisp(floats[0],floats[1],floats[2]);
          m_octree->updateNode(floats[0],floats[1],floats[2],true);
          m_octree->set_node_info(floats[0],floats[1],floats[2],int(floats[3]),floats[4],floats[5],floats[6],floats[7],0,0);
        }
      if(floats[3]==f_map::FRONT)
      {
              octomap::OcTreeKey* thisF(new octomap::OcTreeKey);
              *thisF = m_octree->coordToKey(floats[0],floats[1],floats[2]);//找到key          
              frontier.insert(thisF);
              if(thisF->k[0]==32759&&thisF->k[1]==32734&&thisF->k[2]==32768)
              {
              std::cout<<"Here"<<std::endl;
              std::cout<<"x "<<floats[0]<<"y "<<floats[1]<<"z "<<floats[2]<<std::endl;
              }
              std::pair<int,int> temp_pair = sub_space::Frontier2Subspaceindex(floats[0],floats[1],floats[2]);
              subspace_array[temp_pair.first][temp_pair.second]->addFrontier(thisF);
      }
    }
    std::cout<<"He"<<std::endl;
    //frontier初始化。
}
void Teclas::DebugSave2txt()
{
  gazebo_msgs::GetModelState rq;
  rq.request.model_name="scout/";
  rq.request.relative_entity_name="map";
  while(!states_client.call(rq))
  std::cout<<"Waiting for gzbo";
  float posx=rq.response.pose.position.x;
  float posy=rq.response.pose.position.y;
  float posz=rq.response.pose.position.z;
  std::ofstream file("/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/debug_map.txt");
  if (file.is_open()) 
  {
                      file << std::fixed << std::setprecision(3) << posx;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << posy;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << posz;
                      file << "\n";
  }

  float pa=0;  float pb=0;  float pc=0;
  float p1a=0;  float p1b=0;  float p1c=0;
  float ttt;
  float cr1=0;  float cr2=0;
  if (file.is_open()) 
  {
            for(f_map::FrontierOcTree::leaf_iterator it = m_octree->begin_leafs(m_octree->getTreeDepth()), end = m_octree->end_leafs();it != end; ++it)
            {
                      octomap::point3d tp =  it.getCoordinate();
                      it->get_plane(pa,pb,pc,ttt);
                      file << std::fixed << std::setprecision(3) << tp.x();
                      file << " ";
                      file << std::fixed << std::setprecision(3) << tp.y();
                      file << " ";
                      file << std::fixed << std::setprecision(3) << tp.z();
                      file << " ";
                      file << std::fixed << std::setprecision(3) << it->get_state();
                      file << " ";
                      file << std::fixed << std::setprecision(3) << pa;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << pb;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << pc;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << ttt;
                      file << "\n";

            }
  }
        file.close();
  return;    
}

Teclas::Teclas(const ros::NodeHandle &nh_):nh(nh_)
{
  planningspaceanchor_Pub = nh.advertise<geometry_msgs::PoseStamped>("/planning_space_anchor", 1);
  planningspaceanchor_Pub1 = nh.advertise<geometry_msgs::PoseStamped>("/planning_space_anchor1", 1);
  pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/testpcd", 1);
  ims=nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw",5,&Teclas::insertCloudCallback,this);
  showp = nh.advertise<visualization_msgs::MarkerArray>("Frontier_Marker",5);
  showMap = nh.advertise<visualization_msgs::MarkerArray>("PlaneMap",5);
  showone = nh.advertise<visualization_msgs::Marker>("onemarker",5);  
  savetxt = nh.advertiseService("save2txt",&Teclas::SaveMap2txt, this);
  savedebug = nh.advertiseService("savedebug",&Teclas::DebugService,this);  
  showLPS_VP = nh.advertise<visualization_msgs::MarkerArray>("LPS_VP",5);
  showLPS = nh.advertise<visualization_msgs::MarkerArray>("LPSMarker",5);
  showSUBS = nh.advertise<visualization_msgs::MarkerArray>("SUBMarker",5); 
  showcenter = nh.advertise<visualization_msgs::MarkerArray>("Cluster_Center",5); 
  gridmapPub = nh.advertise<nav_msgs::OccupancyGrid>("map2d",5);
  states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  initial_debug_client  = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  subspace_infor_server = nh.advertiseService("get_subspace",&Teclas::ShowSubspaceInfor, this);
  map_server = nh.advertiseService("gridmap_server", &Teclas::MapService, this);
  m_octree = new f_map::FrontierOcTree(0.25);
  double tem_res = m_octree->getResolution();
  // for(double i=-4.0;i<4.0;i=i+1.0)
  // for(double j=-4.0;j<4.0;j=j+1.0)
  // {//初始化的机器人位置必须要设定为安全已知
  //   m_octree->updateNode(tem_res*i,tem_res*j,tem_res,true);
  //   m_octree->set_node_info(tem_res*i,tem_res*j,tem_res,1,0,0,1,-tem_res,0,1);
  // }
  InitialOctomapBoarder();
  cuboidpoint.clear();
  for(int i=0;i<=31;i++)
  for(int j=0;j<=31;j++)
  {
    subspace_array[i][j]=new sub_space::Subspace;
    subspace_array[i][j]->state = sub_space::UNEXPLORED;
	subspace_array[i][j]->center_anchor_x = (0-15*20-10+i*20)*0.25;
	subspace_array[i][j]->center_anchor_y = (0-15*20-10+j*20)*0.25;
	//中心数据位置
  }//初始化
  pcl::PointXYZ tp;
  tp.x=-0.125;tp.y=-0.125;tp.z=0.125;
  cuboidpoint.push_back(tp);//1
  tp.x=-0.125;tp.y=0.125;tp.z=0.125;
  cuboidpoint.push_back(tp);//2
  tp.x=0.125;tp.y=0.125;tp.z=0.125;
  cuboidpoint.push_back(tp);//3
  tp.x=0.125;tp.y=-0.125;tp.z=0.125;
  cuboidpoint.push_back(tp);//4
  tp.x=-0.125;tp.y=-0.125;tp.z=-0.125;
  cuboidpoint.push_back(tp);//5
  tp.x=-0.125;tp.y=0.125;tp.z=-0.125;
  cuboidpoint.push_back(tp);//6
  tp.x=0.125;tp.y=0.125;tp.z=-0.125;
  cuboidpoint.push_back(tp);//7
  tp.x=0.125;tp.y=-0.125;tp.z=-0.125;
  cuboidpoint.push_back(tp);//8

  for(int xxx = -2;xxx<=2;xxx++)
  for(int yyy = -2;yyy<=2;yyy++)
  for(int zzz = -2;zzz<=2;zzz++)
  {
    octomap::point3d tp(xxx,yyy,zzz);
    checklist2.push_back(tp);
  }

  for(int xxx = -1;xxx<=1;xxx++)
  for(int yyy = -1;yyy<=1;yyy++)
  for(int zzz = -1;zzz<=1;zzz++)
  {
    if(xxx==0&&yyy==0&&zzz==0) continue;
    octomap::point3d tp(float(xxx)*0.25,float(yyy)*0.25,float(zzz)*0.25);
    checklist1.push_back(tp);
  }

  for(int xxx = -1;xxx<=1;xxx++)
  for(int yyy = -1;yyy<=1;yyy++)
  for(int zzz = -2;zzz<=2;zzz++)
  {
    octomap::point3d tp(float(xxx)*0.25,float(yyy)*0.25,float(zzz)*0.25);
    checklist1_1_2.push_back(tp);
  }

  for(int xxx = -3;xxx<=3;xxx++)
  for(int yyy = -3;yyy<=3;yyy++)
  {
    std::pair<int,int> tp;
    tp.first=xxx;
    tp.second = yyy;
    checklist3.push_back(tp);
  }
  center2surround5.clear();
  for(int i=-5;i<=5;i++)
  for(int j=-5;j<=5;j++)
  center2surround5.push_back(pair<int,int>(i,j));
  std::sort(center2surround5.begin(), center2surround5.end(), compare_center);
  // ShowSubspace();
  InitailMapDateDebug();
}

bool Teclas::compare_center(const std::pair<int, int>& a, const std::pair<int, int>& b) 
{
    int sumA = std::abs(a.first) + std::abs(a.second);
    int sumB = std::abs(b.first) + std::abs(b.second);
    return std::sqrt(sumA) < std::sqrt(sumB);
}

bool Teclas::SaveMap2txt(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res)
{
  // std::lock_guard<std::mutex> lock(oct_mutex);
  float pa=0;  float pb=0;  float pc=0;
  float p1a=0;  float p1b=0;  float p1c=0;
  float ttt;
  float cr1=0;  float cr2=0;
  std::ofstream file("/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/mapdata.txt");
  for(f_map::FrontierOcTree::leaf_iterator it = m_octree->begin_leafs(m_octree->getTreeDepth()), end = m_octree->end_leafs();it != end; ++it)
  {
    if(it->get_state()<0)
    {
            octomap::point3d tp =  it.getCoordinate();
            file << std::fixed << std::setprecision(3) << tp.x();
            file << " ";
            file << std::fixed << std::setprecision(3) << tp.y();
            file << " ";
            file << std::fixed << std::setprecision(3) << tp.z();
            file << " ";
            file << std::fixed << std::setprecision(3) << -1;
            file << " ";
            file << std::fixed << std::setprecision(3) << -1;
            file << " ";
            file << std::fixed << std::setprecision(3) << -1;
            file << " ";
            file << std::fixed << std::setprecision(3) << -1;
            file << "\n";
            continue;
    } 
    it->get_plane(pa,pb,pc,ttt);
    octomap::point3d pppp =  it.getCoordinate();    
    p1a = pppp.x();
    p1b = pppp.y();
    p1c = pppp.z();
    if (file.is_open()) 
    {
            file << std::fixed << std::setprecision(3) << pa;
            file << " ";
            file << std::fixed << std::setprecision(3) << pb;
            file << " ";
            file << std::fixed << std::setprecision(3) << pc;
            file << " ";
            file << std::fixed << std::setprecision(3) << ttt;
            file << " ";
            file << std::fixed << std::setprecision(3) << p1a;
            file << " ";
            file << std::fixed << std::setprecision(3) << p1b;
            file << " ";
            file << std::fixed << std::setprecision(3) << p1c;
            file << "\n";
    }
  }
        file.close();
  return true;
}
void Teclas::InitialOctomapBoarder()
{
  for(double i=-10.0;i<=10.0;i=i+0.25)
  {//初始化的机器人位置必须要设定为安全已知
    for(double z = -1.0;z<=1.0;z=z+0.25)
    {
      m_octree->updateNode(i,10.0,z,true);
      m_octree->set_node_info(i,10.0,z,-1,0,0,1,1,0,1);

      m_octree->updateNode(i,-10.0,z,true);
      m_octree->set_node_info(i,-10.0,z,-1,0,0,1,1,0,1);

      m_octree->updateNode(10.0,i,z,true);
      m_octree->set_node_info(10.0,i,z,-1,0,0,1,1,0,1);

      m_octree->updateNode(-10.0,i,z,true);
      m_octree->set_node_info(-10.0,i,z,-1,0,0,1,1,0,1);
    }
  }
}

//将子空间信息展示
bool Teclas::ShowSubspaceInfor(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res)
{
  visualization_msgs::MarkerArray pathNodes;//节点数据结构，用于可视化
  visualization_msgs::Marker bbox_marker;
  pathNodes.markers.clear();
  bbox_marker.header.stamp = ros::Time::now();
  bbox_marker.header.frame_id = "map";
  bbox_marker.ns = "subs_text";
  bbox_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  bbox_marker.color.r = 0.0f;
  bbox_marker.color.g = 1.0f;
  bbox_marker.color.b = 0.0f;
  bbox_marker.color.a = 1;
  bbox_marker.scale.x = 1.0;
  bbox_marker.scale.y = 1.0;
  bbox_marker.scale.z = 1.0;
  int ct =0 ;
  for(int i=0;i<=31;i++)
  for(int j=0;j<=31;j++)
  {
    bbox_marker.id = ct;
    bbox_marker.pose.position.x=(i-16)*20*0.25+10*0.25;
    bbox_marker.pose.position.y=(j-16)*20*0.25+10*0.25;
    bbox_marker.pose.position.z=2.0;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.lifetime = ros::Duration();
    if(subspace_array[i][j]->state==sub_space::EXPLORING)
    bbox_marker.text = std::to_string(subspace_array[i][j]->frontiers.size());
    else if(subspace_array[i][j]->state==sub_space::EXPLORED)
    bbox_marker.text = "FINISH";
    else if(subspace_array[i][j]->state==sub_space::UNEXPLORED)
    bbox_marker.text = "UN";
    pathNodes.markers.push_back(bbox_marker);
    ct++;
  }
  PublishFrontierMarker();
  showSUBS.publish(pathNodes);
  return true;
}

//这里被修改了!!
void Teclas::PublishPlane()
{
  PlaneMarker.markers.clear();
  visualization_msgs::Marker marker;    //Ｍａｒｋｅｒ是一个结构体模板
  //设置frame ID 和 timstamp ，具体信息可以参考TF 教程
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  //为marker设置命名空间和ＩＤ，创建不同的ＩＤ
  //任何带有相同的命名空间和ＩＤ被发送，都会覆盖先前的那个marker
  marker.ns = "array_plane";
  //设置marker的类型。初始化是CUBE,　接着循环SPHERE,ARROW,CYLINDER
  // marker.type = visualization_msgs::Marker::ARROW;
  marker.type = visualization_msgs::Marker::CUBE;
  //设置marker的操作：ADD, DELETE, and new in ROS indigo:3(DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  //设置marker的位姿。这是6个自由度，相对于header中特定的坐标系和时间戳
  //设定marker的大小----- 1x1x1 表示边长为１m
  // marker.color.r = 0.0f;
  // marker.color.g = 1.0f;
  // marker.color.b = 0.0f;

  marker.color.r = 0.8f;
  marker.color.g = 0.8f;
  marker.color.b = 0.8f;

  marker.color.a = 0.6;
  marker.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  float pa=0;  float pb=0;  float pc=0;
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
  int free_cct=0;
  std::ofstream file_effi("/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/efficiency.txt",std::ios::app);
  for(f_map::FrontierOcTree::leaf_iterator it = m_octree->begin_leafs(m_octree->getTreeDepth()), end = m_octree->end_leafs();it != end; ++it)
  {
    attc++;free_cct++;
    // if(it->get_state()<0)continue; 
    it->get_plane(pa,pb,pc,ttt);
    // pa = 1;    pb = 1;    pc = 1;
    octomap::point3d pppp =  it.getCoordinate();    
  // marker.color.r = 0.0f;
  // marker.color.g = 1.0f;
  // marker.color.b = 0.0f;

  marker.color.r = 0.7f;
  marker.color.g = 0.7f;
  marker.color.b = 0.7f;
  marker.color.a = 0.7;

    marker.id = attc;
    marker.pose.position.x = pppp.x();
    marker.pose.position.y = pppp.y();
    marker.pose.position.z = pppp.z();
    map_min_x = std::min(map_min_x,pppp.x());    
    map_min_y = std::min(map_min_y,pppp.y());
    map_max_x = std::max(map_max_x,pppp.x());    
    map_max_y = std::max(map_max_y,pppp.y());
    // marker.pose.position.z = pa*pppp.x()+pb*pppp.y()+pc*pppp.z()+ttt;
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
    // marker.pose.orientation.x = q4.x;
    // marker.pose.orientation.y = q4.y;
    // marker.pose.orientation.z = q4.z;
    // marker.pose.orientation.w = q4.w;
    cr1 = 1-it->get_per();
    cr2 = (it->get_avd())/0.25;
    if(it->get_state()<0)
    {
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.color.a = 1;
      free_cct--;
    }
    else if(it->get_state()==0) 
    {

      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 1;
    }
    PlaneMarker.markers.push_back(marker);
  }

    if (file_effi.is_open()) 
    {
      double real_time_n = ros::WallTime::now().toSec();
      file_effi << std::fixed << std::setprecision(3) << real_time_n;
      file_effi << " ";
      file_effi << std::fixed << std::setprecision(3) << free_cct;
      file_effi << "\n";
    }
    file_effi.close();

    showMap.publish(PlaneMarker);
}


void Teclas::PublishFrontierMarker()
{
  visualization_msgs::MarkerArray pathNodes;//节点数据结构，用于可视化
  visualization_msgs::Marker bbox_marker;
  pathNodes.markers.clear();
  bbox_marker.header.frame_id = "map";
  bbox_marker.frame_locked = true;
  bbox_marker.ns = "all_frontier";
  bbox_marker.color.r = 1.0f;
  bbox_marker.color.g = 0.0f;
  bbox_marker.color.b = 1.0f;
  bbox_marker.color.a = 1;
  bbox_marker.scale.x = 0.25;
  bbox_marker.scale.y = 0.25;
  bbox_marker.scale.z = 0.25;
  ftcounter=0;
  for(auto i = frontier.begin();i!=frontier.end();i++)
  {
    octomap::point3d otpp = m_octree->keyToCoord(**i);
    bbox_marker.id = ftcounter;
    bbox_marker.pose.position.x=otpp.x();
    bbox_marker.pose.position.y=otpp.y();
    bbox_marker.pose.position.z=2.0;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    // bbox_marker.action = visualization_msgs::Marker::ADD;
    pathNodes.markers.push_back(bbox_marker);
    ftcounter++;
  }
  showp.publish(pathNodes);
}

//传入当前局部规划区中的frontier
void Teclas::PublishLocalPlanningSpaceFrontierMarker(std::vector<octomap::point3d>* tempfv,int LPScentersubspace_idx,int LPScentersubspace_idy)
{
  visualization_msgs::MarkerArray pathNodes;//节点数据结构，用于可视化
  visualization_msgs::Marker bbox_marker;
  pathNodes.markers.clear();
  bbox_marker.header.frame_id = "map";
  bbox_marker.frame_locked = true;
  bbox_marker.ns = "partial_frontier";
  bbox_marker.color.r = 0.0f;
  bbox_marker.color.g = 0.0f;
  bbox_marker.color.b = 1.0f;
  bbox_marker.color.a = 1.0;
  bbox_marker.scale.x = 0.25*20*3;
  bbox_marker.scale.y = 0.25*20*3;
  bbox_marker.scale.z = -1.00;
  int cct =0;
  bbox_marker.id = cct;
  bbox_marker.pose.position.x=(LPScentersubspace_idx-16)*20*0.25+10*0.25;
  bbox_marker.pose.position.y=(LPScentersubspace_idy-16)*20*0.25+10*0.25;
  // std::cout<<"LPScentersubspace_idx "<<LPScentersubspace_idx<<"LPScentersubspace_idy "<<LPScentersubspace_idy<<std::endl;
  bbox_marker.pose.position.z=0.75;
  bbox_marker.header.stamp = ros::Time::now();
  bbox_marker.lifetime = ros::Duration();
  bbox_marker.type = visualization_msgs::Marker::CUBE;
  // pathNodes.markers.push_back(bbox_marker);  蓝色的大方块不显示!!!
  cct++;  

  bbox_marker.color.r = 0.0f;
  bbox_marker.color.g = 0.0f;
  bbox_marker.color.b = 1.0f;
  bbox_marker.color.a = 1.0;
  bbox_marker.scale.x = 0.25;
  bbox_marker.scale.y = 0.25;
  bbox_marker.scale.z = 0.25;
  // std::cout<<"The size of input is "<<tempfv->size()<<std::endl;
  for(std::vector<octomap::point3d>::iterator ite = tempfv->begin();ite!=tempfv->end();ite++)
  {
    bbox_marker.id = cct;
    bbox_marker.pose.position.x=double(ite->x());
    bbox_marker.pose.position.y=double(ite->y());
    bbox_marker.pose.position.z=2.0;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.lifetime = ros::Duration();
    // bbox_marker.type = visualization_msgs::Marker::CUBE;
    // bbox_marker.action = visualization_msgs::Marker::ADD;
    pathNodes.markers.push_back(bbox_marker);
    cct++;
  }
  clear_marker_counter  = std::max(cct,clear_marker_counter);
  bbox_marker.color.g = 0.0f;
  bbox_marker.color.a = 0.0;
  for(cct=cct;cct<clear_marker_counter;cct++)
  {
    bbox_marker.id = cct;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.lifetime = ros::Duration();
    // bbox_marker.type = visualization_msgs::Marker::CUBE;
    // bbox_marker.action = visualization_msgs::Marker::ADD;
    pathNodes.markers.push_back(bbox_marker);
  }
  // while(1)
  showLPS.publish(pathNodes);
}


void Teclas::PublishLPS_VP(std::vector<octomap::point3d>* tempVP)
{
  visualization_msgs::MarkerArray pathNodes;//节点数据结构，用于可视化
  visualization_msgs::Marker bbox_marker;
  pathNodes.markers.clear();
  bbox_marker.header.frame_id = "map";
  bbox_marker.frame_locked = true;
  bbox_marker.ns = "LPS_viewpoint";
  int cct =0;
  bbox_marker.color.r = 1.0f;
  bbox_marker.color.g = 1.0f;
  bbox_marker.color.b = 0.0f;
  bbox_marker.color.a = 1.0;
  bbox_marker.scale.x = 0.25;
  bbox_marker.scale.y = 0.25;
  bbox_marker.scale.z = 0.25;
  for(std::vector<octomap::point3d>::iterator ite = tempVP->begin();ite!=tempVP->end();ite++)
  {
    bbox_marker.id = cct;
    bbox_marker.pose.position.x=double(ite->x());
    bbox_marker.pose.position.y=double(ite->y());
    bbox_marker.pose.position.z=double(ite->z())+0.75;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    pathNodes.markers.push_back(bbox_marker);
    cct++;
  }
  clear_marker_counter1  = std::max(cct,clear_marker_counter1);
  bbox_marker.color.g = 0.0f;
  bbox_marker.color.a = 0.0;
  for(cct=cct;cct<clear_marker_counter1;cct++)
  {
    bbox_marker.id = cct;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    pathNodes.markers.push_back(bbox_marker);
  }
  showLPS_VP.publish(pathNodes);
}

void Teclas::GetPlaneNeib(float vcx,float vcy,float vcz,float vpa,float vpb,float vpc,float vpd,std::vector<pcl::PointXYZ>*neicenter)
{
  int flag_plane[6]={0,0,0,0,0,0};
  int flag_point[8]={0,0,0,0,0,0,0,0};
  for(int i=0;i<=7;i++)
  {
    float dd=vpa*(vcx+cuboidpoint[i].x)+vpb*(vcy+cuboidpoint[i].y)+vpc*(vcz+cuboidpoint[i].z)+vpd;
    if(dd>0)flag_point[i]=1;//依次检测正方体的八个点在平面的上方还是下方。
  }
  flag_plane[0]=flag_point[0]+flag_point[1]+flag_point[2]+flag_point[3];//上
  flag_plane[1]=flag_point[4]+flag_point[5]+flag_point[6]+flag_point[7];//下
  flag_plane[2]=flag_point[0]+flag_point[1]+flag_point[4]+flag_point[5];//左
  flag_plane[3]=flag_point[2]+flag_point[3]+flag_point[6]+flag_point[7];//右
  flag_plane[4]=flag_point[1]+flag_point[2]+flag_point[5]+flag_point[6];//前
  flag_plane[5]=flag_point[0]+flag_point[3]+flag_point[4]+flag_point[7];//后
  //只有四个点全部在上方，或者全部在下方，才说明拟合的平面没有穿过正方体的一个面
  pcl::PointXYZ tp;
  //==0说明全部在下方，==4说明全部在上方
  if((flag_plane[0]!=0)&&(flag_plane[0]!=4))
  {
    tp.x=vcx;    tp.y=vcy;    tp.z=vcz+resolu;
    neicenter->push_back(tp);    
  }//上

  if((flag_plane[1]!=0)&&(flag_plane[1]!=4))
  {
    tp.x=vcx;    tp.y=vcy;    tp.z=vcz-resolu;
    neicenter->push_back(tp);    
  }//下

  if((flag_plane[2]!=0)&&(flag_plane[2]!=4))
  {
    tp.x=vcx-resolu;    tp.y=vcy;    tp.z=vcz;
    neicenter->push_back(tp);    
  }//左

  if((flag_plane[3]!=0)&&(flag_plane[3]!=4))
  {
    tp.x=vcx+resolu;    tp.y=vcy;    tp.z=vcz;
    neicenter->push_back(tp);    
  }//右

  if((flag_plane[4]!=0)&&(flag_plane[4]!=4))
  {
    tp.x=vcx;    tp.y=vcy+resolu;    tp.z=vcz;
    neicenter->push_back(tp);    
  }//前

  if((flag_plane[5]!=0)&&(flag_plane[5]!=4))
  {
    tp.x=vcx;    tp.y=vcy-resolu;    tp.z=vcz;
    neicenter->push_back(tp);    
  }//后
}


void Teclas::GetPlaneNeib(float vcx,float vcy,float vcz,std::vector<pcl::PointXYZ>*neicenter)
{
    pcl::PointXYZ tp;
    for(int ge_x=-1;ge_x<=1;ge_x=ge_x+2)
    {
     tp.x=vcx+resolu*float(ge_x);    
     tp.y=vcy;    
     tp.z=vcz;
     neicenter->push_back(tp);    
    }
    for(int ge_y=-1;ge_y<=1;ge_y=ge_y+2)
    {
     tp.x=vcx;    
     tp.y=vcy+resolu*float(ge_y);    
     tp.z=vcz;
     neicenter->push_back(tp);    
    }
    for(int ge_z=-1;ge_z<=1;ge_z=ge_z+2)
    {
     tp.x=vcx;
     tp.y=vcy;    
     tp.z=vcz+resolu*float(ge_z);  
     neicenter->push_back(tp);    
    }
}



// 正常一帧下，时间开销为
// 深度转换时间0.011
// 滤波时间0.026
// oct索引时间0.007
// 平面拟合时间0
// Frontier动态更新时间0
// 总时间0.045
// 深度信息是50帧，一秒拟合两次，每次用2帧

void Teclas::insertCloudCallback(const sensor_msgs::ImageConstPtr & msg)
{

  if(cameracct%12!=0){cameracct++;return;}
  image_in_process = true;
  cameracct = 1;
  temc++;
  double t0=ros::Time::now().toSec();
  //std::cout<<"IO"<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclimg(new(pcl::PointCloud<pcl::PointXYZ>));
  double st_time = ros::Time().now().toSec();
  int u =0; int v=0;
  int wi = int(msg->width);
  int he = int(msg->height);
  std::cout<<"wi"<<wi<<"he"<<he<<std::endl;
  int d = wi*he;
  float z_c = 0;

                                                  // gradient_image_pub.publish(msg_gradient);
                                                        // double time_gray_start =  ros::Time::now().toSec();
                                                        // // 创建灰度图像
                                                        // cv::Mat depth_image_test(1280, 720, CV_8UC1);
                                                        // // 生成随机像素值
                                                        // cv::randu(depth_image_test, cv::Scalar(0), cv::Scalar(255));
                                                        // // Canny 边缘检测参数
                                                        // double threshold1 = 50.0;
                                                        // double threshold2 = 150.0;
                                                        // int apertureSize = 3;
                                                        // // 应用 Canny 边缘检测
                                                        // cv::Mat edges;
                                                        // cv::Canny(depth_image_test, edges, threshold1, threshold2, apertureSize);
                                                        // double time_gray_end =  ros::Time::now().toSec();
                                                        // std::cout<<"time_gray_end - time_gray_start = "<<time_gray_end - time_gray_start<<std::endl;

  pcl::PointXYZ thisp;
  Eigen::Matrix4f Twi;  Eigen::Matrix4f Tic;
  Eigen::Matrix4f sensorToWorld;//这里拿到的就是map->cloud-head变换 

  gazebo_msgs::GetModelState rq;
  rq.request.model_name="scout/";
  rq.request.relative_entity_name="map";
  while(!states_client.call(rq))
  std::cout<<"Waiting for gzbo";
  float posx=rq.response.pose.position.x;
  float posy=rq.response.pose.position.y;
  float posz=rq.response.pose.position.z;
  std::cout<<"pos "<<posx<<" "<<posy<<" "<<posz<<std::endl;
                        // geometry_msgs::PoseStamped rbp;
                        // rbp.header.frame_id = "map";
                        // rbp.pose.position = rq.response.pose.position;
                        // rbp.pose.orientation = rq.response.pose.orientation;
                        // robpos.publish(rbp);

  // float yaw_h = tf::getYaw(rq.response.pose.orientation);//这里是车的yaw角
  tf::Quaternion qw1={rq.response.pose.orientation.x,rq.response.pose.orientation.y,rq.response.pose.orientation.z,rq.response.pose.orientation.w};
  tf::Matrix3x3 matrixw1;
  matrixw1.setRotation(qw1);
  double xita =0.6 ;
  Twi<<matrixw1[0][0],matrixw1[0][1],matrixw1[0][2],posx,
       matrixw1[1][0],matrixw1[1][1],matrixw1[1][2],posy,
       matrixw1[2][0],matrixw1[2][1],matrixw1[2][2],posz,
              0,0,0,1;
  // <origin rpy="0 0.6 0" xyz="0.25 -0.025 0.5"/>              
  // <origin rpy="0 0.4 0" xyz="0.3 -0.025 0.15"/>
  Tic<<0,-sin(xita),cos(xita),0.25,
      -1,         0,        0,  0,
       0,-cos(xita),-sin(xita),0.5,
       0,         0,         0,  1;
  double t1=ros::Time::now().toSec();
        //std::cout<<"0000000000000000000000"<<std::endl;
  int wi_msg = msg->width;
  std::array<int,8> nei={0-wi_msg-1,0-wi_msg,0-wi_msg+1,0-1,0+1,0+wi_msg-1,0+wi_msg,0+wi_msg+1};
  ushort tmp;int idx_c=0;int ind_s=0;
  std::vector<std::pair<int,int>> obs_pair_list;
  float z_s=0;
  int down_sample_ratio = 4;
  cv::Mat depth_image_test(1280/down_sample_ratio, 720/down_sample_ratio, CV_8U);
  for(v=0;v<he;v++)
  for(u=0;u<wi;u++)
  {
    // if(v%10==0&&u%10==0)
    idx_c = u+v*msg->width;
    float tmp = float(Littlesw(msg->data[idx_c * 2], msg->data[idx_c * 2 + 1]));
    z_c = tmp/1000.0;    
    if((z_c>0.3)&&(z_c<=camera_depth)&&(v>50))
    {
      thisp.x =  z_c*(K_inv[0]*u+K_inv[1]*v+K_inv[2]*1);   
      thisp.y =  z_c*(K_inv[3]*u+K_inv[4]*v+K_inv[5]*1);   
      thisp.z =  z_c;   
      pclimg->points.push_back(thisp);
    }
    if(z_c>camera_depth)z_c = camera_depth;
    if(v%down_sample_ratio==0&&u%down_sample_ratio==0)
    depth_image_test.at<uchar>(u/down_sample_ratio, v/down_sample_ratio) = 255 - int((z_c-0.3)/(camera_depth-0.3)*255);//因为边界点会落在灰度从大到小的一侧，所以希望距离近，深度大。
  }
                  double time_gray_start =  ros::Time::now().toSec();
//                  std::cout<<"深度图像转换时间 "<<time_gray_start - t1<<std::endl;
                  double threshold1 = 80.0;
                  double threshold2 = 180.0;
                  int apertureSize = 3;
                  // 应用 Canny 边缘检测
                  cv::Mat edges;
                  cv::Canny(depth_image_test, edges, threshold1, threshold2, apertureSize);
                                                        double time_gray_end =  ros::Time::now().toSec();
                                                        //std::cout<<"Canny边缘检测时间 "<<time_gray_end - time_gray_start<<std::endl;


                                                  sensor_msgs::Image msg_gradient;  
                                                  msg_gradient.width = 1280/down_sample_ratio;
                                                  msg_gradient.height = 720/down_sample_ratio;
                                                  msg_gradient.encoding = "mono8"; // 单通道灰度图像
                                                  msg_gradient.step = 1280/down_sample_ratio;
                                                  msg_gradient.header.frame_id = "map";
                                                  msg_gradient.data.clear();
                                                  msg_gradient.data.resize(720*1280/down_sample_ratio/down_sample_ratio);
                                                  int pixel_index = 0;
                                                  for(int i = 0; i < 720/down_sample_ratio; i++)
                                                  {
                                                      for(int j = 0; j < 1280/down_sample_ratio; j++)
                                                      {
                                                        pixel_index = i * 1280/down_sample_ratio + j;
                                                        msg_gradient.data[pixel_index] = 255;//白
                                                      }
                                                  }
  double time_cliff_start =  ros::Time::now().toSec();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cliff_points(new(pcl::PointCloud<pcl::PointXYZ>));
  pcl::PointXYZ point_cliff;

	// sor.setInputCloud(pclimg);
	// sor.setDownsampleAllData(1);
	// sor.setLeafSize(0.4, 0.4, 0.4);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
	// sor.filter(*mesh_points);//提速非常明显。

    cv::Mat indices;
    cv::findNonZero(edges, indices);
    int cct_gd = 0;
    int pat_size = 3;
    // std::array<int,8> nei_pat={0-1280/down_sample_ratio-pat_size,0-1280/down_sample_ratio,0-1280/down_sample_ratio+pat_size,
    //                        0-pat_size,0+pat_size,
    //                        0+1280/down_sample_ratio-pat_size,0+1280/down_sample_ratio,0+1280/down_sample_ratio+pat_size};
    // 打印边界的像素索引
    for (int i = 0; i < indices.rows; i++)
    {
        cv::Point pt = indices.at<cv::Point>(i);
        int idx = pt.y + pt.x * 1280/down_sample_ratio;
        msg_gradient.data[idx] = 0;
        //利用深度进行筛选，然后把边界的点保存起来
        u = pt.y;
        v = pt.x;
        depth_image_test.at<uchar>(u, v) = 0;
        u*=down_sample_ratio;
        v*=down_sample_ratio;
        idx_c = u + v * 1280;//原始尺寸图像的索引
        float tmp = float(Littlesw(msg->data[idx_c * 2], msg->data[idx_c * 2 + 1]));
        z_c = tmp/1000.0;    
        point_cliff.x =  z_c*(K_inv[0]*u+K_inv[1]*v+K_inv[2]*1);   
        point_cliff.y =  z_c*(K_inv[3]*u+K_inv[4]*v+K_inv[5]*1);   
        point_cliff.z =  z_c;   
        if(z_c<=2.0&&z_c>=0.5) 
        {        
          cct_gd++;
          cliff_points->push_back(point_cliff);
        }
    }
  // std::cout<<"the size of obs cct_gd is "<<cct_gd<<std::endl;
  // gradient_image_pub.publish(msg_gradient);
  double time_cliff_end =  ros::Time::now().toSec();
  //std::cout<<"危险点提取时间"<<time_cliff_end-time_cliff_start<<std::endl;

  double t2=ros::Time::now().toSec();


  pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(pclimg);
	sor.setDownsampleAllData(1);
	sor.setLeafSize(0.01, 0.01, 0.01);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
	sor.filter(*pclimg);//提速非常明显。
  pcl::transformPointCloud(*pclimg, *pclimg, Twi*Tic);//转到世界坐标系下


	sor.setInputCloud(cliff_points);
	sor.setDownsampleAllData(1);
	sor.setLeafSize(0.2, 0.2, 0.2);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
	sor.filter(*cliff_points);//提速非常明显。
  pcl::transformPointCloud(*cliff_points, *cliff_points, Twi*Tic);//转到世界坐标系下
  // std::cout<<"降采样后的 障碍物数量为 "<<cliff_points->points.size()<<std::endl;


  // Eigen::Matrix4f Twc = Twi*Tic;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_points(new(pcl::PointCloud<pcl::PointXYZ>));
	// sor.setInputCloud(pclimg);
	// sor.setDownsampleAllData(1);
	// sor.setLeafSize(0.4, 0.4, 0.4);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
	// sor.filter(*mesh_points);//提速非常明显。
/*
  //这里会将所有的点云都保存下来。
  allpoints+=*pclimg;
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(allpoints);
	sor.setInputCloud(cloudPtr);
	sor.setDownsampleAllData(1);
	sor.setLeafSize(0.05, 0.05, 0.05);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
	sor.filter(*cloudPtr);//提速非常明显。
  allpoints = *cloudPtr;
  sensor_msgs::PointCloud2 tpo;   
  pcl::toROSMsg(*cloudPtr, tpo); //convert from PCL to ROS type this way
  tpo.header.frame_id="map"; 
  tpo.header.stamp=ros::Time::now(); 
  std::cout<<"the size of allpoints is "<< allpoints.size();
  pubCloud.publish(tpo);//return;
  //这里会将所有的点云都保存下来。
*/

    //测试区
    // sensor_msgs::PointCloud2 ros_cloud; //here is the ROS-compatible pointCloud message
    // pcl::toROSMsg(*mesh_points, ros_cloud); //convert from PCL to ROS type this way
    // ros_cloud.header.frame_id="map";
    // ros_cloud.header.stamp=ros::Time::now();
    // pubCloud.publish(ros_cloud);//return;
    //测试区
                              
  double t3=ros::Time::now().toSec();
  //std::cout<<"滤波时间"<<t3-t2<<std::endl;
  float t_wx,t_wy,t_wz;
  for(int i=0;i<cliff_points->points.size();i++)
  {
      t_wx = cliff_points->points[i].x;
      t_wy = cliff_points->points[i].y;
      t_wz = cliff_points->points[i].z;
      // octomap::OcTreeKey* thiscliff(new octomap::OcTreeKey);
      // *thiscliff = m_octree->coordToKey(t_wx,t_wy,t_wz);//找到key          
      // f_map::FrontierOcTreeNode * thisn=m_octree->search(*thiscliff);//先在octomap中找到这个节点。

      octomap::OcTreeKey thiscliff = m_octree->coordToKey(t_wx,t_wy,t_wz);//找到key          
      f_map::FrontierOcTreeNode * thisn=m_octree->search(thiscliff);//先在octomap中找到这个节点。

      if(thisn)//如果已知，不允许是frontier
      {
        if(thisn->get_state()==f_map::FRONT)
        {
          thisn->set_state(f_map::CLIFF);
          std::set<octomap::OcTreeKey *, CompareKey>::iterator invalid_frontier = frontier.find(&thiscliff);
          std::pair<int,int> thpp = sub_space::Frontier2Subspaceindex(t_wx,t_wy,t_wz);
          // std::cout<<"before size is "<<subspace_array[thpp.first][thpp.second]->frontiers.size()<<std::endl;
          subspace_array[thpp.first][thpp.second]->deleteFrontier(&thiscliff);
          // std::cout<<"after size is "<<subspace_array[thpp.first][thpp.second]->frontiers.size()<<std::endl;
          //这里删除，虽然传入的是指针，但是由于我们认为定义了判定准则，只要key的三个数值都一样，那么就会被删除。
          update_subspace_set.insert(thpp.first*sub_num+thpp.second);//记录当前的submap发生了更新
          delete (*invalid_frontier);//把空间释放
          frontier.erase(invalid_frontier);    
        }
      }
      else//如果未知，就先进行更新，后续再处理。
      {
        m_octree->updateNode(t_wx,t_wy,t_wz,true); //这里把未知的状态标记为已知,在后续frontier搜索中非常关键。
        m_octree->set_node_info(t_wx,t_wy,t_wz,f_map::CLIFF,0,0,0,0,0,-1);//标记为cliff
      }
  }

  double tobs=ros::Time::now().toSec();
  //std::cout<<"障碍物处理时间"<<tobs-t3<<std::endl;

  // std::vector<octomap::OcTreeKey*> tvk;
  std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr,CompareKey> thismap;//这一帧中，地图新探索到的octonode
  //所有点进行分类
          // std::cout<<"111111111111111111"<<std::endl;
  for(int i =0;i<pclimg->size();i++)
  {
    octomap::OcTreeKey* thisk(new octomap::OcTreeKey);//分配了一块内存空间给指针，指针指向该内存空间
    *thisk=m_octree->coordToKey(pclimg->points[i].x,pclimg->points[i].y,pclimg->points[i].z);
    f_map::FrontierOcTreeNode * thisn=m_octree->search(*thisk);//先在octomap中找到这个节点。
    if(thisn) //如果全局地图中的octonode已经被探索，跳过这个点。
    {
      // i=i+1;
      if((thisn->get_per()>0))//对于所有的cliff,如果第一次扫描到，我们会将get_per设定为-1.如果这里是正数，只有可能是frontier,obs,tra
      {                       //但是如果是负数，就说明是cliff,需要对平面进行拟合。
        delete thisk;
        continue;
      }
    }
  //////////能运行这一段，说明地图中有新的octonode被探索到了////////////////////////////////    
    std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator thisit = thismap.find(thisk);
    if(thisit!=thismap.end())
    {
      thisit->second->points.push_back(pclimg->points[i]);//在当前帧已经标记
    }
    else
    { //在当前帧中还未标记
      // tvk.push_back(thisk);
      pcl::PointCloud<PointXYZ>::Ptr pvector(new pcl::PointCloud<PointXYZ>);
      pvector->points.push_back(pclimg->points[i]);
      thismap[thisk]=pvector;      
    }
  //////////能运行这一段，说明地图中有新的octonode被探索到了////////////////////////////////    
  }
  pclimg.reset(new pcl::PointCloud<PointXYZ>);//释放内存
  double t4=ros::Time::now().toSec();
  //std::cout<<"oct索引时间"<<t4-t3<<std::endl;
  pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);//对点集进行平面拟合操作
  float abs_dis=0;
  float per_inline=0;
  int tra_flag = 0;
//关于指针https://blog.csdn.net/qq_36403227/article/details/96595707
//////////这一段进行平面拟合工作////////////////////////////////    
//如果是CLIFF并且还没有被观测到，那么就进行平面拟合，但是保留CLIFF状态
  for(std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator it = thismap.begin();it!=thismap.end();it++)
  {
    if(it->second->points.size()<250)
    { //无效的平面标记一下
      it->second->points.clear();
      continue;
    }
    tra_flag = fit_plane_to_cloud(plane_model,it->second,0.06,per_inline,abs_dis);
    // std::cout<<"2"<<std::endl;
    m_octree->updateNode(*(it->first),true); //这里把未知的状态标记为已知,在后续frontier搜索中非常关键。
    // std::cout<<"1"<<std::endl;
    f_map::FrontierOcTreeNode* thisn_temp = m_octree->search(*(it->first));//这里不可能是null,因为前面已经更新为true了。
    int now_state;
    if(thisn_temp->get_state()==f_map::CLIFF)
    {
      now_state = f_map::CLIFF;
    }
    else if(tra_flag<0)//角度很大的平面，内点太少，认为是不可穿行区域
    {
      now_state = f_map::UNTRA;
    }
    else if(tra_flag>0)//这里先把frontier和可穿行区域不做区分。0是frontier,1是traversable
    {
      now_state = f_map::TRAVE;
    }
    m_octree->set_node_info(*(it->first),now_state,plane_model->values[0],plane_model->values[1],plane_model->values[2],plane_model->values[3],abs_dis,per_inline);    
  }
  double t5=ros::Time::now().toSec();
  //std::cout<<"平面拟合时间"<<t5-t4<<std::endl;
  //////////这一段进行平面拟合工作，标记了状态是可穿行，不可穿行。////////////////////////////////    


  float fac_a=0;  float fac_b=0;  float fac_c=0;  float fac_d=0;
  float center_x=0;  float center_y=0;  float center_z=0;//小平面中心点
  float extend_x=0;  float extend_y=0;  float extend_z=0;//小平面延伸点
  //对所有新加入的node进行遍历 ，把指针分配到chenode_set和newfnode_set中，没有被分配的指针释放了。
  std::set<octomap::OcTreeKey*,CompareKey> newfnode_set;//存放新加入所有节点
  std::vector<octomap::OcTreeKey*> newFrontier_Set;  
  std::set<octomap::OcTreeKey*,CompareKey> chenode_set;//存放活跃区已经是frontier的节点，是当前帧手动分配的内存，必须要手动释放！！！
        // std::cout<<"22222222222222222"<<std::endl;
  for(std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator it = thismap.begin();it!=thismap.end();it++)
  {
    //上面因为没有到达120个点所以释放了。
    if(it->second->points.size()==0)
    {
      delete (it->first);  //注意这里会产生野指针！！！！但是没有别的办法！！！
      continue;
    }
    f_map::FrontierOcTreeNode *tnode = m_octree->search(*(it->first));//这里node一定不是nullptr
    // if(tnode) std::cout<<"OK"<<std::endl;
    tnode->get_plane(fac_a,fac_b,fac_c,fac_d);//拿到新node平面参数
    std::vector<pcl::PointXYZ> neibvox;
    octomap::point3d oc_cent = m_octree->keyToCoord(*(it->first));//新node的中心
    int im_fronter=0;//判定当前的it是否为新的frontier
    if(tnode->get_state()==1)
    { //新node可穿行,那么就可能是Frontier,因为当前帧的TRA FRON不区分
                      //1、拿到周围的所有点，并且将frotier放入check_list。
                      //2、如果周围所有点中出现了一个未知点，自己加入new_list
                      GetPlaneNeib(oc_cent.x(),oc_cent.y(),oc_cent.z(),fac_a,fac_b,fac_c,fac_d,&neibvox);      
                      //新node平面扩展得到所有邻居
                      for(std::vector<pcl::PointXYZ>::iterator itnei=neibvox.begin();itnei!=neibvox.end();itnei++)
                      {//对邻居进行索引
                        f_map::FrontierOcTreeNode *temnode;//邻居node
                        octomap::OcTreeKey* thisnei(new octomap::OcTreeKey);
                        *thisnei = m_octree->coordToKey(itnei->x,itnei->y,itnei->z);//邻居key          
                        // std::cout<<" "<<itnei->x<<" "<<itnei->y<<" "<<itnei->z<<std::endl;
                        temnode = m_octree->search(*thisnei);      //邻居node,可能不存在    
                        //1、拿到周围的所有点，并且将frotier放入check_list。
                        if(temnode)
                        {//如果邻居node已知
                          if(temnode->get_state()==0)//邻居node是frontier,这里只有可能是当前深度帧之前的voxel,因为当前帧的frontier和TRA不做区分的标记为1。
                          chenode_set.insert(thisnei);            
                          else 
                          {
                          delete thisnei;//邻居node不是frontier，则回收空间          
                          thisnei=NULL;
                          }
                        }
                        else if((im_fronter==0)&&((tnode->get_state()!=f_map::FRONT)))
                        { //如果邻居node是一个未知点，自己还不是frontier,则自己加入new_list
                          newFrontier_Set.push_back(it->first);
                          im_fronter=1;
                          delete thisnei;//邻居node不是frontier，则回收空间
                          thisnei = NULL;
                        }
                        else
                        {
                          delete thisnei;//邻居node不是frontier，则回收空间      
                          thisnei = NULL;
                        } 
                      }      
    }
    else
    {//UNTRA或者是CLIFF
                      //1、拿到当前node周围的6个点，并且将frotier放入check_list。
                      GetPlaneNeib(oc_cent.x(),oc_cent.y(),oc_cent.z(),&neibvox);      
                      //拿到周围的6个点邻居
                      for(std::vector<pcl::PointXYZ>::iterator itnei=neibvox.begin();itnei!=neibvox.end();itnei++)
                      {
                        f_map::FrontierOcTreeNode *temnode;//邻居node
                        octomap::OcTreeKey* thisnei(new octomap::OcTreeKey);
                        *thisnei = m_octree->coordToKey(itnei->x,itnei->y,itnei->z);//邻居key          
                        temnode = m_octree->search(*thisnei);//邻居node          
                        //1、拿到周围的所有点，并且将frotier放入check_list。
                        if(temnode)//邻居node已知
                        {
                        if(temnode->get_state()==0)//周围的点是frontier
                          {
                            chenode_set.insert(thisnei);            
                          }
                          else 
                          {
                          delete thisnei;//这个点不是frontier，则回收空间
                          thisnei = NULL;
                          }
                        }
                        else 
                        {
                          delete thisnei;//这个点未知，则回收空间
                          thisnei = NULL;
                        }
                      }
    }
    //如果可穿行，但是不是frontier,或者不可穿行，那么就释放内存空间。
    if(im_fronter==0)//如果TRAV 或者 CLIFF 或者 UNTRA那么就释放内存空间
    delete (it->first);
  }
  //thismap中的数据分为三类：
  //1、检测为Fron，那么存储到newFrontier_Set中，指针不被释放。——im_fronter=1
  //2、为TRA，UNTRA，CLIFF 指针被释放。——im_fronter=0
  //3、对于每一个node的周围，如果有FRON,则周围的Fron存储到chenode_set中。内存不释放
  
        // std::cout<<"333333333333333333333333"<<std::endl;
  thismap.clear();//这里之所以没有对所有的key指针进行释放，是因为这里面的部分已经在newFrontier_Set中了
  //从thismap中抽解出chenode_set和newFrontier_Set
  //terrain estimation
  //对之前是frontier，并且需要确认的frontier进行确认。



  for(std::set<octomap::OcTreeKey*>::iterator it=chenode_set.begin();it!=chenode_set.end();it++)
  {
    f_map::FrontierOcTreeNode* tnode = m_octree->search(**it);//这里node一定不是nullptr
    tnode->get_plane(fac_a,fac_b,fac_c,fac_d);//拿到需要检测node平面参数
    octomap::point3d oc_cent = m_octree->keyToCoord(**it); //需要检测node的中心
    std::vector<pcl::PointXYZ> neibvox;
    GetPlaneNeib(oc_cent.x(),oc_cent.y(),oc_cent.z(),fac_a,fac_b,fac_c,fac_d,&neibvox);      
    //需要检测node平面扩展得到所有邻居
    int flag_stillfrontier=0;
    for(std::vector<pcl::PointXYZ>::iterator itnei=neibvox.begin();itnei!=neibvox.end();itnei++)
    {//对邻居进行索引
      f_map::FrontierOcTreeNode *temnode;//邻居node
      octomap::OcTreeKey thisnei;
      thisnei = m_octree->coordToKey(itnei->x,itnei->y,itnei->z);//邻居key          
      temnode = m_octree->search(thisnei);      //邻居node,可能不存在    
      if(temnode)
      {//如果邻居node已知
        continue;
      }
      else
      { //如果邻居node未知,说明现在还是frontier!!
        flag_stillfrontier=1;
        break;
      } 
    }
    if(flag_stillfrontier==0)
    {
      //如果现在已经不是frontier了

    // octomap::point3d poi_bf = m_octree->keyToCoord(**it);
    // std::pair<int,int> thpp_bf = sub_space::Frontier2Subspaceindex(poi_bf.x(),poi_bf.y(),poi_bf.z());
    // std::cout<<"before size "<< subspace_array[thpp_bf.first][thpp_bf.second]->frontiers.size()<<std::endl;
    // for(auto itt_bf = subspace_array[thpp_bf.first][thpp_bf.second]->frontiers.begin();itt_bf!=subspace_array[thpp_bf.first][thpp_bf.second]->frontiers.end();itt_bf++)
    // {
    // std::cout<< (*itt_bf)->k[0]<< " "<< (*itt_bf)->k[1]<< " "<< (*itt_bf)->k[2]<< " " <<std::endl;        
    // }


          // std::cout<<(*it)->k[0]<<","<<(*it)->k[1]<<","<<(*it)->k[2]<<std::endl;

    std::set<octomap::OcTreeKey *, CompareKey>::iterator invalid_frontier = frontier.find(*it);
    // std::cout<<(*invalid_frontier)->k[0]<<","<<(*invalid_frontier)->k[1]<<","<<(*invalid_frontier)->k[2]<<std::endl;
    //虽然这里*invalid_frontier和*it两个指针内存空间中存储的内容相同，都是OcTreeKey，但是这是两篇不同的内存空间！！！
    //如果这里就把invalid_frontier释放，那么subspace->frontiers中就会出现野指针，因为frontier和subspace->frontiers是同一个指针，对应同一内存空间。这样再使用
    //subspace_array[thpp.first][thpp.second]->deleteFrontier(*it);就会报错，因为已经无法再通过Key索引指针了
    //12-31以前的所有版本，frontier中的内存空间应该都没有释放！

    // std::cout<<"after size "<< subspace_array[thpp_bf.first][thpp_bf.second]->frontiers.size()<<std::endl;
    tnode->set_state(f_map::TRAVE);
    octomap::point3d poi = m_octree->keyToCoord(**it);
    std::pair<int,int> thpp = sub_space::Frontier2Subspaceindex(poi.x(),poi.y(),poi.z());
    // std::cout<<"Before delete is -----------------------------------------------"<<subspace_array[thpp.first][thpp.second]->frontiers.size()<<std::endl;
    subspace_array[thpp.first][thpp.second]->deleteFrontier(*it);
    // std::cout<<"After delete is -----------------------------------------------"<<subspace_array[thpp.first][thpp.second]->frontiers.size()<<std::endl;
    //这里传入的是一个octomap::OcTreeKey *，那么内部在Set中进行删除的时候实际上删除的是一个迭代器，但是和指针本身指向的这片空间无关
    update_subspace_set.insert(thpp.first*sub_num+thpp.second);//记录当前的submap发生了更新
    delete (*invalid_frontier);//把原来的frontier空间释放  
    frontier.erase(invalid_frontier);//再把迭代器从set中删除,如果先删除迭代器，就无法释放空间了。
    }
    delete (*it);//这里也会产生野指针！！！
  }

  chenode_set.clear();
  f_map::FrontierOcTreeNode* tnode; 
  for(std::vector<octomap::OcTreeKey*>::iterator it=newFrontier_Set.begin();it!=newFrontier_Set.end();it++)
  {
    frontier.insert(*it);
    //TODO::ADD TO SUBSPACE
    octomap::point3d poi = m_octree->keyToCoord(**it);
    std::pair<int,int> thpp = sub_space::Frontier2Subspaceindex(poi.x(),poi.y(),poi.z());
    if(thpp.first<0||thpp.first>31||thpp.second<0||thpp.second>31)
    {std::cout<<"Subspace out!!"<<std::endl; ROS_ASSERT(0);}

    // std::cout<<"size"<<thpp.first<<" "<<thpp.second<<" "<<subspace_array[thpp.first][thpp.second]->frontiers.size()<<std::endl;
    subspace_array[thpp.first][thpp.second]->addFrontier(*it);
    // std::cout<<"size"<<subspace_array[thpp.first][thpp.second]->frontiers.size()<<std::endl;

    // octomap::OcTreeKey temp_test_key = m_octree->coordToKey(poi);这里的删除测试正常
    // std::pair<int,int> thpp_temp_test_delete = sub_space::Frontier2Subspaceindex(poi.x(),poi.y(),poi.z());
    // std::cout<<"size"<<thpp_temp_test_delete.first<<" "<<thpp_temp_test_delete.second<<" "<<subspace_array[thpp_temp_test_delete.first][thpp_temp_test_delete.second]->frontiers.size()<<std::endl;
    // subspace_array[thpp_temp_test_delete.first][thpp_temp_test_delete.second]->deleteFrontier(&temp_test_key);
    // std::cout<<"size"<<subspace_array[thpp_temp_test_delete.first][thpp_temp_test_delete.second]->frontiers.size()<<std::endl;


    update_subspace_set.insert(thpp.first*sub_num+thpp.second);//记录当前的submap发生了更新
    tnode = m_octree->search(**it);//这里node一定不是nullptr
    if(tnode);else{std::cout<<"tnode==NULL, chECK!!"<<std::endl; ROS_ASSERT(0);}
    tnode->set_state(f_map::FRONT);//状态设定为frontier
  }
  // for(map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator it = thismap.begin();it!=thismap.end();it++)
  //   delete (it->first); 不能delete的原因，是frontier的指针也在里面！！！
  newFrontier_Set.clear();


  // 已知frontier_list
  // 所有新覆盖的节点，如果在平面延续的8个点中出现了frontier，frotier放入check_list。
  // 如果出现了未知点，则将自己放入new_list
  // check_list遍历，如果不符合条件，从frontier_list中删除。
  // new_list加入frontier_list
  //在探索决策时，对frontier进行断层检测。即沿着frontier的两个法方向进行扩散得到两片的octonode，
  //如果发现两片octonode的z纬度差异过大，则认为是断层。
  double t6=ros::Time::now().toSec();
  //std::cout<<"Frontier动态更新时间"<<t6-t5<<std::endl;
  std::pair<int,int> now_center = sub_space::Frontier2Subspaceindex(posx,posy,posz);//LPS的中心subspace在[32][32]中的索引
  update_subspace_set.insert(now_center.first*sub_num+now_center.second);//把当前机器人所处的空间中所有Frontier加入检查范围
                                                        frontier_filer_counter++;frontier_filer_counter%=5; 
                                                        int f_s_x = 0;  int f_s_y = 0;
                                                        int f_e_x = 0;  int f_e_y = 0;
                                                        if(frontier_filer_counter==4)
                                                        {
                                                          f_s_x = now_center.first-1;    f_e_x = now_center.first+1;
                                                          f_s_y = now_center.second-1;    f_e_y = now_center.second+1;
                                                          if(now_center.first==0){f_s_x = 0;f_e_x = 2;}
                                                          if(now_center.first==31){f_s_x = 29;f_e_x = 31;}
                                                          if(now_center.second==0){f_s_y = 0;f_e_y = 2;}
                                                          if(now_center.second==31){f_s_y = 29;f_e_y = 31;}
                                                          for(int i=f_s_x;i<=f_e_x;i++)
                                                          for(int j=f_s_y;j<=f_e_y;j++)
                                                          update_subspace_set.insert(i*sub_num+j);//把当前机器人所处的空间中所有Frontier加入检查范围  
                                                        }

    
  Update_SubspaceFrontier();
  update_subspace_set.clear();
  double t7=ros::Time::now().toSec();
  //std::cout<<"子地图更新时间"<<t7-t6<<std::endl;
  std::cout<<"地图更新总时间"<<t7-t1<<std::endl;
  int d1=0;
  PublishPlane();
  // PublishFrontierMarker();
  // LPS_Planner();
  image_in_process = false;
  ForCheck();
}



void Teclas::ForCheck()
{
  int total_cct = 0;
  for(int i =0 ;i<=31;i++)
  for(int j =0 ;j<=31;j++)
  {
   if(subspace_array[i][j]->state==sub_space::EXPLORING)
   {
      total_cct += subspace_array[i][j]->frontiers.size();
   }
  }
  std::cout<<"Sum of subspace is - Sum of frontier is "<<total_cct - frontier.size()<<std::endl;
}


//返回<0,表示平面不可穿行，返回>=0，表示散点到平面的距离绝对值之和。
int Teclas::fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes,float& precent_inline,float& varia)
{
  float sqd = 0;
  float all_dis=0;
  // std::cout<<"fit_plane_to_cloud"<<std::endl;
  try 
  {
  // std::cout<<"fit_plane_to_cloud1"<<std::endl;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;//平面拟合
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_thes);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    float a=coefficients->values[0];
    float b=coefficients->values[1];
    float c=coefficients->values[2];
    float d=coefficients->values[3];
    sqd=std::sqrt(a*a+b*b+c*c);
  // std::cout<<"fit_plane_to_cloud2"<<std::endl;
    float xita = acos(c/sqd);//法向量与垂直z的夹角，相当平面与水平面的夹角。
    precent_inline = inliers->indices.size()/cloud->points.size();
    if(xita>xita_thred||(precent_inline<0.7))
    {
      varia=-1;
//      std::cout<<"Not"<<std::endl;
      return -1;//内点不够多，角度太大，放弃
    } 
    all_dis=0;
    for(int i=0;i<cloud->points.size();i++)
    {
      all_dis+=std::abs(a*cloud->points[i].x+
                        b*cloud->points[i].y+
                        c*cloud->points[i].z+
                        d);
    }
  // std::cout<<"fit_plane_to_cloud3"<<std::endl;
  all_dis = all_dis/(cloud->points.size());
  varia=all_dis/sqd;
  // std::cout<<"varia "<<varia<<"precent_inline "<<precent_inline<<std::endl;
  return 1;
  }
  catch (...) 
  {
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    return -1;
  }
}

//这里的更新策略是对活跃区域的更新。
//在subspace中删除frontier
//根据subspace中frontier的数量来更新subspace的状态
void Teclas::Update_SubspaceFrontier()
{

  // visualization_msgs::Marker marker;    //Ｍａｒｋｅｒ是一个结构体模板
  // marker.header.frame_id = "map";
  // marker.ns = "onemarker";
  // marker.type = visualization_msgs::Marker::SPHERE;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.color.r = 0.8f;
  // marker.color.g = 0.0f;
  // marker.color.b = 0.0f;
  // marker.color.a = 0.6;
  // marker.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
  // marker.scale.x = 0.25;
  // marker.scale.y = 0.25;
  // marker.scale.z = 0.25;
  // marker.id = 1;



    // std::cout<<"Started "<<std::endl;        
  //1、对当前的Subspace状态进行更新,现在是explored exploring 中的哪一种
  //2、对孤立frontier, 危险frontier 直接删除
    // int cct_imp = 0;
  std::pair<int,int> tp = sub_space::Frontier2Subspaceindex(-2.125,-8.375,0);  
  update_subspace_set.insert(tp.first*sub_num+tp.second);
  for (auto it = update_subspace_set.begin();it!=update_subspace_set.end();it++)
  {
    int idx = *it/sub_num;    int idy = *it%sub_num;//这里的索引就是这样，没有问题！！！
    std::set<octomap::OcTreeKey*,CompareKey> thisfv = subspace_array[idx][idy]->frontiers;
    //1、对当前的Subspace状态进行更新,现在是explored exploring 中的哪一种
    if(thisfv.size()>0){subspace_array[idx][idy]->state=sub_space::EXPLORING;}
    else {subspace_array[idx][idy]->state=sub_space::EXPLORED; continue;}
    //只有删除了Frontier和添加了Frontier的时候才会被检查，所以只有可能是探索完毕
    //1、对当前的Subspace状态进行更新,现在是explored exploring 中的哪一种

    //2、check frontier safety, delete isolated fronteir.
    std::vector<octomap::OcTreeKey*> removeFrontier;
    octomap::point3d thisfp;    octomap::point3d shiftfp;
    int surround_know_z_counter = 0;
    int isolated_danger = -1;//先默认是isolated.
    for(auto subfr_it = thisfv.begin();subfr_it!=thisfv.end();subfr_it++)
    {//对子空间中的每一个frontier进行遍历
      thisfp = m_octree->keyToCoord(**subfr_it);      
      isolated_danger = -1;//先默认是isolated.
      surround_know_z_counter = 0;//周围已知方格数量清空
        // if((thisfp.x()<=-2.0)&&(thisfp.x()>=-2.5)&&(thisfp.y()<=-8.0)&&(thisfp.y()>=-8.5))
        // std::cout<<"marker pose "<<thisfp.x()<<" "<<thisfp.x()<<" "<<thisfp.x()<<std::endl;
      int  frontier_cct_surround = 0;
      for(int shift_int =0; shift_int<checklist1.size();shift_int++)
      {
        surround_know_z_counter = std::min(surround_know_z_counter,int(shift_int/3)+1);
        shiftfp = thisfp+checklist1[shift_int];
        f_map::FrontierOcTreeNode * thisn=m_octree->search(shiftfp);//先在octomap中找到这个节点。   
        if(thisn)
        {
          surround_know_z_counter++;//这里代表已知节点数量
          if(thisn->get_state()==f_map::UNTRA)//
          {
            isolated_danger  = 1;//是危险的frontier，直接删除
            break;
          }
          else if(thisn->get_state()==f_map::FRONT)
          frontier_cct_surround++;
        }  
      }
      //是危险节点/孤立节点/被已知包围的节点
      if(isolated_danger==1||surround_know_z_counter>=8||surround_know_z_counter<=2||frontier_cct_surround==0)
      removeFrontier.push_back(*subfr_it);
    }


    //2、check frontier safety, delete isolated fronteir.
    //把孤立的frontier和危险的frontier都删除
    f_map::FrontierOcTreeNode * thisn;
    for(auto rmit = removeFrontier.begin();rmit!=removeFrontier.end();rmit++)
    {
      thisn=m_octree->search(**rmit);//先在octomap中找到这个节点。   
      if(thisn)
      {
      thisn->set_state(f_map::TRAVE);//认为这个frontier 可穿行
      subspace_array[idx][idy]->frontiers.erase(*rmit);//先从submap中删掉。
      frontier.erase(*rmit);//再从frontier保管区域中删掉。
      delete(*rmit);//释放这个指针的空间
      }
      else
      {
        std::cout<<"Impossible frontier is here!!!"<<std::endl;        
      }
    }
    removeFrontier.clear();
    //由于又删掉了一些frontier，所以subspace可能会成为explored的状态，即没有frontier.
    if(subspace_array[idx][idy]->frontiers.size()==0)
    subspace_array[idx][idy]->state=sub_space::EXPLORED;
  }
  update_subspace_set.clear();
}


bool Teclas::DebugService(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res)
{
  gazebo_msgs::GetModelState rq;
  rq.request.model_name="scout/";
  rq.request.relative_entity_name="map";
  while(!states_client.call(rq))
  std::cout<<"Waiting for gzbo";
  float posx=rq.response.pose.position.x;
  float posy=rq.response.pose.position.y;
  float posz=rq.response.pose.position.z;
  std::ofstream file("/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/debug_map.txt");
  if (file.is_open()) 
  {
                      file << std::fixed << std::setprecision(3) << posx;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << posy;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << posz;
                      file << "\n";
  }

  float pa=0;  float pb=0;  float pc=0;
  float p1a=0;  float p1b=0;  float p1c=0;
  float ttt;
  float cr1=0;  float cr2=0;
  if (file.is_open()) 
  {
            for(f_map::FrontierOcTree::leaf_iterator it = m_octree->begin_leafs(m_octree->getTreeDepth()), end = m_octree->end_leafs();it != end; ++it)
            {
                      octomap::point3d tp =  it.getCoordinate();
                      it->get_plane(pa,pb,pc,ttt);
                      file << std::fixed << std::setprecision(3) << tp.x();
                      file << " ";
                      file << std::fixed << std::setprecision(3) << tp.y();
                      file << " ";
                      file << std::fixed << std::setprecision(3) << tp.z();
                      file << " ";
                      file << std::fixed << std::setprecision(3) << it->get_state();
                      file << " ";
                      file << std::fixed << std::setprecision(3) << pa;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << pb;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << pc;
                      file << " ";
                      file << std::fixed << std::setprecision(3) << ttt;
                      file << "\n";

            }
  }
        file.close();
  return true;  
}


//st_sub1 st_sub2是目标子空间的位置，posx y z是当前位置，这里规划从当前位置到目标子空间位置的路径，并且得到这条路径覆盖的最小地图区域。


// 原先是找到最近的subspace,搜索到frontier的路径
// 这种法方案不能保证一定有解，因为frontier未必可达
// 现在修改为在全局地图中搜索可达frontier,不可达的frontier直接删除。

void Teclas::PrepareServiceMsg_WholeMap(srvbg::getlcplan::Response* temprqst,float posx,float posy,float posz)
{
  elevation_map wholemap;
  wholemap.org_x = map_min_x-0.125;
  wholemap.org_y = map_min_y-0.125;
  wholemap.resolution = resolu;
  int wd = int((map_max_x-map_min_x)/resolu)+2;
  int hi = int((map_max_y-map_min_y)/resolu)+2;
  wholemap.width = wd;
  wholemap.height = hi;
  wholemap.data.resize(wd*hi,-100);
  std::cout<<"wholemap prepare begin  !!"<<std::endl;
  float tempx;  float tempy; int temp_index = 0;
  for(int i =0;i<PlaneMarker.markers.size();i++)
  {
    if(PlaneMarker.markers[i].color.r<0.999)
    { //安全
      tempx = PlaneMarker.markers[i].pose.position.x;
      tempy = PlaneMarker.markers[i].pose.position.y;
      temp_index = int((tempx-map_min_x)/resolu)+int((tempy-map_min_y)/resolu)*wd;
      wholemap.data[temp_index] = std::max(float(PlaneMarker.markers[i].pose.position.z),wholemap.data[temp_index]);
    }    
  }
  std::cout<<"wholemap prepare finished  !!"<<std::endl;
  //这里做的是全局地图投影
  //这里做的是全局地图投影
  //这里做的是全局地图投影
  //这里做的是全局地图投影



//将部分不可达frontier删掉
//将部分不可达frontier删掉
//将部分不可达frontier删掉
//将部分不可达frontier删掉
//将部分不可达frontier删掉
//搜索得到frontie可达路径temp_plan
float ff_x = 0;    float ff_y = 0; float ff_z = 0;
  Eigen::Vector3f at_least_frontier;

  astar_planner::AstarPlanner Ap(&(wholemap.data),wholemap.width,wholemap.height,wholemap.org_x,wholemap.org_y,wholemap.resolution);//这里不会被障碍物膨胀覆盖
  geometry_msgs::PoseStamped startp;    startp.pose.position.x = posx;  startp.pose.position.y = posy;
  geometry_msgs::PoseStamped endp;  
  std::vector<geometry_msgs::PoseStamped> temp_plan;//得到的轨迹是全局坐标系
  bool search_goal_flag = false;
  vector<octomap::OcTreeKey*> frontier_unreach_set;
for(std::set<octomap::OcTreeKey *>::iterator f_it = frontier.begin();f_it!=frontier.end();f_it++)
{
  ff_x = m_octree->keyToCoord(**f_it).x();
  ff_y = m_octree->keyToCoord(**f_it).y();
  ff_z = m_octree->keyToCoord(**f_it).z();
  endp.pose.position.x = ff_x;  endp.pose.position.y = ff_y;
  temp_plan.clear();
  Ap.makePlan(startp,endp,&temp_plan);//终点合法搜索路径
  if(temp_plan.size()>0)
  {
    at_least_frontier[0] = ff_x;
    at_least_frontier[1] = ff_y;
    at_least_frontier[2] = ff_z;
    search_goal_flag=true;
    break;
  }//路径合法，得到轨迹
  frontier_unreach_set.push_back(*f_it);  
}
                                              if(frontier_unreach_set.size()>0)
                                              for(int tempcct = 0;tempcct<frontier_unreach_set.size();tempcct++)
                                              {
                                                octomap::point3d dele_front = m_octree->keyToCoord(*frontier_unreach_set[tempcct]);
                                                std::pair<int,int> sub_p = sub_space::Frontier2Subspaceindex(dele_front.x(),dele_front.y(),dele_front.z());
                                                subspace_array[sub_p.first][sub_p.second]->deleteFrontier(frontier_unreach_set[tempcct]);//subspace更新
                                                frontier.erase(frontier_unreach_set[tempcct]);//删除迭代器
                                                delete frontier_unreach_set[tempcct];//再释放指针空间
                                              }
                                              frontier_unreach_set.clear();
//将部分不可达frontier删掉
//将部分不可达frontier删掉
//将部分不可达frontier删掉
//将部分不可达frontier删掉
//将部分不可达frontier删掉
//搜索得到frontie可达路径temp_plan

  std::cout<<"temp_plan.size() "<<temp_plan.size()<<std::endl;
  if(!search_goal_flag) 
  {
      DebugSave2txt();
      while(1)
      std::cout<<"search no map save!!!!"<<std::endl;
  }

  float sub_minx=10000;   float sub_miny=10000;
  float sub_maxx=-10000;  float sub_maxy=-10000;
  for(int i =0;i<temp_plan.size();i++)
  {
    tempx = temp_plan.at(i).pose.position.x;
    tempy = temp_plan.at(i).pose.position.y;

    temp_plan.at(i).pose.position.z = wholemap.data[
    int((tempx - wholemap.org_x)/wholemap.resolution) + 
    int((tempy - wholemap.org_y)/wholemap.resolution) *
    wholemap.width
    ];

    sub_minx = std::min(tempx,sub_minx);
    sub_miny = std::min(tempy,sub_miny);
    sub_maxx = std::max(tempx,sub_maxx);
    sub_maxy = std::max(tempy,sub_maxy);
  }//所经过的区域
    // sub_minx = int((sub_minx-map_min_x)/resolu);
    // sub_miny = int((sub_miny-map_min_y)/resolu);
    // sub_maxx = int((sub_maxx-map_max_x)/resolu);
    // sub_maxy = int((sub_maxy-map_max_y)/resolu);
    std::pair<int,int> left_down_subspace_corner = sub_space::Frontier2Subspaceindex(sub_minx,sub_miny,0);
    std::pair<int,int> right_up_subspace_corner = sub_space::Frontier2Subspaceindex(sub_maxx,sub_maxy,0);

    //对搜索区域进行稍微扩张一下
    if(left_down_subspace_corner.first>0)
    left_down_subspace_corner.first = left_down_subspace_corner.first-1;
    if(left_down_subspace_corner.second>0)
    left_down_subspace_corner.second = left_down_subspace_corner.second-1;
    if(right_up_subspace_corner.first<31)
    right_up_subspace_corner.first = right_up_subspace_corner.first+1;
    if(right_up_subspace_corner.second<31)
    right_up_subspace_corner.second = right_up_subspace_corner.second+1;
    //如果找不到frontier，可以直接用这两个进行索引


	double t1 = 	ros::Time::now().toSec();
  std::vector<int> obs_list;
	float sz = subspace_array[left_down_subspace_corner.first][left_down_subspace_corner.second]->subspace_size;
	float rs = subspace_array[left_down_subspace_corner.first][left_down_subspace_corner.second]->octo_resolution;
	float sub2d_startx  = subspace_array[left_down_subspace_corner.first][left_down_subspace_corner.second]->center_anchor_x-(sz/2+2)*rs;//左下角
	float sub2d_starty  = subspace_array[left_down_subspace_corner.first][left_down_subspace_corner.second]->center_anchor_y-(sz/2+2)*rs;
	float sub2d_endx  = subspace_array[right_up_subspace_corner.first][right_up_subspace_corner.second]->center_anchor_x+(sz/2+2)*rs;//右上角
	float sub2d_endy  = subspace_array[right_up_subspace_corner.first][right_up_subspace_corner.second]->center_anchor_y+(sz/2+2)*rs;
	float sub2d_startz = posz-2;
	float sub2d_endz = posz+2;
                                        gdmap_show.data.clear();
                                        gdmap_show.info.origin.position.x =  sub2d_startx;
                                        gdmap_show.info.origin.position.y =  sub2d_starty;
                                        gdmap_show.info.origin.position.z =  -1.0;
                                        gdmap_show.info.resolution = rs;
                                        gdmap_show.data.clear();
                                        gdmap_show.header.frame_id = "map";
  temprqst->frontier_x.clear();
  temprqst->frontier_y.clear();
  temprqst->frontier_z.clear();
  temprqst->map2d_data.clear();
	int temph=int((sub2d_endy-sub2d_starty)*4.0)+1;
	int tempw=int((sub2d_endx-sub2d_startx)*4.0)+1;
  temprqst->map2d_data.resize(temph*tempw,-100);//99表示未知
  temprqst->height = temph;
  temprqst->width = tempw;
  temprqst->res = rs;
  temprqst->org_x = sub2d_startx;
  temprqst->org_y = sub2d_starty;
                                        gdmap_show.info.height = temph;
                                        gdmap_show.info.width = tempw;
                                        gdmap_show.data.resize(temph*tempw,100);
  std::cout<<"sub2d_startx "<<sub2d_startx<<" sub2d_starty "<<sub2d_starty<<" sub2d_endx "<<sub2d_endx<<" sub2d_endy "<<sub2d_endy<<std::endl;
	f_map::FrontierOcTreeNode* start_z_node = NULL;
                                                                          float temp_map_min_z  = sub2d_endz;
                                                                          float temp_map_max_z  = sub2d_startz;
	for(float ssx = sub2d_startx;ssx<sub2d_endx;ssx=ssx+rs)
	for(float ssy = sub2d_starty;ssy<sub2d_endy;ssy=ssy+rs)
  {
            int exp_gdx=int((ssx-sub2d_startx)/rs);
            int exp_gdy=int((ssy-sub2d_starty)/rs);
            int this_idx = exp_gdx + exp_gdy*tempw;
            float pre_ssz = sub2d_startz;
            int temp_state;
            start_z_node = NULL;
            for(float ssz = sub2d_startz;ssz<sub2d_endz;ssz=ssz+rs)
            {
              octomap::point3d pthis(ssx,ssy,ssz);
              f_map::FrontierOcTreeNode* tempnode =  m_octree->search(pthis);
              if(tempnode)
              {
                temp_state =  tempnode->get_state();               
                                                                                    gdmap_show.data[this_idx]=0;  
                if(temp_state ==f_map::UNTRA)
                {
                  temprqst->map2d_data[this_idx] = -50 ;//100表示障碍物		
                  obs_list.push_back(exp_gdx);      obs_list.push_back(exp_gdy);
                                                                                    gdmap_show.data[this_idx]=100;  
                  break;
                }
                else if(temp_state==f_map::FRONT)
                {
                  temprqst->frontier_x.push_back(ssx);
                  temprqst->frontier_y.push_back(ssy);
                  temprqst->frontier_z.push_back(ssz);//标记frontier信息。
                                                                                    gdmap_show.data[this_idx]=50;  
                }
                  start_z_node = tempnode; 
                  pre_ssz = ssz;
              }
              else if(start_z_node)
              {
                temprqst->map2d_data[this_idx] = pre_ssz;		    
                temp_map_max_z = std::max(temp_map_max_z,pre_ssz);
                temp_map_min_z = std::min(temp_map_min_z,pre_ssz);
                break;//一列出现断层，可以跳过了
              }
            }
  }
                                                                                    temprqst->max_z = temp_map_max_z;
                                                                                    temprqst->min_z = temp_map_min_z;
  //这里对两张地图进行膨胀
  //这里对两张地图进行膨胀
  //这里对两张地图进行膨胀
  for(int i=0;i<obs_list.size();i+=2)
  {
      int c_x = obs_list[i];
      int c_y = obs_list[i+1];
      for(int f_x=-1;f_x<=1;f_x++)
      for(int f_y=-1;f_y<=1;f_y++)
      {
        if(Checklegal(c_x+f_x,c_y+f_y,tempw,temph))
        {
          int this_idx = (c_x+f_x) + (c_y+f_y)*tempw;
          // gdmap_show.data[this_idx]=100;
          temprqst->map2d_data[this_idx] = -50 ;//-100表示未知，-50表示障碍物		      
        }
      }
  }
  //这里对两张地图进行膨胀
  //这里对两张地图进行膨胀
  //这里对两张地图进行膨胀


  //保证可行路径不要被影响到
  //保证可行路径不要被影响到
  //保证可行路径不要被影响到
  //保证可行路径不要被影响到
  //保证可行路径不要被影响到
  for(int i=0;i<temp_plan.size();i++)
  {
    // temprqst->map2d_data[
    // int((temp_plan[i].pose.position.x - sub2d_startx)/rs) + 
    // int((temp_plan[i].pose.position.y - sub2d_starty)/rs) *
    // tempw
    // ] = temp_plan[i].pose.position.z;

    gdmap_show.data[
    int((temp_plan[i].pose.position.x - sub2d_startx)/rs) + 
    int((temp_plan[i].pose.position.y - sub2d_starty)/rs) *
    tempw
    ] = 30;  

  }
  //保证可行路径不要被影响到
  //保证可行路径不要被影响到
  //保证可行路径不要被影响到
  //保证可行路径不要被影响到
  //保证可行路径不要被影响到

  // for(int i=left_down_subspace_corner.first;i<=right_up_subspace_corner.first;i++)
  // for(int j=left_down_subspace_corner.second;j<=right_up_subspace_corner.second;j++)
  // {

  //   for()
  //   int this_idx = i + j*tempw;
  //   gdmap_show.data[this_idx]=20;  
  // }


  //这里保证至少有一个frontier在内。
  if(1)
  {
      temprqst->frontier_x.push_back(at_least_frontier[0]);
      temprqst->frontier_y.push_back(at_least_frontier[1]);
      temprqst->frontier_z.push_back(at_least_frontier[2]);//标记frontier信息。
      int exp_gdx=int((at_least_frontier[0]-sub2d_startx)/rs);
      int exp_gdy=int((at_least_frontier[1]-sub2d_starty)/rs);
      int this_idx = exp_gdx + exp_gdy*tempw;
      gdmap_show.data[this_idx]=20;  
  }
  
  //这里对两张地图进行膨胀
  gridmapPub.publish(gdmap_show);
	double t2 = 	ros::Time::now().toSec();
  std::cout<<"temprqst->frontier_x size is "<<temprqst->frontier_x.size()<<std::endl;
  std::cout<<"Message prepare time is "<<t2 - t1<<std::endl;
  std::cout<<"Replanning done!!!"<<std::endl;
}

void Teclas::PrepareServiceMsg(srvbg::getlcplan::Response* temprqst,int st_sub1,int st_sub2,float posx,float posy,float posz)
{
	double t1 = 	ros::Time::now().toSec();
  std::vector<int> obs_list;
	float sz = subspace_array[st_sub1+0][st_sub2+0]->subspace_size;
	float rs = subspace_array[st_sub1+0][st_sub2+0]->octo_resolution;
	float sub2d_startx  = subspace_array[st_sub1+0][st_sub2+0]->center_anchor_x-(sz/2+2)*rs;//左下角
	float sub2d_starty  = subspace_array[st_sub1+0][st_sub2+0]->center_anchor_y-(sz/2+2)*rs;
	float sub2d_endx  = subspace_array[st_sub1+LPS_size-1][st_sub2+LPS_size-1]->center_anchor_x+(sz/2+2)*rs;//右上角
	float sub2d_endy  = subspace_array[st_sub1+LPS_size-1][st_sub2+LPS_size-1]->center_anchor_y+(sz/2+2)*rs;


  geometry_msgs::PoseStamped tp;
  tp.header.frame_id="map";tp.pose.position.x = sub2d_startx;tp.pose.position.y = sub2d_starty;
  planningspaceanchor_Pub.publish(tp);
  tp.header.frame_id="map";tp.pose.position.x = sub2d_endx;tp.pose.position.y = sub2d_endy;
  planningspaceanchor_Pub1.publish(tp);
	float sub2d_startz = posz-2;
	float sub2d_endz = posz+2;
                                        gdmap_show.data.clear();
                                        gdmap_show.info.origin.position.x =  sub2d_startx;
                                        gdmap_show.info.origin.position.y =  sub2d_starty;
                                        gdmap_show.info.origin.position.z =  -1.0;
                                        gdmap_show.info.resolution = rs;
                                        gdmap_show.data.clear();
                                        gdmap_show.header.frame_id = "map";
  
  temprqst->frontier_x.clear();
  temprqst->frontier_y.clear();
  temprqst->frontier_z.clear();
  temprqst->map2d_data.clear();
	int temph=int(sub2d_endy-sub2d_starty)*4+1;
	int tempw=int(sub2d_endx-sub2d_startx)*4+1;
  temprqst->map2d_data.resize(temph*tempw,-100);//99表示未知
  temprqst->height = temph;
  temprqst->width = tempw;
  temprqst->res = rs;
  temprqst->org_x = sub2d_startx;
  temprqst->org_y = sub2d_starty;
                                        gdmap_show.info.height = temph;
                                        gdmap_show.info.width = tempw;
                                        gdmap_show.data.resize(temph*tempw,100);
  std::cout<<"sub2d_startx "<<sub2d_startx<<" sub2d_starty "<<sub2d_starty<<" sub2d_endx "<<sub2d_endx<<" sub2d_endy "<<sub2d_endy<<std::endl;





	f_map::FrontierOcTreeNode* start_z_node = NULL;
                                                                          float temp_map_min_z  = sub2d_endz;
                                                                          float temp_map_max_z  = sub2d_startz;  
	for(float ssx = sub2d_startx;ssx<sub2d_endx;ssx=ssx+rs)
	for(float ssy = sub2d_starty;ssy<sub2d_endy;ssy=ssy+rs)
  {
            int exp_gdx=int((ssx-sub2d_startx)/rs);
            int exp_gdy=int((ssy-sub2d_starty)/rs);
            int this_idx = exp_gdx + exp_gdy*tempw;
            float pre_ssz = sub2d_startz;
            int temp_state;
            start_z_node = NULL;
            for(float ssz = sub2d_startz;ssz<sub2d_endz;ssz=ssz+rs)
            {
              octomap::point3d pthis(ssx,ssy,ssz);
              f_map::FrontierOcTreeNode* tempnode =  m_octree->search(pthis);
              if(tempnode)
              {
                temp_state =  tempnode->get_state();               
                                                                                    gdmap_show.data[this_idx]=0;  
                if(temp_state ==f_map::UNTRA)
                {
                  temprqst->map2d_data[this_idx] = -50 ;//-50表示障碍物		
                  obs_list.push_back(exp_gdx);      obs_list.push_back(exp_gdy);
                                                                                    gdmap_show.data[this_idx]=100;  
                  break;
                }
                else if(temp_state==f_map::FRONT)
                {
                  temprqst->frontier_x.push_back(ssx);
                  temprqst->frontier_y.push_back(ssy);
                  temprqst->frontier_z.push_back(ssz);//标记frontier信息。
                                                                                    gdmap_show.data[this_idx]=20;  
                }
                  start_z_node = tempnode; 
                  pre_ssz = ssz;
              }
              else if(start_z_node)
              {
                temprqst->map2d_data[this_idx] = pre_ssz;		    
                temp_map_max_z = std::max(temp_map_max_z,pre_ssz);
                temp_map_min_z = std::min(temp_map_min_z,pre_ssz);                
                break;//一列出现断层，可以跳过了
              }
            }
  }
                                                                                    temprqst->max_z = temp_map_max_z;
                                                                                    temprqst->min_z = temp_map_min_z;
  //这里对两张地图进行膨胀
  int this_idx = 0;
  for(int i=0;i<obs_list.size();i+=2)
  {
      int c_x = obs_list[i];
      int c_y = obs_list[i+1];
      for(int f_x=-1;f_x<=1;f_x++)
      for(int f_y=-1;f_y<=1;f_y++)
      {
        if(Checklegal(c_x+f_x,c_y+f_y,tempw,temph))
        {
          this_idx = (c_x+f_x) + (c_y+f_y)*tempw;
          gdmap_show.data[this_idx]=100;
          temprqst->map2d_data[this_idx] = -50 ;//0表示free		      
        }
      }
  }
  //这里对两张地图进行膨胀
  gridmapPub.publish(gdmap_show);
	double t2 = 	ros::Time::now().toSec();
  std::cout<<"Message prepare time is "<<t2 - t1<<std::endl;
  std::cout<<"Message prepare time is "<<t2 - t1<<std::endl;
  std::cout<<"Message prepare time is "<<t2 - t1<<std::endl;
  std::cout<<"Message prepare time is "<<t2 - t1<<std::endl;
  std::cout<<"Message prepare time is "<<t2 - t1<<std::endl;
  std::cout<<"Message prepare time is "<<t2 - t1<<std::endl;
}

bool inline Teclas::Checklegal(int grid2d_x,int grid2d_y,int grid_w,int grid_h)
{
  return(grid2d_x>=0&&grid2d_x<grid_w&&grid2d_y>=0&&grid2d_y<grid_h);
}

bool inline Teclas::Checklegal(float w_map_x,float w_map_y)
{
  return(w_map_x>map_min_x 
      && w_map_x<map_max_x 
      && w_map_y>map_min_y 
      && w_map_y<map_max_y);
}

bool Teclas::MapService(srvbg::getlcplan::Request& req,srvbg::getlcplan::Response& res)
{
  gazebo_msgs::GetModelState rq;
  rq.request.model_name="scout/";
  rq.request.relative_entity_name="map";
  while(!states_client.call(rq))
  std::cout<<"Waiting for gzbo";
  float posx=rq.response.pose.position.x;
  float posy=rq.response.pose.position.y;
  float posz=rq.response.pose.position.z;
  std::pair<int,int> center_sub = sub_space::Frontier2Subspaceindex(posx,posy,posz);//LPS的中心subspace在[32][32]中的索引
  bool ext_flag = false;
  if(req.local_map)//正常情况下，如果局部规划区中有Frontier
  {
          //保证当前帧的图像已经处理完
        //   std::cout<<"center_sub_idx "<<center_sub.first<<"center_sub_idy "<<center_sub.second<<std::endl;
          int st_sub1;  int st_sub2;//LPS的左下角subspace在subspace[32][32]中的索引
          if(center_sub.first==0)st_sub1=0;
          else if(center_sub.first==31)st_sub1=center_sub.first-2;
          else st_sub1=center_sub.first-1;
          if(center_sub.second==0)st_sub2=0;
          else if(center_sub.second==31)st_sub2=center_sub.second-2;
          else st_sub2=center_sub.second-1;
          submapfrontier_v.clear();//先进行清空
          int shiftx=0;  int shifty=0;
          //先得到所有的Frontier
          int teps = 0;
          float sub2d_startx=0;  float sub2d_starty=0;float sub2d_startz=0;
          float sub2d_endx=0;  float sub2d_endy=0;float sub2d_endz=0;
          //   std::cout<<"center_sub_idx "<<std::endl;
          for(int i=0;i<=LPS_size-1;i++)
          {//这是以自身为中心，向外扩展出的3-3子空间
            for(int j=0;j<=LPS_size-1;j++)
            {
              shiftx = st_sub1+i;
              shifty = st_sub2+j;
              teps+=subspace_array[shiftx][shifty]->frontiers.size();
              for(auto itf = subspace_array[shiftx][shifty]->frontiers.begin();itf!=subspace_array[shiftx][shifty]->frontiers.end();itf++)
              submapfrontier_v.push_back(m_octree->keyToCoord(**itf)); 
            }
          }
          PublishLocalPlanningSpaceFrontierMarker(&submapfrontier_v,center_sub.first,center_sub.second);//将局部规划区中的frontier全部发出去
          now_subspace_center = center_sub;//更新中心点，目前还没有用
          if(image_in_process) return false;

          if(teps>0)
          {
              std::cout<<"expect size "<<teps<<std::endl;
              PrepareServiceMsg(&res,st_sub1,st_sub2,posx,posy,posz);//拿到二维地图和frontier在lcpsrvmsg中
              std::cout<<"actual size "<<res.frontier_x.size()<<std::endl;
              if(res.frontier_x.size()==0) 
              {
                std::cout<<"frontier is blocked by obstacle, here we use frontiers in submap instead!!!!!!!"<<std::endl;
                std::cout<<"frontier is blocked by obstacle, here we use frontiers in submap instead!!!!!!!"<<std::endl;
                std::cout<<"frontier is blocked by obstacle, here we use frontiers in submap instead!!!!!!!"<<std::endl;
                std::cout<<"frontier is blocked by obstacle, here we use frontiers in submap instead!!!!!!!"<<std::endl;
                std::cout<<"frontier is blocked by obstacle, here we use frontiers in submap instead!!!!!!!"<<std::endl;
                std::cout<<"frontier is blocked by obstacle, here we use frontiers in submap instead!!!!!!!"<<std::endl;
                std::cout<<"frontier is blocked by obstacle, here we use frontiers in submap instead!!!!!!!"<<std::endl;
                for(auto itf = submapfrontier_v.begin();itf!=submapfrontier_v.end();itf++)
                {
                  res.frontier_x.push_back(itf->x());
                  res.frontier_y.push_back(itf->y());
                  res.frontier_z.push_back(itf->z());
                }
              }
          }
          else ext_flag=true;
  }
  if(ext_flag||req.local_map==false)
  {
          std::cout<<"Replanning!!!"<<std::endl;
          std::cout<<"Replanning!!!"<<std::endl;
          std::cout<<"Replanning!!!"<<std::endl;
          std::cout<<"Replanning!!!"<<std::endl;
          std::cout<<"Replanning!!!"<<std::endl;
          if(image_in_process) return false;
          std::cout<<"Get closest subspace !!"<<std::endl;
          PrepareServiceMsg_WholeMap(&res,posx,posy,posz);//拿到二维地图和frontier在lcpsrvmsg中
  }
            return true;
}


int main (int argc, char** argv)
{
  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  Teclas T(nh);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return (0);
}

