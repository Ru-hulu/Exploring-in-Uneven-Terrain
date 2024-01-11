#include "planner.h"

using namespace HybridAStar;

Planner::Planner() 
{
  ref_v.clear();
  ref_w.clear();
  ref_x.clear();
  ref_y.clear();
  ref_yaw.clear();
};

//###################################################
// 对configurationSpace进行了更新
// 对voronoiDiagram进行了更新
//###################################################
void Planner::setMap(nav_msgs::OccupancyGrid::Ptr map) 
{
  grid = map;//更新地图指针
  configurationSpace.updateGrid(map);
  int height = map->info.height;//单位grid80
  int width = map->info.width;//单位grid80
  bool** binMap;//二维数组，相当于bool binMap[][]
  std::cout <<"height"<<"  "<<height<<"width"<<"  "<<width<<std::endl;
  binMap = new bool*[width];
  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }//这里可简化为一次申请
  for (int x = 0; x < width; ++x) 
  {
    for (int y = 0; y < height; ++y) 
    {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }
  //转化为二值地图
  // voronoiDiagram.initializeMap(width, height, binMap);//注意这里传入到DynamicVoronoi里并进行保存，此地图目前和尺度无关，因为ESDF距离为infinite
  // voronoiDiagram.update();
  // voronoiDiagram.visualize("/home/r/Mysoftware/hybridAR/hybridA/testmap.pgm");//将Voronoi Diagram初始化、更新并显示
  // double time22 = ros::Time::now().toSec();
  // std::cout<<"Time cost is "<<time22-time11<<std::endl;
  //80-80的范围内需要2ms进行初始化
}


void Planner::Clear_data()
{
      ref_v.clear();
      ref_w.clear();
      ref_x.clear();
      ref_y.clear();
      ref_yaw.clear();
      Pathmotion_primitive.clear();
}
void Planner::GenerateRefCmd(std::vector<Node3D> nodePath)//现在传入的已经是以m为单位的路径点了
{
//nodePath 的首元素实际上是最后一个轨迹点
  int len = int(nodePath.size());
  double this_ref_w=0;
  double this_ref_x=0;
  double this_ref_y=0;
  double this_ref_v = 0;
  double this_ref_yaw=0;
  double pre_x=0;
  double pre_y=0;
  double pre_yaw=0;
  float grid_orix = grid->info.origin.position.x;
  float grid_oriy = grid->info.origin.position.y;
  for(int i =len-1;i>=0;i--)
  {
    int prim = nodePath[i].getPrim();
    if(prim==0||prim==3) this_ref_w=0;//HybridAstar一个节点可能有6种状态，决定当前的参考角速度的取值。
    else if (prim==1||prim==5) this_ref_w = Constants::ref_w;
    else this_ref_w = 0-Constants::ref_w;
    if(prim<=2) this_ref_v = Constants::ref_v;
    else this_ref_v = 0-double(Constants::ref_v);
    this_ref_x = double(nodePath[i].getX())*Constants::cellSize+grid_orix;
    ref_x.push_back(this_ref_x);
    this_ref_y = double(nodePath[i].getY())*Constants::cellSize+grid_oriy;
    ref_y.push_back(this_ref_y);
    this_ref_yaw=double(nodePath[i].getT());
    ref_yaw.push_back(this_ref_yaw);//弧度
    ref_v.push_back(this_ref_v);
    ref_w.push_back(this_ref_w);

    /*
    if(i!=len-1)
    switch (prim)
    {
      case 0:
      {
        if(int((this_ref_yaw-pre_yaw)*10000)==0)
        std::cout<<"right prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        else
        std::cout<<"false prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        break;      
      }
      case 1:
      {
        if(int((this_ref_yaw-pre_yaw)*10000)==873)
        std::cout<<"right prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        else
        std::cout<<"false prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        break;      
      }
      case 2:
      {
        if(int((this_ref_yaw-pre_yaw)*10000)==-873)
        std::cout<<"right prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        else
        std::cout<<"false prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        break;      
      }
      case 3:
      {
        if(int((this_ref_yaw-pre_yaw)*10000)==0)
        std::cout<<"right prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        else
        std::cout<<"false prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        break;      
      }
      case 4:
      {
        if(int((this_ref_yaw-pre_yaw)*10000)==-873)
        std::cout<<"right prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        else
        std::cout<<"false prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        break;      
      }
      case 5:
      {
        if(int((this_ref_yaw-pre_yaw)*10000)==873)
        std::cout<<"right prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        else
        std::cout<<"false prim "<<prim<<" this_ref_x "<<this_ref_x<<" this_ref_y "<<this_ref_y<<" this_ref_yaw "<<this_ref_yaw<<std::endl;        
        break;      
      }
    }
    */
  pre_x = this_ref_x;
  pre_y = this_ref_y;
  pre_yaw  = this_ref_yaw;
  }
}

// !!! 核心函数，规划过程函数

void Planner::tracePath(const Node3D* node) 
{
  const Node3D* track_node = node;
  int te1=0;  int te2=0;  int te3=0;
  while(track_node!=nullptr)
  {
    te1 = te2;    te2 = te3; te3 = track_node->getIdx();
    // std::cout<<"id is "<<track_node->getIdx()<<std::endl;
    // std::cout<<" tracePath"<<track_node->getX()<<" "<<track_node->getY()<<" "<<track_node->getT()<<std::endl;
    Pathmotion_primitive.push_back(*track_node);
    track_node = track_node->getPred();
    if(te1!=0&&te1 == te3)while(1);
  }
}


//输入的起点和终点都是以grid作为单位
bool Planner::plan(float x1,float y1,float yaw1,float &x2,float &y2,float &yaw2)
{
    //目标点和起始点有效
    // if (!(validStart && validGoal)) return;
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    Node3D* nodes3D = new Node3D[length]();//创建了一个Node3d数组，返回数组首地址。这个数组被分配了内存空间，如果不delete,即便是函数结束，内存空间也不会被分配给其他变量。很容易泄漏。
    std::cout<<"Here"<<std::endl;
    //将终点转换为以grid为单位
    // float x= 10;
    // float y=35;
    float t= 0;
    if(yaw1<0)yaw1 = yaw1+2.f*M_PI;
    t = Helper::normalizeHeadingRad(yaw1);
    Node3D nStart(x1, y1, t, 0, 0, nullptr);
    // Node3D nStart(35.5818, 24.6863, 4.62862, 0, 0, nullptr);
  //拿到目标点位置  //位姿、启发值、代价值、前一个节点

    // x = 60;
    // y = 60;
    // t = 0;

    // x = 50;
    // y = 50;
    // t = 0;
    if(yaw2<0)yaw2 = yaw2+2.f*M_PI;
    t = Helper::normalizeHeadingRad(yaw2);
    const Node3D nGoal(x2, y2, t, 0, 0, nullptr);
    // const Node3D nGoal(32.2233, 19.0499, 4.17502, 0, 0, nullptr);
  //拿到起始点位置
    // path.clear();
    // smoothedPath.clear();
    //核心步骤：
    // 1) 调用hybridAStar()函数获取一条路径
    // 2) 获取路径点(3D Node) -> 原始路径
    // 3) 对路径依据Voronoi图进行平滑->平滑路径
    //单位已经全部统一为grid
    // Start is 35.5818 24.6863 4.62862 End is 32.2233 19.0499 4.17502
    std::cout<<"Start is "<<nStart.getX()<<" "<<nStart.getY()<<" "<<nStart.getT()<<" End is "<<nGoal.getX()<<" "<<nGoal.getY()<<" "<<nGoal.getT()<<std::endl;
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, width, height, configurationSpace);
    if(nSolution==nullptr) 
    {
      std::cout<<"No solution"<<std::endl;
      delete [] nodes3D;
      return false;
    }
//    std::cout<<"Start is "<<nStart.getX()<<" "<<nStart.getY()<<" "<<nStart.getT()<<" End is "<<nGoal.getX()<<" "<<nGoal.getY()<<" "<<nGoal.getT()<<std::endl;
    tracePath(nSolution);
    GenerateRefCmd(Pathmotion_primitive);//得到控制轨迹以及参考命令
    x2 =  nSolution->getX();
    y2 =  nSolution->getY();
    yaw2 = nSolution->getT();
    delete [] nodes3D;
    return true;
}
    // while(1);
    // smoother.tracePath(nSolution);//smoother.path更新为grid为单位的路径
    // path.updatePath(smoother.getPath());//path.path为m为单位的路径
    // smoother.smoothPath(voronoiDiagram);//smoother以grid为单位,对smooth.path做平滑
    // smoothedPath.updatePath(smoother.getPath());//smooth.path更新为以m为单位的路径
    // while(1)
    // {
    // path.publishPathNodes();//显示节点
    // path.publishPathVehicles();//节点处显示车体面积
    // smoothedPath.publishPathNodes();
    // smoothedPath.publishPathVehicles();
    // }
    //两个问题：问题1：杜斌曲线片段，没有运动源语prim的赋值
    //问题2：杜斌曲线片段的角度，和我们定义的角度是不同的。
    // if(nSolution)
    //     delete [] nSolution;