/**
 * @file algorithm.h
 * @brief Hybrid A* 算法的核心过程函数，只有一个函数hybridAStar()
 * 输入：
 *      始点、
 *      目标点、
 *      配置空间的3维和2维表示（2D用来A*，3D用于hybrid A*）、
 *      搜索网格的宽度及高度、
 *      配置空间的查找表、
 *      Dubins查找表（程序实际上没有使用该表，而是直接调用OMPL库计算）、
 *      RVIZ可视化类(用于显示结果)
 * 返回：
 *      满足约束条件的节点（数据结构用指针表示）
 * 
 * @date 2023-8-8
 */

#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>
using namespace HybridAStar;

//原始A*算法，用来搜索计算 holonomic-with-obstacles heuristic
float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);
//计算start到目标点goal的启发式代价(即：cost-to-go),改变start的H，单位grid
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);
//尝试在起点和终点之间直接生成Dubins shot的路径
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace,Node3D* nodes3D,int width, int height);

/**
 * 重载运算符，用来生成节点的比较 逻辑，該函數在“boost::heap::compare<CompareNodes>”获得使用
 */
struct CompareNodes 
{
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                       3D A*
//  Hybrid A* 的主调用函数，输入参数的意义如该文件开始说明
// nodes2D是配置空间的二维表示，一个数组，用于A-STAR
// nodes3D是配置空间的三维表示，一个数组，用于Hybrid A-star
//  配置空间的查找表、
//  Dubins查找表（程序实际上没有使用该表，而是直接调用OMPL库计算）
//###################################################
Node3D* Algorithm::hybridAStar(Node3D& start,const Node3D& goal,Node3D* nodes3D,int width,int height,CollisionDetection& configurationSpace)
{
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  int dir = Constants::reverse ? 6 : 3;//如果是后退，那么可能的前进方向为6；否则为3
  int iter = 0;//迭代计数
  Node2D* nodes2D = new Node2D[width * height]();
  //二项堆binomial_heap的介绍和二项树binomial_tree的介绍 
  //最小堆节点关键字大于等于其父节点的值，因为默认是最大堆，所以对比较进行重载。
  // https://zh.wikipedia.org/wiki/%E4%BA%8C%E9%A1%B9%E5%A0%86
  typedef boost::heap::binomial_heap<Node3D*,boost::heap::compare<CompareNodes> > priorityQueue;
  priorityQueue O;//open集
  updateH(start, goal, nodes2D, width, height, configurationSpace);
  start.open();//将start加入open 集合: 1)将点标记为open
  O.push(&start);// 2) 加入集合
  iPred = start.setIdx(width, height); //计算索引位置
  nodes3D[iPred] = start;

  Node3D* nPred;
  Node3D* nSucc;

  while (!O.empty()) 
  {
    //从集合中取出一个最低代价的点，代价=g+h
    nPred = O.top();
    iPred = nPred->setIdx(width, height);//获取该点在nodes3D的索引 (前缀i表示index, n表示node)
    iter++;//记录迭代次数
    std::cout<<" iteration is "<<iter<<std::endl;

    // std::cout<<" id is "<< iPred <<" iteration is "<<"close "<<nodes3D[iPred].isClosed()<<"open  "<<nodes3D[iPred].isOpen()<<iter<<std::endl;
    // if(iterations==2900) 
    // std::cout<<"nPred"<<std::endl;
    if (nodes3D[iPred].isClosed()) 
    {
      O.pop();
      // std::cout<<"isclosed "<<std::endl;      
      continue;//如果为closed，说明该点已经处理过，忽略(将它从open set中移除)
    }
    else if (nodes3D[iPred].isOpen()) 
    {//如果该点是在open状态，即正在扩张的点
//      std::cout<<"nPred"<<nPred->getX()<<"  "<<nPred->getY()<<"  "<<nPred->getT()<<std::endl;
      // std::cout<<"isopen "<<std::endl;      
      nodes3D[iPred].close();
      O.pop();
      if (nPred->ReachGoal(goal) || iter > Constants::iterations) 
      {
            delete [] nodes2D;
            if(iter>Constants::iterations)   
            {
               std::cout<<"iterations out!!!"<<std::endl;
               return nullptr;//到达
            }
            return nPred;//到达
      }
      else 
      {//未到达
        //车子是在前进方向，优先考虑用Dubins去命中目标点
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) 
        {
          nSucc = dubinsShot(*nPred, goal, configurationSpace,nodes3D, width, height); 
          if (nSucc != nullptr && nSucc->ReachGoal(goal)) 
          {
          // std::cout<<"nSucc"<<nSucc->getX()<<"  "<<nSucc->getY()<<"  "<<nSucc->getT()<<std::endl;
          // std::cout<<"goal"<<goal.getX()<<"  "<<goal.getY()<<"  "<<goal.getT()<<std::endl;
          delete [] nodes2D;
          std::cout<<"HybridA done."<<std::endl;
          return nSucc;
          }
          //如果Dubins方法能直接命中，即不需要进入Hybrid A*搜索了，直接返回结果
        }
        // 半径为6，6.75°前后左右转，共 6个方向扩展节点
        for (int i = 0; i < dir; i++) 
        {
          nSucc = nPred->createSuccessor(i);//扩展节点，i为运动原语，此时已经把nPred作为nSucc的前一个指针了
          iSucc = nSucc->setIdx(width, height);//索引值

//一共有A B C 三大类情况，A1 A2-1表示第一类情况的第一种逻辑分支，A1 A2-2表示第一种情况的第二种逻辑分支

// A1、nSucc节点已经被扩展过了,存放在nodes3D中,是open状态，但是id和前一个节点id不同，
        // A2-1、沿着当前这条路径进行扩展（即经过nPred）不能降低他的代价G，放弃,不把nSucc放入node3D中
        // A2-2、沿着当前这条路径进行扩展（即经过nPred）能降低他的代价G，那就计算nSucc当前和之前路径的G+H，如果降低就把nSucc的父节点设定为nPred，更新node3D[iSucc]=nSucc

// B1、如果当前Id和前一个节点id一样iPred == iSucc——代表在同一个grid中并且角度误差为5度内
    // B2-1、沿着当前这条路径进行扩展（即经过nPred）不能降低他的代价G+H,放弃
    // B2-2、沿着当前这条路径进行扩展（即经过nPred）能降低他的代价G+H,则更新node3d[iSucc]=nSucc,并且用nSucc替换nPred

// C1、nSucc节点没扩展过,所以不是close状态，也不是open状态
    // C2、那就把nSucc加入到node3D中，设定为open状态，并且把nSucc的父节点设定为nPred
          // 产生的节点在范围内；不会产生碰撞；
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) 
          {
            //可能下一个节点在同一个grid(index相同，两个节点的空间位置是不同)，或者下一个节点还没有被扩展过。
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) //A1 C1 || B1 
            {
              nSucc->updateG();//补充从Pred ->nSucc的运动代价
              newG = nSucc->getG();
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) //  C1 || A2-2|| B1
              {
                float ng = newG;
                float pg = nodes3D[iSucc].getG();
                // calculate H value: 更新到目标的启发值H
                updateH(*nSucc, goal, nodes2D, width, height, configurationSpace);
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker)// B2-1
                {
                float nh = nSucc->getH();
                float ph = nPred->getH();
                float nc = nSucc->getC();
                float pc = nPred->getC();
                  delete nSucc;
                  continue;
                }
                // 如果下一节点仍在相同的cell, 但是代价值要小，则用当前successor替代前一个节点（这里仅更改指针，数据留在内存中）
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) //B2-2
                {
                  nSucc->setPred(nPred->getPred());//如果下一个点仍在相同的cell、并且cost变小，成功 
                }
                if (nSucc->getPred() == nSucc) std::cout << "looping";//给出原地踏步的提示
                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;//A2-2 B2-2 C2
                O.push(&nodes3D[iSucc]);
                delete nSucc;
              } 
              else { delete nSucc; }//A2-1
            } 
            else { delete nSucc; }
          } 
          else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) 
  {
    std::cout<<"O is empty!!"<<std::endl;
    delete [] nodes2D;
    return nullptr;
  }
  delete [] nodes2D;
  return nullptr;
}

//###################################################
//                2D A*
//每一次用的时候nodes2D都会先清空一次。
// 原始A*算法，返回start到goal的cost-so-far，其中nodes2D会被改变，存储的是从起点到终点的路径。
//即传入参数的start 到goal的astar距离，单位grid
//###################################################
float aStar(Node2D& start,Node2D& goal,Node2D* nodes2D,int width,int height,CollisionDetection& configurationSpace) 
{
  int iPred, iSucc;
  float newG;
  // 将open list和close list重置
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>> O;//Open list, 注意是一个heap
  start.updateH(goal);
  start.open();
  O.push(&start);
  iPred = start.setIdx(width);//单位是grid
  nodes2D[iPred] = start;

  Node2D* nPred;
  Node2D* nSucc;
  while (!O.empty()) 
  {
    nPred = O.top();//从Open集合中找出代价最低的元素
    iPred = nPred->setIdx(width);//相应的index

    if (nodes2D[iPred].isClosed()) {//检查：如果已扩展，则从open set中移除，处理下一个
      O.pop();
      continue;
    }
    else if (nodes2D[iPred].isOpen()) {//没有进行扩展
      nodes2D[iPred].close();//标记为close
      nodes2D[iPred].discover();
      O.pop();

      if (*nPred == goal) //这里是2D的比较，不需要更改
      {
        return nPred->getG();//返回G值，即传入参数的start 到goal的astar距离，正好就是现在的G，单位grid
      }
      else 
      {//非目标点，则从可能的方向寻找
        for (int i = 0; i < Node2D::dir; i++) 
        {//A*算法是8个方向：4个正方向和4个45度的方向
          nSucc = nPred->createSuccessor(i);
          iSucc = nSucc->setIdx(width);
          // 约束性检查：在有效网格范围内、且不是障碍、没有扩展过
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) 
          {
            //更新G值
            nSucc->updateG();//单位是grid
            newG = nSucc->getG();//单位是grid
            // 如果子节点不在openset中，或者他已经在openset中(上面的if已经保证了)但是现在路径让当前G比之前的G小了
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) 
            {
              nSucc->updateH(goal);//计算H值，单位grid
              nSucc->open();//将该点移到open set中
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
// 计算到目标的启发值(cost)
// 这里的cost由三项组成：《Practical Search Techniques in Path Planning for Autonomous Driving》
// 1) "non-holonomic-without-obstacles" heuristic:（用于指导搜索向目标方向前进）
//    受运动学约束的无障碍启发式值。论文的计算建议为： max(Reed-Shepp距离/Dubins距离, 欧氏距离) 表示
//    至于用Reed-Shepp距离还是Dubins距离取决于车辆是否可倒退
// 2) "holonomic-with-obstacles" heuristic：（用于发现U形转弯(U-shaped obstacles)/死路(dead-ends)）
//    （不受运动学约束的）有障约束启发式值(即：A*)
// 注1： 实际计算时，优先考虑运动学启发式值，A*作为可选项。至于是否启用欧氏距离和A*的启发式值，取决于计算
//      的精度和CPU性能（可作为调优手段之一）
// 注2： 实际计算与论文中的描述存在差异：
//      （1）实际计算的第一步用的启发式值为“Reed-Shepp距离/Dubins距离”，而论文为“max(Reed-Shepp距离/Dubins距离, 欧氏距离)”
//      （2）实际计算的第二步用的启发式值为A*的启发式值 减去 “start与goal各自相对自身所在2D网格的偏移量(二维向量)的欧氏距离”
//函数中重新设定了start的G
//并且重置了nodes2D，将他设定为从起点到终点的Astar路径
//传入的单位全部是grid,最后会改变start.setH,单位是grid
void  updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D,  int width, int height, CollisionDetection& configurationSpace)
{
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;
  if (Constants::dubins) 
  {
    //这里改用open motion planning library的算法 
    ompl::base::DubinsStateSpace dubinsPath(float(Constants::r)*10);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    //这里的单位是grid，所以上面的半径需要换算
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    //这里的单位是grid，所以上面的半径需要换算
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
    //杜斌曲线的距离是grid，而不是m
  }

  //假如车子可以后退，则可以启动Reeds-Shepp 算法
  if (Constants::reverse && !Constants::dubins) 
  {
    ompl::base::ReedsSheppStateSpace reedsSheppPath(float(Constants::r)*10);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    //这里的单位是grid，所以上面的半径需要换算
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    //这里的单位是grid，所以上面的半径需要换算
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    //杜斌曲线的距离是grid，而不是m
    // https://blog.csdn.net/a_xiaoning/article/details/130218380
  }
  if (Constants::twoD && !nodes2D[int(start.getY()) * width + int(start.getX())].isDiscovered()) 
  {
    Node2D start2d(int(start.getX()), int(start.getY()), 0, 0, nullptr);
    Node2D goal2d(int(goal.getX()), int(goal.getY()), 0, 0, nullptr);
    nodes2D[int(start.getY()) * width + int(start.getX())].setG(
      aStar(goal2d, start2d, nodes2D, width, height, configurationSpace)
      );
    //为什么这里不是设定H值，而是setG?这里Astar已经找到一条路径了，此处返回的就是Astar的路径代价。
      //调用A*算法，返回cost-so-far, 并在2D网格中设置相应的代价值
  }

  if (Constants::twoD) 
  {
    // // offset for same node in cell
    twoDoffset = sqrt(    
                       ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * 
                       ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                       ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * 
                       ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY()))
                      );//可以认为这个值就是0，或者非常小。
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;//因为Astar求出的是grid距离
    //getG()返回A*的启发式代价，twoDoffset为 start与goal各自相对自身所在2D网格的偏移量的欧氏距离
  }

  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));//将两个代价中的最大值作为启发式值，单位是grid
  //注：虽然这里有三个数值，但Dubins Cost的启用和Reeds-Shepp Cost的启用是互斥的，所以实际上是计算两种cost而已
}
//reedsShepp曲线
//https://blog.csdn.net/qq_44339029/article/details/126200191
//###################################################
// 这里涉及到了m和grid的转换，统一单位为GRID___思路：如果不检测碰撞，是否可以得到dubin曲线？
//###################################################
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace,Node3D* nodes3D,int width, int height) 
{
  double q0[] = { start.getX(), start.getY(), start.getT() };
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  DubinsPath path;
  dubins_init(q0, q1, (Constants::r)*int(1.0/Constants::cellSize), &path);//q0 q1的单位是grid,r的单位是m,所以统一到grid单位中
  // dubins_init(q0, q1, Constants::r, &path);//q0 q1的单位是grid,r的单位是m

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);


  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];


  //杜斌曲线上进行点采样
//  std::cout << "Try dubin"<<std::endl;
  while (x <  length) 
  {
    double q[3];
    dubins_path_sample(&path, x, q);//从起点到终点前进x m，得到一个采样点 q
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));
    //这里的单位是grid
    //杜斌曲线中，没有对prim运动源语进行设定！！！！！！
    //跳出循环的条件之二：生成的路径存在碰撞节点
    if (configurationSpace.isTraversable(&dubinsNodes[i])) 
    {
      // set the predecessor to the previous step
      // if (i > 0) 
      // {
      //   dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      // } 
      // else 
      // {
      //   dubinsNodes[i].setPred(&start);
      // }
      // if (&dubinsNodes[i] == dubinsNodes[i].getPred()) 
      // {
      //   std::cout << "looping shot";
      // }
      x += Constants::dubinsStepSize;
      i++;
    }
    else 
    {
      delete [] dubinsNodes;
      // std::cout<<"Dubin fail collision"<<std::endl;
      return nullptr;
    } 
  }
  int pre_ind = -1;
  int now_ind = -1;
  pre_ind = start.getIdx(); //std::cout<<"Pre id is "<<pre_ind<<std::endl;
  now_ind = dubinsNodes[1].setIdx(width, height); //计算索引位置
  if(pre_ind!=now_ind)
  {
    dubinsNodes[1].setPred(&start);
    nodes3D[now_ind] = dubinsNodes[1];
  }
  pre_ind = now_ind;
  for(int j = 2; j <= i-1; j++)
  {
    now_ind = dubinsNodes[j].setIdx(width, height); //计算索引位置    
    if(pre_ind==now_ind)continue;
    dubinsNodes[j].setPred(&nodes3D[pre_ind]); //计算索引位置
    nodes3D[now_ind]  = dubinsNodes[j];    
    pre_ind = now_ind;
  }
  delete [] dubinsNodes;
//  std::cout<<"Dubin success, size of dubin is "<< i-1<<std::endl;
  // while(1);
  return &nodes3D[now_ind];
}