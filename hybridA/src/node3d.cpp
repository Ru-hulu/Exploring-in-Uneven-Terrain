/**
 * @file node3d.cpp
 * @brief 实现了前向行驶和逆向行驶的代价计算
 * 
 */
#include "node3d.h"
#include<iostream>
using namespace HybridAStar;

const int Node3D::dir = 3; //三个方向。注意2D Node为8个

//EXAMPLE grid 分辨率为0.1,转弯半径为1
// const float Node3D::dx[] = { 0.873,  0.8718903,    0.8718903};//直行时，前进0.873个grid，即0.0873m,如果在圆弧上，对应0.0873弧度，此时x前进为sin(0.0873)*半径=0.087189032m,转换为0.87189032grid
// const float Node3D::dy[] = { 0,       0.03808215,-0.03808215};//此时y前进(1-cos(0.0873))*半径 = 0.003808215m，转换为0.03808215grid
// const float Node3D::dt[] = { 0,       0.0873,   -0.0873};//0.0873弧度 > 5 度
//注意：数组中的单位是grid

//grid 分辨率为0.25，转弯半径为1
const float Node3D::dx[] = { 0.873,  0.866115935, 0.866115935};//直行时，前进0.873个grid，即0.21825m,如果在圆弧上，对应0.21825弧度，此时x前进为sin(0.21825)*半径=0.216528984m,转换为0.866115935grid
const float Node3D::dy[] = { 0,      0.094888311,-0.094888311};//此时y前进(1-cos(0.21825))*半径 = 0.023722078m，转换为0.094888311grid
const float Node3D::dt[] = { 0,      0.21825,    -0.21825};//0.21825弧度 > 5 度
//注意：数组中的单位是grid

//判断是否在3D网格上，传入的参数以grid为单位
bool Node3D::isOnGrid(const int width, const int height) const {
  return x >= 0 && x < width && y >= 0 && y < height && (int)(t / Constants::deltaHeadingRad) >= 0 && (int)(t / Constants::deltaHeadingRad) < Constants::headings;
}

bool Node3D::isInRange(const Node3D& goal) const
{
  int random = rand() % 10 + 1;//产生位于[1, 10]的随机数
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;//距离的平方和在100以内则认为可达
}//单位是grid

// 根据dx, dy, dt产生Successor, i 是一个方向,会被赋值为successor的prim(运动原语)
// 当前源码的dx, dy, dt为人为指定的值，可以根据实际需要进行修改
//这里的单位也是grid，因为在dx dy dt中已经是grid为单位的数值
Node3D* Node3D::createSuccessor(const int i) 
{
  float xSucc;
  float ySucc;
  float tSucc;

  if (i < 3) 
  {//前向 Successor
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
    //setpred的时候是否改变运动prim?
  }
  else 
  {//后向 Successor
    xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
    ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);//如果此处认为旋转矩阵不变，相当于dx被加了一负号
    switch (i)
    {
    case 3:
    tSucc = Helper::normalizeHeadingRad(t + dt[0]);
      break;
    case 4:
    tSucc = Helper::normalizeHeadingRad(t + dt[2]);
      break;    
    case 5:
    tSucc = Helper::normalizeHeadingRad(t + dt[1]);
      break;
    }
  }
//  std::cout<<i<<"  "<<xSucc<<"  "<<ySucc<<"  "<<tSucc<<std::endl;
  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}

//目前的移动代价
//这里的单位是grid
void Node3D::updateG() 
{
  if (prim < 3) 
  {//前进情况
    if (pred->prim != prim) 
    {//方向发生改变时
      if (pred->prim > 2) 
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;//改变方向的惩罚  1.47
      } else 
      {
        g += dx[0] * Constants::penaltyTurning;//没有改变方向  0.735
      }
    } 
    else 
    {//方向没有发生变化
      g += dx[0];//0.7
    }
  }
  else 
  {//后退
    if (pred->prim != prim) 
    {
      if (pred->prim < 3) 
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      } 
      else 
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    } 
    else 
    {
      g += dx[0] * Constants::penaltyReversing;
    }
  }
}
//3d节点的比较函数：x和y同时相同、并且theta在一个阈值范围内时可以认为是同一个Node
//这里的单位是grid
bool Node3D::operator == (const Node3D& rhs) const 
{
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
bool Node3D::ReachGoal(const Node3D& goaln)const
{
  double dis =std::sqrt((x-goaln.x)*(x-goaln.x)+(y-goaln.y)*(y-goaln.y)) ;
  double dis_ya =  std::abs(t - goaln.t);//注意!!!;这里所有的判定单位都是grid,比如reach_th_dis的单位就是0.8,即0.8个grid!而不是m!
  return (dis <=Constants::reach_th_dis)&& (dis_ya <= Constants::reach_th_xita1 || dis_ya >= Constants::reach_th_xita2);
}