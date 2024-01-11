#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include "constants.h"
#include "helper.h"
namespace HybridAStar {
class Node3D {//所有信息都以grid为单位
 public:

  Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}
  //位姿、代价值、启发值、前一个节点
  //虽然是float，但是单位是grid 
  Node3D(float x, float y, float t, float g, float h, const Node3D* pred, int prim = 0) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
  }

  float getX() const { return x; }  //单位是grid 
  float getY() const { return y; }  //单位是grid 
  float getT() const { return t; }
  /// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }
  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed open表示一个点的8个方向至少一个没有被探索过。并不表示这个点不可以是通往终点路径上的点。
  bool isClosed() const { return c; }
  /// determine whether the node is open
  const Node3D* getPred() const { return pred; }

  // SETTER METHODS：设置方法，用新值替代旧值
  /// set the x position
  void setX(const float& x) { this->x = x; }  //单位是grid 
  /// set the y position
  void setY(const float& y) { this->y = y; }  //单位是grid 
  /// set the heading theta
  void setT(const float& t) { this->t = t; }
  /// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }

  //这里传进来的是ros中gridmap，栅格地图的尺寸，长度几个grid，宽度几个grid
  //这里的index，是1~[72(一个格子的状态量)*height*width]  
  //t / Constants::deltaHeadingRad对应72个状态量中的一个
// 这里的id和当前车辆的位置(int)以及姿态有关  
  int setIdx(int width, int height) { this->idx = (int)(t / Constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x); return idx;}  //单位是grid 
  void open() { o = true; c = false;}
  void close() { c = true; o = false; }
  void setPred(const Node3D* pred) { this->pred = pred; }

  void updateG();//更新从predecessor到该节点的 cost-so-far，同时标记该节点为已探查

  bool operator == (const Node3D& rhs) const;//位置相同且朝向相似则认为是同一节点
  bool ReachGoal(const Node3D& goaln)const;//当前点达到目标点
  bool isInRange(const Node3D& goal) const;//检测是否可以分析方法找到解

  bool isOnGrid(const int width, const int height) const;//检查该点是否在3D array范围内

  Node3D* createSuccessor(const int i);//在连续空间中创建successor

  static const int dir;//方向数量
  /// Possible movements in the x direction
  static const float dx[];//在X方向的可能移动步长
  /// Possible movements in the y direction
  static const float dy[];//在Y方向的可能移动步长
  /// Possible movements regarding heading theta
  static const float dt[];//在theta方向的可能移动步长

 private:
  /// the x position
  float x;  //单位是grid 
  /// the y position
  float y;  //单位是grid 
  /// the heading theta
  float t;
  /// the cost-so-far
  float g;//目前代价值
  /// the cost-to-go
  float h;//启发式值
  /// the index of the node in the 3D array
  int idx;//节点的索引位置
  /// the open value
  bool o;//是否属于open set 和 c相反
  /// the closed value
  bool c;//是否属于close set 和 o 相反
  /// the motion primitive of the node，即代表当前节点是父节点的6个运动搜索方向中的哪一个
  int prim;
  /// the predecessor pointer
  const Node3D* pred;//祖先节点
};
}
#endif // NODE3D_H
