#include "smoother.h"
using namespace HybridAStar;
//###################################################
//                                     CUSP DETECTION
//###################################################
//交点检测函数
//看有没有前进后退混合在一起的四个点。此处单位为grid
inline bool isCusp(std::vector<Node3D> path, int i) 
{
  bool revim2 = path[i - 2].getPrim() > 3 ? true : false;
  bool revim1 = path[i - 1].getPrim() > 3 ? true : false;
  bool revi   = path[i].getPrim() > 3 ? true : false;
  bool revip1 = path[i + 1].getPrim() > 3 ? true : false;
  //  bool revip2 = path[i + 2].getPrim() > 3 ? true : false;

  if (revim2 != revim1 || revim1 != revi || revi != revip1) { return true; }

  return false;
}
//###################################################
//                                SMOOTHING ALGORITHM
//不仅更新了当前节点的位置，而且更新了前一个节点的角度朝向
//###################################################
void Smoother::smoothPath(DynamicVoronoi& voronoi) 
{
  // load the current voronoi diagram into the smoother
  this->voronoi = voronoi;
  this->width = voronoi.getSizeX();
  this->height = voronoi.getSizeY();
  int iterations = 0;
  int maxIterations = 500;//最大迭代次数
  int pathLength = 0;

  pathLength = path.size();
  std::vector<Node3D> newPath = path;//单位是grid

  float totalWeight = wSmoothness + wCurvature + wVoronoi + wObstacle;//四项的权重数
  //利用数值的方法对目标函数求解。
  while (iterations < maxIterations) 
  {
    // choose the first three nodes of the path
    for (int i = 2; i < pathLength - 2; ++i) 
    {
     //后面2个点，当前点，前面2个点
      Vector2D xim2(newPath[i - 2].getX(), newPath[i - 2].getY());
      Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());
      Vector2D xi(newPath[i].getX(), newPath[i].getY());
      Vector2D xip1(newPath[i + 1].getX(), newPath[i + 1].getY());
      Vector2D xip2(newPath[i + 2].getX(), newPath[i + 2].getY());
      Vector2D correction;//这里的点单位都是grid

      if (isCusp(newPath, i)) { continue; }//若为交点，不做平滑

      correction = correction - obstacleTerm(xi);//向障碍物反方向移动，障碍物信息是通过Voronoi图获取的
      if (!isOnGrid(xi + correction)) { continue; }//假如校正方向超出当前监视的网格范围，不做处理
      correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      //这里是利用五个点做了一个平滑
      if (!isOnGrid(xi + correction)) { continue; }

      //限制曲率，做了平滑单位都是grid
      correction = correction - curvatureTerm(xim1, xi, xip1);
      if (!isOnGrid(xi + correction)) { continue; }

      xi = xi + alpha * correction/totalWeight;
      newPath[i].setX(xi.getX());
      newPath[i].setY(xi.getY());
      Vector2D Dxi = xi - xim1;
      newPath[i - 1].setT(std::atan2(Dxi.getY(), Dxi.getX()));
    }
    iterations++;
  }

  path = newPath;
}

//函数执行以后，Node3d中的数据会转换为对象本身的path
void Smoother::tracePath(const Node3D* node, int i, std::vector<Node3D> path) 
{
  if (node == nullptr) 
  {
    this->path = path;
    return;
  }
  i++;
  path.push_back(*node);
  std::cout<<"id is "<<node->getIdx()<<std::endl;
  std::cout<<" tracePath"<<node->getX()<<" "<<node->getY()<<" "<<node->getT()<<std::endl;
  tracePath(node->getPred(), i, path);
}

//###################################################
//                                      OBSTACLE TERM
//###################################################
//返回指向最近障碍的梯度方向
Vector2D Smoother::obstacleTerm(Vector2D xi) 
{
  Vector2D gradient;
  float obsDst = voronoi.getDistance(xi.getX(), xi.getY());//这里单位为grid
  int x = (int)xi.getX();//这里单位为grid
  int y = (int)xi.getY();//这里单位为grid
  if (x < width && x >= 0 && y < height && y >= 0) //这里单位为grid
  {
    ////由障碍物指向当前位置方向,以grid为单位进行运算
    Vector2D obsVct(xi.getX() - voronoi.data[(int)xi.getX()][(int)xi.getY()].obstX,
                    xi.getY() - voronoi.data[(int)xi.getX()][(int)xi.getY()].obstY);

    //以grid为单位进行优化    
    if (obsDst < obsDMax) {
      return gradient = wObstacle * 2 * (obsDst - obsDMax) * obsVct / obsDst;
      //这个方向是指向障碍物的
    }
  }
  return gradient;//有潜在风险，前面没有赋值
}

//###################################################
//                                     CURVATURE TERM
//###################################################
//返回梯度方向，以grid为单位进行计算，kappaMax进行了m到grid的转换
//https://blog.csdn.net/condom10010/article/details/125566814
Vector2D Smoother::curvatureTerm(Vector2D xim1, Vector2D xi, Vector2D xip1) 
{//前，此，后三个点
  Vector2D gradient;
  Vector2D Dxi = xi - xim1;
  Vector2D Dxip1 = xip1 - xi;
  Vector2D p1, p2;

  // the distance of the vectors
  float absDxi = Dxi.length();
  float absDxip1 = Dxip1.length();

  // ensure that the absolute values are not null
  if (absDxi > 0 && absDxip1 > 0) {
    // the angular change at the node
    float Dphi = std::acos(Helper::clamp(Dxi.dot(Dxip1) / (absDxi * absDxip1), -1, 1));
    float kappa = Dphi / absDxi;

    // if the curvature is smaller then the maximum do nothing
    if (kappa <= kappaMax) {
      Vector2D zeros;
      return zeros;
    } else {
      //代入原文公式(2)与(3)之间的公式
      //参考：
      // Dolgov D, Thrun S, Montemerlo M, et al. Practical search techniques in path planning for 
      //  autonomous driving[J]. Ann Arbor, 2008, 1001(48105): 18-80.
      float absDxi1Inv = 1 / absDxi;
      float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
      float u = -absDxi1Inv * PDphi_PcosDphi;
      // calculate the p1 and p2 terms
      p1 = xi.ort(-xip1) / (absDxi * absDxip1);//公式(4)
      p2 = -xip1.ort(xi) / (absDxi * absDxip1);
      // calculate the last terms
      float s = Dphi / (absDxi * absDxi);
      Vector2D ones(1, 1);
      Vector2D ki = u * (-p1 - p2) - (s * ones);
      Vector2D kim1 = u * p2 - (s * ones);
      Vector2D kip1 = u * p1;

      // calculate the gradient
      gradient = wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);
      if (std::isnan(gradient.getX()) || std::isnan(gradient.getY())) {
        std::cout << "nan values in curvature term" << std::endl;
        Vector2D zeros;
        return zeros;
      }
      // return gradient of 0
      else {
        return gradient;
      }
    }
  }
  // return gradient of 0
  else {
    std::cout << "abs values not larger than 0" << std::endl;
    Vector2D zeros;
    return zeros;
  }
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
  return wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}//以grid为单位进行平滑
//公式推导见https://zhuanlan.zhihu.com/p/118666410

