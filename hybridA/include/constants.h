#ifndef CONSTANTS
#define CONSTANTS

//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid
/**
 * 约定: 
 *     HEADING： [0, 359]度，0表示朝向北(指向Y)
 *     X - 表示网格的宽度
 *     Y - 表示网格的高度
 */
#include <cmath>

namespace HybridAStar 
{
namespace Constants {
static const bool reverse = true; //true表示可以倒退；false表示只能前进不能倒退
static const bool dubinsShot = true; //切换Dubin路径的开关
static const bool dubins = false;//Dubin路径的切换开关: 若车子可以倒退，值为false
static const bool dubinsLookup = false && dubins;
static const bool twoD = true;//使用astar距离作为启发距离

static const int iterations = 2000; //最大迭代次数
static const double bloating = 0; //膨胀范围
static const double width = 0.6 + 2 * bloating;//车的宽度
static const double length = 0.8 + 2 * bloating;//车的长度
static const float r = 1.0;//最小转弯半径
static const int headings = 36;
static const int check_move_cm = 10;//以10cm作为单位在车体身上设置检查点。
//车体朝向的离散数量，应该是一个状态栅格中，车体的朝向只有72个离散状态。
//节点的扩展过程中有6个方向，与72 这个数字无关。
//车辆的yaw角只有72种，5度为一个。可以影响搜索计算量。
static const float deltaHeadingDeg = 360 / (float)headings; //朝向离散步长(以度表示)
static const float deltaHeadingRad = 2 * M_PI / (float)headings; //朝向离散步长(以弧度表示)允许差异30度
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;//允许差异30度
static const float cellSize = 0.25; //在2D网格中cell的大小
static const float ref_deltaT=0.1;//两个轨迹点之间的时间步长0.1s
static const float ref_v = 0.7;//0.5m/s
static const float ref_w = 0.7;//半径=1m
static const float reach_th_dis = 0.8;//当目标点和当前位置距离在0.8个grid的时候,默认已经达到.
static const float reach_th_xita1 = 2*M_PI/12;//当目标点和当前位置角度差异阈值在此范围内,认为到达
static const float reach_th_xita2 = 2*M_PI*11/12;//当目标点和当前位置角度差异阈值在此范围内,认为到达


/**
 * 如果cost-so-far的启发式值比cost-to-come的启发式值更大时，算法应选择predecessor而不是successor。
 * 这样会导致successor从不会被选择的情况发生，该单元格永远只能扩展一个节点。tieBreaker可以人为地增加
 * predecessor的代价，允许successor放置在同一个单元中。它的使用见algorithm.cpp, 
 *     if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) 
 */
static const float tieBreaker = 0.001;//单位应该是grid
static const float penaltyTurning = 1.05;
static const float penaltyReversing = 2.0;
static const float penaltyCOD = 2.0;
static const float dubinsShotDistance = 100;
static const float dubinsStepSize = 1;
static const int dubinsWidth = 15;//Dubin搜索区域的宽度
static const int dubinsArea = dubinsWidth * dubinsWidth; //Dubin搜索区域的面积

// SMOOTHER SPECIFIC路径平滑
//当前车的最小行驶宽度
// static const float minRoadWidth = 2;
static const float minRoadWidth = 6;//车中心距离障碍物小于6grid的时候，需要对轨迹进行优化


struct color 
{
  float red;
  float green;
  float blue;
};
static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
}
}

#endif // CONSTANTS

