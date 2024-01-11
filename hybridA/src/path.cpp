#include "path.h"

using namespace HybridAStar;


//###################################################
//                                         CLEAR PATH
//###################################################
//清除路径：清除以前的路径位姿点、节点信息、及车辆标记符
//         用空值的节点、车辆标记符对外进行发布（当然显示是空白）
void Path::clear() 
{
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);
  publishPathNodes();
  publishPathVehicles();
}


// 对每个3D节点信息，分别进行：增加线性addSegment(栅格转世界)、节点addNode(可视化)、和车辆标记符addVehicle(可视化)
void Path::updatePath(std::vector<Node3D> nodePath)
{
  path.header.stamp = ros::Time::now();
  int k = 0;
  for (size_t i = 0; i < nodePath.size(); ++i) 
  {
    addSegment(nodePath[i]);//将栅格坐标转换为世界坐标，这里的栅格坐标也是小数的，不是整数的。
    addNode(nodePath[i], k);//可视化
    k++;
    addVehicle(nodePath[i], k);//可视化
    k++;
  }
  return;
}
// ___________
// ADD SEGMENT增加线性:将每个node的信息转为geometry_msgs放入向量path中
void Path::addSegment(const Node3D& node) 
{
  geometry_msgs::PoseStamped vertex; 
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}

// ________
// ADD NODE：节点标记信息放入变量pathNodes
void Path::addNode(const Node3D& node, int i) 
{
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }
  pathNode.ns = "basic_shapes";
  pathNode.header.frame_id = "map";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.05;
  pathNode.scale.y = 0.05;
  pathNode.scale.z = 0.05;
  pathNode.color.a = 1.0;

  if (smoothed) {//粉红表示平滑的路径点
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {//紫色表示没有平滑的路径点
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNodes.markers.push_back(pathNode);
}

/**
 * @brief 将node节点处的车的显示信息加入向量中pathVehicles，用于可视化
 * 
 * @param node 
 * @param i 
 */
void Path::addVehicle(const Node3D& node, int i) 
{
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1) {
    pathVehicle.action = 3;
  }
  pathVehicle.ns = "vehical";
  pathVehicle.header.frame_id = "map";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicles.markers.push_back(pathVehicle);
}
