#include "astar_planner.h"
namespace astar_planner
{   //高程图中，标记了障碍物（-100） 和 各个位置的高度信息。
    AstarPlanner::AstarPlanner(std::vector<float>* height_map , int map_width, int map_height,float start_x, float start_y, float resolution)
    {
        width = map_width;
        height = map_height;
        gdstart_x = start_x;
        gdstart_y = start_y;
        map_size = width * height;
        rz = resolution;
        OGM.resize(map_size,0);
        for (int i = 0; i < map_size; i++)
        OGM[i]  = (*height_map)[i];
        // for data <= -50 (-100), this is dangerous areas.
    }
    void AstarPlanner::worldToMap(double wx, double wy, unsigned int &m_x,unsigned int &m_y)
    {
        m_x = (wx - gdstart_x)/rz;
        m_y = (wy - gdstart_y)/rz;
    }

    int AstarPlanner::getIndex(int m_x,int m_y)
    {
        return (m_x+m_y*width);
    }
    void AstarPlanner::indexToCells(int current_cell,unsigned int &tmp1,unsigned int &tmp2)
    {
        tmp1 = current_cell%width;
        tmp2 = current_cell/width;
    }
    void AstarPlanner::mapToWorld(unsigned int tmp1,unsigned int tmp2, double &x, double &y)
    {
        x = double(tmp1)*rz+gdstart_x;
        y = double(tmp2)*rz+gdstart_y;
    }    
    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal, 
    std::vector<geometry_msgs::PoseStamped>* plan)
    {

        double wx = start.pose.position.x;
        double wy = start.pose.position.y;
        unsigned int start_x, start_y;
        worldToMap(wx, wy, start_x, start_y);
        int start_index = getIndex(start_x, start_y);
        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        unsigned int goal_x, goal_y;
        worldToMap(wx, wy, goal_x, goal_y);
        int goal_index = getIndex(goal_x, goal_y);
        
        vector<float> gCosts(map_size, infinity);
        vector<int> cameFrom(map_size, -1);
        
        multiset<Node2d> priority_costs;
        
        gCosts[start_index] = 0;
        Node2d currentNode;
        currentNode.index = start_index;
        currentNode.cost = gCosts[start_index] + 0;
        priority_costs.insert(currentNode);
        plan->clear();
        
        while(!priority_costs.empty())
        {
            // Take the element from the top
            currentNode = *priority_costs.begin();
            //Delete the element from the top
            priority_costs.erase(priority_costs.begin());
            if (currentNode.index == goal_index){
                break;
            }
            // Get neighbors
            vector<int> neighborIndexes = get_neighbors(currentNode.index);
            
            for(int i = 0; i < neighborIndexes.size(); i++){
                if(cameFrom[neighborIndexes[i]] == -1){
                  gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]);
                  Node2d nextNode;
                  nextNode.index = neighborIndexes[i];
                  nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                  cameFrom[neighborIndexes[i]] = currentNode.index;
                  priority_costs.insert(nextNode);
                }
            }
        }
        
        if(cameFrom[goal_index] == -1){
            cout << "Goal not reachable, failed making a global path." << endl;
            return false;
        }
        
        if(start_index == goal_index)
            return false;
        //Finding the best path
        vector<int> bestPath;
        currentNode.index = goal_index;
        while(currentNode.index != start_index){
            bestPath.push_back(cameFrom[currentNode.index]);
            currentNode.index = cameFrom[currentNode.index];
        }
        reverse(bestPath.begin(), bestPath.end());
        
        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < bestPath.size(); i++)
        {
          unsigned int tmp1, tmp2;
          //gdmap->data[bestPath[i]]=50;
          indexToCells(bestPath[i], tmp1, tmp2);
          double x, y;
          mapToWorld(tmp1,tmp2, x, y);
          geometry_msgs::PoseStamped pose;
          pose.header.stamp = plan_time;
          pose.header.frame_id = "map";
          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.position.z = 0.0;
          // std::cout<<"Astar path "<<x<<"-"<<y<<" "<<tmp1<<"-"<<tmp2<<std::endl;
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;

          plan->push_back(pose);
        }
        plan->push_back(goal);
        return true;    
    }
    
    double AstarPlanner::getMoveCost(int firstIndex, int secondIndex)
    {
        unsigned int tmp1, tmp2;
        indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1,firstYCord = tmp2;
        indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;
        
        int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
        // Error checking
        if(difference != 1 && difference != 2){
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        if(difference == 1)
            return 1.0;
        else
            return 1.4;
    }
    
    double AstarPlanner::getHeuristic(int cell_index, int goal_index)
    {
        unsigned int tmp1, tmp2;
        indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;
        return abs(goalY - startY) + abs(goalX - startX);
    }
    
    bool AstarPlanner::isInBounds(int x, int y)
    {
        if( x < 0 || y < 0 || x >= width || y >= height)
            return false;
        return true;
    }



    //拿到一个节点的邻居是搜索算法的关键，这里做高度的限制。
    vector<int> AstarPlanner::get_neighbors(int current_cell)
    {   
        vector<int> neighborIndexes;
        
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                unsigned tmp1, tmp2;
                indexToCells(current_cell, tmp1, tmp2);
                int nextX = tmp1 + i;
                int nextY = tmp2 + j;
                int nextIndex = getIndex(nextX, nextY);
                if(!( i == 0 && j == 0) && isInBounds(nextX, nextY)&&(OGM[nextIndex]>=-50.0))//保证邻居节点在地图范围内 并且扩展节点是安全的。
                {
                    if(std::abs(OGM[nextIndex] - OGM[current_cell])<=search_elevation_th_gap)//满足扩展节点和当前节点之间的高度差异在范围内。
                    {
                    neighborIndexes.push_back(nextIndex);
                              //40+3200 -- 79  +  3200
                              //40+3120 -- 79  +  3120
                    // if((nextIndex<=3279)&&(nextIndex>=3240))
                    // std::cout<<"delta "<<abs(OGM[nextIndex] - OGM[current_cell])<<std::endl;
                    // if((current_cell<=3279)&&(current_cell>=3240))
                    // std::cout<<"delta "<<abs(OGM[nextIndex] - OGM[current_cell])<<std::endl;
                    // std::cout<<"current_cell "<<current_cell<<"nextIndex "<<nextIndex<<std::endl;
                    }
                }
            }
        }
        return neighborIndexes;
    }
};
// Required for multiset sorting
bool operator <(const Node2d& x, const Node2d& y) {
  return x.cost < y.cost;
}
