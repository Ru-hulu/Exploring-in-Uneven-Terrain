#include<subspace.hpp>
#include<ros/ros.h>
//[32][32]是从左下角到右上角的索引
std::pair<int,int>  sub_space::Frontier2Subspaceindex(float x,float y,float z)
{       
    int ix = x/0.25;
    if(x<0) ix--;
    int iy = y/0.25;
    if(y<0) iy--;
    int rx = ix/20;
    if(ix<0) rx--;    
    int ry = iy/20;//因为frontier是世界坐标下的，现在转到32 32 下
    if(iy<0) ry--;    
    rx +=16;
    ry +=16;
    return std::pair<int,int>(rx,ry);//[x][y]索引是这样对应的！！！！不要混淆
}

int sub_space::Subspace::deleteFrontier(octomap::OcTreeKey* tk)
{
    if (frontiers.size()==0)
        ROS_ASSERT(0);
    frontiers.erase(tk);
    if (frontiers.size()==0)
    state = sub_space::EXPLORED;
    return state;    
}//可以从正在探索 到 探索完毕

int sub_space::Subspace::addFrontier(octomap::OcTreeKey* tk)
{
    frontiers.insert(tk);
    if(state==sub_space::UNEXPLORED)
    state = sub_space::EXPLORING;
    return state;    
}//可以从为探索到正在探索