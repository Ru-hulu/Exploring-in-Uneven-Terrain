#ifndef SUBSPACE_HPP
#define SUBSPACE_HPP

#include<iostream>
#include<memory>
#include<stdlib.h>
#include<surfeltree.hpp>
#include<set>
namespace sub_space
{
typedef int sub_state;
static const sub_state UNEXPLORED = -1;
static const sub_state EXPLORED = 0;
static const sub_state EXPLORING = 1;
std::pair<int,int> Frontier2Subspaceindex(float x,float y,float z);

class Subspace
{   //尺寸是5m - 5m
    public:
        float center_anchor_x;
        float center_anchor_y;
        sub_state state;
        int subspace_size = 20;
        float octo_resolution = 0.25;
        std::set<octomap::OcTreeKey*,CompareKey> frontiers;
        int deleteFrontier(octomap::OcTreeKey* tk);
        int addFrontier(octomap::OcTreeKey* tk);
};
}

#endif 
