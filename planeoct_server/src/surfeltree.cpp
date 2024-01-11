#include "surfeltree.hpp"

using namespace f_map;

FrontierOcTree::FrontierOcTree( double in_resolution ): OccupancyOcTreeBase<FrontierOcTreeNode>(in_resolution) 
{
    frontierOcTreeMemberInit.ensureLinking();
}

FrontierOcTree::StaticMemberInitializer FrontierOcTree::frontierOcTreeMemberInit;
