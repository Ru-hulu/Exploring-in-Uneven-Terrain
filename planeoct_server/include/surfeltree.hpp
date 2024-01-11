#ifndef FRONTIERMAP_HPP
#define FRONTIERMAP_HPP
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

//需要的数据类型
//状态量   int      -1:不可穿行 0:frontier  1:可穿行
//平面量   float[4] Ax+By+Cz+D=0; A B C D 
//平坦度   float    平面距离均值


struct CompareKey
{
	bool operator()(octomap::OcTreeKey* a, octomap::OcTreeKey* b)const //作为key得告诉map如何比较
	{
    if(a->k[0]<b->k[0])return true;
    else if((a->k[0]==b->k[0])&&(a->k[1]<b->k[1]))
      return true;
    else if((a->k[0]==b->k[0])&&(a->k[1]==b->k[1])&&(a->k[2]<b->k[2]))
      return true;
    return false;
	}
};

namespace f_map
{
typedef bool Frontier_t;
static const Frontier_t FRONTIER     = true;
static const Frontier_t NON_FRONTIER = false;

//状态量   int      -1:不可穿行 0:frontier  1:可穿行
typedef int nState;
static const nState UNTRA = -1;
static const nState FRONT =  0;
static const nState TRAVE =  1;
static const nState CLIFF =  2;
//状态量   int      -1:不可穿行 0:frontier  1:可穿行

//平面量   float[4] Ax+By+Cz+D=0; A B C D 
typedef float PlaneA;
typedef float PlaneB;
typedef float PlaneC;
typedef float PlaneD;
//平面量   float[4] Ax+By+Cz+D=0; A B C D 

//平坦度   float    平面距离均值
typedef float AverageDist;
typedef float PercentInliner;
//内点数量   float    

using namespace octomap;

class FrontierOcTreeNode : public OcTreeNode 
{
public:
    FrontierOcTreeNode(): 
        OcTreeNode(),
        fv(NON_FRONTIER),
        nstate(UNTRA),
        np_a(0),
        np_b(0),
        np_c(0),
        np_d(0),
        avd(0),
        per(-1)        
    {}

    FrontierOcTreeNode(Frontier_t f,nState nst,PlaneA a,PlaneB b,PlaneC c,PlaneD d,AverageDist da,PercentInliner pe): 
      OcTreeNode(),
      fv(f),
      nstate(nst),
      np_a(a),
      np_b(b),
      np_c(c),
      np_d(d),
      avd(da),
      per(pe)
    {}

    FrontierOcTreeNode( const FrontierOcTreeNode &other )
    : OcTreeNode(other),
      fv(other.fv),
      nstate(other.nstate),
      np_a(other.np_a),
      np_b(other.np_b),
      np_c(other.np_c),
      np_d(other.np_d),
      avd(other.avd),
      per(other.per)        
      {}

    bool operator == ( const FrontierOcTreeNode& other ) const 
    {
        return ( other.value == value && 
                 other.nstate == nstate &&  
                 other.np_a == np_a &&  
                 other.np_b == np_b &&  
                 other.np_c == np_c &&  
                 other.np_d == np_d &&   
                 other.avd == avd &&
                 other.per == per
                );
    }   //等于号运算符重载

    void copyData( const FrontierOcTreeNode &other ) 
    {
        OcTreeNode::copyData(other);
        this->fv = other.is_frontier() ? FRONTIER : NON_FRONTIER;
        this->nstate = other.get_state();
        float aaa,bbb,ccc,ddd;
        other.get_plane(aaa,bbb,ccc,ddd);
        this->np_a = aaa;
        this->np_b = bbb;
        this->np_c = ccc;
        this->np_d = ddd;
        this->avd = other.get_avd();
        this->per = other.get_per();
    }   //数据拷贝函数
    bool is_frontier() const { return fv; }//判断是否为frontier
    void set_frontier() { fv = FRONTIER; }//设置状态
    void clear_frontier() { fv = NON_FRONTIER; }//清空状态

    int get_state() const { return nstate; }
    void set_state(int ss) {nstate = ss;}

    void get_plane(float& a_,float& b_,float& c_,float& d_) const 
    {a_=np_a;b_=np_b;c_=np_c;d_=np_d;}
    void set_plane(float a_,float b_,float c_,float d_) 
    {np_a=a_;np_b=b_;np_c=c_;np_d=d_;}

    float get_avd() const {return avd;}
    void set_avd(float avd_) {avd=avd_;}

    float get_per() const {return per;}
    void set_per(float per_) {per=per_;}

    //获取状态
    std::istream& readData( std::istream &ins ) 
    {
        ins.read((char*) &value, sizeof(value)); // Occupancy.
        ins.read((char*) &fv, sizeof(Frontier_t));  // Frontier.
        ins.read((char*) &nstate, sizeof(nState));  // nstate.
        ins.read((char*) &np_a, sizeof(PlaneA));  // a.
        ins.read((char*) &np_b, sizeof(PlaneB));  // b.
        ins.read((char*) &np_c, sizeof(PlaneC));  // c.
        ins.read((char*) &np_d, sizeof(PlaneD));  // d.
        ins.read((char*) &avd, sizeof(AverageDist));  // avd.
        ins.read((char*) &per, sizeof(PercentInliner));  // per.
        return ins;
    }

    //设置状态
    std::ostream& writeData( std::ostream &out ) const 
    {
        out.write((const char*) &value, sizeof(value)); // Occupancy.
        out.write((const char*) &fv, sizeof(Frontier_t));  // Frontier.
        out.write((const char*) &nstate, sizeof(nState));  // nstate.
        out.write((const char*) &np_a, sizeof(PlaneA));  // a.
        out.write((const char*) &np_b, sizeof(PlaneB));  // b.
        out.write((const char*) &np_c, sizeof(PlaneC));  // c.
        out.write((const char*) &np_d, sizeof(PlaneD));  // d.
        out.write((const char*) &avd, sizeof(AverageDist));  // avd.
        out.write((const char*) &per, sizeof(PercentInliner));  // per.
        return out;
    }

public:
    Frontier_t fv;
    nState nstate;
    PlaneA np_a;
    PlaneB np_b;
    PlaneC np_c;
    PlaneD np_d;
    AverageDist avd;//距离方差
    PercentInliner per;//内点百分比
};

class FrontierOcTree : public OccupancyOcTreeBase<FrontierOcTreeNode> 
{
public:
    FrontierOcTree( double in_resolution );
    FrontierOcTree* create() const { return new FrontierOcTree(resolution); }
    std::string getTreeType() const { return "FrontierOcTree"; }


    //设定为frontier，清空frontier的两种方式
    FrontierOcTreeNode* set_node_info(const OcTreeKey &key,int nstate_,float a_,float b_,float c_,float d_,float avd_,float per_)
    {
        FrontierOcTreeNode *node = search(key);
        if ( node ) 
        {
            // std::cout<<"know"<<std::endl;
            node->set_frontier();
            node->set_state(nstate_);
            node->set_plane(a_,b_,c_,d_);
            node->set_avd(avd_);
            node->set_per(per_);
        }
        else return nullptr;
        return node;
    }
    FrontierOcTreeNode* set_node_info( float x, float y, float z,int nstate_,float a_,float b_,float c_,float d_,float avd_,float per_) 
    {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) )  return nullptr;
        return set_node_info(key,nstate_,a_,b_,c_,d_,avd_,per_);
    }
    FrontierOcTreeNode* clear_node_info( const OcTreeKey &key ) 
    {
        FrontierOcTreeNode *node = search(key);
        if ( node ) 
        {
            node->clear_frontier();
            node->set_state(-1);
            node->set_plane(0,0,0,0);
            node->set_avd(0);
            node->set_per(0);
        }
        return node;
    }
    FrontierOcTreeNode* clear_node_info( float x, float y, float z ) 
    {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) return nullptr;
        return clear_node_info(key);
    }
    //设定为frontier，清空frontier的两种方式


protected:
    class StaticMemberInitializer
    {
        public:
        StaticMemberInitializer() 
        {
            FrontierOcTree *tree = new FrontierOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }
        void ensureLinking() {}
    };
    static StaticMemberInitializer frontierOcTreeMemberInit;
};

} // namespace f_map

#endif // FRONTIERMAP_HPP


