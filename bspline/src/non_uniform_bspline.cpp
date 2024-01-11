#include "non_uniform_bspline.h"

NonUniformBspline::NonUniformBspline(const ros::NodeHandle &nh_):nh(nh_)
{
  sdfmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("sdf_map",3);
  showspline = nh.advertise<nav_msgs::Path>("showcmd_node",5);
  showspeci = nh.advertise<nav_msgs::Path>("showspeci",5);
  showcontrp = nh.advertise<nav_msgs::Path>("showcontrp",5);
  showcontrp_smf = nh.advertise<nav_msgs::Path>("showcontrp_opt_smo_fro_obs",5);
  showcontrp_smo = nh.advertise<nav_msgs::Path>("showcontrp_opt_smo_obs",5);
}
void NonUniformBspline::saveplanning()
{
        std::string fileName = "/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/"+std::to_string(numFiles) + ".txt";
        std::ofstream file(fileName);
        if (file.is_open()) 
        {
            file << control_points_; // 将矩阵写入文件
            file.close();
            std::cout << "文件 " << fileName << " 已创建并数据已保存." << std::endl;
        } 
        else 
        {
            std::cout << "无法打开文件 " << fileName << std::endl;
        }
        numFiles++;
}

void NonUniformBspline::GetCmd(float now_w_x,float now_w_y,float now_w_yaw)
{
  int n_cp = control_points_.rows();
  int n_pt = u_.size();
  double u_min = u_(0);
  double u_max = u_(n_cp-1);
  double rs = interval_/10;//节点的步长为0.1
  double sum_s = 0;
  double this_s = 0;
  Eigen::VectorXd thisp1;  Eigen::VectorXd thisp2;
  bsline_list.setZero(800,5);
  non_zero_line=0;
  double st_u;
  double now_yaw = now_w_yaw;//
  double pre_yaw = now_w_yaw;//
  double temp_delta_yaw;
  double temp_w=0;
  if(u_min>=u_max-interval_*2)//极端情况下，如果控制点数量只有2个，头、尾部两个，有可能出现无法插值的特殊情况。
  {
      bsline_list(non_zero_line,0)= control_points_(n_cp-1,0);//x
      bsline_list(non_zero_line,1)= control_points_(n_cp-1,1);//y
      now_yaw = std::atan2(control_points_(n_cp-1,1)-now_w_y,control_points_(n_cp-1,0)-now_w_x); 
      if(std::isnan(now_yaw))   bsline_list(non_zero_line,2)= pre_yaw;//yaw
      else                      bsline_list(non_zero_line,2)= now_yaw;      
      bsline_list(non_zero_line,3)= 0;//v
      bsline_list(non_zero_line,4)= 0;//a
      non_zero_line++;
      ROS_INFO("u_min>=u_max-interval_*2");
      return;
  }
  for(st_u = u_min; st_u<u_max-rs;st_u = st_u + rs)
  {
   thisp1 = evaluateDeBoor(st_u);
   thisp2 = evaluateDeBoor(st_u+rs);
   this_s = (thisp2-thisp1).norm();
   if(sum_s>car_control_dis_step)
   {
      non_zero_line++;
      bsline_list(non_zero_line-1,0)= thisp1(0);  //x   
      bsline_list(non_zero_line-1,1)= thisp1(1);  //y   
      now_yaw = std::atan2(thisp2(1)-thisp1(1),thisp2(0)-thisp1(0)); 
      if(std::isnan(now_yaw))   bsline_list(non_zero_line-1,2)= pre_yaw;    //yaw
      else                      bsline_list(non_zero_line-1,2)= now_yaw;    //yaw      
      bsline_list(non_zero_line-1,3)= sum_s/car_cont_time_step;//v
      temp_delta_yaw = now_yaw-pre_yaw;
      if(temp_delta_yaw>M_PI)temp_delta_yaw = temp_delta_yaw-2*M_PI;
      else if(temp_delta_yaw<-M_PI)temp_delta_yaw = temp_delta_yaw+2*M_PI;
      temp_w= temp_delta_yaw/car_cont_time_step;
      if(temp_w>2)temp_w=2;
      else if(temp_w<-2)temp_w=-2;
      bsline_list(non_zero_line-1,4)= temp_w;//w
      pre_yaw = now_yaw;      
      sum_s = 0;  
      if(non_zero_line==bsline_list.rows())break;    
   }
   else
   {
     sum_s+= this_s;
   }
  }
  if((sum_s>0)&&(non_zero_line<bsline_list.rows()))
  {
   thisp1 = evaluateDeBoor(st_u);
   thisp2 = evaluateDeBoor(u_max);
   non_zero_line++;
   bsline_list(non_zero_line-1,0)= thisp1(0);  //x   
   bsline_list(non_zero_line-1,1)= thisp1(1);  //y   
   now_yaw = std::atan2(thisp2(1)-thisp1(1),thisp2(0)-thisp1(0)); 
   if(std::isnan(now_yaw))   bsline_list(non_zero_line-1,2)= pre_yaw;    //yaw
   else                      bsline_list(non_zero_line-1,2)= now_yaw;    //yaw
   bsline_list(non_zero_line-1,3)= sum_s/car_cont_time_step;//v
   bsline_list(non_zero_line-1,4)= (now_yaw-pre_yaw)/car_cont_time_step;//w
   pre_yaw = now_yaw;      
   sum_s = 0;      
  }
  if(non_zero_line==0)
  {
  while(1)
  {
  std::cout<<"?00"<<std::endl;
  }
  } 
  return;
}
void NonUniformBspline::CutControlPoint()
{
  //找到从头开始的具有观测效应的最长控制点序列
  int all_len = control_points_.rows();
  bool valid_start_flag = 0;
  bool get_cut = false;
  int invalid_anchor = 0;
  if(all_len<20) return;

  bool mask_v[4];
  mask_v[0]=check_close_enough(0);
  mask_v[1]=check_close_enough(1);
  mask_v[2]=check_close_enough(2);
  mask_v[3]=check_close_enough(3);

  for(int i=4;i<all_len;i++)
  {
    mask_v[0]=mask_v[1];    mask_v[1]=mask_v[2];    mask_v[2]=mask_v[3];
    mask_v[3] = check_close_enough(i);

    if(!valid_start_flag)
    {//假如最长控制序列还没有开始
      if(mask_v[0]&&mask_v[1]&&mask_v[2]&&mask_v[3])
      valid_start_flag = true;//连续三个符合条件
    }
    else//假如最长控制序列已经开始了
    {
      if((!mask_v[0])&&(!mask_v[1])&&(!mask_v[2])&&(!mask_v[3]))
      {
        invalid_anchor = i;
        get_cut = true;
        break;
      } 
    }
  }
  if(!get_cut)invalid_anchor = all_len-1;
  else
  {
  invalid_anchor -= 8;
  invalid_anchor = std::max(10,invalid_anchor);
  }
  Eigen::MatrixXd  temp_X= control_points_.block(0,0,invalid_anchor,2);
  Eigen::MatrixXd  temp_Y= refine_contr_list.block(0,0,invalid_anchor,2);
  control_points_.setZero(invalid_anchor+1,2);
  refine_contr_list.setZero(invalid_anchor+1,2);//cut后的优化结果
  control_points_ = temp_X;
  refine_contr_list = temp_Y;
  if(get_cut)  Publishspecific();//如果这里cut, 就单独对优化结果可视化
}    
// fro_voro.getClosestObs(x(0),x(1),ox,oy); //从voronoi图中得到距离当前位置最近的障碍物
bool NonUniformBspline::check_close_enough(int ii)
{
  float wx=0;  float wy=0;
  fro_voro.getClosestObs(control_points_(ii,0),control_points_(ii,1),wx,wy);
  if(std::sqrt((control_points_(ii,0)-wx)*(control_points_(ii,0)-wx)+(control_points_(ii,1)-wy)*(control_points_(ii,1)-wy))<=2)
  return true;
  return false;
}

void NonUniformBspline::Publishspecific()
{
  int n_cp=refine_contr_list.rows();
  int n_pt=u_.size();
  geometry_msgs::PoseStamped thisp;
  thisp.header.frame_id = "map";
                nav_msgs::Path control_path;
                control_path.header.frame_id = "map";
                control_path.poses.clear();
                for(int i=0;i<n_cp;i++)
                {
                thisp.pose.position.x=refine_contr_list.row(i)(0);
                thisp.pose.position.y=refine_contr_list.row(i)(1);
                thisp.pose.position.z= 0.5 ;
                control_path.poses.push_back(thisp);
                }
                showspeci.publish(control_path);
}

//将优化前的控制点control_points_和优化后的控制点refine_contr_list可视化。
void NonUniformBspline::PublishBspline(bool fron_opt)
{
  int n_cp=control_points_.rows();
  int n_pt=u_.size();
  geometry_msgs::PoseStamped thisp;
  thisp.header.frame_id = "map";
                                              nav_msgs::Path control_path;
                                              control_path.header.frame_id = "map";
                                              control_path.poses.clear();

                                              for(int i=0;i<n_cp;i++)
                                              {
                                              thisp.pose.position.x=control_points_.row(i)(0);
                                              thisp.pose.position.y=control_points_.row(i)(1);
                                              control_path.poses.push_back(thisp);
                                              }
                                              showcontrp.publish(control_path);

                                              nav_msgs::Path control_path_opt;
                                              control_path_opt.header.frame_id = "map";
                                              control_path_opt.poses.clear();
                                              for(int i=0;i<n_cp;i++)
                                              {
                                              thisp.pose.position.x=refine_contr_list.row(i)(0);
                                              thisp.pose.position.y=refine_contr_list.row(i)(1);
                                              control_path_opt.poses.push_back(thisp);
                                              }

  if(fron_opt)showcontrp_smf.publish(control_path_opt);//smooth-obs-front
  else showcontrp_smo.publish(control_path_opt);//smooth-obs
 
}

NonUniformBspline::~NonUniformBspline() {}

void NonUniformBspline::InitialSpline(const int& order, const double& interval) 
{
  p_              = order;//3
  interval_       = interval;
  n_ = control_points_.rows() - 1;
  m_ = n_ + p_ + 1;
  //t0 = -p_ * interval=-3*intervel;
  //t1 = -2*intervel;
  //t2 = -1*intervel;
  //t3 = 0;即tp = 0;
  u_ = Eigen::VectorXd::Zero(m_ + 1);
  for (int i = 0; i <= m_; ++i) 
  {
    if (i <= p_-1)u_(i) = 0;//0 1 2 开始三个值必须一致
    else if (i > p_-1 && i <= m_ - p_) 
    u_(i) = u_(i - 1) + interval_;
    else u_(i) = u_(i - 1); //结尾三个的值必须一致
  }
  // for (int i = 0; i <= m_; ++i) 
  // {
  //   if (i <= p_) 
  //   {
  //     u_(i) = double(-p_ + i) * interval_;
  //   } else if (i > p_ && i <= m_ - p_) {
  //     u_(i) = u_(i - 1) + interval_;
  //   } else if (i > m_ - p_) {
  //     u_(i) = u_(i - 1) + interval_;
  //   }
  // }
}

//https://en.wikipedia.org/wiki/De_Boor%27s_algorithm
//这里使用德布尔算法得到b-spline曲线的值.
//输入:Bspline的参数u 输出:三维空间中一个点的坐标
Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double& u) 
{
  double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));// 确保u在u_{p},u_{m_-p_}这两个数值之间，因为u_是递增节点序列
  int k = p_;//次数3
  while (true) {
    if (u_(k + 1) >= ub) break;
    ++k;
  }//从u_{4}开始找找到输入u所在区间的右手端点，可见输入u落在区间u_(k),u(k+1)中

  /* deBoor's alg */
  std::vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    // d.push_back(control_points_.row(k - p_ + i));
    // cout << d[i].transpose() << endl;
    d.push_back(refine_contr_list.row(k - p_ + i));
  }
  // 内循环通过递归公式求得高一阶的P+1个控制点，外循环提高阶数。
  for (int r = 1; r <= p_; ++r) {
    for (int i = p_; i >= r; --i) 
    {
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // cout << "alpha: " << alpha << endl;
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }
  return d[p_];
}// [ t p , t m − p ] [t_p,t_{m-p}] [tp​,tm−p​]


//建议：直接使用Bspline曲线，对控制点的平滑、碰撞、观测三个项进行优化，在计算加速度和速度的时候根据曲线的长度(积分式)来进行插值，这样线速度就可以设定为恒定。
//传入的是世界坐标系。
void NonUniformBspline::InitControl_list(std::vector<std::vector<geometry_msgs::PoseStamped>>* allcontrol_point)
{
  //这里其实可以先做一步的平滑和优化。但是这里采用直接取值的方式。
  // std::uniform_int_distribution<unsigned> u(0,9);
  // srand( (unsigned)time( NULL ) ); 
  // float xita = 0;
  // float di_r = 2;
  // float rs = 0.16;
  // int l = int(2*M_PI_2/rs)-1;//实际上的循环应该是l+1次
  // control_points_.setZero(l,2); 
  // for(int i =0;i<=l;i++)
  // {
  //   float j= rand()%10;
  //   if(j==0)j=1;
  //   j = j/10.0;
  //   xita = i*rs-j*rs;
  //   float xx = di_r * std::cos(xita);
  //   float yy = di_r * std::sin(xita);
  //   // control_points_(int(xita/rs),0)=xx;
  //   // control_points_(int(xita/rs),1)=yy;
  //   control_points_(int(xita/rs),0)=xita+7;
  //   control_points_(int(xita/rs),1)=xita+7;
  // }
  int cct = 0;
  for(int i = 0;i<allcontrol_point->size();i++)
  cct = cct + (*allcontrol_point)[i].size();
  control_points_.setZero(cct,2);
  cct = 0;
  for(int i = 0;i<allcontrol_point->size();i++)
  for(int j = 0;j<(*allcontrol_point)[i].size();j++)
  {
   control_points_(cct,0)=(*allcontrol_point)[i][j].pose.position.x;
   control_points_(cct,1)=(*allcontrol_point)[i][j].pose.position.y;
   cct++;
  } 
  std::cout<<"总控制点数量为 "<<cct<<std::endl; 
}

void NonUniformBspline::InitialPara()
{
  nh.param<float>("Bspline_param/opt_time", opt_time, 0.1);//优化时间
  nh.param<int>("Bspline_param/opt_iter", opt_iter, 10);//优化变量
  nh.param<bool>("Bspline_param/use_frontier", use_frontier, false);  
  nh.param<bool>("Bspline_param/use_cut", use_cut,false);  

  nh.param<float>("Bspline_param/smo_weight", smo_weight, 0);//优化权重
  nh.param<float>("Bspline_param/vor_weight", vor_weight, 0);//优化权重
  nh.param<float>("Bspline_param/cur_weight", cur_weight, 0);//优化权重
  nh.param<float>("Bspline_param/fro_weight", fro_weight, 0);//优化权重
  nh.param<float>("Bspline_param/obs_optthr", obs_optthr, 0);
  nh.param<float>("Bspline_param/relax_x", relax_x, 0.01);
  nh.param<float>("Bspline_param/gra_weight_sum", gra_weight_sum, 0);  
  nh.param<float>("Bspline_param/observe_optdis", observe_optdis, 0);  
  nh.param<float>("Bspline_param/obst_safe_dis", obst_safe_dis, 0);  
  nh.param<float>("Bspline_param/car_control_dis_step", car_control_dis_step, 0.1);  
  nh.param<float>("Bspline_param/car_cont_time_step", car_cont_time_step, 0.1);  
} 

inline Eigen::Vector2d NonUniformBspline::conjugate(const Eigen::Vector2d &a,const Eigen::Vector2d &b)
{
  Eigen::Vector2d conju_pro;
  conju_pro = a - (a.dot(b)) * b / b.norm() / b.norm();
  return conju_pro;
}

inline Eigen::Vector2d NonUniformBspline::SmoothGradTerm(const Eigen::Vector2d &p_s2,const Eigen::Vector2d &p_s1,const Eigen::Vector2d &p_n,const Eigen::Vector2d &p_a1,const Eigen::Vector2d &p_a2)
{//平滑项的梯度应该减
  Eigen::Vector2d correction;
  correction = (p_s2 - 4 * p_s1 + 6 * p_n - 4 * p_a1 + p_a2);
  return correction;
}
inline Eigen::Vector2d NonUniformBspline::CurvatureGradTerm(const Eigen::Vector2d &p_s1,const Eigen::Vector2d &p_n,const Eigen::Vector2d &p_a1,double &cost)
{//曲率项的梯度应该减
  Eigen::Vector2d del_xi;  Eigen::Vector2d del_xip;
  Eigen::Vector2d ret_grad;ret_grad.setZero();
  del_xi = p_n - p_s1;  del_xip = p_a1 - p_n;
  double temp_v = del_xi.dot(del_xip) / (del_xi.norm()*del_xip.norm());
  if(temp_v>1)return ret_grad;
  double del_fai = std::acos( temp_v );
  double cos_del_fai = std::cos(del_fai);
  double sqrt_1_cos2del_fai  = std::sqrt(1-cos_del_fai*cos_del_fai);
  if(sqrt_1_cos2del_fai==0) return ret_grad;
  double del_xi_norm = del_xi.norm();
  if(del_xi_norm==0) return ret_grad;
  double del_xip_norm = del_xip.norm();
  Eigen::Vector2d p1 = conjugate(del_xi,del_xip*(-1));
  p1 = p1 / del_xi.norm() / del_xip.norm();
  Eigen::Vector2d p2 = conjugate(del_xip*(-1),del_xi);
  p2 = p2 / del_xi.norm() / del_xip.norm();

  Eigen::Vector2d deri_Ki_xi = (0-del_fai) * del_xi / del_xi_norm / del_xi_norm / del_xi_norm  - 1 / del_xi_norm / sqrt_1_cos2del_fai * (-p1-p2);
  Eigen::Vector2d deri_Ki_xs =    del_fai  * del_xi / del_xi_norm / del_xi_norm / del_xi_norm  - 1 / del_xi_norm / sqrt_1_cos2del_fai * p2;
  Eigen::Vector2d deri_Ki_xp =                                                                  -1 / del_xi_norm / sqrt_1_cos2del_fai  * p1;  

  ret_grad = 0.25*deri_Ki_xs+0.25*deri_Ki_xp+0.5*deri_Ki_xi;
  cost = del_fai/del_xi_norm;
  return ret_grad;
}
inline Eigen::Vector2d NonUniformBspline::FroGradTerm(const Eigen::Vector2d &x,double &cost)
{
 //损失函数(|x-f_i|-observe_optdis)*(|x-f_i|-observe_optdis)
 //根据向量求导法则，对x求导，结果为2*(|x-f_i|-observe_optdis) * (x-f_i) / |x-f_i|
 //更新的时候减去梯度
  Eigen::Vector2d ret_grad;
  Eigen::Vector2d f_i;//最近障碍物，世界坐标
  float ox,oy;
  fro_voro.getClosestObs(x(0),x(1),ox,oy); //从voronoi图中得到距离当前位置最近的障碍物
  f_i<<ox,oy;
  Eigen::Vector2d del_x = x-f_i;//最近障碍物指向自身的向量
  double dis = del_x.norm();//最近障碍物到自身的距离——真实距离，以m为单位
  // cost = dis-observe_optdis;
  // cost *=cost;
  ret_grad = 2.0*(dis-observe_optdis) * del_x / dis; //障碍物距离减更新阈值
  if(std::isnan(ret_grad(0))||std::isnan(ret_grad(1))){ret_grad.setZero(); std::cout<<"front grad nan"<<std::endl;}
  // std::cout<<observe_optdis<<std::endl;
  return ret_grad;
}

inline Eigen::Vector2d NonUniformBspline::ObsGradTerm(const Eigen::Vector2d &x,double &cost)
{//损失函数(|x-o_i|-d_max)*(|x-o_i|-d_max)
 //当|x-o_i|<d_max时，为0.
 //根据向量求导法则，对x求导，结果为2*(|x-o_i|-d_max) * (x-o_i) / |x-o_i|
 //更新的时候减去梯度
  Eigen::Vector2d ret_grad;
  Eigen::Vector2d o_i;//最近障碍物，世界坐标
  float ox,oy;
  obs_voro.getClosestObs(x(0),x(1),ox,oy); //从voronoi图中得到距离当前位置最近的障碍物
  o_i<<ox,oy;
  Eigen::Vector2d del_x = x-o_i;//最近障碍物指向自身的向量
  double dis = del_x.norm();//最近障碍物到自身的距离——真实距离，以m为单位
  // std::cout<<dis <<std::endl;
  if(dis>obs_optthr){ret_grad.setZero();cost = 0;return ret_grad;}
  cost = dis-obs_optthr;cost *=cost;
  ret_grad = 2.0*(dis-obs_optthr) * del_x / dis; //障碍物距离减更新阈值
  return ret_grad;
}

bool NonUniformBspline::PrepareObsVoronoi(nav_msgs::OccupancyGrid* lcpmap2d)
{
    obs_voro.initializeMap(lcpmap2d,100);//这个检测障碍物    lcpmap2d指针在内部不会被保留
    obs_voro.update(true);
    fro_voro.initializeMap(lcpmap2d,20);//这个检测frontier  lcpmap2d指针在内部不会被保留
    fro_voro.update(true);
    nav_msgs::OccupancyGrid tempSDF;
    fro_voro.getsdf(&tempSDF);
    sdfmap_pub.publish(tempSDF);
    lcpmap2d_ = lcpmap2d;//保留指针是为了优化函数中的使用
    std::cout<<"voro 准备完毕"<<std::endl;
    return true;
}






bool NonUniformBspline::Optimize_nlopt(bool fron_opt)
{ //该函数的调用要先使用PrepareObsVoronoi
  int opt_row = control_points_.rows()-2;//头部和尾部不进行优化
  dim_ = control_points_.cols();
  p_  =3;
  if(opt_row<0)while(1)std::cout<<"bspline control point length is less than 2, illegal!!!"<<std::endl;
  else if(opt_row==0)
  {
      refine_contr_list = control_points_;
      Freemaps();//释放内存空间
      return true;
  }
  Hybrid_data data_bag;
  data_bag.contr_list = &control_points_;
  data_bag.fro_voro = &fro_voro;
  data_bag.obs_voro = &obs_voro;
  data_bag.temp_use_pointer_lcpmap2d = lcpmap2d_;
  data_bag.opt_front_flag = fron_opt;
  nlopt::opt opt_ct(nlopt::algorithm(15),opt_row*2);//优化器初始化
  opt_ct.set_min_objective(NonUniformBspline::costFunction,&data_bag); //参数：优化函数，在优化过程中需要的变量
  opt_ct.set_maxeval(opt_iter);//优化次数
  opt_ct.set_maxtime(opt_time);
  opt_ct.set_xtol_rel(relax_x);//当dx / x 小于该数值，优化停止

  std::vector<double> initial_x;//被优化的控制点
  std::vector<double> lb_x;
  std::vector<double> ub_x;
  double thd=0;
  for(int i=1;i<opt_row+1;i++)
  for(int j=0;j<dim_;j++)
  {
    thd = control_points_(i,j);
    initial_x.push_back(thd);
    lb_x.push_back(thd-0.75);
    ub_x.push_back(thd+0.75);
  }
  opt_ct.set_lower_bounds(lb_x);
  opt_ct.set_upper_bounds(ub_x);

  // for(int j=0;j<ub_x.size();j++)
  // std::cout<<lb_x[j]<<" ";
  // std::cout<<std::endl;
  // for(int j=0;j<ub_x.size();j++)
  // std::cout<<ub_x[j]<<" ";
  // std::cout<<std::endl;


  // std::cout<<"Before"<<std::endl;
  // for(int i=0;i<initial_x.size();i++)std::cout<<initial_x[i]<<" ";
  // std::cout<<std::endl;
  // std::cout<<"Before"<<std::endl;
  // std::cout<<"After"<<std::endl;
  // for(int i=0;i<initial_x.size();i++)std::cout<<initial_x[i]<<" ";
  // std::cout<<std::endl;
  bool success_flag = true;
  try
  {
    double cost_r=0;
    nlopt::result this_result = opt_ct.optimize(initial_x,cost_r);
    std::cout<<"Finish optimization"<<std::endl;
  }
  catch(std::exception& e)
  {
    std::cout<<"nlopt exception: "<<e.what()<<std::endl;
    success_flag = false;
  }
  refine_contr_list = control_points_;//如果失败就保留上一次的控制点

  if(success_flag)
  for(int i=1;i<opt_row+1;i++)
  for(int j=0;j<dim_;j++)
  refine_contr_list(i,j) = initial_x[(i-1)*dim_+j];

  return success_flag;  
}

void NonUniformBspline::Freemaps()
{
  lcpmap2d_ = NULL;
  obs_voro.FreeAll();//释放内存空间
  fro_voro.FreeAll();
}
double NonUniformBspline::costFunction(const std::vector<double> &x, std::vector<double>& grad ,void* func_data)
{
  double grad_weight = 0.2;//平滑优化权重
  double fron_weight = 0.0;
  double obs_weight = 0.30;//障碍物优化权重
  double observe_optdis=0.5;//frontier最佳观测距离
  double danger_front_dist = 0.7;//具有高度差异的frontier危险距离。 0 <= danger_front_dist <= 2*observe_optdis
  double obst_safe_dis= 0.8;//obs优化范围
  Hybrid_data* hy_data = reinterpret_cast<Hybrid_data*>(func_data);
  Eigen::MatrixXd* conto_p_orig = hy_data->contr_list; //传入的是所有的控制点，并非是优化控制点
  DynamicVoronoi* obs_vo = hy_data->obs_voro;
  DynamicVoronoi* fro_vo = hy_data->fro_voro;
                                                  nav_msgs::OccupancyGrid* temp_elevation_map = hy_data->temp_use_pointer_lcpmap2d;  
                                                  float ele_org_x = temp_elevation_map->info.origin.position.x;
                                                  float ele_org_y = temp_elevation_map->info.origin.position.y;
                                                  float ele_width = temp_elevation_map->info.width;
                                                  float ele_height = temp_elevation_map->info.height;
                                                  float ele_resolu = temp_elevation_map->info.resolution;
  bool front_opt = hy_data->opt_front_flag;
  if(front_opt)    fron_weight = 0.4;//这个项的引入还是有效的
  Eigen::MatrixXd control_points_ = *conto_p_orig;
  int p_s = 3;//暂时处理为3阶
  int dim_s = control_points_.cols();
  int ct_pn = control_points_.rows();
  std::vector<double> gd_x;//控制点的展开
  gd_x.resize(control_points_.size(),0.0);
  int gd_size = x.size(); //优化变量的尺寸 
  for(int i =0 ;i<grad.size();i++)grad[i]=0.0;

  //头控制点不进行优化，直接取值
  for(int j=0;j<dim_s;j++) gd_x[0*dim_s+j]=control_points_(0,j);
  //头控制点不进行优化，直接取值

  //先对控制点进行赋值
  for(int i=0;i<x.size();i++)
  gd_x[1*dim_s+i]=x[i];  //对尾部进行优化,就是x包含了所有的除了头部的控制点
  //先对控制点进行赋值

  //尾巴的这几个控制点不进行优化
  for(int j=0;j<dim_s;j++)gd_x[dim_s*1 + x.size() + j] = control_points_(ct_pn-1,j);        
  //尾巴的这几个控制点不进行优化

//先对控制点进行赋值
//可以开始利用gd_x计算梯度了
//被优化的控制点的索引范围是  p_   ct_pn-p_-1
//因此 根据优化项 |q_(i+1) - q_i - (q_i-q_(i-1))|^2
//求解对q_(i-1)的梯度 2q_(i-1)+2q_(i+1)-4q_i
//求解对q_(i)的梯度 8q_i-4q_(i+1)-4q_(i-1)
//求解对q_(i+1)的梯度 2q_(i+1)-4q_i+2q_(i-1)
//涉及被优化控制点地图的求解范围： p_ - 1   ct_pn-1-p_+1

  double return_cost=0;
  double cost_this = 0;
                            Eigen::Vector2d p_is1;//pre in world corr
                            Eigen::Vector2d p_i;//now in world corr
                            Eigen::Vector2d p_ip1;//next in world corr
  p_is1.setZero(dim_s);  p_i.setZero(dim_s);  p_ip1.setZero(dim_s);
  Eigen::Vector2d g_s1,g_1,g_p1;//pre now next
  g_s1.setZero(dim_s);  g_1.setZero(dim_s);  g_p1.setZero(dim_s);
  int start_idx = 1;  int end_idx = ct_pn-1-1;//第一个和最后一个不进行优化
  //start 是被优化节点的第一个，end是被优化节点的最后一个
  for(int i = start_idx;i<=end_idx;i++)
  {
    for(int c=0;c<dim_s;c++)
    {
      p_is1(c)=gd_x[(i-1)*dim_s+c];   
      p_i(c)=gd_x[i*dim_s+c];   
      p_ip1(c)=gd_x[(i+1)*dim_s+c];   
      g_s1 =( 2.0*p_is1-4.0*p_i+2.0*p_ip1)*grad_weight;
      g_1 = (-4.0*p_is1+8.0*p_i-4.0*p_ip1)*grad_weight;
      g_p1 =( 2.0*p_is1-4.0*p_i+2.0*p_ip1)*grad_weight;
      cost_this=(p_ip1+p_is1-2*p_i).norm();
      return_cost+= cost_this*cost_this*grad_weight;
    }//拿到三个点各自的梯度
    if(i==start_idx)
    {
      for(int cct=0;cct<dim_s;cct++)
      grad[cct] += g_1(cct);
      for(int cct=0;cct<dim_s;cct++)
      grad[dim_s+cct] += g_p1(cct);
    }
    else if(i==end_idx)
    {
      int thisd = gd_size-2*dim_s;
      for(int cct=0;cct<dim_s;cct++)
      grad[cct+thisd] += g_s1(cct);
      thisd = gd_size-dim_s;
      for(int cct=0;cct<dim_s;cct++)
      grad[cct+thisd] += g_1(cct);
    }
    else
    {
      int thisidx = i-start_idx;
      for(int cct=0;cct<dim_s;cct++)
      {
        grad[(thisidx-1)*dim_s+cct] += g_s1(cct);
        grad[thisidx*dim_s+cct] += g_1(cct);
        grad[(thisidx+1)*dim_s+cct] += g_p1(cct);
      }
      //这里对曲率进行优化
      // double cur_loss = 0;
      // if(CurvatureGradTerm(p_is1,p_i,p_ip1,&g_s1,&g_1,&g_p1,cur_loss))
      // for(int cct=0;cct<dim_;cct++)
      // {
      //   grad[(thisidx-1)*dim_+cct] += g_s1(cct);
      //   grad[thisidx*dim_+cct] += g_1(cct);
      //   grad[(thisidx+1)*dim_+cct] += g_p1(cct);
      // }
      // return_cost+=cur_loss;
      //这里对曲率进行优化
    }



  //注意，frontier梯度场和 obs梯度场采取的形式是一样的。都是让障碍物/frontier保证在一定范围内，二次函数的形式。
 //损失函数(|x-f_i|-observe_optdis)*(|x-f_i|-observe_optdis)
 //根据向量求导法则，对x求导，结果为2*(|x-f_i|-observe_optdis) * (x-f_i) / |x-f_i|
 //更新的时候减去梯度
                                    int now_p_ele_index = int((p_i(0)-ele_org_x)/ele_resolu) + int((p_i(1)-ele_org_y)/ele_resolu) * ele_width;//当前路径点在elevationmap中的index
                                    float now_p_ele_h = temp_elevation_map->data[now_p_ele_index];//当前路径点的高程
  Eigen::Vector2d ret_grad;
  Eigen::Vector2d f_i;//最近障碍物，世界坐标
  float ox,oy;
  Eigen::Vector2d del_x;
  double dis;
  //Frontier项梯度更新
  bool fron_valid = fro_vo->getClosestObs(p_i(0),p_i(1),ox,oy); //从voronoi图中得到距离当前位置最近的frontier
  if(fron_valid)
  {
                                    int now_o_ele_index = int((ox-ele_org_x)/ele_resolu) + int((oy-ele_org_y)/ele_resolu) * ele_width;//最近frontier在elevationmap中的index
                                    float now_o_ele_h = temp_elevation_map->data[now_o_ele_index];//最近frontier的高程

                              f_i<<ox,oy;
                              del_x = p_i-f_i;//最近障碍物指向自身的向量
                              dis = del_x.norm();//最近障碍物到自身的距离——真实距离，以m为单位
                              if(dis<=observe_optdis*2.5)//只有当距离足够近的时候才会触发frontier梯度。
                              {
                                if(std::abs(now_p_ele_h-now_o_ele_h)<=0.3)//高度差不能太大
                                {
                                          //二次函数只取2边，所以是非单调的
                                    ret_grad = 2.0*(dis-observe_optdis) * del_x / dis; //障碍物距离减更新阈值
                                    int thisidx = i-start_idx;
                                    if(std::isnan(ret_grad(0))||std::isnan(ret_grad(1)))
                                    return_cost+=(dis-observe_optdis)*(dis-observe_optdis)*fron_weight;
                                    else
                                    {
                                      for(int cct=0;cct<dim_s;cct++)
                                      grad[thisidx*dim_s+cct]+=ret_grad[cct]*fron_weight;
                                      return_cost+=(dis-observe_optdis)*(dis-observe_optdis)*fron_weight;
                                    }  
                                }
                                else//如果高度差太大就将路径点从附近推开
                                {
                                          //二次函数只取半边，所以是单调的
                                          if((0<dis)&&(dis<danger_front_dist))
                                          {
                                              ret_grad = 2.0*(dis-danger_front_dist) * del_x / dis; //障碍物距离减更新阈值
                                              int thisidx = i-start_idx;
                                              if(std::isnan(ret_grad(0))||std::isnan(ret_grad(1)))
                                              return_cost+=(dis-danger_front_dist)*(dis-danger_front_dist)*fron_weight;
                                              else
                                              {
                                                for(int cct=0;cct<dim_s;cct++)
                                                grad[thisidx*dim_s+cct]+=ret_grad[cct]*fron_weight;
                                                return_cost+=(dis-danger_front_dist)*(dis-danger_front_dist)*fron_weight;
                                              }  
                                          }
                                }
                              }
  }
  //Frontier项梯度更新

  //Obstacle项梯度更新
  bool obs_valid = obs_vo->getClosestObs(p_i(0),p_i(1),ox,oy);//voronoi图中得到最近的障碍物
  if(obs_valid)
  {
                                f_i<<ox,oy;
                                del_x = p_i-f_i;//最近障碍物指向自身的向量
                                dis = del_x.norm();//最近障碍物到自身的距离——真实距离，以m为单位
                                ret_grad.setZero();
                                          //二次函数只取半边，所以是单调的
                                if((0<dis)&&(dis<obst_safe_dis))
                                { //梯度有效
                                  ret_grad = 2.0*(dis-obst_safe_dis) * del_x / dis; //障碍物距离减更新阈值  
                                  // ret_grad = -1.0 * del_x / dis / dis /dis; //障碍物距离减更新阈值  
                                  int thisidx = i-start_idx;
                                  if(std::isnan(ret_grad(0))||std::isnan(ret_grad(1)))
                                  return_cost += (dis-obst_safe_dis)*(dis-obst_safe_dis)*obs_weight;//梯度无效，代价有效
                                  else
                                  {
                                    // if(ret_grad.norm()>10.0)ret_grad = ret_grad/4.0;
                                    for(int cct=0;cct<dim_s;cct++)//梯度有效，代价有效
                                    grad[thisidx*dim_s+cct]+=ret_grad[cct]*obs_weight;
                                    return_cost += (dis-obst_safe_dis)*(dis-obst_safe_dis)*obs_weight;
                                  }
                                }
                                else if(dis>obst_safe_dis){return_cost+=0;}//安全
                                else{return_cost+=300;}//不合法
  }
  //Obstacle项梯度更新
  }
    // std::cout<<"Grad info "<<std::endl;
    // for(int i =0 ;i<grad.size();i++)
    // {std::cout<<grad[i]<<" ";} 
    // std::cout<<std::endl;
    // std::cout<<"Grad info "<<std::endl;
    // std::cout<<"x info "<<std::endl;
    // for(int i =0 ;i<grad.size();i++)
    // {std::cout<<x[i]<<" ";} 
    // std::cout<<std::endl;
    // std::cout<<"x info "<<std::endl;
  return return_cost;
}



double NonUniformBspline::costFunction_smooth_fro_only(const std::vector<double> &x, std::vector<double>& grad ,void* func_data)
{
  double grad_weight = 0.6;
  double fron_weight = 0.2;
  double obs_weight = 0.2;
  double observe_optdis=0.5;
  double obst_safe_dis= 0.5;
  Hybrid_data* hy_data = reinterpret_cast<Hybrid_data*>(func_data);
  Eigen::MatrixXd* conto_p_orig = hy_data->contr_list; //传入的是所有的控制点，并非是优化控制点
  DynamicVoronoi* obs_vo = hy_data->obs_voro;
  DynamicVoronoi* fro_vo = hy_data->fro_voro;

  Eigen::MatrixXd control_points_ = *conto_p_orig;
  int p_s = 3;//暂时处理为3阶
  int dim_s = control_points_.cols();
  int ct_pn = control_points_.rows();
  std::vector<double> gd_x;//控制点的展开
  gd_x.resize(control_points_.size(),0.0);
  int gd_size = x.size(); //优化变量的尺寸 
  for(int i =0 ;i<grad.size();i++)grad[i]=0.0;

  //头控制点不进行优化，直接取值
  for(int j=0;j<dim_s;j++) gd_x[0*dim_s+j]=control_points_(0,j);
  //头控制点不进行优化，直接取值

  //先对控制点进行赋值
  for(int i=0;i<x.size();i++)
  gd_x[1*dim_s+i]=x[i];  //对尾部进行优化,就是x包含了所有的除了头部的控制点
  //先对控制点进行赋值

  //尾巴的这几个控制点不进行优化
  for(int j=0;j<dim_s;j++)gd_x[dim_s*1 + x.size() + j] = control_points_(ct_pn-1,j);        
  //尾巴的这几个控制点不进行优化

//先对控制点进行赋值
//可以开始利用gd_x计算梯度了
//被优化的控制点的索引范围是  p_   ct_pn-p_-1
//因此 根据优化项 |q_(i+1) - q_i - (q_i-q_(i-1))|^2
//求解对q_(i-1)的梯度 2q_(i-1)+2q_(i+1)-4q_i
//求解对q_(i)的梯度 8q_i-4q_(i+1)-4q_(i-1)
//求解对q_(i+1)的梯度 2q_(i+1)-4q_i+2q_(i-1)
//涉及被优化控制点地图的求解范围： p_ - 1   ct_pn-1-p_+1

  double return_cost=0;
  double cost_this = 0;
  Eigen::Vector2d p_is1,p_i,p_ip1;//pre now next
  p_is1.setZero(dim_s);  p_i.setZero(dim_s);  p_ip1.setZero(dim_s);
  Eigen::Vector2d g_s1,g_1,g_p1;//pre now next
  g_s1.setZero(dim_s);  g_1.setZero(dim_s);  g_p1.setZero(dim_s);
  int start_idx = 1;  int end_idx = ct_pn-1-1;//第一个和最后一个不进行优化
  //start 是被优化节点的第一个，end是被优化节点的最后一个
  for(int i = start_idx;i<=end_idx;i++)
  {
    for(int c=0;c<dim_s;c++)
    {
      p_is1(c)=gd_x[(i-1)*dim_s+c];   
      p_i(c)=gd_x[i*dim_s+c];   
      p_ip1(c)=gd_x[(i+1)*dim_s+c];   
      g_s1 =( 2.0*p_is1-4.0*p_i+2.0*p_ip1)*grad_weight;
      g_1 = (-4.0*p_is1+8.0*p_i-4.0*p_ip1)*grad_weight;
      g_p1 =( 2.0*p_is1-4.0*p_i+2.0*p_ip1)*grad_weight;
      cost_this=(p_ip1+p_is1-2*p_i).norm();
      return_cost+= cost_this*cost_this*grad_weight;
    }//拿到三个点各自的梯度
    if(i==start_idx)
    {
      for(int cct=0;cct<dim_s;cct++)
      grad[cct] += g_1(cct);
      for(int cct=0;cct<dim_s;cct++)
      grad[dim_s+cct] += g_p1(cct);
    }
    else if(i==end_idx)
    {
      int thisd = gd_size-2*dim_s;
      for(int cct=0;cct<dim_s;cct++)
      grad[cct+thisd] += g_s1(cct);
      thisd = gd_size-dim_s;
      for(int cct=0;cct<dim_s;cct++)
      grad[cct+thisd] += g_1(cct);
    }
    else
    {
      int thisidx = i-start_idx;
      for(int cct=0;cct<dim_s;cct++)
      {
        grad[(thisidx-1)*dim_s+cct] += g_s1(cct);
        grad[thisidx*dim_s+cct] += g_1(cct);
        grad[(thisidx+1)*dim_s+cct] += g_p1(cct);
      }
      //这里对曲率进行优化
      // double cur_loss = 0;
      // if(CurvatureGradTerm(p_is1,p_i,p_ip1,&g_s1,&g_1,&g_p1,cur_loss))
      // for(int cct=0;cct<dim_;cct++)
      // {
      //   grad[(thisidx-1)*dim_+cct] += g_s1(cct);
      //   grad[thisidx*dim_+cct] += g_1(cct);
      //   grad[(thisidx+1)*dim_+cct] += g_p1(cct);
      // }
      // return_cost+=cur_loss;
      //这里对曲率进行优化
    }




 //损失函数(|x-f_i|-observe_optdis)*(|x-f_i|-observe_optdis)
 //根据向量求导法则，对x求导，结果为2*(|x-f_i|-observe_optdis) * (x-f_i) / |x-f_i|
 //更新的时候减去梯度
  Eigen::Vector2d ret_grad;
  Eigen::Vector2d f_i;//最近障碍物，世界坐标
  float ox,oy;

  //Frontier项梯度更新
  fro_vo->getClosestObs(p_i(0),p_i(1),ox,oy); //从voronoi图中得到距离当前位置最近的frontier
  f_i<<ox,oy;
  Eigen::Vector2d del_x = p_i-f_i;//最近障碍物指向自身的向量
  double dis = del_x.norm();//最近障碍物到自身的距离——真实距离，以m为单位
  if(!(dis>observe_optdis*2))
  {
    ret_grad = 2.0*(dis-observe_optdis) * del_x / dis; //障碍物距离减更新阈值
    int thisidx = i-start_idx;
    if(std::isnan(ret_grad(0))||std::isnan(ret_grad(1)))
    return_cost+=(dis-observe_optdis)*(dis-observe_optdis)*fron_weight;
    else
    {
      for(int cct=0;cct<dim_s;cct++)
      grad[thisidx*dim_s+cct]+=ret_grad[cct]*fron_weight;
      return_cost+=(dis-observe_optdis)*(dis-observe_optdis)*fron_weight;
    }  
  }
  //Frontier项梯度更新
  }
    // std::cout<<"Grad info "<<std::endl;
    // for(int i =0 ;i<grad.size();i++)
    // {std::cout<<grad[i]<<" ";} 
    // std::cout<<std::endl;
    // std::cout<<"Grad info "<<std::endl;
    // std::cout<<"x info "<<std::endl;
    // for(int i =0 ;i<grad.size();i++)
    // {std::cout<<x[i]<<" ";} 
    // std::cout<<std::endl;
    // std::cout<<"x info "<<std::endl;
  return return_cost;
}

//利用fast_planner的方法，将整体平滑项作为优化目标，效果不佳。
//综合考虑，不再采用fast_planner\ego_planner类似思路，而采用车辆相关的方法来解决问题。
//fast_planner：hybridA搜索路径+bspline拟合搜索结果+构造优化函数，对所有控制点统一优化。