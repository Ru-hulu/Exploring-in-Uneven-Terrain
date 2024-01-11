#include"mpc_control_node.hpp"
MPC_Control::MPC_Control(ros::NodeHandle &n):nh(n)
{
    showSUBS = nh.advertise<visualization_msgs::MarkerArray>("test_path",5);   
    robpos = nh.advertise<geometry_msgs::PoseStamped>("robotpos",5);   
    // cmd_v_w_pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel",5);
    cmd_v_w_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",5);
    mpc_server = nh.advertiseService("mpc_solver",&MPC_Control::MPC_Top_Handle,this);
    gz_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    Param_Init();
}
int inline MPC_Control::rotdir(float tar_yaw, float now_yaw)
{
  float del_yaw = tar_yaw-now_yaw;
  if(std::abs(del_yaw)<0.1) return 0;
  if(del_yaw>0)
  {
    if(del_yaw<M_PI)return 1;
    else return -1;
  }
  else
  {
    if(del_yaw>-M_PI)return -1;
    else return 1;
  }
}

void MPC_Control::Param_Init()
{
  nh.param<int>("mpc_param/Np", Np, 5);//优化时间
  nh.param<int>("mpc_param/Nc", Nc, 4);//优化变量
  nh.param<float>("mpc_param/delta_mpc_t", delta_mpc_t, 0.1);//时间步长
  nh.param<float>("mpc_param/max_v", max_v, 3);//优化权重
  nh.param<float>("mpc_param/min_v", min_v, -3);//优化权重
  nh.param<float>("mpc_param/max_w", max_w, 2);//优化权重
  nh.param<float>("mpc_param/min_w", min_w, -2);//优化权重
  nh.param<float>("mpc_param/temp_x", temp_x, -2);//优化权重
  nh.param<float>("mpc_param/temp_w", temp_w, -2);//优化权重
  U_max.resize(Nu*Nc+1,1);//带上松弛因子
  U_min.resize(Nu*Nc+1,1);//带上松弛因子
  for(int i =0;i<Nu*Nc;i++)
  {
      if(i%2==0)//赋值速度
      {
          U_max(i,0)=max_v;
          U_min(i,0)=min_v;
      }
      else//奇数赋值角速度
      {
          U_max(i,0)=max_w;
          U_min(i,0)=min_w;
      }
  }
  U_max(Nu*Nc,0)=1;
  U_min(Nu*Nc,0)=0;
  // Ur.resize(Nc*Nu+1,1);
  A_qua.resize(2*(Nu*Nc+1),(Nu*Nc+1));
  // A_qua.block<Nu*Nc+1,Nu*Nc+1>(0,0) = 
  //这里都是不变化的参数
}
void MPC_Control::BreakPointRotate(double target_yaw)
{
    if(std::isnan(target_yaw))target_yaw = 0;
    gazebo_msgs::GetModelState this_state;
    this_state.request.model_name="scout/";
    this_state.request.relative_entity_name="map";
    double tyaw = 0;
    double delta_rotate = 0;
    double ro_dir = 0;
    ros::Rate control_hz(10);
    while(ros::ok())
    {
        gz_client.call(this_state);
        tyaw = tf::getYaw(this_state.response.pose.orientation);//这里是车的yaw角
        std::cout<<"Break Point!!  target_yaw "<<target_yaw<<" now_yaw "<<tyaw<<std::endl;
        ro_dir = rotdir(target_yaw,tyaw);
        if(std::abs(ro_dir)<0.05)break;
        // geometry_msgs::TwistStamped this_cmd;
        geometry_msgs::Twist this_cmd;
        this_cmd.linear.x = 0;
        this_cmd.angular.z = ro_dir*0.5;
        cmd_v_w_pub.publish(this_cmd);
        control_hz.sleep();
    }
}
bool MPC_Control::MPC_Top_Handle(srvbg::mpcref::Request& req,srvbg::mpcref::Response& res)
{
    std::cout<<"In call back "<<std::endl;
    int sizelen = req.ref_x.size();
    path_len = sizelen;
    ref_x.clear();
    ref_y.clear();
    ref_yaw.clear();
    ref_v.clear();
    ref_w.clear();
    path_track_counter=0;
    A_qua.block<2*4+1,2*4+1>(0,0) = Eigen::MatrixXf::Identity(2*4+1,2*4+1);
    A_qua.block<2*4+1,2*4+1>(2*4+1,0) = Eigen::MatrixXf::Zero(2*4+1,2*4+1)-Eigen::MatrixXf::Identity(2*4+1,2*4+1);
    for(int i=0;i<sizelen;i++)
    {
        ref_x.push_back(req.ref_x[i]);
        ref_y.push_back(req.ref_y[i]);
        ref_yaw.push_back(req.ref_yaw[i]);
        ref_v.push_back(req.ref_v[i]);
        ref_w.push_back(req.ref_w[i]);
    }//参考轨迹初始化
    for(int i=0;i<Nc;i++)
    {
        ref_v.push_back(0.0);
        ref_w.push_back(0.0);
    }//由于MPC控制过程中需要未来控制的参考，所以这里要补充长度。
    now_carstate<<0,0,0;//当前状态初始化
    std::pair<double,double> now_control_cmd;
    double thi=ros::Time::now().toSec();
    geometry_msgs::PoseStamped rbp;
    rbp.header.frame_id = "map";
    gazebo_msgs::GetModelState this_state;
    this_state.request.model_name="scout/";
    this_state.request.relative_entity_name="map";
    double troll, tpitch, tyaw;
    geometry_msgs::Twist this_cmd;
    // geometry_msgs::TwistStamped this_cmd;
    // this_cmd.header.frame_id = "vehicle";
    int i=0;
    std::cout<<"In mpc "<<std::endl;
    ros::Rate control_hz(10);
    float delta_rotate = 10;
                                    gz_client.call(this_state);
                                    float now_x_temp=this_state.response.pose.position.x;
                                    float now_y_temp=this_state.response.pose.position.y;
                                    if(std::sqrt((now_x_temp-ref_x[0])*(now_x_temp-ref_x[0])+
                                    (now_y_temp-ref_y[0])*(now_y_temp-ref_y[0]))>0.75)
                                    {
                                    res.started = true;
                                    std::cout<<"now pos is "<< now_x_temp<<" "<<now_y_temp<<std::endl;
                                    std::cout<<"Start pos is "<< ref_x[0]<<" "<<ref_y[0]<<std::endl;
                                    std::cout<<"Start of trajectory is too far away from start point, not follow!!"<<std::endl;
                                    return true;//轨迹出发点和当前位置偏移太远，重新规划
                                    }

    BreakPointRotate(ref_yaw[0]);
    // while(ros::ok())
    // {
    //     gz_client.call(this_state);
    //     tyaw = tf::getYaw(this_state.response.pose.orientation);//这里是车的yaw角
    //     delta_rotate = ref_yaw[0]-tyaw;
    //     if(std::abs(delta_rotate)<0.01)break;

    //     this_cmd.twist.linear.x = 0;
    //     if(delta_rotate>0) this_cmd.twist.angular.z = 0.2;
    //     else this_cmd.twist.angular.z = -0.2;        

    //     cmd_v_w_pub.publish(this_cmd);        
    //     control_hz.sleep();
    // }
                                                                                    std::ofstream file_traj("/home/r/Mysoftware/Paper_Expriment_Simulation/Wheel_uneven/traj.txt",std::ios::app);
                                                                                    if (file_traj.is_open()); 
    double time1 = ros::Time::now().toSec();
    while(ros::ok()&&i<path_len)
    {
        gz_client.call(this_state);
        now_carstate(0,0)=this_state.response.pose.position.x;
        now_carstate(1,0)=this_state.response.pose.position.y;

                                                                                    double real_time_n = ros::WallTime::now().toSec();
                                                                                    file_traj << std::fixed << std::setprecision(3) << real_time_n;
                                                                                    file_traj << " ";
                                                                                    file_traj << std::fixed << std::setprecision(3) << now_carstate(0,0);
                                                                                    file_traj << " ";
                                                                                    file_traj << std::fixed << std::setprecision(3) << now_carstate(1,0);
                                                                                    file_traj << " ";
                                                                                    file_traj << "\n";
        tyaw = tf::getYaw(this_state.response.pose.orientation);//这里是车的yaw角
        now_carstate(2,0)=tyaw;
        now_control_cmd = MPC_Solve();//根据参考轨迹、状态信息，生成控制律
        this_cmd.linear.x = now_control_cmd.first;
        this_cmd.angular.z = now_control_cmd.second;
        cmd_v_w_pub.publish(this_cmd);
        i++;
        control_hz.sleep();
        double time2 = ros::Time::now().toSec();
        std::cout<<i<<" "<<path_len<<std::endl;
        std::cout<<"delta time is "<<time2-time1<<std::endl;
        time1 = time2;
        //这里把机器人当前的状态发布出去。
        rbp.pose.position = this_state.response.pose.position;
        rbp.pose.orientation = this_state.response.pose.orientation;
        robpos.publish(rbp);
    }
        this_cmd.linear.x = 0;
        this_cmd.angular.z = 0;
        cmd_v_w_pub.publish(this_cmd);        
    res.started = true;
                                                                                        file_traj.close();
    return true;
}

void MPC_Control::TEST()
{   
    Eigen::MatrixXd ref;
    std::ifstream file("/home/r/Mysoftware/TARE/1.txt");  // 打开要读取的文本文件
    double this_d = 0;
    if (file.is_open())
    {
        int rows=197;int cols=5;
        ref.setZero(rows, cols);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                file >> this_d;
                if((j==cols-1)&&std::abs(this_d)>2.0) this_d = ref(i-1,j);
                ref(i, j) = this_d;
            }
        }
        file.close();  // 关闭文件
    }

    gazebo_msgs::SetModelState set_state;
    set_state.request.model_state.reference_frame = "map";
    set_state.request.model_state.pose.position.x=ref(0,0);
    set_state.request.model_state.pose.position.y=ref(0,1);
    set_state.request.model_state.pose.position.z=0.0;
    geometry_msgs::Quaternion thiq= tf::createQuaternionMsgFromYaw(ref(0,2));
    set_state.request.model_state.pose.orientation=thiq;    
    set_state.request.model_state.model_name = "robot";
    

    gazebo_msgs::GetModelState this_state;
    this_state.request.model_name="robot";
    this_state.request.relative_entity_name="map";
    gz_client.call(this_state);
    std::cout<<"posx "<<this_state.response.pose.position.x<<" posy "<<this_state.response.pose.position.y;

    // std::cout<<ref<<std::endl;
    ref_x.clear();
    ref_y.clear();
    ref_yaw.clear();
    ref_v.clear();
    ref_w.clear();
    float te_yaw;
    for(int i=0;i<ref.rows();i++)
    {
        ref_x.push_back(ref(i,0)-ref(0,0));
        ref_y.push_back(ref(i,1)-ref(0,1));
        te_yaw = ref(i,2);// [-pi, pi]
//        if(te_yaw<0)te_yaw += 2*M_PI;
        ref_yaw.push_back(te_yaw);
        ref_v.push_back(ref(i,3));
        ref_w.push_back(ref(i,4));
    }
    ShowSubspace();
    // geometry_msgs::TwistStamped this_cmd;
    // this_cmd.header.frame_id = "vehicle";
    // this_cmd.twist.linear.x = temp_x;
    // this_cmd.twist.angular.z = temp_w;
    // cmd_v_w_pub.publish(this_cmd);        
}


void MPC_Control::ShowSubspace()
{
  visualization_msgs::MarkerArray pathNodes;//节点数据结构，用于可视化
  visualization_msgs::Marker bbox_marker;
  pathNodes.markers.clear();
  bbox_marker.header.frame_id = "map";
  bbox_marker.frame_locked = true;
  bbox_marker.ns = "test_path";
  bbox_marker.color.r = 0.0f;
  bbox_marker.color.g = 1.0f;
  bbox_marker.color.b = 0.0f;
  bbox_marker.color.a = 0.1;
  bbox_marker.scale.x = 0.1;
  bbox_marker.scale.y = 0.1;
  bbox_marker.scale.z = 0.1;

  int ct =0 ;
  for(int i=0;i<ref_y.size();i++)
  {
    bbox_marker.id = ct;
    bbox_marker.pose.position.x=ref_x[i]-ref_x[0];
    bbox_marker.pose.position.y=ref_y[i]-ref_y[0];
    bbox_marker.pose.position.z=0;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    pathNodes.markers.push_back(bbox_marker);
    ct++;
  }
  showSUBS.publish(pathNodes);
}
bool MPC_Control::MPC_Test_Handle()
{
    std::cout<<"In call back "<<std::endl;
    int sizelen = ref_y.size();
    path_len = sizelen;
    path_track_counter=0;
    A_qua.block<2*4+1,2*4+1>(0,0) = Eigen::MatrixXf::Identity(2*4+1,2*4+1);
    A_qua.block<2*4+1,2*4+1>(2*4+1,0) = Eigen::MatrixXf::Zero(2*4+1,2*4+1)-Eigen::MatrixXf::Identity(2*4+1,2*4+1);
    now_carstate<<0,0,0;//当前状态初始化
    std::pair<double,double> now_control_cmd;
    double thi=ros::Time::now().toSec();
    gazebo_msgs::GetModelState this_state;
    this_state.request.model_name="robot";
    this_state.request.relative_entity_name="map";

    double tyaw;
    geometry_msgs::TwistStamped this_cmd;
    this_cmd.header.frame_id = "vehicle";
    int i=0;
    std::cout<<"In mpc "<<std::endl;
    ros::Rate control_hz(10);
    float delta_rotate = 10;

    while(ros::ok())
    {
        while(ros::ok())
        {
        gz_client.call(this_state);
        tyaw = tf::getYaw(this_state.response.pose.orientation);//这里是车的yaw角
        if(!std::isnan(tyaw))break;
        }

        delta_rotate = ref_yaw[0]-tyaw;
        if(delta_rotate>M_PI)delta_rotate = M_PI-delta_rotate;
        else if(delta_rotate<-M_PI)delta_rotate = delta_rotate+2*M_PI;
        std::cout<<"delta is "<<ref_yaw[0]<<" "<<tyaw<<" "<<delta_rotate<<std::endl;
        if(std::abs(delta_rotate)<0.01)break;
        this_cmd.twist.linear.x = 0;
        if(delta_rotate>0) this_cmd.twist.angular.z = 0.2;
        else this_cmd.twist.angular.z = -0.2;        

        cmd_v_w_pub.publish(this_cmd);        
        control_hz.sleep();
    }
    double time1 = ros::Time::now().toSec();
    while(ros::ok()&&i<path_len)
    {
        gz_client.call(this_state);
        now_carstate(0,0)=this_state.response.pose.position.x;
        now_carstate(1,0)=this_state.response.pose.position.y;
        tyaw = tf::getYaw(this_state.response.pose.orientation);//这里是车的yaw角
        now_carstate(2,0)=tyaw;
        now_control_cmd = MPC_Solve();//根据参考轨迹、状态信息，生成控制律
        this_cmd.twist.linear.x = now_control_cmd.first;
        this_cmd.twist.angular.z = now_control_cmd.second;
        cmd_v_w_pub.publish(this_cmd);        
        i++;
        control_hz.sleep();
    double time2 = ros::Time::now().toSec();
    std::cout<<"delta time is "<<time2-time1<<std::endl;
    time1 = time2;
    }
        this_cmd.twist.linear.x = 0;
        this_cmd.twist.angular.z = 0;
        cmd_v_w_pub.publish(this_cmd);        
    return true;
}


std::pair<double,double> MPC_Control::MPC_Solve()
{
    // ref yaw is -2.94841 now_yaw is -2.94482ref x is -1.9707 now_x is -1.94412ref y is -5.56449 now_y is -5.56479ref v is 0.509743ref w is -0.484682 delta xita is 0.00359058The control cmd is :  v 0.748214  w 2
    float now_ref_x=ref_x[path_track_counter];
    float now_ref_y=ref_y[path_track_counter];
    float now_ref_yaw=ref_yaw[path_track_counter];//由于输入的角度在[-pi,pi],所以需要转换为[0,2pi]
    float now_ref_v=ref_v[path_track_counter];
    float now_ref_w=ref_w[path_track_counter];
    //异常原因:在曲线求解的时候,会出现一些很特殊的点,这些点的曲率本身没有得到良好的优化,因此会出现异常值,进而导致求解出的控制指令出现异常
    // path_track_counter = 92;
    // float now_ref_x=-1.9707;
    // float now_ref_y=-5.56449;
    // float now_ref_yaw=-2.94841;//由于输入的角度在[-pi,pi],所以需要转换为[0,2pi]
    // float now_ref_v=0.509743;
    // float now_ref_w=-0.484682;
    // now_carstate.setZero();
    // now_carstate<<-1.94412,-5.56479,-2.94482;
    Eigen::Matrix<float,3,1>  now_state_ref;
    now_state_ref<<now_ref_x,now_ref_y,now_ref_yaw;
    now_state_ref<<now_ref_x,now_ref_y,now_ref_yaw;
    std::cout<<path_track_counter<<std::endl;
    // std::cout<<now_carstate<<std::endl;//这个范围是-pi,pi
    // std::cout<<now_ref_x<<","<<now_ref_y<<","<<now_ref_yaw<<","<<now_ref_v<<","<<now_ref_w<<","<<std::endl;
    Eigen::Matrix3f a;
    a<< 1,0,(0-delta_mpc_t*now_ref_v*std::sin(now_ref_yaw)),
             0,1,(delta_mpc_t*now_ref_v*std::cos(now_ref_yaw)),
             0,0,1;
    // std::cout<<a<<std::endl;             
    Eigen::Matrix<float, 3,2> b;
    b<< (delta_mpc_t*std::cos(now_ref_yaw)),0,
             (delta_mpc_t*std::sin(now_ref_yaw)),0,
             0,delta_mpc_t;
    // std::cout<<b<<std::endl;             
    x_bo = now_carstate-now_state_ref;
    if(x_bo(2)<-M_PI) x_bo(2)=x_bo(2)+2*M_PI;
    if(x_bo(2)>M_PI) x_bo(2)=x_bo(2)-2*M_PI;//这里要保证很重要。否则可能出现一个接近2pi的跳跃。
    if(std::abs(x_bo(2))>M_PI_2*0.5){BreakPointRotate(now_state_ref(2));return std::pair<double,double>(0,0);}
    std::cout<<"ref yaw is "<<now_ref_yaw<<" now_yaw is "<<now_carstate(2)
    <<"ref x is "<<now_ref_x<<" now_x is "<<now_carstate(0)
    <<"ref y is "<<now_ref_y<<" now_y is "<<now_carstate(1)
    <<"ref v is "<<now_ref_v
    <<"ref w is "<<now_ref_w
    <<" delta xita is "<<x_bo(2);
    std::cout<<"ref v ref w "<<now_ref_v<<" "<<now_ref_w<<std::endl;
    Eigen::MatrixXf A_hat(Np*Nx,Nx);
    Eigen::Matrix3f A_temp; A_temp = a;
    Eigen::MatrixXf B_hat(Np*Nx,Nc*Nu);
    Eigen::MatrixXf zerob=Eigen::MatrixXf::Zero(Nx,Nu);
    for(int i =0;i<Np;i++)
    {
        A_hat.block<3,3>(i*Nx,0)=A_temp;
        A_temp = A_temp*a;
    }
    for(int i =0;i<Np;i++)
    for(int j=0;j<Nc;j++)
    {
        if(i==j)
            B_hat.block<3,2>(i*Nx,j*Nu) = b;
        if(j<i)
            B_hat.block<3,2>(i*Nx,j*Nu) = A_hat.block<3,3>((i-j-1)*Nx,0)*b;
        else
            B_hat.block<3,2>(i*Nx,j*Nu) = zerob;
    }
    Eigen::MatrixXf Q=Eigen::MatrixXf::Zero(Np*Nx,Np*Nx);
    for(int i =0;i<Np*Nx;i++)Q(i,i)=1.0;
    Eigen::MatrixXf R=Eigen::MatrixXf::Zero(Nc*Nu,Nc*Nu);
    for(int i =0;i<Nc*Nu;i++)R(i,i)=0.001;
    Eigen::MatrixXf H=Eigen::MatrixXf::Zero(Nc*Nu+1,Nc*Nu+1);
    // H.block<Nc*Nu,Nc*Nu>(0,0)=B_hat.transpose()*Q*B_hat+R;
    H.block<4*2,4*2>(0,0)=B_hat.transpose()*Q*B_hat+R;
    H(Nc*Nu,Nc*Nu)=1;//松弛因子
    // std::cout<<H<<std::endl;
    Eigen::MatrixXf f=Eigen::MatrixXf::Zero(1,Nc*Nu+1);
    // f.block<1,Nc*Nu>(0,0)
    f.block<1,4*2>(0,0)=x_bo.transpose()*A_hat.transpose()*Q*B_hat;
    f(0,Nc*Nu)=1;//松弛因子
    Eigen::MatrixXf b_qua(2*(Nc*Nu+1),1);
    Eigen::MatrixXf Ur(Nc*Nu+1,1);
    for(int i=0;i<Nc*Nu;i++)
    {
        // if(ref_v.size()-2==path_track_counter) 
        // std::cout<<""<<std::endl;
        Ur(i,0)=ref_v[int(i/2)+path_track_counter];
        // std::cout<<Ur(i,0)<<std::endl;
        i++;
        Ur(i,0)=ref_w[int(i/2)+path_track_counter];
        // std::cout<<Ur(i,0)<<std::endl;
    }//拿到两个参考控制量
    Ur(Nc*Nu,0)=0;//松弛因子
    Eigen::MatrixXf ub = U_max-Ur;
    Eigen::MatrixXf lb = U_min-Ur;//拿到优化变量的上界下界

    b_qua.block<4*2+1,1>(0,0)=U_max-Ur;
    b_qua.block<4*2+1,1>(4*2+1,0)=Ur-U_min;
    // std::cout<<b_qua<<std::endl;
    // std::cout<<ub<<std::endl;
    // std::cout<<lb<<std::endl;

    //将数据转换为qpOASES需要的形式
    int  size_H_qp=(Nc*Nu+1)*(Nc*Nu+1);
    qpOASES::real_t H_qp[size_H_qp];
    for(int i = 0;i<size_H_qp;i++)
    H_qp[i]=H(int(i/(Nc*Nu+1))  ,  (i%(Nc*Nu+1)));

    int  size_f_qp=Nc*Nu+1;
    qpOASES::real_t f_qp[size_f_qp];
    for(int i = 0;i<size_f_qp;i++)
    f_qp[i]=f(0,i);//这里f已经是转置了

    int  size_lb_qp=Nc*Nu+1;
    qpOASES::real_t lb_qp[size_lb_qp];
    for(int i = 0;i<size_lb_qp;i++)
    lb_qp[i]=lb(i,0);//优化变量的上下界

    qpOASES::real_t ub_qp[size_lb_qp];
    for(int i = 0;i<size_lb_qp;i++)
    ub_qp[i]=ub(i,0);//优化变量的上下界

    int  size_b_qua_qp=2*(Nc*Nu+1);
    qpOASES::real_t b_qua_qp[size_b_qua_qp];
    for(int i = 0;i<size_b_qua_qp;i++)
    b_qua_qp[i]=b_qua(i,0);

    int  size_A_qua_qp=2*(Nc*Nu+1)*(Nc*Nu+1);
    qpOASES::real_t A_qua_qp[size_A_qua_qp];
    for(int i = 0;i<size_A_qua_qp;i++)
    A_qua_qp[i]=A_qua( int(i/(Nc*Nu+1))  ,  (i%(Nc*Nu+1)) );
    int nV = (Nc*Nu+1);//变量个数
    int nC = 2*(Nc*Nu+1);//约束条件数量

    qpOASES::Options options;
    options.setToMPC();
	//options.enableFlippingBounds = BT_FALSE;
	// options.initialStatusBounds = qpOASES::ST_INACTIVE;
	// options.numRefinementSteps = 1;
	// options.enableCholeskyRefactorisation = 1;
    options.printLevel=qpOASES::PL_NONE;
    // qpOASES::QProblem example( nV,nC );
    qpOASES::QProblemB example( nV );
	example.setOptions( options );
    qpOASES::int_t nWSR = 100;
    // example.init( H_qp,f_qp,A_qua_qp,lb_qp,ub_qp,nullptr,b_qua_qp, nWSR );
    // if(path_track_counter==92)
    // {
    // std::cout<<H<<std::endl;
    // std::cout<<f<<std::endl;
    // std::cout<<lb<<std::endl;
    // std::cout<<ub<<std::endl;
    // }
    example.init( H_qp,f_qp,lb_qp,ub_qp, nWSR );
    qpOASES::real_t xOpt[nV];
    int result_solve = example.getPrimalSolution(xOpt);
    path_track_counter++;
    // std::cout<<path_track_counter<<std::endl;
    std::pair<double,double> ans(xOpt[0]+Ur(0,0),xOpt[1]+Ur(1,0));
    // std::cout<<"Result is "<<result_solve<<std::endl;
    std::cout<<"The control cmd is :  v "<<ans.first<<"  w "<<ans.second<<std::endl;
    return ans;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"mpc_node");
    ros::NodeHandle n;
    MPC_Control control_handle(n); //轨迹已经初始化完成,初始状态已经生成，参考轨迹、输入已经完成
    ros::Rate rate(5);
    while(ros::ok())
    {
        std::cout<<"In mpc while "<<std::endl;
        rate.sleep();
        ros::spinOnce();        
    }
    return 1;
}
//问题2：mpc求解序列不完全。
