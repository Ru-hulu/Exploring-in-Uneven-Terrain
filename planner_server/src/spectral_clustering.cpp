#include"spectral_clustering.hpp"
#include "dkm.hpp"
#include "dkm_parallel.hpp"
#include "dkm_utils.hpp"
template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v)
{
  // 初始化索引向量
  std::vector<size_t> idx(v.size());
  //使用iota对向量赋0~？的连续值
  std::iota(idx.begin(), idx.end(), 0);
  // 通过比较v的值对索引idx进行排序
  std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });
  return idx;
}


//A为待开方的矩阵，N为A矩阵的行数
Eigen::MatrixXf  MatrixSqrt( const Eigen::MatrixXf & A, int N )
{//只进行开方，不检查矩阵是否可以开方，当矩阵不能开方时，返回的结果会有错误

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> adjoint_eigen_solver((A + A.transpose()) / 2.);//求伴随阵的特征值和特征向量
	
	Eigen::MatrixXf mata = Eigen::MatrixXf::Zero(N, 1);

	return adjoint_eigen_solver.eigenvectors() * adjoint_eigen_solver.eigenvalues().cwiseMax(mata).cwiseSqrt().asDiagonal() * adjoint_eigen_solver.eigenvectors().transpose();
}

//这个函数按照最多6类进行处理，谱聚类的算法见//https://www.cnblogs.com/pinard/p/6221564.html
int Spectral_Clustering(std::vector<octomap::point3d>* Frontier_List,std::vector<int>* class_label_in)
{   //这里没有对frontier数量小于聚类个数的情况进行处理
    const int max_cluster_num =  15;//最多有15个聚类
    const int num_per_cluster = 5;//每个聚类5个Frontier

    int sample_num = Frontier_List->size();
    int class_num_actual = std::min(max_cluster_num,int(sample_num/num_per_cluster));

    if((class_num_actual<=1)&&(sample_num>0))
    {
        class_label_in->clear();
        for(int i =0;i<sample_num;i++)
        (*class_label_in).push_back(i);  
        class_num_actual = sample_num;
        return class_num_actual;
    }
    // std::cout<<"Sample size is "<<sample_num<<std::endl;
    Eigen::MatrixXf W_matrix(sample_num,sample_num);
    Eigen::MatrixXf D_matrix(sample_num,sample_num);
    Eigen::MatrixXf D2_matrix(sample_num,sample_num);//D开方
    Eigen::MatrixXf Dinv2_matrix(sample_num,sample_num);//D开方的逆
    Eigen::MatrixXf L_matrix(sample_num,sample_num);
    D_matrix.setZero();
    D2_matrix.setZero();
    //只对上三角形进行索引。
    float row_sum_count=0;
    float w_ij=0;
    float xigam_2 = 0.4;
    octomap::point3d delt;
    for(int i = 0;i<sample_num;i++)
    {
        row_sum_count = 0;
        for(int j = 0;j<sample_num;j++)
        {
            delt = (*Frontier_List)[i]-(*Frontier_List)[j];
            w_ij = std::exp((0-delt.norm())/2.0/xigam_2);
            // w_ij = delt.norm();
            W_matrix(i,j)=w_ij;
            // std::cout<<w_ij<<std::endl;
            row_sum_count += w_ij;//对行求和
        }
        D_matrix(i,i)=row_sum_count;//对角线的数值直接开根号        
        D2_matrix(i,i)=std::sqrt(row_sum_count);//对角线的数值直接开根号        
    }
    L_matrix = D_matrix-W_matrix;
    // std::cout<<D_matrix<<std::endl;
    Dinv2_matrix =  D2_matrix.inverse();//D开方求逆
    L_matrix = Dinv2_matrix*L_matrix*Dinv2_matrix;//归一化拉普拉斯矩阵
    Eigen::VectorXf L_evalue(sample_num);
    Eigen::MatrixXf L_evector(sample_num,sample_num);
    //这里默认就是实数！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
    Eigen::EigenSolver<Eigen::MatrixXf> eig(L_matrix);     // [vec val] = eig(A) 
    L_evalue = eig.eigenvalues().real();                // diag(val)与前边的是一样的结果
    L_evector = eig.eigenvectors().real();               // vec 特征值对应的特征向量
    std::vector<float> tempf;
    for(int i=0;i<L_evalue.size();i++)
    tempf.push_back(L_evalue(i));
	auto idx = sort_indexes<float>(tempf);//拿到从小到大的排序
    Eigen::MatrixXf Select_evector(sample_num,class_num_actual);
    float tn = 0;
    int select_id = 0;
    for(int i=0;i<class_num_actual;i++)
    {
        select_id = idx[i];//拿到最小的几个列向量索引
       // std::cout<<L_evalue(select_id)<<std::endl;
        Select_evector.col(i) = L_evector.col(select_id);//拿到最小的几个列向量
    }



    std::vector<std::array<float, max_cluster_num>> featuer_vec;
    std::array<float, max_cluster_num> one_row{};//按照最大的来初始化，但是长度不足就是0，这样在kmean的时候效果也不受影响。

    for(int j=0;j<sample_num;j++)//给定的一个行，进行归一化
    {
        Select_evector.row(j) = Select_evector.row(j) / Select_evector.row(j).norm();
        for(int i =0;i<class_num_actual;i++)
        one_row[i]=Select_evector(j,i);
        featuer_vec.push_back(one_row);
    }
    std::tuple<std::vector<std::array<float, max_cluster_num>>, std::vector<uint32_t>> result;//pair只能有两个成员,但是tuple可以有多个.这里明显可以使用pair.
	result = dkm::kmeans_lloyd(featuer_vec, class_num_actual);



//   0: A vector holding the means for each cluster from 0 to k-1.
//   1: A vector containing the cluster number (0 to k-1) for each corresponding element of the input
// 	 data vector.
    std::vector<uint32_t> result2 = std::get<1>(result);
    class_label_in->clear();
    for (const auto& lab : result2)
    {
	// std::cout << int(lab)<<std::endl;
    class_label_in->push_back(int(lab));
    }
    std::cout<<"聚类完成 class_label_in size is "<<class_label_in->size()<<std::endl;
    return class_num_actual;
}


    //1、计算相似度矩阵W
    //2、计算D矩阵
    //3、计算拉普拉斯矩阵L
    //4、对L进行归一化，并计算特征向量和特征值。
    //5、最小的几个特征值对应的特征向量
    //6、特征向量按照行归一化
    //7、对归一化后的特征向量进行KMEAN
    //8、得到cluster_index;

