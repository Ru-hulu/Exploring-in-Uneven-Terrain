#include <eigen3/Eigen/Dense>
#include "Eigen/Eigenvalues"
#include<vector>
#include<numeric>
#include<algorithm>
#include <octomap/octomap.h>

//输入：vector<vector3f> class_number
//输出：vector<int> class_label
Eigen::MatrixXf  MatrixSqrt( const Eigen::MatrixXf & A, int N );
// void Spectral_Clustering(std::vector<Eigen::Vector3f>* Frontier_List,std::vector<int>* class_label, int class_num);
int Spectral_Clustering(std::vector<octomap::point3d>* Frontier_List,std::vector<int>* class_label_in);
template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v);
