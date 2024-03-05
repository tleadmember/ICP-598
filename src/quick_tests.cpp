#include <iostream>
#include <Eigen/Dense>

int main()
{
       Eigen::MatrixXf mat(3,3);
       mat << 1, 4, 7,
              2, 5, 8,
              3, 6, 9;

       Eigen::Vector3f v(1, 1, 1);

       auto temp1 = mat(0,0);
       auto temp2 = mat.size();

       // Eigen::Matrix3f zero_mat = Eigen::Matrix3f::Zero();
       // zero_mat(2,2) = 1;

       std::cout << temp1 << '\n' << '\n';
       std::cout << temp2 << '\n' << '\n';

       return 0;
}