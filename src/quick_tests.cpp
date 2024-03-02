#include <iostream>
#include <Eigen/Dense>

int main()
{
       Eigen::MatrixXf mat(3,3);
       mat << 1, 4, 7,
              2, 5, 8,
              3, 6, 9;

       Eigen::Vector3f v(1, 1, 1);

       Eigen::MatrixXf res = mat.colwise() - v;

       res.block(2, 0, 1, 3).setZero();

       Eigen::Matrix3f zero_mat = Eigen::Matrix3f::Zero();
       zero_mat(2,2) = 1;

       std::cout << res << '\n' << '\n';
       std::cout << zero_mat << '\n' << '\n';
}