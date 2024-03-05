#include <iostream>
#include <Eigen/Dense>

int main()
{
       Eigen::MatrixXd A(1,3);
       A << 1, 2, 3;
       Eigen::MatrixXd B(3,1);
       B << 1, 
            2, 
            3;

       auto res = A*B;
       // std::cout << res << '\n' << '\n';
       std::cout << res.size() << '\n' << '\n';

       double number = res(0,0);
       std::cout << number << '\n' << '\n';

       return 0;
}