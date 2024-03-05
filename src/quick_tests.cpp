#include <iostream>
#include <Eigen/Dense>

int main()
{
     //   Eigen::MatrixXd A(1,3);
     //   A << 1, 2, 3;
     //   Eigen::MatrixXd B(3,1);
     //   B << 1, 
     //        2, 
     //        3;
     //   auto res = A*B;
     //   // std::cout << res << '\n' << '\n';
     //   std::cout << res.size() << '\n' << '\n';

     //   double number = res(0,0);
     //   std::cout << number << '\n' << '\n';


     // Eigen::VectorXd v(12);
     // v.setOnes();
     // for (int i = 0; i < 12; i++) {
     //      // v(i) = i+1;
     //      v(i) = double(i+1);
     // }
     // std::cout << v << '\n';


     // Eigen::MatrixXd A(2,2);
     // A << 1, 2,
     //      3, 4;
     // for (int i = 0; i < A.size(); i++) {
     //      std::cout << A(i) << '\n';
     // }
     // std::cout << A.data() << '\n' << '\n';

     // Reshape the matrix into a column vector
     // Eigen::VectorXd vectorized_matrix = Eigen::Map<Eigen::VectorXd>(A.data(), A.size()); 
     // std::cout << "vectorized_matrix =\n" << vectorized_matrix << '\n' << '\n';

     // A = Eigen::MatrixXd::Identity(2,2);
     // std::cout << "A =\n" << A << '\n' << '\n';

     // double norm = A.norm();
     // std::cout << "Frobeinus norm =\n" << norm << '\n' << '\n';
     // double squared_norm = A.squaredNorm();
     // std::cout << "Squared Frobenius norm =\n" << squared_norm << '\n' << '\n';


     // Eigen::VectorXd start_point(12); 
     // start_point.setOnes();
     // Eigen::VectorXd v1 = start_point.block(0,0,4,1);
     // std::cout << v1 << '\n';
     

     return 0;
}