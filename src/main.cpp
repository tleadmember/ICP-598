#include "matcher.h"
#include <iostream>
#include <cmath>
#include <random>


int main() {
    int num_points = 8; // determine how many 3d points

    Eigen::MatrixXd set_original(3,num_points);         // original set of points
    Eigen::MatrixXd set_original_aug(4,num_points);     // augmented matrix of original set
    Eigen::MatrixXd set_moved(3,num_points);            // moved set of points
    Eigen::MatrixXd set_moved_aug(4,num_points);        // augmented matrix of moved set
    Eigen::Matrix<double, 3, 3> rot_matrix;
    Eigen::VectorXd transl_vec(3);
    double degrees;
    double radians;
    Eigen::MatrixXd T(4,4); // homogenerous transf matrix
    Eigen::VectorXd aug_vec(4);
    aug_vec << 0, 0, 0, 1; // vector to augment T

    // Form matrix of original points (random)
    std::random_device rd;
    std::mt19937 gen(rd());  // here you could set the seed, but std::random_device already does that
    std::uniform_real_distribution<double> dis1(-20.0, 20.0);
    set_original = Eigen::MatrixXd::NullaryExpr(3,num_points,[&](){return dis1(gen);});
    // std::cout << "Matrix of original points: \n" << set_original << '\n' << '\n';

    // Augment matrix of original points
    Eigen::VectorXd ones_vec(num_points);
    ones_vec.setOnes();
    set_original_aug.block(0, 0, 3, num_points) = set_original; // augmented matrix
    set_original_aug.block(3, 0, 1, num_points) = ones_vec.transpose(); 
    // std::cout << "Augmented matrix of original points: \n" << set_original_aug << '\n' << '\n';

    // Set the rotation and translation
    // std::uniform_real_distribution<double> dis2(-179, 179);
    degrees = 10;
    std::cout << "Degrees: \n" << degrees << '\n';
    radians = (degrees/180)*3.14159;
    // rot_matrix << cos(radians), -sin(radians), 0,
    //               sin(radians), cos(radians), 0,
    //               0, 0, 1;
    rot_matrix = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    transl_vec << 1, -2, 3;
    std::cout << "Translation: \n" << transl_vec << "\n\n";

    // Form the actual transformation matrix
    T.block(0, 0, 3, 3) = rot_matrix;
    T.block(0, 3, 3, 1) = transl_vec;
    T.block(3, 0, 1, 4) = aug_vec.transpose();
    std::cout << "Actual transformation matrix: \n" << T << '\n' << '\n';
    Eigen::Quaterniond quaternion1(rot_matrix);
    std::cout << "Actual quaternion of rotation: \n" << quaternion1.coeffs().transpose() << '\n' << '\n';

    // Perform sample transformation changes on original points
    set_moved_aug = T*set_original_aug;
    // std::cout << "Augmented moved matrix: \n" << set_moved_aug << '\n' << '\n';

    // Extract matrix of moved points
    set_moved = set_moved_aug.block(0, 0, 3, num_points);
    // std::cout << "Moved matrix: \n" << set_moved << '\n' << '\n';

    // Initialize an object of Matcher class
    Matcher matcher{set_original, set_moved};

    matcher.print_sets();

    /* 8-point algorithm */
    // Calculate matrix A for linearization
    matcher.calc_A();
    // Check if columns of A are linearly independent
    Eigen::Matrix<double, 8, 9> matrix_A = matcher.matrix_A();
    // std::cout << "Matrix A: \n" << matrix_A << '\n' << '\n';
    // std::cout << "A_transpose: \n" << matrix_A.transpose() << '\n' << '\n';
    Eigen::Matrix<double, 9, 9> AT_A = matrix_A.transpose() * matrix_A;
    // std::cout << "A_transpose*A: \n" << AT_A << '\n' << '\n';
    Eigen::Matrix<double, 9, 9> _2_AT_A = 2*AT_A;
    std::cout << "2*A_transpose*A: \n" << _2_AT_A << '\n' << '\n';
    Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(_2_AT_A);
    Eigen::VectorXd eigen_values = eigen_solver.eigenvalues().real(); // AT_A is symmetric so all its eigenvalues are real
    std::cout << "Eigenvalues of 2*AT*A: \n" << eigen_values << '\n' << '\n';

    double smallest_eigvalue = eigen_values.minCoeff();
    std::cout << "Smallest eigenvalue of AT*A: " << smallest_eigvalue << '\n' << '\n';

    return 0;
}