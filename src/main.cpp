#include "matcher.h"
#include <iostream>
#include <cmath>
#include <random>


int main() {
    int num_points = 8; // determine how many points

    Eigen::MatrixXd set_original(3,num_points);         // original set of points
    Eigen::MatrixXd set_original_aug(3,num_points);     // augmented matrix of original set
    Eigen::MatrixXd set_moved(3,num_points);            // moved set of points
    Eigen::MatrixXd set_moved_aug(3,num_points);        // augmented matrix of moved set
    Eigen::Matrix<double, 2, 2> rot_matrix;
    Eigen::VectorXd transl_vec(2);
    double degrees;
    double radians;
    Eigen::MatrixXd T(3,3); // homogenerous transf matrix
    Eigen::VectorXd aug_vec(3);
    aug_vec << 0, 0, 1; // vector to augment T

    // Form matrix of original points (random)
    std::random_device rd;
    std::mt19937 gen(rd());  // here you could set the seed, but std::random_device already does that
    std::uniform_real_distribution<double> dis1(-20.0, 20.0);
    set_original = Eigen::MatrixXd::NullaryExpr(2,num_points,[&](){return dis1(gen);});
    // std::cout << "Matrix of original points: \n" << set_original << '\n' << '\n';

    // Make every point [a b 1]' like in 8-point algorithm lectures found online
    Eigen::VectorXd ones_vec(num_points);
    ones_vec.setOnes();
    // Augment matrix of original points
    set_original_aug.block(0, 0, 2, num_points) = set_original; // augmented matrix
    set_original_aug.block(2, 0, 1, num_points) = ones_vec.transpose(); 
    std::cout << "Augmented matrix of original points: \n" << set_original_aug << '\n' << '\n';

    // Set the rotation and translation
    // std::uniform_real_distribution<double> dis2(-179, 179);
    degrees = 10;
    std::cout << "Degrees: \n" << degrees << '\n' << '\n';
    radians = (degrees/180)*3.14159;
    rot_matrix << cos(radians), -sin(radians),
                  sin(radians), cos(radians);
    std::cout << "Rotation matrix: \n" << rot_matrix << '\n' << '\n';
    // rot_matrix = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    transl_vec << 1, -2;
    std::cout << "Translation: \n" << transl_vec << '\n' << '\n';

    // Form the actual transformation matrix
    T.block(0, 0, 2, 2) = rot_matrix;
    T.block(0, 2, 2, 1) = transl_vec;
    T.block(2, 0, 1, 3) = aug_vec.transpose();
    std::cout << "Actual transformation matrix: \n" << T << '\n' << '\n';

    // Perform sample transformation changes on original points
    set_moved_aug = T*set_original_aug;
    std::cout << "Augmented matrix of moved points: \n" << set_moved_aug << '\n' << '\n';

    // // Extract matrix of moved points
    // set_moved = set_moved_aug.block(0, 0, 2, num_points);
    // std::cout << "Moved matrix: \n" << set_moved << '\n' << '\n';

    // Initialize an object of Matcher class
    Matcher matcher{set_original_aug, set_moved_aug};
    // matcher.print_sets();

    /* 8-point algorithm */
    // Calculate matrix A for linearization
    Eigen::Matrix3d F = matcher.eight_pt();
    std::cout << "Fundamental matrix F: \n" << F << '\n' << '\n';
    // Check results
    bool is_correct = matcher.check_epipolar(F);
    if (is_correct) {
        std::cout << "Epipolar constraint satisfied. " << '\n' << '\n';
    } else {
        std::cout << "ERROR: Epipolar constraint NOT satisfied! " << '\n' << '\n';
    }

    // Check if columns of A are linearly independent
    // Eigen::Matrix<double, 8, 9> matrix_A = matcher.matrix_A();
    // std::cout << "Matrix A: \n" << matrix_A << '\n' << '\n';
    // std::cout << "A_transpose: \n" << matrix_A.transpose() << '\n' << '\n';
    // Eigen::Matrix<double, 9, 9> AT_A = matrix_A.transpose() * matrix_A;
    // std::cout << "A_transpose*A: \n" << AT_A << '\n' << '\n';

    // Eigen::Matrix<double, 9, 9> _2_AT_A = 2*AT_A;
    // std::cout << "2*A_transpose*A: \n" << _2_AT_A << '\n' << '\n';
    // Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(_2_AT_A);
    // Eigen::VectorXd eigen_values = eigen_solver.eigenvalues().real(); // AT_A is symmetric so all its eigenvalues are real
    // std::cout << "Eigenvalues of 2*AT*A: \n" << eigen_values << '\n' << '\n';
    // double smallest_eigvalue = eigen_values.minCoeff();
    // std::cout << "Smallest eigenvalue of AT*A: " << smallest_eigvalue << '\n' << '\n';

    // Get rank of A
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(matrix_A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // auto rank_A = svd_A.rank();
    // std::cout << "Rank of A: " << rank_A << '\n' << '\n';

    // // Get rank of AT_A
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd_ATA(AT_A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // auto rank_ATA = svd_ATA.rank();
    // std::cout << "Rank of AT*A: " << rank_ATA << '\n' << '\n';
    // std::cout << "Size of AT*A: " << AT_A.rows() << 'x' << AT_A.cols() << '\n' << '\n';
    
    // // Calculate pseudoinverse of AT_A*x = AT*b = 0
    // Eigen::VectorXd b = Eigen::VectorXd::Zero(9);
    // // std::cout << "Right hand side: \n" << b << '\n' << '\n';
    // Eigen::MatrixXd x = svd_ATA.solve(b);
    // std::cout << "x* =  \n" << x << '\n' << '\n';

    return 0;
}