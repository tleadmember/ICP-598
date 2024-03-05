#include "gradient_descent.h"
#include <iostream>
#include <cmath>


double quadratic_objective_function(Eigen::VectorXd x) {
    Eigen::Matrix2d A;
    A <<    2, -1,
            -1, 1;

    return x.transpose()*A*x;
}


Eigen::Matrix4d pack44(Eigen::VectorXd& vec) {
    Eigen::Matrix4d mat;
    mat.setZero();
    mat(0,0) = vec(0);
    mat(0,1) = vec(2);
    mat(1,2) = vec(0);
    mat(1,3) = vec(2);
    mat(2,0) = vec(1);
    mat(2,1) = vec(3);
    mat(3,2) = vec(1);
    mat(3,3) = vec(3);
    return mat;
}


Eigen::VectorXd mat_to_vec(Eigen::Matrix2d& mat) { // turn matrix into vector, column-major way
    Eigen::VectorXd vec = Eigen::Map<Eigen::VectorXd>(mat.data(), mat.size());
    return vec;
}


double rot_matrices_objective_function(Eigen::VectorXd x) {
    Eigen::VectorXd vec_R_w_2 = x.block(0,0,4,1); // extract 4-dim vectors out of x
    Eigen::VectorXd vec_R_w_3 = x.block(4,0,4,1);
    Eigen::VectorXd vec_R_w_4 = x.block(8,0,4,1);

    Eigen::Matrix2d R_1_2, R_2_3, R_3_4, R_4_1, R_2_1, R_w_1, R_1_3, R_3_1;
    R_1_2 <<    0.17, -0.98,
                0.98, 0.17;
    R_2_3 <<    -0.17, -0.98,
                0.98, -0.17;
    R_3_4 <<    0.71, -0.71,
                0.71, 0.71;
    R_4_1 <<    -0.71, -0.71,
                0.71, -0.71;
    R_2_1 = R_1_2.transpose();
    R_1_3 = R_1_2 * R_2_3;
    R_3_1 = R_1_3.transpose();
    R_w_1 = Eigen::Matrix2d::Identity(2,2); // given as identity matrix

    Eigen::VectorXd vec_R_w_1 = mat_to_vec(R_w_1); // turn matrix into vector, column-major way
    // Eigen::VectorXd vec_R_2_1 = mat_to_vec(R_2_1);

    double result = 0; // initialization
    Eigen::VectorXd temp_v1(4);
    Eigen::Matrix4d temp_mat;
    Eigen::VectorXd temp_v2(4);
    Eigen::VectorXd temp_subtraction(4);

    // 1st squared norm in f(x)
    temp_v1 =  vec_R_w_1;
    temp_mat = pack44(vec_R_w_2);
    temp_v2 = mat_to_vec(R_2_1);
    temp_subtraction = temp_v1 - temp_mat * temp_v2;
    result += temp_subtraction.squaredNorm();
    // std::cout << temp_v1 << '\n' << '\n';
    // std::cout << temp_mat << '\n' << '\n';
    // std::cout << temp_v2 << '\n' << '\n';
    // std::cout << temp_subtraction << '\n' << '\n';
    // std::cout << result << '\n' << '\n';

    // 2nd squared norm in f(x)
    temp_v1 =  vec_R_w_1;
    temp_mat = pack44(vec_R_w_3);
    temp_v2 = mat_to_vec(R_3_1);
    temp_subtraction = temp_v1 - temp_mat * temp_v2;
    result += temp_subtraction.squaredNorm();
    std::cout << temp_subtraction << '\n' << '\n';
    std::cout << result << '\n' << '\n';


    return 1212;
}


int main() {
    double epsilon = 1e-4;
    double step_size = 1;
    double h = 1e-3;
    double armijo_tau = 0.5;
    double armijo_c = 0.5;

    /***********************************************************/

    // Eigen::VectorXd start_point(2); 
    // start_point <<  -100, 
    //                 -100;

    // // Call gradient descent function(s)
    // GradientDescent<Eigen::VectorXd> gd(&quadratic_objective_function, 
    //                                     start_point, 
    //                                     step_size, 
    //                                     h, 
    //                                     epsilon,
    //                                     armijo_tau,
    //                                     armijo_c);
    // gd.test_print();
    // gd.calculate();

    // std::cout << "Arg min =\n" << gd.arg_min() << '\n' << '\n';
    // std::cout << "Minimum =\n" << gd.min() << '\n' << '\n';

    /***********************************************************/

    Eigen::VectorXd start_point(12); 
    // start_point.setOnes();
    start_point << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
    rot_matrices_objective_function(start_point);

    // // Call gradient descent function(s)
    // GradientDescent<Eigen::VectorXd> gd(&rot_matrices_objective_function, 
    //                                     start_point, 
    //                                     step_size, 
    //                                     h, 
    //                                     epsilon,
    //                                     armijo_tau,
    //                                     armijo_c);
    // gd.test_print();
    // gd.calculate();

    // std::cout << "Arg min =\n" << gd.arg_min() << '\n' << '\n';
    // std::cout << "Minimum =\n" << gd.min() << '\n' << '\n';

    /***********************************************************/

    return 0;
}