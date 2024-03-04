#include "gradient_descent.h"
#include <iostream>
#include <cmath>

double quadratic_objective_function(Eigen::Vector2d x) {
    Eigen::Matrix2d A;
    A <<    2, -1,
            -1, 1;

    return x.transpose()*A*x;
}


int main() {
    double epsilon = 1e-4;
    Eigen::Vector2d start_point; 
    start_point << 100, 100;


    // Call gradient descent function(s)
    GradientDescent<Eigen::Vector2d> gd(&quadratic_objective_function, start_point);
    gd.test_print();
    gd.central_difference();
    gd.numerical_gradient();


    /***********************************************************/

    // // Example of creating a vector
    // std::vector<Eigen::Matrix2d> my_vector;

    // // Assign values to the vector
    // Eigen::Matrix2d R_1_2, R_2_3, R_3_4, R_4_1;
    // R_1_2 <<    0.17, -0.98,
    //             0.98, 0.17;
    // R_2_3 <<    -0.17, -0.98,
    //             0.98, -0.17;
    // R_3_4 <<    0.71, -0.71,
    //             0.71, 0.71;
    // R_4_1 <<    -0.71, -0.71,
    //             0.71, -0.71;
    // my_vector.push_back(R_1_2);
    // my_vector.push_back(R_2_3);
    // my_vector.push_back(R_3_4);

    // // Display the vector
    // for (const auto& element : my_vector) {
    //     std::cout << element << '\n' << '\n';
    // }
    // std::cout << "Second element:\n" << my_vector[1] << '\n';

    return 0;
}