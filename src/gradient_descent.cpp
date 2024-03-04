#include "gradient_descent.h"
#include <vector>
#include <tuple>
#include <chrono>
#include <iostream>

// Constructor
template <typename start_point_T>
GradientDescent<start_point_T>::GradientDescent(double (*fun)(start_point_T), start_point_T& start_point)
    :   start_point_{start_point},
        objective_function_{fun} {}

template <typename start_point_T>
void GradientDescent<start_point_T>::test_print() const {
    std::cout << "Start point from class:\n" << start_point_ << '\n' << '\n';
    std::cout << "Result of objective function with initial start point:\n" << objective_function_(start_point_) << '\n' << '\n';
}

template <typename start_point_T>
double GradientDescent<start_point_T>::central_difference() const {
    double res = 0;
    std::cout << "Hello from central difference: \n" << res << '\n' << '\n';
    return res;
}

template <typename start_point_T>
Eigen::Vector3d GradientDescent<start_point_T>::numerical_gradient() const {
    Eigen::Vector3d nabla;
    nabla.setZero();
    std::cout << "Hello from numerical gradient: \n" << nabla << '\n' << '\n';

    return nabla;
}


// Explicit instantiations
template class GradientDescent<Eigen::Vector2d>;
template class GradientDescent<Eigen::MatrixXd>;