#include "gradient_descent.h"
#include <chrono>
#include <iostream>

// Constructor
// template <typename start_point_T>
// GradientDescent<start_point_T>::GradientDescent(double (*fun)(start_point_T), start_point_T& start_point)
//     : start_point_{start_point} {}
template <typename start_point_T>
GradientDescent<start_point_T>::GradientDescent(start_point_T& start_point)
    : start_point_{start_point} {}

template <typename start_point_T>
void GradientDescent<start_point_T>::test_print() const {
    std::cout << "Start point from class:\n" << start_point_ << '\n';
}


// Explicit instantiations
template class GradientDescent<Eigen::Vector2d>;