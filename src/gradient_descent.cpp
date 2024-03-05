#include "gradient_descent.h"
#include <vector>
#include <tuple>
#include <chrono>
#include <iostream>
#include<unistd.h>   

// Constructor
template <typename start_point_T>
GradientDescent<start_point_T>::GradientDescent( double (*fun)(start_point_T), 
                                                 start_point_T& start_point, 
                                                 double& step_size, 
                                                 double& h,
                                                 double& epsilon)
    :   start_point_{start_point},
        next_point_{start_point},
        num_elms_{start_point.rows()/start_point.cols()},
        objective_function_{fun},
        step_size_{step_size},
        h_{h},
        epsilon_{epsilon},
        count_{0} {}


// Getters
template <typename start_point_T>
double GradientDescent<start_point_T>::min() const {
    return minimum_;
};
  
template <typename start_point_T>
start_point_T GradientDescent<start_point_T>::arg_min() const {
    return start_point_;
};


// Test print private members from class
template <typename start_point_T>
void GradientDescent<start_point_T>::test_print() const {
    std::cout << "Start point from class:\n" << start_point_ << '\n' << '\n';
    std::cout << "Num of \"elements\" from class:\n" << num_elms_ << '\n' << '\n';
    std::cout << "Alpha (step size) from class:\n" << step_size_ << '\n' << '\n';
    std::cout << "h (amount to vary in central difference) from class:\n" << h_ << '\n' << '\n';
    std::cout << "Result of objective function with initial start point:\n" << objective_function_(start_point_) << '\n' << '\n';
}


// Calculate central difference separately
// template <typename start_point_T>
// double GradientDescent<start_point_T>::central_difference() const {
//     double res = 0;
//     std::cout << "Hello from central_difference():\n" << res << '\n' << '\n';
    

//     std::cout << "Hello again from central_difference():\n" << res << '\n' << '\n';
//     return res;
// }


// Numerical_gradient() function uses central difference
template <typename start_point_T>
Eigen::MatrixXd GradientDescent<start_point_T>::numerical_gradient() const {
    // std::cout << "Hello from numerical_gradient():\n" << nabla << '\n' << '\n';
    Eigen::MatrixXd nabla(1,num_elms_); // gradient "vector", horizontal format

    if (start_point_.cols() == 1) {
        // std::cout << "Got vector start point!\n\n";
        for (auto i = 0; i < num_elms_; i++) {
            start_point_T temp_start_point1 = start_point_;
            start_point_T temp_start_point2 = start_point_;
            temp_start_point1(i,0) = temp_start_point1(i,0) + h_;
            temp_start_point2(i,0) = temp_start_point2(i,0) - h_;
            nabla(0,i) = ( objective_function_(temp_start_point1) - objective_function_(temp_start_point2) ) / ( 2*h_ ); // central difference
        }
    } else if (start_point_.cols() > 1) {
        std::cout << "Got matrix start point!\n\n";
    } else {
        std::cout << "ERROR: Start point has either 0 columns or negative columns!\n\n";
        return nabla;
    }

    // std::cout << "Hello again from numerical_gradient():\n" << nabla << '\n' << '\n';
    return nabla;
}


// Calculate function to loop until convergence
template <typename start_point_T>
void GradientDescent<start_point_T>::calculate() {
    std::cout << "Hello from calculate()\n" << '\n';
    Eigen::MatrixXd nabla(1,num_elms_); // gradient "vector", horizontal format
    nabla.setOnes(); // initialize with something not less than epsilon

    while(count_ < 1000) {
        nabla = numerical_gradient();
        std::cout << "Current gradient: " << nabla << '\n' << '\n';
        
        // Check if norm gradient is close enough to 0
        if (nabla.norm() < epsilon_) break; 

        // Update decision variable
        if (start_point_.cols() == 1) {
            // std::cout << "Got vector start point!\n\n";
            next_point_ = start_point_ - step_size_ * nabla.transpose();
            // Check if next_point_ helps reduce objective function
            double new_f = objective_function_(next_point_);
            std::cout << "New f = " << new_f << '\n' << '\n';
            double current_f = objective_function_(start_point_);
            std::cout << "Current f = " << current_f << '\n' << '\n';
            while (new_f >= current_f) { // while not good, update step_size_ and recalculate next_point_
                step_size_ = 0.75*step_size_;
                std::cout << "Updated step_size_: " << step_size_ << '\n' << '\n';
                sleep(1);
                next_point_ = start_point_ - step_size_ * nabla.transpose();
                new_f = objective_function_(next_point_);
                std::cout << "New f = " << new_f << '\n' << '\n';
            }
            start_point_ = next_point_; // Good
        } else if (start_point_.cols() > 1) {
            std::cout << "Got matrix start point!\n\n";
        } else {
            std::cout << "ERROR: Start point has either 0 columns or negative columns!\n\n";
        }

        ++count_; // update count of guesses of start_point_
    }

    std::cout << "Hello again from calculate(). Convergence reached after ";
    std::cout << count_ << " guesses!\n" << '\n';
}


// Explicit instantiations
template class GradientDescent<Eigen::Vector2d>;
template class GradientDescent<Eigen::MatrixXd>;