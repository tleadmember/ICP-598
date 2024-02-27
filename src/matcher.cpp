#include "matcher.h"
#include <chrono>
#include <iostream>
#include <nanoflann.hpp>

// Constructor
Matcher::Matcher(const Eigen::MatrixXd &set_org, const Eigen::MatrixXd &set_mov)
    : set_org_{set_org}, 
      set_mov_{set_mov},
      matrix_A_{Eigen::Matrix<double, 8, 9>::Zero(8, 9)} {}

// Getter functions
Eigen::MatrixXd Matcher::set_org() const { return set_org_; }
Eigen::MatrixXd Matcher::set_mov() const { return set_mov_; }
Eigen::Matrix<double, 8, 9> Matcher::matrix_A() const {return matrix_A_;};

// Other functions
void Matcher::print_sets() const {
  std::cout << "Matrix of original points: \n" << set_org() << '\n' << '\n';
  std::cout << "Moved matrix: \n" << set_mov() << '\n' << '\n';
}


/* Function to calculate matrix A in the linearization of the 8-point equation to min ||Ax-b||^2 */
void Matcher::calc_A() {
  int r = matrix_A_.rows();
  int c = matrix_A_.cols();
  for (int i = 0; i < r; i++) {
    for (int j = 0; j < c; j++) {
      matrix_A_(i, j) = set_org_(j/3, i) * set_mov_(j%3, i); // see hand notes in HW3 on how to calculate
    }
  }
  std::cout << "Matrix A for linearization: \n" << matrix_A_ << '\n' << '\n';
}


/* Eight point algorithm*/
void Matcher::eight_pt() {
  // Normalize the points
  Eigen::MatrixXd normalMat1 = normalize_matrix(set_org_);
  std::cout << "Normalization matrix for original points: \n" << normalMat1 << '\n' << '\n';
  Eigen::MatrixXd normalMat2 = normalize_matrix(set_mov_);
  std::cout << "Normalization matrix for moved points: \n" << normalMat2 << '\n' << '\n';
}

/* Normalization of matrix of points*/
Eigen::Matrix3d Matcher::normalize_matrix(const Eigen::MatrixXd& points) const {
  Eigen::Matrix3d normalMat = Eigen::Matrix3d::Identity();

  double meanX = points.row(0).mean();
  double meanY = points.row(1).mean();

  std::cout << "Mean x: \n" << meanX << '\n' << '\n';
  std::cout << "Mean y: \n" << meanY << '\n' << '\n';

  return normalMat;
}