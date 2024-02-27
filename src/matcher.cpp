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
  std::cout << "Augmented matrix of original points: \n" << set_org() << '\n' << '\n';
  std::cout << "Augmented matrix of moved points: \n" << set_mov() << '\n' << '\n';
}


/* Function to calculate matrix A in the linearization of the 8-point equation to min ||Ax-b||^2 */
void Matcher::calc_A(const Eigen::MatrixXd& normal_set1, const Eigen::MatrixXd& normal_set2) {
  int r = 8;
  int c = 9;
  for (int i = 0; i < r; i++) {
    for (int j = 0; j < c; j++) {
      matrix_A_(i, j) = normal_set2(j/3, i) * normal_set1(j%3, i); // see hand notes in HW3 on how to calculate
    }
  }
  std::cout << "Matrix A for linearization: \n" << matrix_A_ << '\n' << '\n';
}


/* Eight point algorithm*/
Eigen::Matrix3d Matcher::eight_pt() {
  // Normalize the points
  Eigen::MatrixXd normal_mat1 = normalize_matrix(set_org_);
  // std::cout << "Normalization matrix for original points: \n" << normal_mat1 << '\n' << '\n';
  Eigen::MatrixXd normal_mat2 = normalize_matrix(set_mov_);
  // std::cout << "Normalization matrix for moved points: \n" << normal_mat2 << '\n' << '\n';

  Eigen::MatrixXd normal_set1 = normal_mat1 * set_org_;
  std::cout << "Normalized org points: \n" << normal_set1 << '\n' << '\n';
  Eigen::MatrixXd normal_set2 = normal_mat2 * set_mov_;
  std::cout << "Normalized moved points: \n" << normal_set2 << '\n' << '\n';

  calc_A(normal_set1, normal_set2);

  // Recommended by ChatGPT - it gives the same result as calc_A
  // Eigen::MatrixXd A(8, 9);
  // for (int i = 0; i < 8; ++i) {
  //   A.row(i) << normal_set2(0, i) * normal_set1(0, i),
  //               normal_set2(0, i) * normal_set1(1, i),
  //               normal_set2(0, i),
  //               normal_set2(1, i) * normal_set1(0, i),
  //               normal_set2(1, i) * normal_set1(1, i),
  //               normal_set2(1, i),
  //               normal_set1(0, i),
  //               normal_set1(1, i),
  //               1.0;
  // }
  // std::cout << "(ChatGPT) Matrix A for linearization: \n" << A << '\n' << '\n';

  // // Solve the linear system using SVD
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(matrix_A_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Eigen::MatrixXd V = svd_A.matrixV();
  // std::cout << "Matrix V in SVD of A: \n" << V << '\n' << '\n';

  // Solve the linear system using SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_ATA(matrix_A_.transpose()*matrix_A_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd V = svd_ATA.matrixV();
  std::cout << "Matrix V in SVD of AT*A: \n" << V << '\n' << '\n';
  Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(matrix_A_.transpose()*matrix_A_);
  Eigen::VectorXd eigen_values = eigen_solver.eigenvalues().real(); // AT_A is symmetric so all its eigenvalues are real
  std::cout << "Eigenvalues of AT*A: \n" << eigen_values << '\n' << '\n';

  // Extract the fundamental matrix from the last column of V
  Eigen::Matrix3d F;
  F << V(0, 8), V(1, 8), V(2, 8),
       V(3, 8), V(4, 8), V(5, 8),
       V(6, 8), V(7, 8), V(8, 8);

  // Enforce the rank-2 constraint by performing SVD again and setting the smallest singular value to 0
  Eigen::JacobiSVD<Eigen::MatrixXd> svdF(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U = svdF.matrixU();
  Eigen::MatrixXd S = svdF.singularValues().asDiagonal();
  Eigen::MatrixXd Vt = svdF.matrixV().transpose();
  S(2, 2) = 0;
  F = U * S * Vt;
  // Unnormalize F
  F = normal_mat2.transpose() * F * normal_mat1;

  return F;
}


/* Normalization of matrix of points*/
Eigen::Matrix3d Matcher::normalize_matrix(const Eigen::MatrixXd& points) const {
  Eigen::Matrix3d normal_mat = Eigen::Matrix3d::Identity();

  double meanX = points.row(0).mean();
  double meanY = points.row(1).mean();
  // std::cout << "Mean x: \n" << meanX << '\n' << '\n';
  // std::cout << "Mean y: \n" << meanY << '\n' << '\n';

  double scale = std::sqrt(2) / points.colwise().norm().mean();
  // std::cout << points.colwise().norm() << '\n' << '\n';
  // std::cout << "Scale: \n" << scale << '\n' << '\n';

  normal_mat(0, 0) = scale;
  normal_mat(1, 1) = scale;
  normal_mat(0, 2) = -scale * meanX;
  normal_mat(1, 2) = -scale * meanY;

  return normal_mat;
}

bool Matcher::check_epipolar(const Eigen::Matrix3d& F) const {
  double sum = 0;
  for (int i = 0; i < 8; ++i) {
       double epipolar_constr = set_mov_.col(i).transpose() * F * set_org_.col(i);
       std::cout << "Point index " << i << ", epipolar constraint = " << epipolar_constr << '\n' << '\n';
       sum += epipolar_constr;
  }

  if (std::abs(sum) > 1e-5) {
    return false;
  } else {
    return true;
  }
}