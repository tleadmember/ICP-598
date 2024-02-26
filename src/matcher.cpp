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


/* Function icp() takes in original set of points and moved set of points,
respectively*/
Eigen::Matrix<double, 3, 3> Matcher::icp() const {
  // Start timing
  auto start = std::chrono::system_clock::now();

  // Print out arguments given
  // std::cout << "Original points set: \n" << set_org_ << '\n' << '\n';
  // std::cout << "Moved points set: \n" << set_mov_ << '\n' << '\n';

  Eigen::MatrixXd set_mov_reordered = nearest_neighbor();

  Eigen::Matrix<double, 3, 3> R = kabsch_umeyama(set_mov_reordered);

  // Finish timing and print out elapsed time in seconds
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  printf("Finished ICP in %.3f seconds!\n\n", elapsed_seconds.count());

  return R;
}


/* Function nearest_neighbor() returns a matrix of points being the corresponding 
closest neighbor of each of the query points in original set (data association )*/
Eigen::MatrixXd Matcher::nearest_neighbor() const {
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> my_matrix_type;
  typedef nanoflann::KDTreeEigenMatrixAdaptor<my_matrix_type, 3 /*dimension*/, nanoflann::metric_L2, false> my_kd_tree_t; // false = not row_major
  my_kd_tree_t mat_index(3, std::cref(set_mov_), /* max leaf */ 10);
  int num_neighbors = 1;
  Eigen::MatrixXi mat_idx_nn(num_neighbors, set_org_.cols()); // initialize matrix to hold indices of nn's

  for (int i = 0; i < set_org_.cols(); i++)
  {
    // Query point in set of origninal points
    std::vector<double> qp{set_org_(0, i), set_org_(1, i), set_org_(2, i)};
    // Search for nn of query point
    std::vector<size_t> idx_nn(num_neighbors);
    std::vector<double> dists_nn(num_neighbors); // not used
    nanoflann::KNNResultSet<double> resultSet(num_neighbors);
    resultSet.init(&idx_nn[0], &dists_nn[0]);
    mat_index.index->findNeighbors(resultSet, &qp[0], nanoflann::SearchParams(10));
    // Save indices of nn to matrix
    for (int j = 0; j < num_neighbors; ++j) {
      mat_idx_nn(j, i) = idx_nn[j];
    }
  }
  // std::cout << "Matrix of indices in moved set (nn's of points in original set): \n" << mat_idx_nn << '\n' << '\n';

  // Reorder points in moved set to be in corresponding nn order to original set
  Eigen::MatrixXd set_mov_reordered(3, set_org_.cols());
  for (int i = 0; i < set_org_.cols(); ++i) {
    const int idx = mat_idx_nn(0, i);
    set_mov_reordered.block(0, i, 3, 1) = set_mov_.block(0, idx, 3, 1);
  }
  // std::cout << "Re-ordered moved points set: \n" << set_mov_reordered << '\n' << '\n';

  return set_mov_reordered;
}


/* Function kabsch_umeyama() return the estimated rotation matrix */
Eigen::Matrix<double, 3, 3> Matcher::kabsch_umeyama(const Eigen::MatrixXd& set_mov_reordered) const {
  Eigen::MatrixXd M = set_org_ * (set_mov_reordered.transpose());
  // Calculate SVD of M
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd V = svd.matrixU();
  // Eigen::MatrixXd S = svd.singularValues().asDiagonal();
  Eigen::MatrixXd W = svd.matrixV();
  // std::cout << "V matrix in VSWt: \n" << V << '\n' << '\n';
  // std::cout << "W matrix in VSWt: \n" << W << '\n' << '\n';
  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3,3);
  S.diagonal().setOnes();
  // Calculate determinent of V*W
  double det = (W*(V.transpose())).determinant();
  if (det < 0) S(2, 2) = -S(2, 2);
  // std::cout << "S tilde matrix: \n" << S << '\n' << '\n';
  // Calculate the rotation matrix
  Eigen::Matrix<double, 3, 3> R = W*S*(V.transpose());
  // std::cout << "Calculated rotation matrix: \n" << R << '\n' << '\n';
  // std::cout << "Determinant of rotation transformation: \n" << R.determinant() << '\n' << '\n';
  return R;
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