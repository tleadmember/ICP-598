#include "icp.h"
#include <iostream>
#include <chrono>
#include <nanoflann.hpp>
 
 /* Function icp() takes in original set of points and moved set of points, respectively*/
Eigen::Matrix<double, 3, 3> icp(const Eigen::MatrixXd& set_org, 
                                const Eigen::MatrixXd& set_mov)
{
  // Start timing
  auto start = std::chrono::system_clock::now();


  // Print out arguments given
  // std::cout << "Original points set: \n" << set_org << '\n' << '\n';
  // std::cout << "Moved points set: \n" << set_mov << '\n' << '\n';


  /* Data association (nearest neighbor search) */
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> my_matrix_type;
  typedef nanoflann::KDTreeEigenMatrixAdaptor<my_matrix_type, 3 /*dimension*/, nanoflann::metric_L2, false> my_kd_tree_t; // false = not row_major
  my_kd_tree_t mat_index(3, std::cref(set_mov), /* max leaf */ 10);
  int num_neighbors = 1;
  Eigen::MatrixXi mat_idx_nn(num_neighbors, set_org.cols()); // initialize matrix to hold indices of nn's
  for (int i = 0; i < set_org.cols(); i++)
  {
    // Query point in set of origninal points
    std::vector<double> qp{set_org(0, i), set_org(1, i), set_org(2, i)};
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
  Eigen::MatrixXd set_mov_reordered(3, set_org.cols());
  for (int i = 0; i < set_org.cols(); ++i) {
    const int idx = mat_idx_nn(0, i);
    set_mov_reordered.block(0, i, 3, 1) = set_mov.block(0, idx, 3, 1);
  }
  // std::cout << "Re-ordered moved points set: \n" << set_mov_reordered << '\n' << '\n';


  /* Calculate rotation matrix, using Kabsch-Umeyama algorithm */
  Eigen::MatrixXd M = set_org * (set_mov_reordered.transpose());
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


  // Finish timing and print out elapsed time in seconds
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  printf("Finished ICP in %.3f seconds!\n\n", elapsed_seconds.count());

  return R;
}