#include "icp.h"
#include <iostream>
#include <chrono>
#include <nanoflann.hpp>
 
 /* Function icp() takes in original set of points and moved set of points, respectively*/
Eigen::Matrix<double, 4, 4> icp(const Eigen::MatrixXd& set_org, const Eigen::MatrixXd& set_mov)
{
  // Start timing
  auto start = std::chrono::system_clock::now();

  // Initializations
  Eigen::Matrix<double, 4, 4> result_T; // result transformation

  // Print out arguments given
  std::cout << "Original points set: \n" << set_org << '\n' << '\n';
  std::cout << "Moved points set: \n" << set_mov << '\n' << '\n';

  /* Data association (nearest neighbor search) */
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> my_matrix_type;
  typedef nanoflann::KDTreeEigenMatrixAdaptor<my_matrix_type, 3 /*dimension*/, nanoflann::metric_L2, false> my_kd_tree_t; // false = not row_major
  my_kd_tree_t mat_index(3, std::cref(set_org), /* max leaf */ 10);
  int num_neighbors = 1;
  Eigen::MatrixXi mat_idx_nn(num_neighbors, set_mov.cols()); // initialize matrix to hold indices of nn's
  for (int i = 0; i < set_mov.cols(); i++)
  {
    // Query point in set of moved points
    std::vector<double> qp{set_mov(0, i), set_mov(1, i), set_mov(2, i)};
    // Search for nn of query point
    std::vector<size_t> idx_nn(num_neighbors);
    std::vector<double> dists_nn(num_neighbors); // not used
    nanoflann::KNNResultSet<double> resultSet(num_neighbors);
    resultSet.init(&idx_nn[0], &dists_nn[0]);
    mat_index.index->findNeighbors(resultSet, &qp[0], nanoflann::SearchParams(10));
    // Save indices of nn to matrix
    for (int j = 0; j < num_neighbors; j++)
    {
      mat_idx_nn(j, i) = idx_nn[j];
    }
  }
  std::cout << "Matrix of indices in moved set (nn's of points in original set): \n" << mat_idx_nn << '\n' << '\n';
  

  // Finish timing and print out elapsed time in seconds
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  printf("Finished ICP in %.3f seconds!\n", elapsed_seconds.count());

  return result_T;
}