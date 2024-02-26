#ifndef MATCHER_H
#define MATCHER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

class Matcher {
public:
  // Constructor
  Matcher(const Eigen::MatrixXd &, const Eigen::MatrixXd &);

  // Getter functions
  Eigen::MatrixXd set_org() const;
  Eigen::MatrixXd set_mov() const;
  Eigen::Matrix<double, 8, 9> matrix_A() const;

  // Other functions
  void                          print_sets() const;
  Eigen::Matrix<double, 3, 3>   icp() const;
  Eigen::MatrixXd               nearest_neighbor() const;
  Eigen::Matrix<double, 3, 3>   kabsch_umeyama(const Eigen::MatrixXd&) const;
  void                          calc_A();

private:
  const Eigen::MatrixXd set_org_; // original set of points
  const Eigen::MatrixXd set_mov_; // moved set of points
  // matrix A in the linearization of the 8-point equation to min ||Ax-b||^2
  Eigen::Matrix<double, 8, 9> matrix_A_;
};

#endif // MATCHER_H