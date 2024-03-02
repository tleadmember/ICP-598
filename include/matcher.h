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

  // Other functions
  void print_raw_sets() const;
  Eigen::Matrix<double, 4, 4>   icp();
  Eigen::MatrixXd               nearest_neighbor() const;
  Eigen::Matrix<double, 3, 3>   kabsch_umeyama(const Eigen::MatrixXd&) const;

private:
  const Eigen::MatrixXd set_org_; // original set of points
  const Eigen::MatrixXd set_mov_; // moved set of points
  Eigen::MatrixXd set_org_centroided_; // original set of points, subtracted by its centroid
  Eigen::MatrixXd set_mov_centroided_; // moved set of points, subtracted by its centroid
};

#endif // MATCHER_H