#ifndef ICP_H
#define ICP_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

Eigen::Matrix<double, 3, 3> icp(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

#endif // ICP_H