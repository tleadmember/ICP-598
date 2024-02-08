#ifndef ICP_H
#define ICP_H

#include <Eigen/Dense>

Eigen::Matrix<double, 4, 4> icp(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

#endif // ICP_H