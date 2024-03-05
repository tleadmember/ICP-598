#ifndef GRADIENT_DESCENT_H
#define GRADIENT_DESCENT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>

template <typename start_point_T>
class GradientDescent {
public:
  // Constructor
  GradientDescent(double (*)(start_point_T), start_point_T&, double&, double&, double&);

  // Getters
  double min() const;
  start_point_T arg_min() const;

  // Other functions
  void test_print() const;
  // double central_difference() const;
  Eigen::MatrixXd numerical_gradient() const;
  void calculate();

private:
  start_point_T start_point_;
  start_point_T next_point_;
  const long int num_elms_; // number of "elements in each start point"
  std::function<double(start_point_T)> objective_function_;
  double step_size_;
  double h_;
  double minimum_;
  double epsilon_;
  int count_;
};

#endif // GRADIENT_DESCENT_H