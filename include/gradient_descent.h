#ifndef GRADIENT_DESCENT_H
#define GRADIENT_DESCENT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>

template <typename start_point_T>
class GradientDescent {
public:
  // Constructor
//   GradientDescent(double (*)(start_point_T), start_point_T&);
  GradientDescent(start_point_T&);
  void test_print() const;

private:
  start_point_T start_point_;
//   double objective_function_(start_point_T);
  std::function<double(start_point_T)> objective_function_;
};

#endif // GRADIENT_DESCENT_H