#ifndef VARIABLE_TYPE_HPP
#define VARIABLE_TYPE_HPP
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using std::cout;
using std::endl;

using Eigen::Matrix;

typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;

#endif