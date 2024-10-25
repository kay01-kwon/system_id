#ifndef EIGNE_TYPE_DEF_HPP
#define EIGEN_TYPE_DEF_HPP

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using std::cout;
using std::endl;

using Eigen::Matrix;
using Eigen::Quaternion;

// Definition of Eigen vectors
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;

// Definition of Eigen matrices
typedef Eigen::Matrix<double, 2, 2> Matrix2d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;

// Definition of Eigen quaternions - float
typedef Eigen::Quaternion<float> Quaternionf;

#endif // EIGNE_TYPE_DEF_HPP