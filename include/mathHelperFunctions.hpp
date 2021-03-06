#pragma once
//Contains basic math functions used across all included files
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <stdio.h>
#include <iostream>
#include "constants.hpp"

namespace gpsimu_odom
{
  //Helper functions
  Eigen::Matrix3d ecef2enu_rotMatrix(Eigen::Vector3d &ECEF);
  Eigen::Vector3d ecef2enu(Eigen::Vector3d &ECEF);
  Eigen::Matrix3d updateRBIfromGamma(const Eigen::Matrix3d &R0, const Eigen::Vector3d &gamma);
  void updateRBIfromGamma(Eigen::Matrix3d &R0, Eigen::Matrix<double,15,1> &x0);
  Eigen::Matrix3d hatmat(const Eigen::Vector3d &v1);
  Eigen::Matrix3d rotMatFromEuler(const Eigen::Vector3d &ee);
  Eigen::Matrix3d rotMatFromWahba(const Eigen::VectorXd &weights, const::Eigen::MatrixXd &vI,
     const::Eigen::MatrixXd &vB);
  Eigen::Vector3d unit3(const Eigen::Vector3d &v1);
  Eigen::Matrix3d orthonormalize(const Eigen::Matrix3d &inmat);
  void saturateBiases(const double baMax, const double bgMax);
  double symmetricSaturationDouble(const double inval, const double maxval);
  Eigen::Matrix3d euler2dcm312(const Eigen::Vector3d &ee);
  double tgpsToSec(const int week, const int secOfWeek, const double fracSec);

}  // namespace gps_odom

