#pragma once
#include <Eigen/Geometry>

//const int SEC_PER_WEEK(604800);
const double SPEED_OF_LIGHT(299792458);
const Eigen::Vector3d GRAVITY_IN_INTERTIAL_FRAME(0,0,-9.81);
const Eigen::MatrixXd LYNX_IMU_ROTATION = (Eigen::MatrixXd(3,3) << 0,-1,0, -1,0,0, 0,0,-1).finished();
const Eigen::MatrixXd SNAP_IMU_ROTATION = (Eigen::MatrixXd(3,3) << 1,0,0, 0,1,0, 0,0,1).finished();
const double MATH_PI(std::atan(1.0)*4.0);

