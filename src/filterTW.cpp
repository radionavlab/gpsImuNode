#include "filterTW.hpp"
#include <Eigen/LU>  // For matrix inverse

namespace gpsimu_odom{

void KalmanTW::initialize(const Eigen::Matrix<double,7,1> &state,
            const Eigen::Matrix<double,7,7> &initial_cov,
            const Eigen::Matrix<double,4,4> &process_noise,
            const Eigen::Matrix<double,3,3> &meas_noise)
{
  x = state;
  P = initial_cov;
  Q = process_noise;
  R = meas_noise;
}


void KalmanTW::processUpdate(double dt, Eigen::Matrix3d &RBI, const double thrott)
{
  //thrott=norm(u_commanded)=TW_0*throttle_[0,1]*m*g
  Eigen::Vector3d uT=RBI*Eigen::Vector3d(0,0,thrott);
  //A, 7x7
  Eigen::Matrix<double,4,4> A = Eigen::Matrix<double,4,4>::Identity();
  A(0,3) = dt; A(1,4)=dt; A(2,5)=dt;
  //B, 7x3
  Bmat_t B = Bmat_t::Zero();
  B.topRows(3) = dt*dt*0.5*Eigen::Matrix3d::Identity();
  B.middleRows(3,3) = Eigen::Matrix3d::Identity()*dt;
  //B*u*tw% is the effect of tw% on x(0-5) so should be the rightmost column of A(7x7)
  Eigen::Matrix<7,1> A_rightCol = B*uT;
  A.rightCols(1) = A_rightCol;
  A(6,6)=1;
  //noise matrix
  Eigen::Matrix<double,7,4> gamma = Eigen::Matrix<double,7,4>::Zero();
  gamma.topLeftCorner(3,3)=dt*dt*0.5*Eigen::Matrix3d::Identity();
  gamma.block(3,0,3,3)=dt*Eigen::Matrix3d::Identity();
  gamma(6,3)=1;

  //propagation
  x = A * x + B * Eigen::Vector3d(0,0,-9.81);
  P = A * P * A.transpose() + gamma*Q*gamma.transpose();
}

void KalmanTW::measurementUpdate(const Eigen::Vector3d &meas)
{
  Eigen::Matrix<double, 3, 7> H;
  H.setZero();
  H.leftCols(3) = Eigen::Matrix3d::Identity();

  const Eigen::Matrix<double, 7, 3> K =
      P * H.transpose() * (H * P * H.transpose() + R).inverse();
  const Eigen::Vector3d inno = meas - H * x;
  x += K * inno;
  P = (Eigen::Matrix<double,7,7>::Identity() - K * H) * P;
}

}  // namespace gps_odom
