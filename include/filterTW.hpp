#include <Eigen/Core>

namespace gpsimu_odom {

class KalmanTW
{
public:
    KalmanTW() {}
    KalmanTW(const Eigen::Matrix<double,7,1> &state, const Eigen::Matrix<double,7,7> &initial_cov,
             const Eigen::Matrix<double,4,4> &process_noise,
             const Eigen::Matrix<double,3,3> &meas_noise)
        : x(state), P(initial_cov), Q(process_noise), R(meas_noise) {}

    void initialize(const Eigen::Matrix<double,7,1> &state, const Eigen::Matrix<double,7,7> &initial_cov,
        const Eigen::Matrix<double,4,4> &process_noise,
        const Eigen::Matrix3d &meas_noise);

    //Process update every time a new mavlink message is received
    //Measurement update every time a new IMU measurement is received
    //RBI is vB=RBI*vW
    void processUpdate(double dt, Eigen::Matrix3d &RBI, const double thrott);
    void measurementUpdate(const Eigen::Vector3d &meas);
    void setState(const Eigen::Matrix<double,7,1> &new_state) {x = new_state;}
    void setEstimateCovariance(const Eigen::Matrix<double,7,7> &new_covariance) { P = new_covariance; }
    void setProcessNoise(const Eigen::Matrix<double,4,4> &process_noise) { Q = process_noise; }
    void setMeasurementNoise(const Eigen::Matrix<double,3,3> &meas_noise) {R = meas_noise;}

    const Eigen::Matrix<double,7,1> &getState() const { return x; }
    const Eigen::Matrix<double,7,7> &getCovariance() const { return P; }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Matrix<double,7,1> x;
    Eigen::Matrix<double,7,7> P;
    Eigen::Matrix<double,4,4> Q;
    Eigen::Matrix<double,3,3> R;
};

}  // namespace
