#pragma once

#include "typedefs.h"
#include "report.h"
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <gbx_ros_bridge_msgs/SingleBaselineRTK.h>
#include <gbx_ros_bridge_msgs/Attitude2D.h>
#include <gbx_ros_bridge_msgs/Imu.h>
#include <gbx_ros_bridge_msgs/ImuConfig.h>
#include <gbx_ros_bridge_msgs/NavigationSolution.h>
#include <gbx_ros_bridge_msgs/ObservablesMeasurementTime.h>
#include "classes.hpp"

//Namespace'd forward declaration
namespace gpsimu_odom
{
    class estimationNode;
}

class viconStream
{
public:
    viconStream() {
        hasAlreadyReceivedA2D_=false;
        hasAlreadyReceivedRTK_=false;
        gpsSec_=0;
        gpsWeek_=0;
        gpsFracSec_=0;
        hasRosHandle=false;
    }

    // More useful functions
    void configure(ros::NodeHandle &nh, Eigen::Vector3d &baseECEF_vector_in,
            Eigen::Matrix3d &Recef2enu_in, Eigen::Matrix3d &Rwrw);
    void donothing(); //test function
    void singleBaselineRTKCallback(const gbx_ros_bridge_msgs::SingleBaselineRTK::ConstPtr &msg);
    void attitude2DCallback(const gbx_ros_bridge_msgs::Attitude2D::ConstPtr &msg);
    void imuConfigCallback(const gbx_ros_bridge_msgs::ImuConfig::ConstPtr &msg);
    void lynxImuCallback(const gbx_ros_bridge_msgs::Imu::ConstPtr &msg);
    void navsolCallback(const gbx_ros_bridge_msgs::NavigationSolution::ConstPtr &msg);
    void tOffsetCallback(const gbx_ros_bridge_msgs::ObservablesMeasurementTime::ConstPtr &msg);    
    void setRosPointer(std::shared_ptr<gpsimu_odom::estimationNode> rosHandle);
    void runRosUKF(const Eigen::Vector3d pose, const Eigen::Vector3d Ls2p, const double ttime);
    void runRosUKFPropagate(const Eigen::Vector3d acc, const Eigen::Vector3d att, const double ttime);
    
    void doSetRBI0(const Eigen::Matrix3d &RBI0);
    void doSetRprimary(const Eigen::Vector3d &rp);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    std::shared_ptr<gpsimu_odom::estimationNode> rosHandle_;
    bool validRTKtest_, validA2Dtest_, hasAlreadyReceivedA2D_, hasAlreadyReceivedRTK_, hasRosHandle, LYNX_IMU;
    int gpsWeek_, gpsSec_, internalSeq, sec_in_week;;
    double gpsFracSec_, dtRX_, minTestStat, lastRTKtime, lastA2Dtime;
    Eigen::Quaterniond internalQuat;
    Eigen::Vector3d internalPose, baseECEF_vector, L_cg2p;
    Eigen::Matrix3d RBI, Recef2enu;

    std::string child_frame_id_;
    //an internal helper is used to contain the time offset from ROSTIME to UTC
    filterHelper lynxHelper_;
    imuMeas lastImuMeasLynx_;

    Eigen::Vector3d zeroInECEF_, rPrimaryMeas_, rS2PMeas_, rPrimaryMeas_mu, Lcg2p_, Ls2p_, Lcg2imu_;
    Eigen::Matrix3d Recef2enu_, Rwrw_, Recef2wrw_, RBI_, QgyroOutput_;
    Eigen::Matrix<double,21,3> rCtildeCalib_, rBCalib_;

    ros::Subscriber gps_sub_, rtkSub_, a2dSub_, imuSub_, imuConfigSub_, tOffsetSub_, navSub_;

    double lastRTKtime_, lastA2Dtime_, minTestStat_, imuConfigAccel_, imuConfigAttRate_, tOffsetRosToUTC_,
        pi, sec_in_week_, dtRXinMeters_;
    bool rbiIsInitialized_, isCalibratedLynx_, isCalibratedSnap_, publish_tf_, hasRosToUTC_;
    long long int tIndexConfig_;
    uint64_t sampleFreqNum_, sampleFreqDen_;

};


