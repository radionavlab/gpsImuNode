#pragma once

#include "rosStreamendpoint.h"
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

class estimationNode;

class rosStreamEndpointGPSKF
{
public:
    rosStreamEndpointGPSKF() {
        hasAlreadyReceivedA2D=false;
        hasAlreadyReceivedRTK=false;
        gpsSec_=0;
        gpsWeek_=0;
        gpsFracSec_=0;
    }

    // More useful functions
    void configure(ros::NodeHandle &nh, Eigen::Vector3d baseECEF_vector_in,
            Eigen::Matrix3d Recef2enu_in);
    void donothing(); //compiler test
    void singleBaselineRTKCallback(const gbx_ros_bridge_msgs::SingleBaselineRTK::ConstPtr &msg);
    void attitude2DCallback(const gbx_ros_bridge_msgs::Attitude2D::ConstPtr &msg);
    void imuConfigCallback(const gbx_ros_bridge_msgs::ImuConfig::ConstPtr &msg);
    void lynxImuCallback(const gbx_ros_bridge_msgs::Imu::ConstPtr &msg);
    void navsolCallback(const gbx_ros_bridge_msgs::NavigationSolution::ConstPtr &msg);
    void tOffsetCallback(const gbx_ros_bridge_msgs::ObservablesMeasurementTime::ConstPtr &msg);    
    void setRosPointer(std::shared_ptr<EstimationNode> rosHandle);
    void runRosUKF(const Eigen::Vector3d pose, const Eigen::Vector3d Ls2p, const double ttime);

    void doSetRBI0(const Eigen::Matrix3d &RBI0);
    void doSetRprimary(const Eigen::Vector3d &rp);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    std::shared_ptr<EstimationNode> rosHandle_;
    bool validRTKtest, validA2Dtest, hasAlreadyReceivedA2D, hasAlreadyReceivedRTK;
    int gpsWeek_, gpsSec_, internalSeq, sec_in_week;;
    double gpsFracSec_, dtRX_, minTestStat, lastRTKtime, lastA2Dtime;
    Eigen::Quaterniond internalQuat;
    Eigen::Vector3d internalPose, baseECEF_vector, L_cg2p;
    Eigen::Matrix3d RBI, Recef2enu;

    // TO-DO: CLEAN UP DUPLICATES LATER
    std::string child_frame_id_;
    gpsImu imuFilterLynx_, imuFilterSnap_;
    filterHelper lynxHelper_;
    imuMeas lastImuMeasLynx_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    Eigen::Vector3d zeroInECEF_, rPrimaryMeas_, rS2PMeas_, rPrimaryMeas_mu, Lcg2p_, Ls2p_, Lcg2imu_;
    Eigen::Matrix3d Recef2enu_, Rwrw_, Recef2wrw_, RBI_, QgyroOutput_;
    Eigen::Matrix<double,21,3> rCtildeCalib_, rBCalib_;

    ros::Subscriber gps_sub_, rtkSub_, a2dSub_, imuSub_, imuConfigSub_, tOffsetSub_, navSub_;

    int internalSeq;
    double lastRTKtime_, lastA2Dtime_, minTestStat_, imuConfigAccel_, imuConfigAttRate_, tOffsetRosToUTC_,
        pi, sec_in_week_, dtRXinMeters_;
    bool validRTKtest_, validA2Dtest_, hasAlreadyReceivedA2D_, hasAlreadyReceivedRTK_, rbiIsInitialized_,
        isCalibratedLynx_, isCalibratedSnap_, publish_tf_, hasRosToUTC_;

    long long int tIndexConfig_;
    uint64_t sampleFreqNum_, sampleFreqDen_;

};