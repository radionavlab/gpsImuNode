#pragma once

#include "gbxstreamendpoint.h"
#include "typedefs.h"
#include "report.h"
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
namespace po = boost::program_options;

class estimationNode;

class GbxStreamEndpointGPSKF
{
public:
    GbxStreamEndpointGPSKF() {
        hasAlreadyReceivedA2D=false;
        hasAlreadyReceivedRTK=false;
        gpsSec_=0;
        gpsWeek_=0;
        gpsFracSec_=0;
    }

    // More useful functions
    virtual ~GbxStreamEndpointGPSKF();
    void configure(ros::NodeHandle &nh, Eigen::Vector3d baseECEF_vector_in,
            Eigen::Matrix3d Recef2enu_in);
    void donothing(); //compiler test
    void setRosPointer(std::shared_ptr<EstimationNode> rosHandle);
    void runRosUKF(const Eigen::Vector3d pose, const Eigen::Vector3d Ls2p, const double ttime);
    void doSetRBI0(const Eigen::Matrix3d &RBI0);
    void doSetRprimary(const Eigen::Vector3d &rp);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    virtual bool openSinkStream_() override;
    virtual void closeSinkStream_() override;
    virtual bool isValidSinkStream_() const;
    virtual bool isSinkEndpoint_() const { return true; }
    virtual bool isProcessEndpoint_() const { return true; }
    virtual bool writeBytes_(const u8* buffer, size_t size);

    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
            std::shared_ptr<const ReportCoda>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportSingleBaselineRtk>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportMultiBaselineRtkAttitude2D>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportImu>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportImuConfig>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportNavigationSolution>&& pReport, const u8 streamId);
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
    filterHelper lynxHelper_, snapHelper_;
    imuMeas lastImuMeasLynx_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    Eigen::Vector3d zeroInECEF_, rPrimaryMeas_, rS2PMeas_, rPrimaryMeas_mu, Lcg2p_, Ls2p_, Lcg2imu_;
    Eigen::Matrix3d Recef2enu_, Rwrw_, Recef2wrw_, RBI_, QgyroOutput_;
    Eigen::Matrix<double,21,3> rCtildeCalib_, rBCalib_;

    ros::Subscriber gps_sub_, rtkSub_, a2dSub_, imuSub_, imuConfigSub_, tOffsetSub_, navSub_, mavrosImuSub_;

    int internalSeq;
    double lastRTKtime_, lastA2Dtime_, minTestStat_, imuConfigAccel_, imuConfigAttRate_, tOffsetRosToUTC_,
        pi, sec_in_week_, dtRXinMeters_;
    bool validRTKtest_, validA2Dtest_, hasAlreadyReceivedA2D_, hasAlreadyReceivedRTK_, rbiIsInitialized_,
        isCalibratedLynx_, isCalibratedSnap_, publish_tf_, hasRosToUTC_;

    long long int tIndexConfig_;
    uint64_t sampleFreqNum_, sampleFreqDen_;

};