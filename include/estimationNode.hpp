#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <gbx_ros_bridge_msgs/SingleBaselineRTK.h>
#include <gbx_ros_bridge_msgs/Attitude2D.h>
#include <gbx_ros_bridge_msgs/Imu.h>
#include <gbx_ros_bridge_msgs/ImuConfig.h>
#include <gbx_ros_bridge_msgs/NavigationSolution.h>
#include <gbx_ros_bridge_msgs/ObservablesMeasurementTime.h>
#include <cmath>
#include <string>
#include <iostream>

#include "classes.hpp" //Included in filterImu.hpp
#include "filterImu.hpp"
#include "mathHelperFunctions.hpp"
#include "constants.hpp"

//gbx includes
#include "gbxstreamendpointin.h"
#include "gbxstream.h"
#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace gpsimu_odom
{
class estimationNode
{
 public:
    estimationNode(ros::NodeHandle &nh);

    //ROS stuff
    GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportSingleBaselineRtk>&& pReport, const u8 streamId);
    GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportMultiBaselineRtkAttitude2D>&& pReport, const u8 streamId);
    GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportImu>&& pReport, const u8 streamId);
    GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportImuConfig>&& pReport, const u8 streamId);
    GbxStreamEndpoint::ProcessReportReturn processReport_(
                std::shared_ptr<const ReportNavigationSolution>&& pReport, const u8 streamId);
    void tOffsetCallback(const gbx_ros_bridge_msgs::ObservablesMeasurementTime::ConstPtr &msg);
    void mavrosImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void publishOdomAndMocap();

 private:
    void PublishTransform(const geometry_msgs::Pose &pose,
                                                const std_msgs::Header &header,
                                                const std::string &child_frame_id);

    ros::Publisher localOdom_pub_, mocap_pub_;
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

    std::string updateType;

};

} // gps_odom
