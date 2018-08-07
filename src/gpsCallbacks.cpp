//Contains all callbacks related to the Lynx as well as a test callback for the Snap imu.
#include <Eigen/Geometry>
#include "estimationNode.hpp"
#include <string>
#include <iostream>

namespace gpsimu_odom
{

//Callback for the imu subscriber for the snapdragon.
//Publishing is disabled, pending microlift testing.
void estimationNode::mavrosImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //If the time offset has not been recorded
    if(~hasRosToUTC_)
    {
        return;
    }

    static double tLastImuS=0;
    static int counterS=0;
    static uint64_t imuSeqS=0;
    static Eigen::Vector3d ba0Snap=Eigen::Vector3d(0,0,0);
    static Eigen::Vector3d bg0Snap=Eigen::Vector3d(0,0,0);

    double tLastProcessed = snapHelper_.getTLastProc();

    double thisTime = (msg->header.stamp).toSec() + snapHelper_.getTOffset();

    if(thisTime - tLastImuS<=1e-9) //Note: NOT abs(dt), as this checks whether or not messages are received out of order as well
    {
        std::cout << "Error: 1ns between IMU measurements" << std::endl;
        return;
    }
    //Only update last time used IF this time is accepted
    
    Eigen::Vector3d imuAccelMeas, imuAttRateMeas;
    imuAccelMeas(0) = msg->linear_acceleration.x;
    imuAccelMeas(1) = msg->linear_acceleration.y;
    imuAccelMeas(2) = msg->linear_acceleration.z;
    imuAttRateMeas(0) = msg->angular_velocity.x;
    imuAttRateMeas(1) = msg->angular_velocity.y;
    imuAttRateMeas(2) = msg->angular_velocity.z;

    //rotate from measurement frame to body frame
    imuAccelMeas = SNAP_IMU_ROTATION*imuAccelMeas;
    imuAttRateMeas = SNAP_IMU_ROTATION*imuAttRateMeas;

    //Run CF if calibrated
    if(isCalibratedSnap_)
    {
        imuSeqS++;
        double dtLastProc = thisTime - tLastProcessed;
        if(dtLastProc>0) //Just in case a gps message is received late
        { 
            //Construct measurements
            const imuMeas thisImuMeas(thisTime,imuAccelMeas,imuAttRateMeas);
            snapHelper_.setLastImuMeas(thisImuMeas);

            imuFilterSnap_.runUKFpropagateOnly(tLastProcessed,thisImuMeas);

            //Publish
            updateType = "imu"; publishOdomAndMocap();
            //RBI_ is updated when publisher is called.

            //Cleanup
            snapHelper_.setTLastProc(thisTime);
        }
    }else if(rbiIsInitialized_)  //if RBI has been calculated but the biases have not been calculated
    { 
        ba0Snap=ba0Snap+0.01*(imuAccelMeas - RBI_.transpose()*Eigen::Vector3d(0,0,9.8)); //inefficient
        bg0Snap=bg0Snap+0.01*imuAttRateMeas;        
        counterS++;
        // Try ground calibration step for simplicity
        if(counterS>=100)
        {
            isCalibratedSnap_ = true;
            //tLastProcessed_ = thisTime;
            Eigen::Vector3d rI0 = rPrimaryMeas_ - RBI_.transpose()*Lcg2p_;
            Eigen::Matrix<double,15,1> xState;
            xState<<rI0(0),rI0(1),rI0(2), 0,0,0, 0,0,0, ba0Snap(0),ba0Snap(1),ba0Snap(2), bg0Snap(0),bg0Snap(1),bg0Snap(2);
            snapHelper_.setTLastProc(thisTime);
            imuFilterSnap_.setState(xState,RBI_);
        }
    }
}


void runGPS(const Eigen::Vector3d pose, const Eigen::Vector3d Ls2p, const double ttime)
{
    stream_ -> letStreamRunGPS(pose, Ls2p, ttime);
    return;
}


void estimationNode::letStreamRunGPS(const Eigen::Vector3d pose, const Eigen::Vector3d Ls2p, const double ttime)
{
    ROS_INFO("Combined GPS callback called");

    const gpsMeas thisGpsMeas(ttime,pose,Ls2p);
    //imuFilterLynx_.runUKF(lynxHelper_.getLastImuMeas(),thisGpsMeas);
    imuFilterSnap_.runUKF(snapHelper_.getLastImuMeas(),thisGpsMeas);

    //Publish messages
    updateType = "gps";
    publishOdomAndMocap();
    //RBI_ is updated when publisher is called.
    //lynxHelper_.setTLastProc(ttime);
    snapHelper_.setTLastProc(ttime);    
    return;
}


void runLynxIMU(const Eigen::Vector3d accel, const Eigen::Vector3d attRate, const double ttime)
{
    stream_ -> letStreamRunIMU(accel, attRate, ttime);
    return;
}


void estimationNode::letStreamRunIMU(const Eigen::Vector3d accel, const Eigen::Vector3d attRate, const double ttime)
{
    ROS_INFO("Combined IMU callback called");
    return;
}



//Publish local_odom and mavros mocap.  Called whenever a measurement is processed
void estimationNode::publishOdomAndMocap()
{
    Eigen::Matrix<double,15,15> Preport;
    imuFilterLynx_.getCovariance(Preport);

    //Update rotation matrix and force orthonormality  
    Eigen::Matrix<double,15,1> xState;
    Eigen::Vector3d imuAccel,imuAttRate;
    double tlastImuPub;
    imuFilterLynx_.getState(xState, RBI_);

    nav_msgs::Odometry localOdom_msg;

    //Generate message
    localOdom_msg.header.stamp = ros::Time::now();
    localOdom_msg.header.frame_id = "world";
    localOdom_msg.child_frame_id = "world";
    localOdom_msg.pose.pose.position.x = xState(0);
    localOdom_msg.pose.pose.position.y = xState(1);
    localOdom_msg.pose.pose.position.z = xState(2);
    localOdom_msg.twist.twist.linear.x = xState(3);
    localOdom_msg.twist.twist.linear.y = xState(4);
    localOdom_msg.twist.twist.linear.z = xState(5);
    Eigen::Quaterniond q0(RBI_); //RBI from lynx only
    localOdom_msg.pose.pose.orientation.x=q0.x();
    localOdom_msg.pose.pose.orientation.y=q0.y();
    localOdom_msg.pose.pose.orientation.z=q0.z();
    localOdom_msg.pose.pose.orientation.w=q0.w();

    //Remove biases
    lastImuMeasLynx_.getMeas(tlastImuPub,imuAccel,imuAttRate);
    localOdom_msg.twist.twist.angular.x=imuAttRate(0)-xState(12);
    localOdom_msg.twist.twist.angular.y=imuAttRate(1)-xState(13);
    localOdom_msg.twist.twist.angular.z=imuAttRate(2)-xState(14);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            localOdom_msg.pose.covariance[6*i + j] = Preport(i, j);
            localOdom_msg.twist.covariance[6*i + j] = Preport(3+i, 3+j);
            //Covariance of attitude rate is gaussian(IMU error) + gaussian(bias estimate)
            localOdom_msg.twist.covariance[6*i + j + 21] = QgyroOutput_(i, j) + Preport(12+i, 12+j);
        }
    }

    //Publish local odometry message
    localOdom_pub_.publish(localOdom_msg);

    //px4 mocap topic to align frames
    geometry_msgs::PoseStamped mocap_msg;
    mocap_msg.pose.position = localOdom_msg.pose.pose.position;
    mocap_msg.pose.orientation = localOdom_msg.pose.pose.orientation;
//    mocap_msg.header = msg->header;
    mocap_msg.header.frame_id = "fcu";
    mocap_pub_.publish(mocap_msg);
}


} //end namespace