#include "viconStream.hpp"
#include "navtoolbox.h"
#include <sys/time.h>
#include "mathHelperFunctions.hpp"
#include "estimationNode.hpp"


void viconStream::configure(ros::NodeHandle &nh, Eigen::Vector3d &baseECEF_vector_in,
            Eigen::Matrix3d &Recef2enu_in, Eigen::Matrix3d &Rwrw)
{
    std::string GPSKFName, posePubTopic;
    GPSKFName = ros::this_node::getName();
    Recef2enu = Recef2enu_in;
    baseECEF_vector_in = baseECEF_vector;
    std::string viconTopic;
    Rwrw_=Rwrw;

    ros::param::get(GPSKFName + "/posePubTopic", posePubTopic);
    ros::param::get(GPSKFName + "/minimumTestStat",minTestStat);
    ros::param::get(GPSKFName + "/runLynx",LYNX_IMU);
    ros::param::get(GPSKFName + "/viconTopic",viconTopic);

    //no harm in TCP for playback
    rtkSub_ = nh.subscribe(viconTopic,1,&viconStream::viconCallback,
                                        this, ros::TransportHints().unreliable().tcpNoDelay());
}


void viconStream::setRosPointer(std::shared_ptr<gpsimu_odom::estimationNode> rosHandle)
{
    rosHandle_=rosHandle;
    hasRosHandle=true;
    return;
}

void viconStream::donothing()
{
    std::cout << "Do nothing called" << std::endl;
    return;
}


//vicon
void viconStream::viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    if(!hasRosHandle)
    {return;}
    Eigen::Quaterniond quat;
    quat.x() = msg->transform.rotation.x;
    quat.y() = msg->transform.rotation.y;
    quat.z() = msg->transform.rotation.z;
    quat.w() = msg->transform.rotation.w;
    //all filters use a single constrained baseline
    Eigen::Matrix3d RR(quat);
    Eigen::Vector3d Ls2p=RR*Eigen::Vector3d(1,0,0);
    rPrimaryMeas_(0) = msg->transform.translation.x;
    rPrimaryMeas_(1) = msg->transform.translation.y;
    rPrimaryMeas_(2) = msg->transform.translation.z;
    
    double ttime = (ros::Time::now()).toSec();

    if(ttime > lastRTKtime_)  //only use newest time
    {
        lastRTKtime_=ttime;
        if(!rbiIsInitialized_)
        {
            //Doing a single iteration is sufficient with mm-level accuracy
            Eigen::Matrix<double,1,3> Lvert(0,0,1);
            Eigen::Matrix<double,2,3> rB, rC;
            Eigen::Matrix<double,2,1> weights(1,1);
            rC.topRows(1)=Lvert;
            rB.topRows(2)=Lvert;
            rC.bottomRows(1)=Ls2p.transpose();
            rB.bottomRows(1)=Eigen::Matrix<double,1,3>(1,0,0);
            RBI_ = gpsimu_odom::rotMatFromWahba(weights, rC,rB);
            doSetRBI0(RBI_);
            rbiIsInitialized_=true;
            doSetRprimary(rPrimaryMeas_);
        }else
        {
            runRosUKF(rPrimaryMeas_,Ls2p,ttime);
        }
    }
    return;
}


void viconStream::runRosUKF(const Eigen::Vector3d pose, const Eigen::Vector3d Ls2p, const double ttime)
{
    rosHandle_->letStreamRunGPS(pose, Ls2p, ttime);
}


void viconStream::doSetRBI0(const Eigen::Matrix3d &RBI0)
{
    rosHandle_->letStreamSetRBI(RBI0);
}


void viconStream::doSetRprimary(const Eigen::Vector3d &rp)
{
    rosHandle_->letStreamSetRprimary(rp);
}


