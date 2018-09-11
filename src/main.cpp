#include <Eigen/Geometry>
#include "estimationNode.hpp"
#include "gbxStreamEndpointGPSKF.hpp"
#include "rosStreamEndpointGPSKF.hpp"
#include "rosPlayback.hpp"
#include <string>
#include <iostream>
#include "mathHelperFunctions.hpp"

const char strSIGTERM[] = "SIGTERM";
const char strSIGINT[] = "SIGINT";
const char strSIGHUP[] = "SIGHUP";
const char *ptrSigString = nullptr;
static volatile sig_atomic_t sigterm_caught = 0;
extern "C" void signalHandler(int signum) {
    if(!sigterm_caught) {
        if(signum == SIGTERM || signum == SIGINT || signum == SIGHUP) {
            if(!ptrSigString) {
                if(signum == SIGTERM) ptrSigString = strSIGTERM;
                if(signum == SIGINT) ptrSigString = strSIGINT;
                if(signum == SIGHUP) ptrSigString = strSIGHUP;
            }
        sigterm_caught = 1;
        }
    }
}


int main(int argc, char **argv)
{
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGHUP, signalHandler);
    std::signal(SIGQUIT, signalHandler);
    ros::init(argc, argv, "gpsimu_odom");

    ros::NodeHandle nh;

    //is LYNX_IMU to be used?
    bool LYNX_IMU, SNAP_IMU;

    //gpsimu_odom::estimationNode gpsimu_odom(nh);
    auto gpsimu = std::make_shared<gpsimu_odom::estimationNode>(nh);

    std::string quadName = ros::this_node::getName();

    //Determine whether to create a GBXStream or a ROSStream
    int mode;
    ros::param::get(quadName + "/nodeType", mode);

    //Common setup
    Eigen::Vector3d baseECEF_vector;
    ros::param::get(quadName + "/arenaCenterX", baseECEF_vector(0));
    ros::param::get(quadName + "/arenaCenterY", baseECEF_vector(1));
    ros::param::get(quadName + "/arenaCenterZ", baseECEF_vector(2));
    Eigen::Matrix3d Recef2enu = gpsimu_odom::ecef2enu_rotMatrix(baseECEF_vector);

    double thetaDeg, thetaRad;
    ros::param::get(quadName + "/thetaWRW", thetaDeg);
    thetaRad = thetaDeg*4.0*atan(1.0)/180.0;
    Eigen::Matrix3d Rwrw(Eigen::Matrix3d::Zero());
    Rwrw(2,2)=0.0;
    Rwrw(0,0)=cos(thetaRad);
    Rwrw(0,1)=-sin(thetaRad);
    Rwrw(1,0)=sin(thetaRad);
    Rwrw(1,1)=cos(thetaRad);

    if(mode==1) //online gbx stream
    {
        int gbxport;

        ros::param::get(quadName + "/gbxport",gbxport);
        ros::param::get(quadName + "/runLynx",LYNX_IMU);
        ros::param::get(quadName + "/runSnap",SNAP_IMU);
        
        auto gbxStream = std::make_shared<GbxStream>();
        gbxStream->pauseStream();

        int port=gbxport;
        auto epOutput = std::make_shared<GbxStreamEndpointGPSKF>();
        epOutput->configure(nh, baseECEF_vector, Recef2enu, Rwrw);
        epOutput->setRosPointer(gpsimu);
        //gpsimu->setGbxPointer(epOutput); //not currently needed
        //epOutput->donothing(); //test
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::CODA);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::SINGLE_BASELINE_RTK);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::ATTITUDE_2D);
        if(LYNX_IMU) //only process imu reports if lynx is to be used
        {
            epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::IMU);
            epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::IMU_CONFIG);
            epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::NAVIGATION_SOLUTION);
        }
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).enableWhitelist();
    
        //make endpoint
        auto epInput = std::make_shared<GbxStreamEndpointIN>("192.168.2.2",port);
        gbxStream->resumeStream();
    }
    else if(mode==2)  //online ros stream
    {
        auto rosStream = std::make_shared<rosStreamEndpointGPSKF>();
        rosStream->configure(nh, baseECEF_vector, Recef2enu,Rwrw);
        rosStream->setRosPointer(gpsimu);
    }else if(mode==3) //online ros stream from vicon
    {
    
    }else if(mode==4) //post-process a ros stream (rosbag)
    {
        auto rosStream = std::make_shared<rosPlayback>();
        rosStream->configure(nh,baseECEF_vector,Recef2enu,Rwrw);
        rosStream->setRosPointer(gpsimu);
    }


    ros::spin();
    
    return 0;
}



