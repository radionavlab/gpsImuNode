#include <Eigen/Geometry>
#include "estimationNode.hpp"
#include "gbxStreamEndpointGPSKF.hpp"
#include <string>
#include <iostream>

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
    ros::spin();

    int gbxport;
    Eigen::Vector3d baseECEF_vector;
    ros::param::get(quadName + "/arenaCenterX", baseECEF_vector(0));
    ros::param::get(quadName + "/arenaCenterY", baseECEF_vector(1));
    ros::param::get(quadName + "/arenaCenterZ", baseECEF_vector(2));
    ros::param::get(quadName + "/gbxport",gbxport);
    ros::param::get(quadName + "/runLynx",LYNX_IMU);
    ros::param::get(quadName + "/runSnap",SNAP_IMU);
    Eigen::Matrix3d Recef2enu = ecef2enu_rotMatrix(baseECEF_vector);
    auto gbxStream = std::make_shared<GbxStream>();
    gbxStream->pauseStream();

    int port = gbxport;
    auto epOutput = std::make_shared<GbxStreamEndpointGPSKF>();
    epOutput->configure(nh, baseECEF_vector, Recef2enu);
    epOutput->setRosPointer(gpsimu);
    //gpsimu->setGbxPointer(epOutput); //not currently needed
    //epOutput->donothing(); //test
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::CODA);
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::SINGLE_BASELINE_RTK);
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::ATTITUDE_2D);
    if(LYNX_IMU) //only process lynx reports if lynx is to be used
    {
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::IMU);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::IMU_CONFIG);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::NAVIGATION_SOLUTION);
    }
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).enableWhitelist();
    
    //make endpoint
    auto epInput = std::make_shared<GbxStreamEndpointIN>(port, OptionObject::protocol_enum::IP_UDP, OptionObject::peer_type_enum::ROVER);
    gbxStream->resumeStream();
    
    return 0;
}