#include "rosPlayback.hpp"
#include "navtoolbox.h"
#include <sys/time.h>
#include "mathHelperFunctions.hpp"
#include "estimationNode.hpp"


void rosStreamEndpointGPSKF::configure(ros::NodeHandle &nh, Eigen::Vector3d baseECEF_vector_in,
            Eigen::Matrix3d Recef2enu_in)
{
    std::string GPSKFName, posePubTopic;
    GPSKFName = ros::this_node::getName();
    Recef2enu = Recef2enu_in;
    baseECEF_vector_in = baseECEF_vector;

    ros::param::get(GPSKFName + "/posePubTopic", posePubTopic);
    ros::param::get(GPSKFName + "/minimumTestStat",minTestStat);
    ros::param::get(GPSKFName + "/runLynx",LYNX_IMU);

    rtkSub_ = nh.subscribe("SingleBaselineRTK",10,&rosStreamEndpointGPSKF::singleBaselineRTKCallback,
                                        this, ros::TransportHints().tcpNoDelay());
    a2dSub_ = nh.subscribe("Attitude2D",10,&rosStreamEndpointGPSKF::attitude2DCallback,
                                        this, ros::TransportHints().tcpNoDelay());
    imuSub_ = nh.subscribe("IMU",10, &rosStreamEndpointGPSKF::lynxImuCallback,
                                        this, ros::TransportHints().tcpNoDelay());
    imuConfigSub_ = nh.subscribe("IMUConfig",10, &rosStreamEndpointGPSKF::imuConfigCallback,
                                        this, ros::TransportHints().tcpNoDelay());
    navSub_ = nh.subscribe("NavigationSolution",10,&rosStreamEndpointGPSKF::navsolCallback,
                                        this, ros::TransportHints().tcpNoDelay());
    tOffsetSub_ = nh.subscribe("ObservablesMeasurementTime",10,&rosStreamEndpointGPSKF::tOffsetCallback,
                                        this, ros::TransportHints().tcpNoDelay());
}


void rosStreamEndpointGPSKF::setRosPointer(std::shared_ptr<gpsimu_odom::estimationNode> rosHandle)
{
    rosHandle_=rosHandle;
    hasRosHandle=true;
    return;
}

void rosStreamEndpointGPSKF::donothing()
{
    std::cout << "Do nothing called" << std::endl;
    return;
}


//A2D callback.  Takes in message from A2D, synchronizes with message from A2D, then calls UKF update
void rosStreamEndpointGPSKF::attitude2DCallback(const gbx_ros_bridge_msgs::Attitude2D::ConstPtr &msg)
{
    if(~hasRosHandle)
    {return;}
    int week, secOfWeek;
    double fracSec, dtRX;
    dtRX_=msg->deltRSec;
    week = msg->tSolution.week;
    secOfWeek = msg->tSolution.secondsOfWeek;
    fracSec = msg->tSolution.fractionOfSecond;
    double ttime=gpsimu_odom::tgpsToSec(week,secOfWeek,fracSec) - dtRX;
    static int rCCalibCounter=0;
    static int calibSamples=20;

    //Ignore zero messages
    if(msg->tSolution.week<=1)
    {return;}

    if(!rbiIsInitialized_ && msg->testStat>=100)
    {
        const Eigen::Vector3d constrainedBaselineECEF(msg->rx, msg->ry, msg->rz);
        const Eigen::Vector3d constrainedBaselineI(gpsimu_odom::unit3(Rwrw_*Recef2enu_*constrainedBaselineECEF));
        rCtildeCalib_(rCCalibCounter%calibSamples,0)=constrainedBaselineI(0);
        rCtildeCalib_(rCCalibCounter%calibSamples,1)=constrainedBaselineI(1);
        rCtildeCalib_(rCCalibCounter%calibSamples,2)=constrainedBaselineI(2);
        rBCalib_(rCCalibCounter%calibSamples,0)=1;
        rBCalib_(rCCalibCounter%calibSamples,1)=0;
        rBCalib_(rCCalibCounter%calibSamples,2)=0;
        //rCB=???
        rCCalibCounter++;

        //estimate initial RBI from wahba
        if(rCCalibCounter>=calibSamples)
        {
            rCtildeCalib_(calibSamples,0)=0; rBCalib_(calibSamples,0)=0;
            rCtildeCalib_(calibSamples,1)=0; rBCalib_(calibSamples,1)=0;
            rCtildeCalib_(calibSamples,2)=1; rBCalib_(calibSamples,2)=1;
            Eigen::MatrixXd weights;
            weights.resize(calibSamples+1,1);
            weights.topRows(calibSamples)=0.5*1/calibSamples*Eigen::MatrixXd::Ones(calibSamples,1);
            weights(calibSamples)=0.5;
            RBI_=gpsimu_odom::rotMatFromWahba(weights,rCtildeCalib_,rBCalib_);
            rbiIsInitialized_=true;

            doSetRBI0(RBI_);
        }
        return;
    }

    //if everything is working
    if(ttime>lastA2Dtime_)  //Only use newest time. Ignore 0 messages.
    {
        hasAlreadyReceivedA2D_=true;
        lastA2Dtime_=ttime;
        if(msg->testStat > minTestStat_)
        {
            validA2Dtest_=true;
            //Store constrained baseline vector in I frame
            const Eigen::Vector3d constrainedBaselineECEF(msg->rx, msg->ry, msg->rz);
            rS2PMeas_ = Recef2wrw_*constrainedBaselineECEF;
            rS2PMeas_ = gpsimu_odom::unit3(rS2PMeas_);
        }else
        {
            validA2Dtest_=false;
    }

    //If the message is new, both messages have been received, and all teststats are good, then publish.
    if(abs(lastRTKtime_-lastA2Dtime_)<.001 && validA2Dtest_ && validRTKtest_
        && hasAlreadyReceivedRTK_ && hasAlreadyReceivedA2D_)  //only resend pose if new
    {
        internalSeq++;
        double dtLastProc = ttime - lynxHelper_.getTLastProc();
        if(isCalibratedLynx_ && dtLastProc>0)
        {

            runRosUKF(rPrimaryMeas_,rS2PMeas_,ttime);
        }

        //Reset to avoid publishing twice
        hasAlreadyReceivedRTK_=false; hasAlreadyReceivedA2D_=false;
        }
    }
    return;    
}


//SBRTK callback.  Takes in message from SBRTK, synchronizes with message from A2D, then calls UKF update
void rosStreamEndpointGPSKF::singleBaselineRTKCallback(const gbx_ros_bridge_msgs::SingleBaselineRTK::ConstPtr &msg)
{
    if(~hasRosHandle)
    {return;}
    int week, secOfWeek;
    double fracSec, dtRX;
    dtRX_=msg->deltRSec;
    week = msg->tSolution.week;
    secOfWeek = msg->tSolution.secondsOfWeek;
    fracSec = msg->tSolution.fractionOfSecond;
    double ttime=gpsimu_odom::tgpsToSec(week,secOfWeek,fracSec) - dtRX;

    if(ttime > lastRTKtime_)  //only use newest time
    {
        hasAlreadyReceivedRTK_=true;
        lastRTKtime_=ttime;
        //If the message is accepted
        if(msg->testStat > minTestStat_)
            {
                validRTKtest_=true;
                Eigen::Vector3d tmpvec;
                //Rotate rECEF to rI and store in rPrimaryMeas_
                tmpvec(0) = msg->rx - zeroInECEF_(0); //error vector from ECEF at init time
                tmpvec(1) = msg->ry - zeroInECEF_(1);
                tmpvec(2) = msg->rz - zeroInECEF_(2);
                rPrimaryMeas_ = Rwrw_*Recef2enu_*tmpvec;
                doSetRprimary(rPrimaryMeas_); //sets rPrimaryMeas_ in estimationNode
            }else
            {
                validRTKtest_=false;
            }

            //If the time is new, both messages for this time have been received, and teststats are good
        if(abs(lastRTKtime_-lastA2Dtime_)<.001 && validA2Dtest_ && validRTKtest_
                 && hasAlreadyReceivedRTK_ && hasAlreadyReceivedA2D_)  //only resend pose if new
        {
            internalSeq++;
            double dtLastProc = ttime - lynxHelper_.getTLastProc();

            if(isCalibratedLynx_ && dtLastProc>0)
            {

                runRosUKF(rPrimaryMeas_,rS2PMeas_,ttime);

            }

            //Reset to avoid publishing twice
            hasAlreadyReceivedRTK_=false; hasAlreadyReceivedA2D_=false; 
        }
    }
    
    return;
}


//Checks navsol to get the most recent figures for dtRX
void rosStreamEndpointGPSKF::navsolCallback(const gbx_ros_bridge_msgs::NavigationSolution::ConstPtr &msg)
{
    dtRXinMeters_ = msg->deltatRxMeters;
    return; 
}


//Get reference RRT time and measurement offset time from Observables message
void rosStreamEndpointGPSKF::tOffsetCallback(const gbx_ros_bridge_msgs::ObservablesMeasurementTime::ConstPtr &msg)
{
    int week, secOfWeek;
    double fracSec;
    week =  msg->tOffset.week;
    secOfWeek = msg->tOffset.secondsOfWeek;
    fracSec = msg->tOffset.fractionOfSecond;
    double ttime=gpsimu_odom::tgpsToSec(week,secOfWeek,fracSec);
    lynxHelper_.setTOffset(ttime);
    return; 
}


//Get upper 32 bits of tIndex counter
void rosStreamEndpointGPSKF::imuConfigCallback(const gbx_ros_bridge_msgs::ImuConfig::ConstPtr &msg)
{
    ROS_INFO("Config message received.");
    imuConfigAccel_ = msg->lsbToMetersPerSecSq; //scaling to m/s2 from "non-engineering units"
    imuConfigAttRate_ = msg->lsbToRadPerSec; //scaling to rad/s from "non-engineering units"
    
    sampleFreqNum_ = msg->sampleFreqNumerator;
    sampleFreqDen_ = msg->sampleFreqDenominator;
    tIndexConfig_ - msg->tIndexk;
    
    return; 
}


//The code is present here but has been disabled due to lynx vibrations
//Callback for imu subscriber for the lynx
void rosStreamEndpointGPSKF::lynxImuCallback(const gbx_ros_bridge_msgs::Imu::ConstPtr &msg)
{
    //If reports are being received but the pointer to the ros node is not available, exit
    if(~hasRosHandle || ~LYNX_IMU)
    {
        return;         
    }


    //Initialization variables
    static int counter=0;
    static uint64_t imuSeq=0;
    static double tLastImu=0;
    static Eigen::Vector3d ba0Lynx=Eigen::Vector3d(0,0,0);
    static Eigen::Vector3d bg0Lynx=Eigen::Vector3d(0,0,0);
    //gps variables
    static const long long int mask = 0xffffffff; // This is: (1 << 32) - 1
    static int warnCounter=0;
    double dt;

    double tLastProcessed = lynxHelper_.getTLastProc();
    //Calculate IMU time
    const uint64_t tIndex = msg->tIndexTrunc;
    const long long int tIndexFull = (tIndexConfig_ & ~mask) | (tIndex & mask); //Bit-mask, don't bit-shift
    const double sampleFreq = sampleFreqNum_/sampleFreqDen_;
    const double tRRT = tIndexFull/sampleFreq; //tIndexFull is in samples, divide by samples/s
    //const double tOffset = toffsetFracSecs_ + toffsetWeek_*SEC_PER_WEEK + toffsetSecOfWeek_;
    const double tOffset = lynxHelper_.getTOffset();
    const double tORT = tRRT + tOffset; //tOffset comes from ObservablesMeasurementTime
    const double tGPS = tORT - dtRXinMeters_/SPEED_OF_LIGHT; //dtrx comes from NavigationSolution
    const double thisTime = tGPS;
    std::cout.precision(17);
    dt = thisTime-tLastImu;
    if(dt<=1e-9) //Note: NOT abs(dt), as this checks whether or not messages are received out of order as well
    {
        std::cout << "Error: 1ns between IMU measurements" << std::endl;
        return;
    }
    //Only update last time used IF this time is accepted
    tLastImu=thisTime;

    //Use scale parameter
    Eigen::Vector3d imuAccelMeas, imuAttRateMeas;
    imuAccelMeas(0) = msg->acceleration[0] * imuConfigAccel_;
    imuAccelMeas(1) = msg->acceleration[1] * imuConfigAccel_;
    imuAccelMeas(2) = msg->acceleration[2] * imuConfigAccel_;
    imuAttRateMeas(0) = msg->angularRate[0] * imuConfigAttRate_;
    imuAttRateMeas(1) = msg->angularRate[1] * imuConfigAttRate_;
    imuAttRateMeas(2) = msg->angularRate[2] * imuConfigAttRate_;

    //LYNX_IMU_ROTATION is defined in constants.hpp
    imuAccelMeas = LYNX_IMU_ROTATION*imuAccelMeas;
    imuAttRateMeas = LYNX_IMU_ROTATION*imuAttRateMeas;

    double tOffsetRosToUTC = tGPS - (ros::Time::now()).toSec();
    hasRosToUTC_=true;
    rosHandle_->lynxHelper_.setTOffset(tOffsetRosToUTC);

    //Run CF if calibrated
    if(isCalibratedLynx_)
    {
        imuSeq++;
        double dtLastProc = thisTime - tLastProcessed;
        if(dtLastProc>0) //Just in case a gps message is received late
        { 
            //Construct measurements
            const imuMeas thisImuMeas(thisTime,imuAccelMeas,imuAttRateMeas);
            lastImuMeasLynx_ = thisImuMeas; //Used for reporting wB in publishOdomAndMocap
            rosHandle_->lynxHelper_.setLastImuMeas(thisImuMeas);

            rosHandle_->imuFilterLynx_.runUKFpropagateOnly(tLastProcessed,thisImuMeas);

            //Cleanup
            rosHandle_->lynxHelper_.setTLastProc(thisTime);

            //Warn if signal is lost
            if( (thisTime-lastRTKtime_ > 0.5) || (thisTime-lastA2Dtime_ > 0.5) )
            {
                if(warnCounter%40==0)
                {ROS_INFO("GPS outage warning!");}
                warnCounter++;
            }else
            {
                if(warnCounter!=0)
                {ROS_INFO("GPS restored.");}
                warnCounter=0;
            }
        }
    }else if(rbiIsInitialized_)  //if RBI has been calculated but the biases have not been calculated
    { 
        ba0Lynx=ba0Lynx+0.01*(imuAccelMeas - RBI_.transpose()*Eigen::Vector3d(0,0,9.8)); //inefficient
        bg0Lynx=bg0Lynx+0.01*imuAttRateMeas;
        counter++;
        // Try ground calibration step for simplicity
        if(counter>=100)
        {
            Eigen::Vector3d rI0 = rPrimaryMeas_ - RBI_.transpose()*Lcg2p_;
            Eigen::Matrix<double,15,1> xState;
            xState<<rI0(0),rI0(1),rI0(2), 0,0,0, 0,0,0, ba0Lynx(0),ba0Lynx(1),ba0Lynx(2), bg0Lynx(0),bg0Lynx(1),bg0Lynx(2);
            isCalibratedLynx_ = true;
            rosHandle_->lynxHelper_.setTLastProc(thisTime);
            rosHandle_->imuFilterLynx_.setState(xState,RBI_);
        }
    }
    
    return; 
}


void rosStreamEndpointGPSKF::runRosUKF(const Eigen::Vector3d pose, const Eigen::Vector3d Ls2p, const double ttime)
{
    rosHandle_->letStreamRunGPS(pose, Ls2p, ttime);
}


void rosStreamEndpointGPSKF::runRosUKFPropagate(const Eigen::Vector3d acc, const Eigen::Vector3d att, const double ttime)
{
    rosHandle_->letStreamRunIMU(acc, att, ttime);
}


void rosStreamEndpointGPSKF::doSetRBI0(const Eigen::Matrix3d &RBI0)
{
    rosHandle_->letStreamSetRBI(RBI0);
}

void rosStreamEndpointGPSKF::doSetRprimary(const Eigen::Vector3d &rp)
{
    rosHandle_->letStreamSetRprimary(rp);
}


