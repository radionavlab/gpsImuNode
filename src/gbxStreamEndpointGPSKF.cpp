#include "gbxStreamEndpointGPSKF.hpp"
#include "navtoolbox.h"
#include <sys/time.h>
#include "mathHelperFunctions.hpp"

GbxStreamEndpointGPSKF::~GbxStreamEndpointGPSKF() {
  closeSinkStream_();
}

void GbxStreamEndpointGPSKF::configure(ros::NodeHandle &nh, Eigen::Vector3d baseECEF_vector_in,
            Eigen::Matrix3d Recef2enu_in)
{
    std::string GPSKFName, posePubTopic;
    GPSKFName = ros::this_node::getName();
    Recef2enu = Recef2enu_in;
    baseECEF_vector_in = baseECEF_vector;

    ros::param::get(GPSKFName + "/posePubTopic", posePubTopic);
    ros::param::get(GPSKFName + "/minimumTestStat",minTestStat);
    internalSeq=0;
    sec_in_week = 604800;
    L_cg2p << 0.1013,-0.0004,0.0472;
}


void GbxStreamEndpointGPSKF::setRosPointer(std::shared_ptr<EstimationNode> rosHandle)
{
    rosHandle_=rosHandle;
    hasRosHandle=true;
    return;
}

void GbxStreamEndpointGPSKF::donothing()
{
    std::cout << "Do nothing called" << std::endl;
    return;
}

bool GbxStreamEndpointGPSKF::openSinkStream_() {
  return true;
}

void GbxStreamEndpointGPSKF::closeSinkStream_() {
  return;
}

bool GbxStreamEndpointGPSKF::isValidSinkStream_() const {
  return true;
}

bool GbxStreamEndpointGPSKF::writeBytes_(const u8* buffer, size_t size) {
  return true;
}

GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportCoda>&& pReport, const u8 streamId) {
  return ProcessReportReturn::REJECTED;
}


//A2D callback.  Takes in message from A2D, synchronizes with message from A2D, then calls UKF update
GbxStreamEndpoint::ProcessReportReturn estimationNode::processReport_(
    std::shared_ptr<const ReportMultiBaselineRtkAttitude2D>&& pReport, const u8 streamId)
{

    int week, secOfWeek;
    double fracSec, dtRX;
    dtRX_=pReport->deltRSec();
    pReport->tSolution.get(week, secOfWeek, fracSec);
    double ttime=tgpsToSec(week,secOfWeek,fracSec) - dtRX;
    static int rCCalibCounter=0;
    static int calibSamples=20;

    //Ignore zero messages
    if(msg->tSolution.week<=1)
    {return;}

    if(!rbiIsInitialized_ && msg->testStat>=100)
    {
        const Eigen::Vector3d constrainedBaselineECEF(pReport->rx(), pReport->ry(), pReport->rz());
        const Eigen::Vector3d constrainedBaselineI(unit3(Rwrw_*Recef2enu_*constrainedBaselineECEF));
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
            RBI_=rotMatFromWahba(weights,rCtildeCalib_,rBCalib_);
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
        if(pReport->testStat() > minTestStat_)
        {
            validA2Dtest_=true;
            //Store constrained baseline vector in I frame
            const Eigen::Vector3d constrainedBaselineECEF(pReport->rx(), pReport->ry(), pReport->rz());
            rS2PMeas_ = Recef2wrw_*constrainedBaselineECEF;
            rS2PMeas_ = unit3(rS2PMeas_);
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

            runRosUKF(internalpose,rS2PMeas_,ttime);
        }

        //Reset to avoid publishing twice
        hasAlreadyReceivedRTK_=false; hasAlreadyReceivedA2D_=false;
        }
    }
    retval = ProcessReportReturn::ACCEPTED;
    return retval;    
}


//SBRTK callback.  Takes in message from SBRTK, synchronizes with message from A2D, then calls UKF update
GbxStreamEndpoint::ProcessReportReturn estimationNode::processReport_(
    std::shared_ptr<const ReportSingleBaselineRtk>&& pReport, const u8 streamId)
{
    int week, secOfWeek;
    double fracSec, dtRX;
    dtRX_=pReport->deltRSec();
    pReport->tSolution.get(week, secOfWeek, fracSec);
    double ttime=tgpsToSec(week,secOfWeek,fracSec) - dtRX;

    if(ttime > lastRTKtime_)  //only use newest time
    {
        hasAlreadyReceivedRTK_=true;
        lastRTKtime_=ttime;
        //If the message is accepted
        if(pReport->testStat() > minTestStat_)
            {
                validRTKtest_=true;
                Eigen::Vector3d tmpvec;
                //Rotate rECEF to rI and store in rPrimaryMeas_
                tmpvec(0) = pReport->rx() - zeroInECEF_(0); //error vector from ECEF at init time
                tmpvec(1) = pReport->ry() - zeroInECEF_(1);
                tmpvec(2) = pReport->rz() - zeroInECEF_(2);
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

                runRosUKF(internalpose,rS2PMeas_,ttime);

            }

            //Reset to avoid publishing twice
            hasAlreadyReceivedRTK_=false; hasAlreadyReceivedA2D_=false; 
        }
    }
    retval = ProcessReportReturn::ACCEPTED;
    return retval;
}



//Checks navsol to get the most recent figures for dtRX
GbxStreamEndpoint::ProcessReportReturn estimationNode::processReport_(
    std::shared_ptr<const ReportNavigationSolution>&& pReport, const u8 streamId)
{
    dtRXinMeters_ = pReport->deltatRxMeters();
    retval = ProcessReportReturn::ACCEPTED;
    return retval; 
}


//Get reference RRT time and measurement offset time from Observables message
GbxStreamEndpoint::ProcessReportReturn estimationNode::processReport_(
    std::shared_ptr<const ReportObservablesMeasurementTime>&& pReport, const u8 streamId)
{
    int week, secOfWeek;
    double fracSec;
    pReport->tOffset.get(week, secOfWeek, fracSec);
    double ttime=tgpsToSec(week,secOfWeek,fracSec);
    lynxHelper_.setTOffset(ttime);
    retval = ProcessReportReturn::ACCEPTED;
    return retval; 
}


//Get upper 32 bits of tIndex counter
GbxStreamEndpoint::ProcessReportReturn estimationNode::processReport_(
    std::shared_ptr<const ReportImuConfig>&& pReport, const u8 streamId)
{
    ROS_INFO("Config message received.");
    imuConfigAccel_ = pReport->lsbToMetersPerSecSq(); //scaling to m/s2 from "non-engineering units"
    imuConfigAttRate_ = pReport->lsbToRadPerSec(); //scaling to rad/s from "non-engineering units"
    
    sampleFreqNum_ = pReport->sampleFreqNumerator();
    sampleFreqDen_ = pReport->sampleFreqDenominator();
    tIndexConfig_ - pReport->tIndexk();
    retval = ProcessReportReturn::ACCEPTED;
    return retval; 
}


//The code is present here but has been disabled due to lynx vibrations
//Callback for imu subscriber for the lynx
GbxStreamEndpoint::ProcessReportReturn estimationNode::processReport_(
    std::shared_ptr<const ReportImu>&& pReport, const u8 streamId)
{
    //If reports are being received but the pointer to the ros node is not available, exit
    if(~hasRosHandle)
    {
        retval = ProcessReportReturn::ACCEPTED;
        return retval;         
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
    const uint64_t tIndex = pReport->tIndexTruncated();
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
    imuAccelMeas(0) = pReport->acceleration[0] * imuConfigAccel_;
    imuAccelMeas(1) = pReport->acceleration[1] * imuConfigAccel_;
    imuAccelMeas(2) = pReport->acceleration[2] * imuConfigAccel_;
    imuAttRateMeas(0) = pReport->angularRate[0] * imuConfigAttRate_;
    imuAttRateMeas(1) = pReport->angularRate[1] * imuConfigAttRate_;
    imuAttRateMeas(2) = pReport->angularRate[2] * imuConfigAttRate_;

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

            //Publish
            updateType = "imu"; publishOdomAndMocap();
            //RBI_ is updated when publisher is called.

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
    retval = ProcessReportReturn::ACCEPTED;
    return retval; 
}


void GbxStreamEndpointGPSKF::runRosUKF(const Eigen::Vector3d pose, const Eigen::Vector3d Ls2p, const double ttime)
{
    rosHandle_->letStreamRunGPS(pose, Ls2p, ttime);
}


void GbxStreamEndpointGPSKF::runRosUKFPropagate(const Eigen::Vector3d acc, const Eigen::Vector3d att, const double ttime)
{
    rosHandle_->letStreamRunGPS(acc, att, ttime);
}


void GbxStreamEndpointGPSKF::doSetRBI0(const Eigen::Matrix3d &RBI0)
{
    rosHandle_->letStreamSetRBI(RBI0);
}

void GbxStreamEndpointGPSKF::doSetRprimary(const Eigen::Vector3d &rp)
{
    rosHandle_->letStreamSetRprimary(rp);
}

