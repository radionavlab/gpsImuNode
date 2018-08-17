//Class containing ROS publishers/subscribers
#include "estimationNode.hpp"

/*Contains two gps helper functions and the main initialization function
for the ROS class*/
namespace gpsimu_odom
{

//Initializes all ROS work
estimationNode::estimationNode(ros::NodeHandle &nh)
{
    // Pause GBX stream during initial reads
    stream_ -> pauseStream();

    //Get data about node and topic to listen
    std::string quadPoseTopic, quadName, rtktopic, a2dtopic, posePubTopic, nodeNamespace;
    double tmax;
    quadName = ros::this_node::getName();
    nodeNamespace = ros::this_node::getNamespace();    
    ros::param::get(quadName + "/quadPoseTopic", quadPoseTopic);
    ros::param::get(quadName + "/arenaCenterX", zeroInECEF_(0));
    ros::param::get(quadName + "/arenaCenterY", zeroInECEF_(1));
    ros::param::get(quadName + "/arenaCenterZ", zeroInECEF_(2));
    ros::param::get(quadName + "/rtktopic", rtktopic);
    ros::param::get(quadName + "/a2dtopic", a2dtopic);
    ros::param::get(quadName + "/posePubTopic", posePubTopic);
    ros::param::get(quadName + "/minimumTestStat",minTestStat_);
    ros::param::get(quadName + "/maxThrust",tmax);
    int gbxport;
    ros::param::get(quadName + "/gbxport",gbxport);

    //Initialize twHelper
    twHelper.setWeight(0.75*9.81);
    twHelper.setTW0(1.75);
    twHelper.setTLastProc(ros::Time::now().toSec());


    //Get additional parameters for the kalkman filter
    nh.param(quadName + "/publish_tf", publish_tf_, true);
    nh.param<std::string>(quadName + "/child_frame_id", child_frame_id_, "base_link");
    if(publish_tf_ && child_frame_id_.empty())
        throw std::runtime_error("gpsimu_odom: child_frame_id required for publishing tf");

    Recef2enu_=ecef2enu_rotMatrix(zeroInECEF_);

    //Account for WRW rotation wrt ENU
    double thetaWRW;
    thetaWRW = 6.2*MATH_PI/180.0; //angle of rooftop coordinate system WRT ENU
    Rwrw_ << cos(thetaWRW), -1*sin(thetaWRW), 0,
                    sin(thetaWRW), cos(thetaWRW), 0,
                    0, 0, 1.0;
    Recef2wrw_ = Rwrw_*Recef2enu_;
    Recef2wrw_=Rwrw_*Recef2enu_;
    RBI_=Eigen::MatrixXd::Identity(3,3);

    //Delcarations before filling in covs
    Eigen::Matrix<double,6,6> Rk;
    Eigen::Matrix<double,12,12> Qk12, Qk12dividedByDt;
    Eigen::Matrix<double,15,15> Pimu;
    Eigen::Matrix<double,6,1> P6diagElements;
    Eigen::Matrix<double,15,1> P15diagElements;
    double tauA, tauG, maxBa, maxBg;
    Pimu=Eigen::Matrix<double,15,15>::Identity();

    //Covariances
    tauA=1000.0;
    tauG=1000.0;    
    //P6diagElements << 0.000036,0.000036,0.000144, 0.000144,0.000144,0.000144;  //Params file
    P6diagElements << 0.000036,0.000036,0.000019, 0.000144,0.000144,0.000144;  //Test values
    Rk = P6diagElements.asDiagonal();

    //IMU params
    const double dtIMU=1.0/73.25; //from (rostopic hz /phoenix/imu -r 10)/10
    const double alphaA = exp(-dtIMU/tauA);
    const double gScale = 9.81/1000.0;
    const double thetaScale = pi/180.0;
    const double alphaG = exp(-dtIMU/tauG);
    //Covariance elements: gyro output, gyro bias, accel output, accel bias

/*  //Filled spec sheet data
    Qk12.topLeftCorner(3,3) = pow(0.1*pi/180/sqrt(dtIMU),2)*Eigen::Matrix3d::Identity(); //see datasheet  
    Qk12.block(3,3,3,3) = pow(thetaScale*100.0/360.0,2)*(1.-alphaG*alphaG)*Eigen::Matrix3d::Identity(); //random tests
    Qk12.block(6,6,3,3) = pow(9.81/1.0e6*150.0/sqrt(dtIMU),2)*Eigen::Matrix3d::Identity(); //see datasheet
    Qk12.bottomRightCorner(3,3) = pow(gScale*1000.0,2)*(1.-alphaA*alphaA)*Eigen::Matrix3d::Identity(); //random tests */

/*  //Testing based on ground test.  Works but velocity is noisy
    Qk12.topLeftCorner(3,3) = 6.95e-4*Eigen::Matrix3d::Identity();
    Qk12.block(3,3,3,3) = pow(thetaScale*100.0/360.0,2)*(1.-alphaG*alphaG)*Eigen::Matrix3d::Identity();
    Qk12.block(6,6,3,3) = 0.0045*Eigen::Matrix3d::Identity();
    Qk12.bottomRightCorner(3,3) = 0.1*pow(gScale*1000.0,2)*(1.-alphaA*alphaA)*Eigen::Matrix3d::Identity(); */

    //Best tracking from ground set
    Eigen::Matrix3d QangularAccel = 0.1*Eigen::Matrix3d::Identity(); //Q caused by angular acceleration
    Eigen::Matrix3d QgyroOutput2 = 1e-3*Eigen::Matrix3d::Identity();
    Qk12=Eigen::Matrix<double,12,12>::Zero();
    Qk12.topLeftCorner(3,3) = 6.95e-4*Eigen::Matrix3d::Identity() + QgyroOutput2;
    Qk12.block(3,3,3,3) = 1.0e-6*pow(thetaScale*100.0/360.0,2)*(1.-alphaG*alphaG)*Eigen::Matrix3d::Identity();
    Qk12.block(6,6,3,3) = 0.0045*Eigen::Matrix3d::Identity() + QangularAccel;
    Qk12.bottomRightCorner(3,3) = 1.0e-6*pow(gScale*80.0,2)*(1.-alphaA*alphaA)*Eigen::Matrix3d::Identity();
    Qk12dividedByDt = Qk12/dtIMU;  //When used in filter, multiply by dt of each update step

    QgyroOutput_=QgyroOutput2;
    
    P15diagElements << 1.0e-4,1.0e-4,1.0e-4, 1.0e-6,1.0e-6,1.0e-6,
            1.0e-5,1.0e-5,1.0e-5, 1.0e-5,1.0e-5,1.0e-5, 1.0e-8, 1.0e-8, 1.0e-8;
    Pimu=P15diagElements.asDiagonal();

    //Stuff for rI
    sec_in_week_ = 604800.0;
    lastRTKtime_=0;
    lastA2Dtime_=0;
    internalSeq=0;

    //Secondary to primary vector in body frame
    Ls2p_<< 0.190,0,0;
    Lcg2imu_<< -0.0884,0.0134,-0.0399;
    Lcg2p_<< 0.1013,-0.0004,0.0472; //kept as class var for use in 

    //First get rbi(0), then get biases(0)
    rbiIsInitialized_ = false;
    isCalibratedLynx_ = false;
    isCalibratedSnap_ = false;
    hasRosToUTC_ = false;

    // Initialize publishers and subscribers
    bool useUDPinsteadOfTCP = false; //false by default
    ros::param::get(quadName + "/useUDP",useUDPinsteadOfTCP);


    //GBX stuff
    auto gbxStream = std::make_shared<GbxStream>();
    gbxStream->pauseStream();
    auto epOutput = std::make_shared<GbxStreamEndpointGPSKF>();
    // Add any other necessary reports here.
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::SINGLE_BASELINE_RTK);
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::ATTITUDE_2D);
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::IMU);
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::IMU_CONFIG);
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::NAVIGATION_SOLUTION);
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::OBSERVABLES_MEASUREMENT_TIME);
    epOutput->filter(GbxStream::DEFAULT_PRIMARY).enableWhitelist();
    //make endpoint
    auto epInput = std::make_shared<GbxStreamEndpointIN>(port, OptionObject::protocol_enum::IP_UDP, OptionObject::peer_type_enum::ROVER);
    gbxStream->resumeStream();    


    //Create pubs/subs as TCP/UDP
    if(~useUDPinsteadOfTCP)  //if using TCP/IP
    {
        //Publishers
        localOdom_pub_ = nh.advertise<nav_msgs::Odometry>("local_odom_INS", 10);
        mocap_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose_INS", 10);

        //Subscribers
        mavrosImuSub_ = nh.subscribe("mavros/imu/data_raw",10,&estimationNode::mavrosImuCallback,
                                                    this, ros::TransportHints().tcpNoDelay());
    }else  //if using UDP
    {
        //Publishers
        localOdom_pub_ = nh.advertise<nav_msgs::Odometry>("local_odom_INS", 10);
        mocap_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose_INS", 10);

        //Subscribers
        mavrosImuSub_ = nh.subscribe("mavros/imu/data_raw",10,&estimationNode::mavrosImuCallback,
                                                    this, ros::TransportHints().unreliable().reliable().tcpNoDelay(true));
        //NOTE:  ros::TransportHints().unreliable().reliable().tcpNoDelay(true)) is "UDP preferred, use tcpnodelay if UDP"
        //       is not available (i.e., when the publishing node is a rospy node rather than roscpp).
    }

    //Load IMU config data, establish saturations
    ROS_INFO("Waiting for IMU config data, this may take a moment...");
    gbx_ros_bridge_msgs::ImuConfig::ConstPtr imuConfigMsg = 
                ros::topic::waitForMessage<gbx_ros_bridge_msgs::ImuConfig>("IMUConfig");
    imuConfigAccel_ = imuConfigMsg->lsbToMetersPerSecSq;
    imuConfigAttRate_ = imuConfigMsg->lsbToRadPerSec;
    sampleFreqNum_ = imuConfigMsg->sampleFreqNumerator;
    sampleFreqDen_ = imuConfigMsg->sampleFreqDenominator;  
    tIndexConfig_ = imuConfigMsg->tIndexk;
    //maxBa = imuConfigAccel_ * 250.0; //imu report units times scalefactor
    //maxBg = imuConfigAttRate_ * 250.0;
    maxBa = imuConfigAccel_*1.0e4;
    maxBg = imuConfigAttRate_*1.0e4;
    ROS_INFO("IMU configuration recorded.");

    //Load offset time data
    ROS_INFO("Waiting for offset time, this may take a moment...");
    gbx_ros_bridge_msgs::ObservablesMeasurementTime::ConstPtr toffsetMsg =
                ros::topic::waitForMessage<gbx_ros_bridge_msgs::ObservablesMeasurementTime>("ObservablesMeasurementTime");
    int toffsetWeek = toffsetMsg->tOffset.week;
    int toffsetSecOfWeek = toffsetMsg->tOffset.secondsOfWeek;
    double toffsetFracSecs = toffsetMsg->tOffset.fractionOfSecond;
    lynxHelper_.setTOffset(tgpsToSec(toffsetWeek,toffsetSecOfWeek,toffsetFracSecs));
    ROS_INFO("Time offset from RRT to ORT recorded.");

    //Get dtRX0
    gbx_ros_bridge_msgs::NavigationSolution::ConstPtr navsolMsg = 
                ros::topic::waitForMessage<gbx_ros_bridge_msgs::NavigationSolution>("NavigationSolution");
    dtRXinMeters_ = navsolMsg->deltatRxMeters;
    ROS_INFO("Time offset from RX to GPS obtained.");

    ROS_INFO("Setting hard parameters for complementary filter.");
    //Lynx
    imuFilterLynx_.setLevers(Lcg2p_,Lcg2imu_,Ls2p_);
    imuFilterLynx_.setCovariances(dtIMU, Qk12, Pimu, Rk);
    imuFilterLynx_.setImuParams(tauA,tauG);
    imuFilterLynx_.setBiasSaturationLimits(maxBa, maxBg);
    
    //Snap
    imuFilterSnap_.setLevers(Lcg2p_,Eigen::Vector3d(0.0684,0.0134,-0.0399),Ls2p_);
    imuFilterSnap_.setCovariances(dtIMU, Qk12, Pimu, Rk);
    imuFilterSnap_.setImuParams(tauA,tauG);
    imuFilterSnap_.setBiasSaturationLimits(maxBa, maxBg);
    

    ROS_INFO("Startup complete.");
}


void estimationNode::PublishTransform(const geometry_msgs::Pose &pose,
                                                             const std_msgs::Header &header,
                                                             const std::string &child_frame_id)
{
    //Publish tf
    geometry_msgs::Vector3 translation;
    translation.x = pose.position.x;
    translation.y = pose.position.y;
    translation.z = pose.position.z;

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header = header;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.transform.translation = translation;
    transform_stamped.transform.rotation = pose.orientation;

    tf_broadcaster_.sendTransform(transform_stamped);
}


} //end namespace
