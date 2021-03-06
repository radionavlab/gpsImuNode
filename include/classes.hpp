#pragma once
#include <Eigen/Geometry>
#include <stdint.h>
#include <cmath>

//Contains data storage classes for gps time, gps measurements (in UTC), imu measurements (in system time).
//Also contains a filterHelper class that contains an offset from tImu to tGps.

class gpsTime
{
  	public:
    	gpsTime(){week_=0.0;sec_=0.0;fracSec_=0.0;}
    	gpsTime(const double wk, const double se, const double fSec){week_=wk;sec_=se;fracSec_=fSec;}
    	void getTime(double &wk, double &se, double &fSec){wk=week_;se=sec_;fSec=fracSec_;}
    	void setTime(const double wk, const double se, const double fSec){week_=wk;sec_=se;fracSec_=fSec;}
    	double toSec(){return (week_*604800.0+sec_+fracSec_);}
    	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  	private:
    	double week_, sec_, fracSec_;
};


//All measurements are expressed in local (WRW) frame
class gpsMeas
{
	public:
		gpsMeas() {tSec_=0.0; rPrimary_=Eigen::Vector3d::Zero(); rS2P_=Eigen::Vector3d::Zero();}
		gpsMeas(const double t, const Eigen::Vector3d &rP, const Eigen::Vector3d &rC)
			{tSec_=t; rPrimary_=rP; rS2P_=rC;};
		void setMeas(const double t, const Eigen::Vector3d &rP, const Eigen::Vector3d &rC)
			{tSec_=t; rPrimary_=rP; rS2P_=rC;}
		void getMeas(double &t, Eigen::Vector3d &rP, Eigen::Vector3d &rC)
			const {t=tSec_; rP=rPrimary_; rC=rS2P_;}
		void getTime(double &t) const {t=tSec_;}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		double tSec_;
		Eigen::Vector3d rPrimary_, rS2P_;
};

class imuMeas
{
	public:
		imuMeas(){tSec_=0.0; a_=Eigen::Vector3d::Zero(); wB_=Eigen::Vector3d::Zero();}
		imuMeas(const double t, const Eigen::Vector3d &aa, const Eigen::Vector3d &ww)
			{tSec_=t; a_=aa; wB_=ww;};
		void setMeas(const double t, const Eigen::Vector3d &aIn, const Eigen::Vector3d &wIn)
			{tSec_=t; a_=aIn; wB_=wIn;};
		void getMeas(double &t, Eigen::Vector3d &aOut, Eigen::Vector3d &wOut)
			const {t=tSec_; aOut=a_; wOut=wB_;}
		void getTime(double &t) const {t=tSec_;}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW	
	private:
		double tSec_;
		Eigen::Vector3d a_, wB_;
};


class filterHelper
{
	public:
		filterHelper() {tOffsetSec_=0.0; tLastProc_=0.0;}
		void setTOffset(const double t) {tOffsetSec_ = t;}
		void setTLastProc(const double t) {tLastProc_ = t;}
		void setLastImuMeas(const imuMeas &lastMeas) {lastImuMeas_=lastMeas;}
		double getTOffset() const {return tOffsetSec_;}
		double getTLastProc() const {return tLastProc_;}
		imuMeas getLastImuMeas() const {return lastImuMeas_;}
	private:
		double tOffsetSec_, tLastProc_;
		imuMeas lastImuMeas_;
};

