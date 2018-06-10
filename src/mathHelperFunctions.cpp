#include "mathHelperFunctions.hpp"
#include <Eigen/SVD>

//Contains matrix helper functions


namespace gpsimu_odom
{

//Rotation matrix, 312 convention
Eigen::Matrix3d euler2dcm312(const Eigen::Vector3d &ee)
{
  	const double cPhi = cos(ee(0));
  	const double sPhi = sin(ee(0));
  	const double cThe = cos(ee(1));
  	const double sThe = sin(ee(1));
  	const double cPsi = cos(ee(2));
  	const double sPsi = sin(ee(2));
  	Eigen::Matrix3d R2;
  	R2 << cPsi*cThe - sPhi*sPsi*sThe, cThe*sPsi + cPsi*sPhi*sThe, -cPhi*sThe,
        -cPhi*sPsi,                                  cPhi*cPsi,       sPhi,
        cPsi*sThe + cThe*sPhi*sPsi, sPsi*sThe - cPsi*cThe*sPhi,  cPhi*cThe;
  	return R2;
}


//rotate by hatmat(gammavec) to new RBI
Eigen::Matrix3d updateRBIfromGamma(const Eigen::Matrix3d &R0, const Eigen::Vector3d &gamma)
{
    //return (Eigen::Matrix3d::Identity()+hatmat(gamma))*R0;
    //return (rotMatFromEuler(gamma))*R0;
    return (euler2dcm312(gamma))*R0;
}


//rotate by hatmat(gammavec) to new RBI.  Also zeros middle elements of x0.
//For use ONLY with 15-element imu state vector
void updateRBIfromGamma(Eigen::Matrix3d &R0, Eigen::Matrix<double,15,1> &x0)
{
	R0=euler2dcm312(x0.middleRows(6,3))*R0;
	x0.middleRows(6,3)=Eigen::Vector3d::Zero();
}


//Cross product equivalent.  Named this way for consistency with nn_imu_dat
Eigen::Matrix3d hatmat(const Eigen::Vector3d &v1)
{
	Eigen::Matrix3d f = Eigen::MatrixXd::Zero(3,3);
	f(0,1)=-v1(2); f(0,2)=v1(1);
	f(1,0)=v1(2); f(1,2)=-v1(0);
	f(2,0)=-v1(1); f(2,1)=v1(0);
	return f;
}


//Wabha solver.  Expects vI, vB as nx3 matrices with n sample vectors
Eigen::Matrix3d rotMatFromWahba(const Eigen::VectorXd &weights,
	const::Eigen::MatrixXd &vI, const::Eigen::MatrixXd &vB)
{
	int n=weights.size();
	Eigen::Matrix3d B=Eigen::Matrix3d::Zero();
	for(int ij=0; ij<n; ij++)
	{
		B=B+weights(ij)*(vB.row(ij)).transpose()*vI.row(ij);
	}
	Eigen::Matrix3d U,V;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullV | Eigen::ComputeFullU);
	U=svd.matrixU();
	V=svd.matrixV();
	Eigen::DiagonalMatrix<double, 3> M(1, 1, U.determinant()*V.determinant());
	//M.asDiagonal(Eigen::Vector3d(1,1,U.determinant()*V.determinant()));
	return U*M*(V.transpose());
}


//Convert to unit. Might be a more efficient Eigen built-in function
Eigen::Vector3d unit3(const Eigen::Vector3d &v1)
{
	return v1/v1.norm();
}


//Saturate a scalar double between -X and +X
double symmetricSaturationDouble(const double inval, const double maxval)
{
	//Handling this with an error to avoid needing an abs() each call
	if(maxval<0)
	{ std::cout <<"ERROR: Saturation bound is negative" << std::endl;}
	if(inval > maxval)
	{
		return maxval;
	}else if(inval < -1.0*maxval)
	{
		return -1.0*maxval;
	}else
	{
		return inval;
	}
}


//Turn matrix into orthonormal version via SVD.  Used to cut down numerical errors in rotation matrix
Eigen::Matrix3d orthonormalize(const Eigen::Matrix3d &inmat)
{
	Eigen::Matrix3d outmat;
	Eigen::Matrix3d U,V;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(inmat, Eigen::ComputeFullV | Eigen::ComputeFullU);
	U=svd.matrixU();
	V=svd.matrixV();
	outmat = U*(V.transpose());
	return outmat;
}


//Adapted with minor changes from
//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
Eigen::Quaterniond rotmat2quat(const Eigen::Matrix3d &RR)
{
	double trace = RR.trace();
	Eigen::Matrix<double,4,1> q;
	if (trace > 0) {// RR_EPSILON = 0
		double s = 0.5 / sqrt(trace + 1.0);
		q << 0.25 / s,
			(RR(2,1) - RR(1,2))*s,
			(RR(0,2) - RR(2,0))*s,
			(RR(1,0) - RR(0,1))*s;
	}
	else {
		if (RR(0,0) > RR(1,1) && RR(0,0) > RR(2,2)) {
			double s = 2.0*sqrt(1.0 + RR(0,0) - RR(1,1) - RR(2,2));
			q << (RR(2,1) - RR(1,2))/s,
				 0.25*s,
				 (RR(0,1) + RR(1,0))/s,
				 (RR(0,2) + RR(2,0))/s;
		}
		else if (RR(1,1) > RR(2,2)) {
			double s = 2.0*sqrt(1.0 + RR(1,1) - RR(0,0) - RR(2,2));
			q << (RR(0,2) - RR(2,0))/s,
			     (RR(0,1) + RR(1,0))/s,
			     0.25 * s,
			     (RR(1,2) + RR(2,1))/s;
		}
		else {
			double s = 2.0*sqrt(1.0 + RR(2,2) - RR(0,0) - RR(1,1));
			q << (RR(1,0) - RR(0,1))/s,
			     (RR(0,2) + RR(2,0))/s,
			     (RR(1,2) + RR(2,1))/s,
			     0.25 * s;
		}
	}

	Eigen::Quaterniond quat;
	q=q/q.norm();
	quat.x()=q(1);
	quat.y()=q(2);
	quat.z()=q(3);
	quat.w()=q(0);
    return quat;	
}


//Convert quaternion into rotation matrix
Eigen::Matrix3d rotMatFromQuat(const Eigen::Quaterniond &qq)
{
	const double xx=qq.x();
	const double yy=qq.y();
	const double zz=qq.z();
	const double ww=qq.w();
	Eigen::Matrix3d RR;
	RR << 1-2*yy*yy-2*zz*zz, 2*xx*yy-2*ww*zz, 2*xx*zz+2*ww*yy,
    	2*xx*yy+2*ww*zz, 1-2*xx*xx-2*zz*zz, 2*yy*zz-2*ww*xx,
    	2*xx*zz-2*ww*yy, 2*yy*zz+2*ww*xx, 1-2*xx*xx-2*yy*yy;
	return RR;
}


//321
Eigen::Matrix3d rotMatFromEuler(const Eigen::Vector3d &ee)
{
  const double phi=ee(0);
  const double theta=ee(1);
  const double psi=ee(2);
  Eigen::Matrix3d RR;
  RR<<cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta),
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(theta)*sin(phi)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta),
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta);
  return RR;
}	


//Convert gps time to seconds.
double tgpsToSec(const int week, const int secOfWeek, const double fracSec)
{
	static const int SEC_PER_WEEK(604800);
	return fracSec + secOfWeek + SEC_PER_WEEK*week;
}


}
