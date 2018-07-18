//Contains matrix helper functions
#include "mathHelperFunctions.hpp"

namespace gpsimu_odom
{
Eigen::Matrix3d ecef2enu_rotMatrix(Eigen::Vector3d &ECEF){


    //Define WGS-84 Earth parameters
    const double aa = 6378137.00000;
    const double bb = 6356752.3142518;
    const double ee = 0.0818191908334158;
    const double ep = sqrt((aa*aa - bb*bb)/(bb*bb));
	const double ee2 = (aa*aa-bb*bb)/(aa*aa);

    //Convert to (phi,lambda,h) geodetic coordinates
    double x = ECEF(0);
    double y = ECEF(1);
    double z = ECEF(2);
    double lambda = atan2(y, x);
    double p = sqrt(x*x + y*y);
    double theta = atan2(z*aa, p*bb);
    double phi=atan2(z,(1-ee2)*p);
    double N,h, phiprev;
    bool contvar=true;
    while(contvar)
    {
        phiprev=phi;
    	N=aa/sqrt(1-ee2*sin(phi)*sin(phi));
    	h=p/cos(phi)-N;
        phi=atan2(z,(1-ee2*N/(N+h))*p);
        if(abs(phiprev-phi)<1e-6)
        {
        	contvar=false;
	    }
    }

    //Form the rotation matrix
    Eigen::Matrix3d Renu_ecef = Eigen::Matrix3d::Zero();
    Renu_ecef(0,0) = -sin(lambda);
    Renu_ecef(0,1) = cos(lambda);
    Renu_ecef(0,2) = 0;
    Renu_ecef(1,0) = -sin(phi)*cos(lambda);
    Renu_ecef(1,1) = -sin(phi)*sin(lambda);
    Renu_ecef(1,2) = cos(phi);
    Renu_ecef(2,0) = cos(phi)*cos(lambda);
    Renu_ecef(2,1) = cos(phi)*sin(lambda);
    Renu_ecef(2,2) = sin(phi);

 return Renu_ecef;
}


Eigen::Vector3d ecef2enu(Eigen::Vector3d &ECEF){
    Eigen::Matrix3d R = ecef2enu_rotMatrix(ECEF);
    Eigen::Vector3d ENU = R*ECEF;
    return ENU;
}


//Rotation matrix, 312 convention
Eigen::Matrix3d euler2dcm312(const Eigen::Vector3d &ee)
{
	const double roll=ee(0);
	const double pitch=ee(1);
	const double yaw=ee(2);
 	const Eigen::AngleAxisd r3(yaw,   Eigen::Vector3d::UnitZ());
	const Eigen::AngleAxisd r2(pitch, Eigen::Vector3d::UnitY());
	const Eigen::AngleAxisd r1(roll,  Eigen::Vector3d::UnitX());
	const Eigen::Quaternion<double> q = r3*r1*r2;
	Eigen::Matrix3d RR = q.matrix();
	return RR;
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
	//note: an asDiagonal call here can cause errors due to non-initialized terms
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


//312 rotation such that vB=RR*vI
Eigen::Matrix3d rotMatFromEuler(const Eigen::Vector3d &ee)
{
	const double roll=ee(0);
	const double pitch=ee(1);
	const double yaw=ee(2);
 	const Eigen::AngleAxisd r3(yaw,   Eigen::Vector3d::UnitZ());
	const Eigen::AngleAxisd r2(pitch, Eigen::Vector3d::UnitY());
	const Eigen::AngleAxisd r1(roll,  Eigen::Vector3d::UnitX());
	const Eigen::Quaternion<double> q = r3*r1*r2;
	Eigen::Matrix3d RR = (q.matrix()).transpose(); //frame convention

	return RR;
}	


//Convert gps time to seconds.
double tgpsToSec(const int week, const int secOfWeek, const double fracSec)
{
	return fracSec + secOfWeek + SEC_PER_WEEK*week;
}


}
