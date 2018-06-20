//IMU/GPS UKF as a complementary filter.
#include "filterImu.hpp"


namespace gpsimu_odom
{

/*Called when a gps measurement is received.  Propagates from the last imu time to
current gps time, then performs measurement update step.  The &imu argument is the
last imu measurement received before the gps measurement was received*/
void gpsImu::runUKF(const imuMeas &imu, const gpsMeas &gps)
{
	double timu, tgps, dt0;
	Eigen::Vector3d imuAccelMeas, imuAttRateMeas, internal_rI, internal_rC;
	imu.getMeas(timu, imuAccelMeas, imuAttRateMeas); //Store current imu time in dt
	gps.getMeas(tgps, internal_rI, internal_rC);
	dt0=tgps-timu; //subtract off t0 to get actual dt
	
	//KF
	Eigen::Matrix<double,15,1> x0;
	Eigen::Matrix<double,15,15> P0;
	spkfPropagate15(xState_, Pimu_, (Qk12dividedByDt_*dt0), dt0, imuAccelMeas, imuAttRateMeas, RBI_, Limu_, x0, P0);
	updateRBIfromGamma(RBI_,x0);
	spkfMeasure6(x0, P0, Rk_, internal_rI, internal_rC, RBI_, Lcg2p_, Ls2p_, xState_, Pimu_);
	updateRBIfromGamma(RBI_,xState_);


	//Test bias saturation to avoid OOM errors
	//saturateBiases(maxBa_,maxBg_);

	RBI_=orthonormalize(RBI_);

	return;
}


//Only run propagation step of UKF, called whenver an imu measurement is received.
void gpsImu::runUKFpropagateOnly(const double tPrev, const imuMeas &imu)
{
	double timu, dt0;
	Eigen::Vector3d imuAccelMeas, imuAttRateMeas;
	imu.getMeas(timu,imuAccelMeas,imuAttRateMeas); 
	dt0=timu-tPrev; //subtract off t0 to get actual dt
	//Qk12_ ~ (QK/dt_est)*dt
	Eigen::Matrix<double,15,1> x0;
	Eigen::Matrix<double,15,15> P0;
	spkfPropagate15(xState_,Pimu_,(Qk12dividedByDt_*dt0),dt0,imuAccelMeas,imuAttRateMeas,RBI_,Limu_, x0, P0);
	updateRBIfromGamma(RBI_,x0);

	xState_=x0;
	Pimu_=P0;

	//Test bias saturation to avoid OOM errors
	//saturateBiases(maxBa_,maxBg_);

	RBI_=orthonormalize(RBI_);

	return;
}


/*Dynamic nonlinear propagation for IMU data. NOTE: This handles noise and noise rate for gyro/accel
Assumes a 15-element state; matrix sizes are hardcoded instead of being dynamically handled to 
speed up operation.*/
Eigen::Matrix<double,15,1> gpsImu::fdynSPKF(const Eigen::Matrix<double,15,1> &x0, const double dt,
	const Eigen::Vector3d &fB0, const Eigen::Matrix<double,12,1> &vk, const Eigen::Vector3d &wB0,
	const Eigen::Matrix3d &RR, const Eigen::Vector3d &lAB)
{
	Eigen::Matrix<double,15,1> x1 = x0;
	//split x0 into component vectors, assuming [x,v,gamma,ba,bg]
	const Eigen::Vector3d x = x0.topRows(3);
	const Eigen::Vector3d v = x0.middleRows(3,3);
	const Eigen::Vector3d gamma = x0.middleRows(6,3);
	const Eigen::Vector3d ba = x0.middleRows(9,3);
	const Eigen::Vector3d bg = x0.bottomRows(3);
	const Eigen::Vector3d vgk = vk.topRows(3);
	const Eigen::Vector3d vgk2 = vk.middleRows(3,3);
	const Eigen::Vector3d vak = vk.middleRows(6,3);
	const Eigen::Vector3d vak2 = vk.bottomRows(3);
	const Eigen::Matrix3d RR2 = updateRBIfromGamma(RR,x0.middleRows(6,3));

	//Approximate propagation
	Eigen::Vector3d xkp1 = x + dt*v;
	Eigen::Vector3d omegaB = wB0 - bg - vgk;
	Eigen::Vector3d wB_x_wB_x_lAB = omegaB.cross(omegaB.cross(lAB));
	Eigen::Vector3d a = RR2.transpose()*(fB0 - wB_x_wB_x_lAB - ba - vak) - Eigen::Vector3d(0,0,9.8);
	Eigen::Vector3d vkp1 = v + dt*a;
	Eigen::Vector3d gammakp1 = gamma + dt*omegaB;

	//Outputs--it is assumed that biases do not vary (time constant sufficiently large such that bkp1=bk)
	x1.topRows(3) = xkp1;
	x1.middleRows(3,3) = vkp1;
	x1.middleRows(6,3) = gammakp1;
	x1.middleRows(9,3) = exp(-dt/tauA_)*x1.middleRows(9,3) + vak2;
	x1.bottomRows(3) = exp(-dt/tauG_)*x1.bottomRows(3) + vgk2;

	return x1;
}

//True state nonlinear measurement equation.  Again using a 15-element state.
Eigen::Matrix<double,6,1> gpsImu::hnonlinSPKF(const Eigen::Matrix<double,15,1> &x0,
	const Eigen::Matrix3d &RR, const Eigen::Vector3d &ls2p, const Eigen::Vector3d &lcg2p,
	const Eigen::Matrix<double,6,1> &vk)
{
	Eigen::Matrix<double,6,1> zhat;

	Eigen::Matrix3d R2 = updateRBIfromGamma(RR,x0.middleRows(6,3));
	Eigen::Vector3d rCB = R2.transpose()*unit3(ls2p)+vk.bottomRows(3);
	zhat.topRows(3)=x0.topRows(3)+R2.transpose()*lcg2p+vk.topRows(3);
	zhat.bottomRows(3)=rCB;
	return zhat;
}


//Propagation step for the UKF with a state vector of length 15
void gpsImu::spkfPropagate15(const Eigen::Matrix<double,15,1> &x0, const Eigen::Matrix<double,15,15> &P0,
	const Eigen::Matrix<double,12,12> &Q, const double dt, const Eigen::Vector3d &fB0, const Eigen::Vector3d &wB0,
	const Eigen::Matrix3d &RR, const Eigen::Vector3d &lAB, Eigen::Matrix<double,15,1> &xkp1, Eigen::Matrix<double,15,15> &Pkp1)
{
	static const double epsilon(1.0e-8);
	static const double alpha(1.0e-3);
	static const double beta(2.0);
	static const double kappa(0.0);
	static const int nn(27);
	static const double lambda = (alpha*alpha)*(kappa+double(nn))-double(nn);
	static const double w_mean_center(lambda/(double(nn)+lambda));
	static const double w_mean_reg(1.0/2.0/(double(nn)+lambda));
	static const double w_cov_center(w_mean_center+1.0-alpha*alpha+beta);
	static const double w_cov_reg(w_mean_reg);
	static const double cp(sqrt(double(nn)+lambda));
	Eigen::Matrix<double,27,27> Paug=Eigen::Matrix<double,27,27>::Zero();
	Eigen::Matrix<double,15,1> xBar, storeDum, xSPoint;
	Eigen::Matrix<double,15,55> xStore;
	Eigen::Matrix<double,27,27> cholP; // compute the Cholesky decomposition of A
	Paug.topLeftCorner(15,15)=P0;
	Paug.bottomRightCorner(12,12)=Q;

	cholP = (Paug.llt().matrixL());

	xBar = fdynSPKF(x0, dt, fB0, Eigen::Matrix<double,12,1>::Zero(), wB0, RR, lAB);
	xStore.col(0) = xBar;
	Eigen::Matrix<double,27,1> xAug, x_sp;
	xAug.topRows(15) = x0;
	xAug.bottomRows(12)=Eigen::Matrix<double,12,1>::Zero();
	xBar = w_mean_center*xBar;
	
	int colno;
	double spSign; //This can be an int (will only have values on +-1), I'm just being careful to avoid implicit conversions.
	//Propagate through sigma points
	spSign=1.0;
	for(int ij=0; ij<2*nn; ij++)
	{
		//iterate left -> right columns, flip sign, iterate left-right columns again
		colno = ij%nn;
		if(ij>=nn)
		{
			spSign=-1.0;
		}
		x_sp = xAug + cp*spSign*cholP.col(colno);
		xSPoint=x_sp.topRows(15);
		storeDum = fdynSPKF(xSPoint, dt, fB0, x_sp.bottomRows(12), wB0, RR, lAB);
		xStore.col(ij+1) = storeDum;
		xBar = xBar + w_mean_reg*storeDum;
	}
	//std::cout << "xbar:" <<std::endl<<xBar<<std::endl;

	//Recombine for covariance
	Eigen::Matrix<double,15,15> PbaRk_p1;
	PbaRk_p1 = w_cov_center*(xStore.col(0)-xBar)*((xStore.col(0)-xBar).transpose());
	for(int ij=0; ij<2*nn; ij++)
	{
		PbaRk_p1 = PbaRk_p1 + w_cov_reg*(xStore.col(ij+1)-xBar)*((xStore.col(ij+1)-xBar).transpose());
	}
	//std::cout << "Pmax: " << PbaRk_p1.maxCoeff() << std::endl;

	//outputs
	xkp1 = xBar;
	Pkp1 = PbaRk_p1;
	//std::cout << "P:" <<std::endl << Pkp1 <<std::endl;
	return;
}


//UKF measurement update.  15-element state, same order as propgation
void gpsImu::spkfMeasure6(const Eigen::Matrix<double,15,1> &x0, const Eigen::Matrix<double,15,15> &P0,
	const Eigen::Matrix<double,6,6> &R, const Eigen::Vector3d &rI_measurement, const Eigen::Vector3d &rCu_measurement,
	const Eigen::Matrix3d &RR, const Eigen::Vector3d &lcg2p, const Eigen::Vector3d &ls2p,
	Eigen::Matrix<double,15,1> &xkp1, Eigen::Matrix<double,15,15> &Pkp1)
{
	static const double epsilon(1.0e-8);
	static const double alpha(1.0e-3);
	static const double beta(2.0);
	static const double kappa(0.0);
	static const int nn(21);
	static const double lambda = (alpha*alpha)*(kappa+double(nn))-double(nn);
	static const double w_mean_center(lambda/(double(nn)+lambda));
	static const double w_mean_reg(1.0/2.0/(double(nn)+lambda));
	static const double w_cov_center(w_mean_center+1.0-alpha*alpha+beta);
	static const double w_cov_reg(w_mean_reg);
	static const double cp(sqrt(nn+lambda));
	Eigen::Matrix<double,21,21> Paug=Eigen::Matrix<double,21,21>::Zero();
	Eigen::Matrix<double,6,1> zBar, storeDum;
	Eigen::Matrix<double,6,43> zStore;
	Eigen::Matrix<double,15,43> xStore;
	Eigen::Matrix<double,15,1> xSPoint;
	Eigen::Matrix<double,21,21> cholP; // compute the Cholesky decomposition of A
	Eigen::Vector3d ls2p_unit3 = unit3(ls2p);

	//Center point and propagation
	Paug.topLeftCorner(15,15)=P0; Paug.bottomRightCorner(6,6)=R;
	cholP = (Paug.llt().matrixL());
	zBar = hnonlinSPKF(x0, RR, ls2p, lcg2p, Eigen::Matrix<double,6,1>::Zero());
	zStore.col(0) = zBar;
	xStore.col(0) = x0;
	Eigen::Matrix<double,21,1> xAug, x_sp;
	xAug.topRows(15) = x0;
	xAug.bottomRows(6)=Eigen::Matrix<double,6,1>::Zero();
	zBar = w_mean_center*zBar;
	int colno;
	double spSign; //This can be an int (will only have values on +-1), I'm just being careful to avoid implicit conversions.

	//Propagate regression points
	spSign=1.0;
	for(int ij=0; ij<2*nn; ij++)
	{
		colno = ij%nn;
		if(ij>=nn)
		{
			spSign=-1.0;
		}
		x_sp = xAug + cp*spSign*cholP.col(colno);
		xSPoint = x_sp.topRows(15);
		storeDum = hnonlinSPKF(xSPoint, RR, ls2p, lcg2p, x_sp.bottomRows(6));
		zStore.col(ij+1) = storeDum;
		xStore.col(ij+1) = x_sp.topRows(15);
		zBar = zBar + w_mean_reg*storeDum;
	}

	//Recombine for covariance
	Eigen::Matrix<double,15,6> Pxz;
	Eigen::Matrix<double,6,6> Pzz;
	Eigen::Matrix<double,6,1> dz;
	Pxz = w_cov_center*(xStore.col(0)-x0)*((zStore.col(0)-zBar).transpose());
	Pzz = w_cov_center*(zStore.col(0)-zBar)*((zStore.col(0)-zBar).transpose());
	for(int ij=0; ij<2*nn; ij++)
	{
		dz = zStore.col(ij+1)-zBar; //For efficiency
		Pxz = Pxz + w_cov_reg*(xStore.col(ij+1)-x0)*(dz.transpose());
		Pzz = Pzz + w_cov_reg*dz*(dz.transpose());
	}

	//LMMSE
	Eigen::Matrix<double,6,1> z_measurement;
	const Eigen::Matrix<double,6,6> PzzInv = Pzz.inverse(); //Efficiency
	z_measurement.topRows(3)=rI_measurement;
	z_measurement.bottomRows(3)=unit3(rCu_measurement);
	xkp1 = x0 + Pxz*PzzInv*(z_measurement-zBar);
	Pkp1 = P0 - Pxz*PzzInv*(Pxz.transpose());
	return;
}


void gpsImu::saturateBiases(const double baMax, const double bgMax)
{
	//Saturation
	for(int ij=0; ij<3; ij++)
	{
		xState_(ij+9) = symmetricSaturationDouble(xState_(ij+9),baMax);
	}
	for(int ij=0; ij<3; ij++)
	{
		xState_(ij+12) = symmetricSaturationDouble(xState_(ij+12),bgMax);
	}
}


}
