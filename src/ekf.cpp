#include "segway_sim/ekf.h"

typedef Eigen::Matrix<double,STATE_LENGTH,STATE_LENGTH> MatrixNN;
typedef Eigen::Matrix<double,MEASUREMENT_LENGTH,STATE_LENGTH> MatrixZN;
typedef Eigen::Matrix<double,STATE_LENGTH,MEASUREMENT_LENGTH> MatrixNZ;
typedef Eigen::Matrix<double,MEASUREMENT_LENGTH,MEASUREMENT_LENGTH> MatrixZZ;
typedef Eigen::Matrix<double,STATE_LENGTH,1> VectorN;
typedef Eigen::Matrix<double,MEASUREMENT_LENGTH,1> VectorZ;
typedef Eigen::Map<MatrixNN> MapMatrixNN;
typedef Eigen::Map<MatrixZN> MapMatrixZN;

EKF::EKF(const std::vector<double> &Q,
	     const std::vector<double> &R):
Q_(Q),
R_(R)
{}

void EKF::init(const std::vector<double> &x0, const std::vector<double> &P0vec)
{
	x_t_ = x0;
	P_t_.resize(STATE_LENGTH*STATE_LENGTH);
	MapMatrixNN P(P_t_.data());
	P.setZero();
	for (int i = 0; i < (int)P0vec.size(); i ++) {
		P(i,i) = P0vec[i];
	}
	initialized = 1;
}

void EKF::update(const std::vector<double> &U,
				 const std::vector<double> &M,
				 const double dt)
{
	//predict x
	double xDot[STATE_LENGTH];
	dynamics(0.,x_t_.data(),U.data(),xDot);
	double xPred[STATE_LENGTH] = {0.};
	for (int i = 0; i < (int)STATE_LENGTH; i++) {
		xPred[i] = x_t_[i]+xDot[i]*dt;
	}

	//get F
	double DfRaw[STATE_LENGTH*STATE_LENGTH];
	dynamicsGradient(0., x_t_.data(), U.data(), DfRaw);
	MapMatrixNN Df(DfRaw);

	//get h and H
	double h[MEASUREMENT_LENGTH];
	measurementH(xPred,h);
	double DhRaw[MEASUREMENT_LENGTH*STATE_LENGTH];
	measurementHGradient(xPred,DhRaw);
	MapMatrixZN Dh(DhRaw);

	//setup Q and R
	MatrixNN Q;
	Q.setZero();

	MatrixZZ R;
	R.setZero();

	for (int i = 0; i < STATE_LENGTH; i++) {
		Q(i,i) = Q_[i];
	}

	for (int i = 0; i < MEASUREMENT_LENGTH; i++) {
		R(i,i) = R_[i];
	}

	//predict P
	const Eigen::Map<const MatrixNN> P_old(P_t_.data());
	MatrixNN P = P_old + dt*(Df*P_old+P_old*Df.transpose()+Q);

	//map xPred and z and h
	const Eigen::Map<const VectorZ> z(M.data());

	const Eigen::Map<const VectorZ> h_mat(h);
	const Eigen::Map<const VectorN> xPred_mat(xPred);

	//compute K
	MatrixNZ K = P*Dh.transpose()*(Dh*P*Dh.transpose()+R).inverse();
//
//	//compute x
	Eigen::Map<VectorN> X_new(x_t_.data());
	X_new = xPred_mat + K*(z-h_mat);
//
//	//compute P
	MapMatrixNN P_new(P_t_.data());
  P_new = ((MatrixNN::Identity()-K*Dh)*P).eval();
	initialized = 1;
}
