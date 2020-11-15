#include "segway_sim/low_level_node.hpp"

// Global declarations
ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;

segway_sim::optSol optSol_;
ros::Subscriber sub_optSol_;

ros::Subscriber sub_state_true_;
ros::Subscriber sub_state_nominal_;
ros::Subscriber sub_input_mpc_;
ros::Subscriber sub_linear_mat_;

ros::Publisher pub_inputAct_;
ros::Publisher pub_lowLevelLog_;

segway_sim::state stateTrue_;
segway_sim::state stateNominal_;
segway_sim::linearMatrices linearMatrices_;
segway_sim::input inputMpc_;
segway_sim::input inputTot_;
segway_sim::lowLevelLog lowLevelLog_;

ros::Time ros_time_init;
ros::Time ros_time_end;

using namespace std;
double offset_angle_;
double dt_;
bool lowLevelActive_;

using namespace ControlBarrierFunction;

CBF *cbf;

// Initialize constant parameters
const int nx = 7;
const int nu = 2;
double X[nx]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};     // true state
double Xn[nx]    = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};    // nominal state
double XnTest[nx]    = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};    // nominal state
double X_IC[nx]  = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};     // true state initial condition to CBF QP
double Xn_IC[nx] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};    // nominal state initial condition to CBF QP
double uMPC[nu]  = {0.0, 0.0};
double delay_ms_;
double Alinear[nx*nx] = {};
double Blinear[nx*nu] = {};
double Clinear[nx] = {};

double xmax        = {};
double ymax        = {};
double thetamax    = {};
double vmax        = {};
double thetaDotmax = {};
double psimax      = {};
double psiDotmax   = {};

const int N_read = 10;
const int N = 40;
int h_counter = 0;

double dt_mpc = 0.05;
double dt_counter = 0;

double xPred[nx*(N_read+1)] = {}; 
double uPred[nu*N_read] = {}; 

int flagInputPub_ = 0;
int flagNominalStatePub_ = 0;
std::vector<double> inputTotBuffer1_;
std::vector<double> inputMpcBuffer1_;
std::vector<double> inputTotBuffer2_;
std::vector<double> inputMpcBuffer2_;

// Functions Definition
void stateTrueCallback(const segway_sim::state::ConstPtr msg)
{
	stateTrue_ = *msg;
	X[0]  = stateTrue_.x;
	X[1]  = stateTrue_.y;
	X[2]  = stateTrue_.theta;
	X[3]  = stateTrue_.v;
	X[4]  = stateTrue_.thetaDot;
	X[5]  = stateTrue_.psi;
	X[6]  = stateTrue_.psiDot;
}

void stateNominalCallback(const segway_sim::state::ConstPtr msg)
{
	flagNominalStatePub_ = 1;

	stateNominal_ = *msg;
	Xn[0] = stateNominal_.x;
	Xn[1] = stateNominal_.y;
	Xn[2] = stateNominal_.theta;
	Xn[3] = stateNominal_.v;
	Xn[4] = stateNominal_.thetaDot;
	Xn[5] = stateNominal_.psi;
	Xn[6] = stateNominal_.psiDot;

	XnTest[0] = stateNominal_.x;
	XnTest[1] = stateNominal_.y;
	XnTest[2] = stateNominal_.theta;
	XnTest[3] = stateNominal_.v;
	XnTest[4] = stateNominal_.thetaDot;
	XnTest[5] = stateNominal_.psi;
	XnTest[6] = stateNominal_.psiDot;
}

void optSolCallback(const segway_sim::optSol::ConstPtr msg)
{
	flagNominalStatePub_ = 1;
	dt_counter = 0;
	h_counter = 0;
	
	optSol_ = *msg;
	for (int j =0; j<N_read; j++){
		for (int i = 0; i<nx; i++){
			if (i == 5){
				xPred[i+j*nx] = optSol_.optimalSolution[i+j*nx] + offset_angle_; //MPC local --> back to global
			}else{
				xPred[i+j*nx] = optSol_.optimalSolution[i+j*nx];
			}
		}
	}

	for (int i = 0; i< nu*N_read; i++){
		uPred[i] = optSol_.optimalSolution[nx*(N+1) + i];
	}

	for (int i = 0; i < nx; ++i)
	{
		Xn[i] = xPred[i];
	}
	// cout << "update xpred and upred" << endl;
}

void linearMatricesCallback(const segway_sim::linearMatrices::ConstPtr msg)
{
	linearMatrices_ = *msg;

	for (int i = 0; i < nx*nx; i++){
		Alinear[i] = linearMatrices_.Alinear[i];
	}
	for (int i = 0; i < nx*nu; i++){
		Blinear[i] = linearMatrices_.Blinear[i];
	}
	for (int i = 0; i < nx; i++){
		Clinear[i] = linearMatrices_.Clinear[i];
	}
}

void inputCallback(const segway_sim::input::ConstPtr msg)
{
	flagInputPub_ = 1;
	inputMpc_ = *msg;
	inputMpcBuffer1_.push_back(inputMpc_.inputVec[0]);
	inputMpcBuffer1_.erase(inputMpcBuffer1_.begin());
	inputMpcBuffer2_.push_back(inputMpc_.inputVec[1]);
	inputMpcBuffer2_.erase(inputMpcBuffer2_.begin());
}

// Main Definition
int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"cpp_lowLevel");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	dt_ = 0.001;
	ros::Rate rate(1/dt_);


	// Read Launch File
	nhParams_->param<double>("offset_angle",offset_angle_,0.);
	nhParams_->param<bool>("low_level_active",lowLevelActive_,false);
	nhParams_->param<double>("low_level_input_delay",delay_ms_,0.);

	nhParams_->param<double>("xmax"       , xmax,0.);
	nhParams_->param<double>("ymax"       , ymax,0.);
	nhParams_->param<double>("thetamax"   , thetamax,0.);
	nhParams_->param<double>("vmax"       , vmax,0.);
	nhParams_->param<double>("thetaDotmax", thetaDotmax,0.);
	nhParams_->param<double>("psimax"     , psimax,0.);
	nhParams_->param<double>("psiDotmax"  , psiDotmax,0.);

	if (delay_ms_ < 0.01){
		delay_ms_ = 1;
	}

	inputMpcBuffer1_.resize(int((delay_ms_/1000)/dt_) );
	inputTotBuffer1_.resize(int((delay_ms_/1000)/dt_) );	
	inputMpcBuffer2_.resize(int((delay_ms_/1000)/dt_) );
	inputTotBuffer2_.resize(int((delay_ms_/1000)/dt_) );	

	// Init pubs, subs and srvs
	sub_optSol_        = nh_->subscribe<segway_sim::optSol>("optimal_sol", 1, optSolCallback);
	// sub_state_nominal_ = nh_->subscribe<segway_sim::state>("state_nominal", 1, stateNominalCallback);

	sub_state_true_    = nh_->subscribe<segway_sim::state>("state_true", 1, stateTrueCallback);
	sub_linear_mat_    = nh_->subscribe<segway_sim::linearMatrices>("linear_matrices", 1, linearMatricesCallback);
	sub_input_mpc_     = nh_->subscribe<segway_sim::input>("mpc_input", 1, inputCallback);
	pub_inputAct_ 	   = nh_->advertise<segway_sim::input>("input", 1);
	pub_lowLevelLog_   = nh_->advertise<segway_sim::lowLevelLog>("lowLevelLog", 1);

	double H_x[3] = {10000,10000,1};
	cbf = new CBF(nx_, nu_, true, H_x,x_eq_, fullDynamics,safetySet,clf);

	double xeq[nx]  = { 0.0, 0.0, 0.0, 0.0, 0.0, offset_angle_, 0.0};  // initial condition to
	Xn[5] = offset_angle_;
	X[5]  = offset_angle_;

	cbf->setIC(X, Xn);

	cbf->setMatrices(Alinear, Blinear, Clinear);
	cbf->evaluateCBFqpConstraintMatrices(uMPC, 0);
	cbf->setUpOSQP(0);
	cbf->solveQP(10);

	// Define parameters
	const int printLevel = 0;

	inputMpc_.inputVec[0] = 0.0;
	inputMpc_.inputVec[1] = 0.0;
	
	double Xn_next[nx] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	if (lowLevelActive_){
		cout << "=========================== Low level is active " << endl;
	} else {
		cout << "=========================== Low level is NOT active " << endl;
	}

	cout << "[LOW LEVEL] Delay MPC input vector ";
	for (int i = 0; i < int((delay_ms_/1000)/dt_) ; ++i){
		cout << inputMpcBuffer1_[i] << ", "<< inputMpcBuffer1_[i] << "; ";
		cout << endl;
	}

	cout << "[LOW LEVEL] cbf->uCBF[0]: " << cbf->uCBF[0] << endl;
	cout << "[LOW LEVEL] cbf->uCBF[1]: " << cbf->uCBF[1] << endl;

	// Take it for a spin	
	
	while(ros::ok())
	{
		//Get latest input
		ros::spinOnce();

		// store ros time
		
		if ((flagInputPub_ == 1) && ( flagNominalStatePub_ == 1 )){ 
			// apply input (note that at time t we compute the input for time t+1)
			
			ros_time_init = ros::Time::now();
			cbf->setIC(X, Xn);
			cbf->setMatrices(Alinear, Blinear, Clinear);
			// Solve CBF-CLF QP
			uMPC[0] = inputMpc_.inputVec[0]; //inputMpcBuffer1_[int((delay_ms_/1000)/dt_)- 1];
			uMPC[1] = inputMpc_.inputVec[1]; //inputMpcBuffer2_[int((delay_ms_/1000)/dt_)- 1];
 			cbf->evaluateCBFqpConstraintMatrices(uMPC, 0);
 			
 			cbf->solveQP(0);
 		// 	if (dt_counter < dt_mpc){
 		// 		cbf->solveQP(0);
			// }else{
			// 	cbf->uCBF[0] = 0;
			// 	cbf->uCBF[1] = 0;
			// 	uMPC[0] = uPred[0+h_counter*nu];
			// 	uMPC[1] = uPred[1+h_counter*nu];
			// }

			inputTot_.inputVec[0] = lowLevelActive_ * (cbf->uCBF[0]) + inputMpc_.inputVec[0];
			inputTot_.inputVec[1] = lowLevelActive_ * (cbf->uCBF[1]) + inputMpc_.inputVec[1];
			
			lowLevelLog_.QPtime = ros_time_init.toSec()-ros_time_end.toSec();

			pub_inputAct_.publish(inputTot_);


			for (int i = 0; i < nx; i++){
				Xn_next[i] = 0.0;
				for (int j = 0; j < nx; ++j)
				{
					if (i==j){
						Xn_next[i] = Xn_next[i] + Xn[j] + (Alinear[i*nx + j] * (Xn[j]-xeq[j]) )*dt_;
					}else{
						Xn_next[i] = Xn_next[i] + (Alinear[i*nx + j] * (Xn[j]-xeq[j]) )*dt_;
					}
				}
				Xn_next[i] = Xn_next[i] + (Blinear[i*nu + 0] * inputMpc_.inputVec[0] + Blinear[i*nu + 1] * inputMpc_.inputVec[1] + Clinear[i])*dt_;
			}

			if (dt_counter > dt_mpc){

				if (h_counter*dt_mpc < dt_counter){
					h_counter = h_counter + 1;
					inputMpc_.inputVec[0] = uPred[0+h_counter*nu];
					inputMpc_.inputVec[1] = uPred[1+h_counter*nu];
					uMPC[0] = uPred[0+h_counter*nu];
					uMPC[0] = uPred[1+h_counter*nu];

					// cout << "inputMpc_.inputVec[0]: " << inputMpc_.inputVec[0] << endl;
					// cout << "inputMpc_.inputVec[0]: " << inputMpc_.inputVec[1] << endl;
					// cout << "=================================== h_counter: " << h_counter << endl;
				}				
			}

			// if (dt_counter <= dt_mpc){
			// 	h_counter = 0;
			// 	for (int i = 0; i < nx; i++){
			// 		Xn_next[i] = 0.0;
			// 		for (int j = 0; j < nx; ++j)
			// 		{
			// 			if (i==j){
			// 				Xn_next[i] = Xn_next[i] + Xn[j] + (Alinear[i*nx + j] * (Xn[j]-xeq[j]) )*dt_;
			// 			}else{
			// 				Xn_next[i] = Xn_next[i] + (Alinear[i*nx + j] * (Xn[j]-xeq[j]) )*dt_;
			// 			}
			// 		}
			// 		Xn_next[i] = Xn_next[i] + (Blinear[i*nu + 0] * inputMpc_.inputVec[0] + Blinear[i*nu + 1] * inputMpc_.inputVec[1] + Clinear[i])*dt_;
			// 	}
			// }else{

			// 	if (h_counter*dt_mpc < dt_counter){
			// 		h_counter = h_counter + 1;
			// 		inputMpc_.inputVec[0] = uPred[0+h_counter*nu];
			// 		inputMpc_.inputVec[1] = uPred[1+h_counter*nu];
			// 		cout << "inputMpc_.inputVec[0]: " << inputMpc_.inputVec[0] << endl;
			// 		cout << "inputMpc_.inputVec[0]: " << inputMpc_.inputVec[1] << endl;
			// 		cout << "=================================== h_counter: " << h_counter << endl;

			// 	}

			// 	double interp = (h_counter*dt_mpc - dt_counter)/dt_mpc;
			// 	cout << "interp: " << interp << endl;

			// 	for (int i = 0; i < nx; ++i)
			// 	{
			// 		// cout << "xPred[i + h_counter*nx]: " << xPred[i + h_counter*nx] << endl;
			// 		// cout << "xPred[i + (h_counter+1)*nx]: " << xPred[i + (h_counter+1)*nx] << endl;

			// 		Xn_next[i] = interp*xPred[i + h_counter*nx];// + (1-interp)*xPred[i + (h_counter+1)*nx];

			// 		// cout << "Xn_next[i]: " << Xn_next[i] << endl;
			// 	}
			// }

			// for (int i = 0; i < nx; i++){
			// 	if	(XnTest[i] != xPred[i])
			// 		cout << "i: " << i << " val: " << XnTest[i] << " , " << xPred[i] << endl;
			// }

			dt_counter = dt_counter + dt_;

			// if (dt_counter > dt_mpc)
			// 	cout << "dt_counter: " << dt_counter << endl;

			for (int i = 0; i < nx; i++){
				Xn[i] = Xn_next[i];
			}

			for (int i = 0 ; i<nx; i++){
				lowLevelLog_.X[i] = cbf->X[i];
				lowLevelLog_.Xn[i] = cbf->Xn[i];
			}

			for (int i = 0; i < nu; i++){
				lowLevelLog_.uCBF[i] = cbf->uCBF[i];
				lowLevelLog_.uMPC[i] = uMPC[i];
				lowLevelLog_.uTot[i] = inputTot_.inputVec[i];
			}

			lowLevelLog_.V = cbf->V[0];
			lowLevelLog_.h = cbf->h[0];
			lowLevelLog_.flagQP = cbf->flagQP;
			pub_lowLevelLog_.publish(lowLevelLog_);
			
			// read ros time and print solver time
			ros_time_end = ros::Time::now();
			// if (printLevel >= 1) cout << "Node Time: " << ros_time_init.toSec()-ros_time_end.toSec() << std::endl;
		}

		//Wait for tick
		rate.sleep();
	}

}