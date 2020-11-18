#include "segway_sim/mpc_drone_node.hpp"

using namespace std;
using namespace ModelPredictiveControllerValFun;

// Global declarations
ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;

ros::Subscriber sub_state_;
ros::Subscriber sub_joy_;
ros::Subscriber sub_goalSetAndState_;

ros::Publisher pub_inputAct_;
ros::Publisher pub_optSol_;

segway_sim::stateDrone stateCurrent_;
ambercortex_ros::ctrl_info stateCurrent_hw_;
segway_sim::cmdDrone inputAct_;
ambercortex_ros::cmd inputAct_hw_;
segway_sim::goalSetAndState goalSetAndState_;
segway_sim::optSol optSol_;
segway_sim::linearMatrices linearMatrices_;

ros::Time ros_time_init;
ros::Time ros_time_end;

std::vector<double> inputBuffer_u1_;
std::vector<double> inputBuffer_u2_;

double dt_;
double dtDelay_;
double delay_ms_;
double x_start;
double y_start;
double vMax = {0.0};
double error_max_[nx_] = {};
double linearization_IC_[nx_] = {};
bool initialized_ = false;
bool takeoff_ = false;


double e_x        = {};
double e_y        = {};
double e_vbx        = {};
double e_vby        = {};

double enlarge    = {};
double lowLevelActive = {};

ros::Time to_init_;

int flag_state_measurement = 0;
int flag_goalSetAndState_measurement = 0;

MPCValFun *mpcValFun; // Define MPC object as a pointer

bool hardware_ = false;
ros::Time last_button;
int btn_init, btn_run, btn_backup, btn_l1, btn_r1;
double uz_ = 0;

void add_uint32_to_vec(std::vector<uint8_t> & vec, uint32_t ui) {
  // add four bits to the end of a std::vector<uint8_t> representing a float
  num32_t n32;
  n32.ui = ui;
  vec.push_back(n32.c[0]);
  vec.push_back(n32.c[1]);
  vec.push_back(n32.c[2]);
  vec.push_back(n32.c[3]);
}

//// Functions Definition
void sendToDrone(void)
{
	if (hardware_)
	{
		ambercortex_ros::cmd drone_msg;
		drone_msg.data.push_back(MSG_CTRL);

			// inputAct_.vDes[0] = mpcValFun->xPred[1*nx+2];
			// inputAct_.vDes[1] = mpcValFun->xPred[1*nx+3];
		double u1 = 0; double u2 = 0;

		for (int i = 0; i < 1; i++)
		{
			if (  !takeoff_ or (mpcValFun->xPred[1*nx+2] > 1.1) or (mpcValFun->xPred[1*nx+2] < -1.1) or (mpcValFun->xPred[1*nx+3] > 1.1) or (mpcValFun->xPred[1*nx+3] < -1.1) ){
				u1 = 0.0;
				u2 = 0.0;
			}else{
				u1 = mpcValFun->xPred[1*nx+2];
				u2 = mpcValFun->xPred[1*nx+3];
			}
			
		}
		if (!initialized_)
			to_init_ = ros::Time::now();

		// Send Nominal State

		if (ros::Time::now() > (to_init_ + ros::Duration(4)) && ros::Time::now() < (to_init_ + ros::Duration(7)))
			{uz_ = 0;
			u1 = 0;
			u2 = 0;
			takeoff_ = true;}
		add_float_to_vec(drone_msg.data, u1);
		add_float_to_vec(drone_msg.data, u2);
		add_float_to_vec(drone_msg.data, uz_);
		add_float_to_vec(drone_msg.data, 0.0);
		add_float_to_vec(drone_msg.data, 0.0);
		add_float_to_vec(drone_msg.data, 0.0);
		add_float_to_vec(drone_msg.data, 0.0);
		add_float_to_vec(drone_msg.data, 0.0);
		add_float_to_vec(drone_msg.data, 0.0);

		// Send matrices

		add_uint32_to_vec(drone_msg.data, stateCurrent_hw_.time);

		drone_msg.chksum = compute_vec_chksum(drone_msg.data);
		if (initialized_)
			pub_inputAct_.publish(drone_msg);				
	}
	else {
		pub_inputAct_.publish(inputAct_);				
	}
}

void stateCallback(const segway_sim::stateDrone::ConstPtr msg)
{
	flag_state_measurement = 1;
	stateCurrent_ = *msg;
}

void stateCallback_hw(const ambercortex_ros::ctrl_info::ConstPtr msg)
{
	flag_state_measurement = 1;
	stateCurrent_hw_ = *msg;
	// DO TO
	stateCurrent_.x = stateCurrent_hw_.data[0];
	stateCurrent_.y = stateCurrent_hw_.data[1];
	stateCurrent_.vbx = stateCurrent_hw_.data[3];
	stateCurrent_.vby = stateCurrent_hw_.data[4];
	if (initialized_)
		uz_ = fmin(.5,(1.4 - stateCurrent_hw_.data[2])) - 1*stateCurrent_hw_.data[5];
	else 
		uz_ = -.2;
	// cout << "uz to send: " << uz_ << endl;
	
	// stateCurrent_.theta = stateCurrent_hw_.state[2];
	// stateCurrent_.v = stateCurrent_hw_.state[3];
	// stateCurrent_.thetaDot = stateCurrent_hw_.state[4];
	// stateCurrent_.psi = stateCurrent_hw_.state[5];
	// stateCurrent_.psiDot = stateCurrent_hw_.state[6];
}

void goalSetAndStateCallback(const segway_sim::goalSetAndState::ConstPtr msg)
{
	flag_goalSetAndState_measurement = 1;
	goalSetAndState_ = *msg;
	cout <<" ===== Drone Goal Updated"<< endl;
	cout << "goalSetAndState_.x: " << goalSetAndState_.x << endl;
	cout << "goalSetAndState_.y: " << goalSetAndState_.y << endl;
	if (initialized_ == false) {
		ambercortex_ros::cmd msg_button;
  		msg_button.data.push_back(MSG_MODE);
	    msg_button.data.push_back(MODE_RUN);
		cout <<" ===== Going into run mode!!! "<< endl;
	    for (int i = 0; i < 39; i++) {
	      msg_button.data.push_back(0);
	    }
	    msg_button.chksum = compute_vec_chksum(msg_button.data);
	    pub_inputAct_.publish(msg_button);

	}
	initialized_ = true;
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"MPC");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	nhParams_->param<bool>("hardware", hardware_,false);

	// Init pubs, subs and srvs
	if (hardware_)
	{
		ROS_INFO("Running on real Drone");
		// TO DO 
		sub_state_ = nh_->subscribe<ambercortex_ros::ctrl_info>("ctrl_info", 1, stateCallback_hw);
		pub_inputAct_ = nh_->advertise<ambercortex_ros::cmd>("cmd", 1);
	} else {
		ROS_INFO("Running in simulation");
		sub_state_ = nh_->subscribe<segway_sim::stateDrone>("/uav_sim_ros/state", 1, stateCallback);
		pub_inputAct_ = nh_->advertise<segway_sim::cmdDrone>("/uav_sim_ros/uav_cmd_des", 1);
	}
	sub_goalSetAndState_   = nh_->subscribe<segway_sim::goalSetAndState>("droneGoalSetAndState", 1, goalSetAndStateCallback);
	pub_optSol_       = nh_->advertise<segway_sim::optSol>("drone_opt", 1);

	// Retrieve ROS parameters
	nhParams_->param<double>("dt", dt_,0.05);
	nhParams_->param<double>("mpc_input_delay", delay_ms_,0.);
	nhParams_->param<double>("x_start", x_start,0.);
	nhParams_->param<double>("y_start", y_start,0.);
	dtDelay_ = delay_ms_/1000.0;
	optSol_.delay_ms = delay_ms_;
	
	nhParams_->param<double>("e_x"       , e_x,0.);
	nhParams_->param<double>("e_vbx"       , e_vbx,0.);
	nhParams_->param<double>("e_y"       , e_y,0.);
	nhParams_->param<double>("e_vby"       , e_vby,0.);

	nhParams_->param<double>("enlarge"   , enlarge,0.);

	lowLevelActive = 0.;

	ros::param::get("~_btn_init", btn_init);
	ros::param::get("~_btn_run", btn_run);
	ros::param::get("~_btn_backup", btn_backup);	

	error_max_[0] = e_x;
	error_max_[2] = e_vbx;
	error_max_[1] = e_y;
	error_max_[3] = e_vby;

	error_max_[0] = 0;
	error_max_[2] = 0;
	error_max_[1] = 0;
	error_max_[3] = 0;
	enlarge= 0;

	double xCurr[nx]{}; // current measured state
	double uCurr[nu]{}; // current applied input

	double goalSetAndStateVector[11] = {0.0};
	double goalSetAndStateVectorOld[11] = {0.0};
	// States are:        x,   y, theta,   v, thetaDot,            psi, psiDot
	double xeq[nx]  = { 0.0, 0.0,   0.0,      0.0};  // initial condition to

	// Initial condition and goal in relative reference frame
	double x_IC[nx] = {  x_start, y_start,   0.0, 0.0,};  // initial condition to
	double x_g[nx]  = {  x_start, y_start-1, 0.0, 0.0};  // initial condition to

	for (int i = 0; i < nx_; i++)
		linearization_IC_[i]=x_IC[i];

	double highLevTime = -1;

	goalSetAndState_.x = x_IC[0];
	goalSetAndState_.y = x_IC[1];

	goalSetAndState_.xmin = x_IC[0] - 10.5;
	goalSetAndState_.xmax = x_IC[0] + 10.5;
	goalSetAndState_.ymin = x_IC[1] - 10.5;
	goalSetAndState_.ymax = x_IC[1] + 10.5;

	goalSetAndState_.highLevTime = goalSetAndState_.highLevTime;
	
	goalSetAndState_.term_xmin = x_IC[0] - 10.5;
	goalSetAndState_.term_xmax = x_IC[0] + 10.5;
	goalSetAndState_.term_ymin = x_IC[1] - 10.5;
	goalSetAndState_.term_ymax = x_IC[1] + 10.5;

	inputAct_.vDes[0] = 0.0;
	inputAct_.vDes[1] = 0.0;

	goalSetAndStateVector[0]  = goalSetAndState_.x*hardware_;
	goalSetAndStateVector[1]  = goalSetAndState_.y*hardware_;
	goalSetAndStateVector[2]  = goalSetAndState_.xmin;
	goalSetAndStateVector[3]  = goalSetAndState_.xmax;
	goalSetAndStateVector[4]  = goalSetAndState_.ymin;
	goalSetAndStateVector[5]  = goalSetAndState_.ymax;
	goalSetAndStateVector[6]  = goalSetAndState_.highLevTime;
	goalSetAndStateVector[7]  = goalSetAndState_.term_xmin;
	goalSetAndStateVector[8]  = goalSetAndState_.term_xmax;
	goalSetAndStateVector[9]  = goalSetAndState_.term_ymin;
	goalSetAndStateVector[10] = goalSetAndState_.term_ymax;

	ROS_INFO("========== START Initializing MPC Drone Object");
	string matrix_prefix_path = ros::package::getPath("segway_sim");
	
	cout << "matrix_prefix_path: " << matrix_prefix_path << endl;
	
	matrix_prefix_path = matrix_prefix_path + "/drone_qp_matrices/";
	
	cout << "matrix_prefix_path: " << matrix_prefix_path << endl;

	mpcValFun = new MPCValFun(nx_, nu_, N, dt_, dtDelay_, printLevel, xeq, error_max_, enlarge, lowLevelActive, linearization_IC_, matrix_prefix_path, linearizedDroneDynamics);
	mpcValFun->setGoalState(x_g);
	mpcValFun->setIC(x_IC);  // Solve QP to check the everything works
	mpcValFun->readAB();
	mpcValFun->readCost();
	
	mpcValFun->initiConstrVector();

	mpcValFun->linearize();
	mpcValFun->updateGoalSetAndState(goalSetAndStateVector);
	
	mpcValFun->buildConstrMatrix();
	mpcValFun->buildCost();
	mpcValFun->buildConstrVector();
	
	mpcValFun->setUpOSQP(1); // Initialize QP with verbose = 1
	ROS_INFO("========== DONE Initializing MPC Drone Object");

	mpcValFun->solveQP();  // Solve QP to check the everything works

	mpcValFun->linearize();			
	mpcValFun->buildCost();
	mpcValFun->buildConstrMatrix();
	mpcValFun->buildConstrVector();
	mpcValFun->solveQP();  // Solve QP to check the everything works

	  cout << "OPTIMAL States New:"<< std::endl;
	  for (int i = 0; i< nx; i++){
		  for (int j = 0; j < N+1; ++j)
			  {cout << mpcValFun->xPred[j*nx+i] <<",";}
		  cout << endl;
	  }	
	  cout << endl;

		cout << "OPTIMAL Inputs:"<< std::endl;
		for (int i = 0; i< nu; i++){
			for (int j = 0; j < N; ++j)
				cout << mpcValFun->uPred[j*nu+i] <<",";
			cout << endl;
		}	
		cout << endl;

	if (printLevel > 0) {
	  cout << "OPTIMAL States New:"<< std::endl;
	  for (int i = 0; i< nx; i++){
		  for (int j = 0; j < N+1; ++j)
			  {cout << mpcValFun->xPred[j*nx+i] <<",";}
		  cout << endl;
	  }	
	  cout << endl;

		cout << "OPTIMAL Inputs:"<< std::endl;
		for (int i = 0; i< nu; i++){
			for (int j = 0; j < N; ++j)
				cout << mpcValFun->uPred[j*nu+i] <<",";
			cout << endl;
		}	
		cout << endl;
	}	

	for (auto i = 0; i < nu_; i++)
		inputAct_.vDes[i] = mpcValFun->xPred[2*i + 1];

	if (delay_ms_>0){
		inputBuffer_u1_.resize( int(dt_/(delay_ms_/1000)) );
		inputBuffer_u2_.resize( int(dt_/(delay_ms_/1000)) );
		cout << "===== MPC delay: " << delay_ms_ << ". Initialize inputBuffer_u1_: ";
		for (int i = 0; i < int(dt_/(delay_ms_/1000)); ++i){
			cout << inputBuffer_u1_[i] << ", ";
		}
		cout << endl;
	}

	int horizonCounter = 0;

	ros::Rate rate(1/dt_);
	// Take it for a spin
	while(ros::ok())
	{
		//Get latest input
		ros::spinOnce();

		if ((flag_state_measurement == 1)){
			// apply input
			if (delay_ms_ > -0.5){
				sendToDrone();
			}
			
			// store ros time
			ros_time_init = ros::Time::now();
			
			// read current input and state in local coordinates
			uCurr[0] = inputAct_.vDes[0];
			uCurr[1] = inputAct_.vDes[1];
			
			xCurr[0] = stateCurrent_.x + x_start*hardware_;
			xCurr[1] = stateCurrent_.y + y_start*hardware_;
			xCurr[2] = stateCurrent_.vbx;
			xCurr[3] = stateCurrent_.vby;

			// xCurr[2] = stateCurrent_.theta;
			// xCurr[4] = stateCurrent_.thetaDot;
			// xCurr[5] = stateCurrent_.psi - offset_angle_;
			// xCurr[6] = stateCurrent_.psiDot;

			goalSetAndStateVector[0] = goalSetAndState_.x;
			goalSetAndStateVector[1] = goalSetAndState_.y;

			goalSetAndStateVector[2] = goalSetAndState_.xmin - 10.0;
			goalSetAndStateVector[3] = goalSetAndState_.xmax + 10.0;
			goalSetAndStateVector[4] = goalSetAndState_.ymin - 10.0;
			goalSetAndStateVector[5] = goalSetAndState_.ymax + 10.0;
		
			goalSetAndStateVector[6] = goalSetAndState_.highLevTime;

			goalSetAndStateVector[7] = goalSetAndState_.term_xmin - 10.0;
			goalSetAndStateVector[8] = goalSetAndState_.term_xmax + 10.0;
			goalSetAndStateVector[9] = goalSetAndState_.term_ymin - 10.0;
			goalSetAndStateVector[10] = goalSetAndState_.term_ymax + 10.0;
	
			mpcValFun->linearize();		
			mpcValFun->setIC(xCurr);
			// compute initial condition by propagating foward dynamics
			if (delay_ms_ > 0.5){
				for (int i = 0; i < int( dt_/(delay_ms_/1000) ); ++i){
					uCurr[0] = inputBuffer_u1_[i];
					uCurr[1] = inputBuffer_u2_[i];
					mpcValFun->oneStepPredictionDelay(uCurr);
				}
			} else if (delay_ms_ > -0.5){
				mpcValFun->oneStepPrediction(uCurr);
			}

			
			if (highLevTime < goalSetAndStateVector[6]){
				highLevTime = goalSetAndStateVector[6];
				cout << "================== High Level Time Updated: " << highLevTime << endl;
				mpcValFun->resetHorizon();
				mpcValFun->updateGoalSetAndState(goalSetAndStateVector);
				horizonCounter = 0;
			}
			mpcValFun->buildCost();
			mpcValFun->buildConstrMatrix();
			mpcValFun->buildConstrVector();
			
			// solve QP
			mpcValFun->solveQP();
			if ((mpcValFun->solverFlag == 1) or (mpcValFun->solverFlag == 2)){
				for (int i = 0; i < 11; i++){
					goalSetAndStateVectorOld[i] = goalSetAndStateVector[i];
				}
			} else {
				cout << "==================== Solving  Drone Contingency plan ======================= " << endl;
				mpcValFun->updateGoalSetAndState(goalSetAndStateVectorOld);
				mpcValFun->updateHorizon();
				mpcValFun->buildCost();
				mpcValFun->buildConstrMatrix();
				mpcValFun->buildConstrVector();
				mpcValFun->solveQP();
				cout << "==================== Flag Drone Contingency Plan: "<< mpcValFun->solverFlag << endl;
				mpcValFun->updateGoalSetAndState(goalSetAndStateVector);			
			}

			if ( horizonCounter == 1){
				mpcValFun->updateHorizon();
				horizonCounter = 0;
			} else {
				horizonCounter += 1;
			}
			inputAct_.vDes[0] = mpcValFun->xPred[1*nx+2];
			inputAct_.vDes[1] = mpcValFun->xPred[1*nx+3];

			if (delay_ms_ > 0){
				inputBuffer_u1_[0] = inputBuffer_u1_[int(dt_/(delay_ms_/1000))-1];
				inputBuffer_u2_[0] = inputBuffer_u2_[int(dt_/(delay_ms_/1000))-1];
				for (int i = 1; i < int( dt_/(delay_ms_/1000) ); ++i){
					inputBuffer_u1_[i] = inputAct_.vDes[0];
					inputBuffer_u2_[i] = inputAct_.vDes[1];
				}
			}
			
			for (int i = 0; i< nx*(N+1); i++){
				optSol_.optimalSolution[i] = mpcValFun->xPred[i];
			}
			for (int i = 0; i< nu*N; i++){
				optSol_.optimalSolution[nx*(N+1) + i] = mpcValFun->uPred[i];
			}

			// read ros time and print solver time
			ros_time_end = ros::Time::now();
			if (printLevel >= 0.5) cout << "Solver Time: " << ros_time_end.toSec()-ros_time_init.toSec() << std::endl;
			optSol_.solverFlag = mpcValFun->solverFlag;
			optSol_.solverTime = ros_time_end.toSec()-ros_time_init.toSec();
			optSol_.x = goalSetAndState_.x; 
			optSol_.y = goalSetAndState_.y;
			
			for (int i = 0; i < nx; i++){
				optSol_.xCurr[i] = xCurr[i];
				optSol_.x_IC[i] = mpcValFun->x_IC_[i];
			}
			pub_optSol_.publish(optSol_);
					

			if (delay_ms_ < -0.5){
				sendToDrone();
			}

		}
		//Wait for tick
		rate.sleep();
	}
}