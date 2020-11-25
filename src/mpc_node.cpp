#include "segway_sim/mpc_node.hpp"
#include "ambercortex_ros/ctrl_info.h"


using namespace std;
using namespace ModelPredictiveControllerValFun;

double time_after_delay_ = 10;
double delay_ = 250;
double t0_;

// Global declarations
ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;

ros::Subscriber sub_state_;
ros::Subscriber sub_joy_;
ros::Subscriber sub_goalSetAndState_;
ros::Subscriber sub_ctrl_info_;

ros::Publisher pub_inputAct_;
ros::Publisher pub_stateNominal_;
ros::Publisher pub_optSol_;
ros::Publisher pub_linearMat;

segway_sim::state stateCurrent_;
ambercortex_ros::state stateCurrent_hw_;
segway_sim::input inputAct_;
ambercortex_ros::cmd inputAct_hw_;
segway_sim::state stateNominal_;
segway_sim::goalSetAndState goalSetAndState_;
segway_sim::optSol optSol_;
segway_sim::linearMatrices linearMatrices_;

ros::Time ros_time_init;
ros::Time ros_time_end;

std::vector<double> inputBuffer_u1_;
std::vector<double> inputBuffer_u2_;

double offset_angle_;
double dt_;
double dtDelay_;
double delay_ms_;
double x_start;
double y_start;
double theta_start;
double vMax = {0.0};
double error_max_[nx_] = {};
double linearization_IC_[nx_] = {};


double e_x        = {};
double e_y        = {};
double e_theta    = {};
double e_v        = {};
double e_thetaDot = {};
double e_psi      = {};
double e_psiDot   = {};
double enlarge    = {};
bool lowLevelActive;

int deltaHorizonUpdate;

int flag_state_measurement = 0;
int flag_goalSetAndState_measurement = 0;

MPCValFun *mpcValFun; // Define MPC object as a pointer

bool hardware_ = false;
ros::Time last_button;
int btn_init, btn_run, btn_backup, btn_l1, btn_r1;

//// Functions Definition

void add_uint32_to_vec(std::vector<uint8_t> & vec, uint32_t ui) {
  // add four bits to the end of a std::vector<uint8_t> representing a float
  num32_t n32;
  n32.ui = ui;
  vec.push_back(n32.c[0]);
  vec.push_back(n32.c[1]);
  vec.push_back(n32.c[2]);
  vec.push_back(n32.c[3]);
}


void sendToSegway(void)
{
	double u1;
	double u2;

	if (hardware_)
	{
		ambercortex_ros::cmd segway_msg;
		segway_msg.data.push_back(MSG_CTRL);

		for (int i = 0; i < N; i++)
		{
			if ((mpcValFun->uPred[nu*i] > 15.5) or (mpcValFun->uPred[nu*i] < -15.5) or (mpcValFun->uPred[nu*i+1] > 15.5) or (mpcValFun->uPred[nu*i+1] < -15.5) ){
				u1 = 0.0;
				u2 = 0.0;
			}else{
				u1 = mpcValFun->uPred[nu*i];
				u2 = mpcValFun->uPred[nu*i+1];
			}
			add_float_to_vec(segway_msg.data, u1);
			add_float_to_vec(segway_msg.data, u2);
		}

		// Send Nominal State
		add_float_to_vec(segway_msg.data, stateNominal_.x-x_start);
		add_float_to_vec(segway_msg.data, stateNominal_.y-y_start);
		add_float_to_vec(segway_msg.data, stateNominal_.theta);
		add_float_to_vec(segway_msg.data, stateNominal_.v);
		add_float_to_vec(segway_msg.data, stateNominal_.thetaDot);
		add_float_to_vec(segway_msg.data, stateNominal_.psi);
		add_float_to_vec(segway_msg.data, stateNominal_.psiDot);

		// Send matrices
		for (int i = 0; i < nx*nx; i++){
			add_float_to_vec(segway_msg.data, mpcValFun->AlinearOut[i]);
		}

		for (int i = 0; i < nx*nu; i++){
			add_float_to_vec(segway_msg.data, mpcValFun->BlinearOut[i]);
		}

		for (int i = 0; i < nx; i++){
			add_float_to_vec(segway_msg.data, mpcValFun->ClinearOut[i]);
		}

		add_uint32_to_vec(segway_msg.data, stateCurrent_hw_.time);

		segway_msg.chksum = compute_vec_chksum(segway_msg.data);

		pub_inputAct_.publish(segway_msg);
	}
	else {
		// cout << inputAct_.inputVec[0] << " , " << inputAct_.inputVec[1] << endl;
		pub_inputAct_.publish(inputAct_);				
		pub_stateNominal_.publish(stateNominal_);
	}
}

void stateCallback(const segway_sim::state::ConstPtr msg)
{
	flag_state_measurement = 1;
	stateCurrent_ = *msg;
}

void stateCallback_hw(const ambercortex_ros::state::ConstPtr msg)
{
	flag_state_measurement = 1;
	stateCurrent_hw_ = *msg;
	stateCurrent_.x = stateCurrent_hw_.state[0];
	stateCurrent_.y = stateCurrent_hw_.state[1];
	stateCurrent_.theta = stateCurrent_hw_.state[2];
	stateCurrent_.v = stateCurrent_hw_.state[3];
	stateCurrent_.thetaDot = stateCurrent_hw_.state[4];
	stateCurrent_.psi = stateCurrent_hw_.state[5];
	stateCurrent_.psiDot = stateCurrent_hw_.state[6];
}

void ctrl_infoCallback_hw(const ambercortex_ros::ctrl_info::ConstPtr msg)
{
	ROS_INFO("mpc delay (ms): %f h: %f", msg->data[0], msg->data[7]);
}

void joy_cb(const sensor_msgs::Joy::ConstPtr msg)
{
  ambercortex_ros::cmd msg_button;
  msg_button.data.push_back(MSG_MODE);

  if (msg->buttons[btn_init]) {
    // request INIT
    msg_button.data.push_back(MODE_INIT);
    for (int i = 0; i < 631; i++) {
      msg_button.data.push_back(0);
    }
    msg_button.chksum = compute_vec_chksum(msg_button.data);
    last_button = ros::Time::now();
    pub_inputAct_.publish(msg_button);
  } else if (msg->buttons[btn_run]) {
    // request RUN
    msg_button.data.push_back(MODE_RUN);
    for (int i = 0; i < 631; i++) {
      msg_button.data.push_back(0);
    }
    msg_button.chksum = compute_vec_chksum(msg_button.data);
    last_button = ros::Time::now();
    pub_inputAct_.publish(msg_button);
  } else if (msg->buttons[btn_backup]) {
    // request BACKUP
    msg_button.data.push_back(MODE_BACKUP);
    for (int i = 0; i < 631; i++) {
      msg_button.data.push_back(0);
    }
    msg_button.chksum = compute_vec_chksum(msg_button.data);
    last_button = ros::Time::now();
    pub_inputAct_.publish(msg_button);
  }

}

void goalSetAndStateCallback(const segway_sim::goalSetAndState::ConstPtr msg)
{
	ROS_INFO("New highlevel goal");
	flag_goalSetAndState_measurement = 1;
	goalSetAndState_ = *msg;
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"MPC");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	nhParams_->param<bool>("hardware", hardware_,false);

	sub_goalSetAndState_   = nh_->subscribe<segway_sim::goalSetAndState>("goalSetAndState", 1, goalSetAndStateCallback);

	// Init pubs, subs and srvs
	if (hardware_)
	{
		ROS_INFO("Running on real Segway");
		sub_joy_ = nh_->subscribe<sensor_msgs::Joy>("joy", 1, joy_cb);
		sub_state_ = nh_->subscribe<ambercortex_ros::state>("state", 1, stateCallback_hw);
		sub_ctrl_info_ = nh_->subscribe<ambercortex_ros::ctrl_info>("ctrl_info", 1, ctrl_infoCallback_hw);	
		pub_inputAct_ = nh_->advertise<ambercortex_ros::cmd>("cmd", 1);
	} else {
		ROS_INFO("Running in simulation");
		sub_state_ = nh_->subscribe<segway_sim::state>("state_true", 1, stateCallback);
		pub_inputAct_ = nh_->advertise<segway_sim::input>("mpc_input", 1);
	}
	
	pub_stateNominal_ = nh_->advertise<segway_sim::state>("state_nominal", 1);
	pub_optSol_       = nh_->advertise<segway_sim::optSol>("optimal_sol", 1);
	pub_linearMat     = nh_->advertise<segway_sim::linearMatrices>("linear_matrices", 1);

	// Retrieve ROS parameters
	nhParams_->param<int>("deltaHorizonUpdate", deltaHorizonUpdate, 0);
	cout << "=================== deltaHorizonUpdate: " << deltaHorizonUpdate << endl;

	nhParams_->param<double>("dt", dt_,0.05);
	nhParams_->param<double>("offset_angle", offset_angle_,0.);
	nhParams_->param<double>("mpc_input_delay", delay_ms_,0.);
	nhParams_->param<double>("x_start", x_start,0.);
	nhParams_->param<double>("y_start", y_start,0.);
	nhParams_->param<double>("theta_start", theta_start,0.);
	dtDelay_ = delay_ms_/1000.0;
	optSol_.delay_ms = delay_ms_;
	
	nhParams_->param<double>("e_x"       , e_x,0.);
	nhParams_->param<double>("e_y"       , e_y,0.);
	nhParams_->param<double>("e_theta"   , e_theta,0.);
	nhParams_->param<double>("e_v"       , e_v,0.);
	nhParams_->param<double>("e_thetaDot", e_thetaDot,0.);
	nhParams_->param<double>("e_psi"     , e_psi,0.);
	nhParams_->param<double>("e_psiDot"  , e_psiDot,0.);
	nhParams_->param<double>("enlarge"   , enlarge,0.);
	nhParams_->param<bool>("/segway_sim/low_level/low_level_active"   , lowLevelActive,false);
	cout << "================= lowLevelActive: " << lowLevelActive << endl;

	ros::param::get("~_btn_init", btn_init);
	ros::param::get("~_btn_run", btn_run);
	ros::param::get("~_btn_backup", btn_backup);	

	error_max_[0] = e_x;
	error_max_[1] = e_y;
	error_max_[2] = e_theta;
	error_max_[3] = e_v;
	error_max_[4] = e_thetaDot;
	error_max_[5] = e_psi;
	error_max_[6] = e_psiDot;	

	double xCurr[nx]{}; // current measured state
	double uCurr[nu]{}; // current applied input

	double goalSetAndStateVector[11] = {0.0};
	double goalSetAndStateVectorOld[11] = {0.0};
	// States are:        x,   y, theta,   v, thetaDot,            psi, psiDot
	double xeq[nx]  = { 0.0, 0.0,   1.57, 0.0,      0.0,  offset_angle_,    0.0};  // initial condition to

	// Initial condition and goal in relative reference frame
	double x_IC[nx] = {  x_start, y_start,   theta_start, 0.0,      0.0,            0.0,    0.0};  // initial condition to
	double x_g[nx]  = {  x_start, y_start-1,   theta_start, 0.0,      0.0,            0.0,    0.0};  // initial condition to

	for (int i = 0; i < nx_; i++)
		linearization_IC_[i]=x_IC[i];

	double highLevTime = -1;

	goalSetAndState_.x = x_IC[0];
	goalSetAndState_.y = x_IC[1];

	goalSetAndState_.xmin = x_IC[0] - 0.5;
	goalSetAndState_.xmax = x_IC[0] + 0.5;
	goalSetAndState_.ymin = x_IC[1] - 0.5;
	goalSetAndState_.ymax = x_IC[1] + 0.5;

	goalSetAndState_.highLevTime = goalSetAndState_.highLevTime;
	
	goalSetAndState_.term_xmin = x_IC[0] - 0.5;
	goalSetAndState_.term_xmax = x_IC[0] + 0.5;
	goalSetAndState_.term_ymin = x_IC[1] - 0.5;
	goalSetAndState_.term_ymax = x_IC[1] + 0.5;

	inputAct_.inputVec[0] = 0.0;
	inputAct_.inputVec[1] = 0.0;

	goalSetAndStateVector[0]  = goalSetAndState_.x;
	goalSetAndStateVector[1]  = goalSetAndState_.y;
	goalSetAndStateVector[2]  = goalSetAndState_.xmin;
	goalSetAndStateVector[3]  = goalSetAndState_.xmax;
	goalSetAndStateVector[4]  = goalSetAndState_.ymin;
	goalSetAndStateVector[5]  = goalSetAndState_.ymax;
	goalSetAndStateVector[6]  = goalSetAndState_.highLevTime;
	goalSetAndStateVector[7]  = goalSetAndState_.term_xmin;
	goalSetAndStateVector[8]  = goalSetAndState_.term_xmax;
	goalSetAndStateVector[9]  = goalSetAndState_.term_ymin;
	goalSetAndStateVector[10] = goalSetAndState_.term_ymax;

	ROS_INFO("========== START Initializing MPC Object");
	string matrix_prefix_path = ros::package::getPath("segway_sim");
	mpcValFun = new MPCValFun(nx_, nu_, N, dt_, dtDelay_, printLevel, x_eq_, error_max_, enlarge, lowLevelActive, linearization_IC_, matrix_prefix_path, linearizedDynamics);
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
	ROS_INFO("========== DONE Initializing MPC Object");

	mpcValFun->solveQP();  // Solve QP to check the everything works

	mpcValFun->linearize();			
	mpcValFun->buildCost();
	mpcValFun->buildConstrMatrix();
	mpcValFun->buildConstrVector();
	mpcValFun->solveQP();  // Solve QP to check the everything works

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
		inputAct_.inputVec[i] = mpcValFun->uPred[i];

	if (delay_ms_>0){
		inputBuffer_u1_.resize( int(dt_/(delay_ms_/1000)) );
		inputBuffer_u2_.resize( int(dt_/(delay_ms_/1000)) );
		cout << "===== MPC delay: " << delay_ms_ << ". Initialize inputBuffer_u1_: ";
		for (int i = 0; i < int(dt_/(delay_ms_/1000)); ++i){
			cout << inputBuffer_u1_[i] << ", ";
		}
		cout << endl;
	}

	stateNominal_.x        = 0.0; // This is xPred[0,0]
	stateNominal_.y        = 0.0;
	stateNominal_.theta    = 0.0;
	stateNominal_.v        = 0.0;
	stateNominal_.thetaDot = 0.0;
	stateNominal_.psi      = 0.0 + xeq[5];
	stateNominal_.psiDot   = 0.0;	
	stateNominal_.time = stateCurrent_.time;

	int horizonCounter = 0;

	ros::Rate rate(1/dt_);
	// Take it for a spin

	ros::Time t0 = ros::Time::now();
	while(ros::ok())
	{
		ros::Time tnow = ros::Time::now();
		if ((ros::Duration(tnow-t0) > ros::Duration(time_after_delay_)) and (delay_ > 1))
		{
			
			// if (delay_ != 0)
			// 	ROS_INFO("before: %i", ros::Time::now());
			// ros::Duration(.25).sleep();
			// if (delay_ != 0)
			// 	ROS_INFO("after: %i",ros::Time::now());
			// delay_ = 0;

			// goalSetAndState_.x = 1.5;
			// goalSetAndState_.y = 4.0;
			// goalSetAndState_.xmin = 0;
			// goalSetAndState_.xmax = 2;
			// goalSetAndState_.ymin = 4;
			// goalSetAndState_.ymax = 5;
			// goalSetAndState_.highLevTime = 1;
			// goalSetAndState_.term_xmin = 1;
			// goalSetAndState_.term_xmax = 2;
			// goalSetAndState_.term_ymin = 4;
			// goalSetAndState_.term_ymax = 5;
		}

		//Get latest input
		ros::spinOnce();

		if ((flag_state_measurement == 1)){
			// apply input
			if (delay_ms_ > -0.5){
				sendToSegway();
			}
			
			// store ros time
			ros_time_init = ros::Time::now();
			
			// read current input and state in local coordinates
			uCurr[0] = inputAct_.inputVec[0];
			uCurr[1] = inputAct_.inputVec[1];
			xCurr[0] = stateCurrent_.x + x_start*hardware_;
			xCurr[1] = stateCurrent_.y + y_start*hardware_;
			xCurr[2] = stateCurrent_.theta;
			
			// cout << xCurr[2] << endl;
			
			// xCurr[2] = fmod(xCurr[2]+M_PI,2.0 * M_PI);
			
			// if (xCurr[2] < 0)
	  //   		xCurr[2] += 2.0 * M_PI;
    		
   //  		xCurr[2] += - M_PI;
			
			// cout << xCurr[2] << endl;

			xCurr[3] = stateCurrent_.v;
			xCurr[4] = stateCurrent_.thetaDot;
			xCurr[5] = stateCurrent_.psi - offset_angle_;
			xCurr[6] = stateCurrent_.psiDot;

			goalSetAndStateVector[0] = goalSetAndState_.x;
			goalSetAndStateVector[1] = goalSetAndState_.y;

			goalSetAndStateVector[2] = goalSetAndState_.xmin;
			goalSetAndStateVector[3] = goalSetAndState_.xmax;
			goalSetAndStateVector[4] = goalSetAndState_.ymin;
			goalSetAndStateVector[5] = goalSetAndState_.ymax;
		
			goalSetAndStateVector[6] = goalSetAndState_.highLevTime;

			goalSetAndStateVector[7] = goalSetAndState_.term_xmin;
			goalSetAndStateVector[8] = goalSetAndState_.term_xmax;
			goalSetAndStateVector[9] = goalSetAndState_.term_ymin;
			goalSetAndStateVector[10] = goalSetAndState_.term_ymax;

			if (printLevel >= 0.5) cout << "x: " << xCurr[0] << ", y: " << xCurr[1] << ", theta: " << xCurr[2]<< ", v: " << xCurr[3] << ", thetaDot: " << xCurr[4]  << ", psi: " << xCurr[5] + offset_angle_ << ", psiDot: " << xCurr[6] << endl;
	
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

			if (printLevel >= 2) cout << "x_next: " << xCurr[0] << ", v_next: " << xCurr[1] << ", psi_next: " << xCurr[2] << ", psiDot_next: " << xCurr[3] << endl;
			
			// solve QP
			mpcValFun->solveQP();
			if ((mpcValFun->solverFlag == 1) or (mpcValFun->solverFlag == 2)){
				optSol_.contPlan = 0;
				for (int i = 0; i < 11; i++){
					goalSetAndStateVectorOld[i] = goalSetAndStateVector[i];
				}
			} else {
				cout << "==================== Solving Contingency plan ======================= " << endl;
				mpcValFun->updateGoalSetAndState(goalSetAndStateVectorOld);
				mpcValFun->updateHorizon();
				mpcValFun->buildCost();
				mpcValFun->buildConstrMatrix();
				mpcValFun->buildConstrVector();
				mpcValFun->solveQP();
				if (mpcValFun->solverFlag==1){
					cout << "==================== Contingency Plan: OPTIMAL! ========= " << endl;					
				}else{
					cout << "==================== Flag Contingency Plan: "<< mpcValFun->solverFlag << endl;					
				}
				mpcValFun->updateGoalSetAndState(goalSetAndStateVector);	
				optSol_.contPlan = 1;

			}

			if ( horizonCounter == deltaHorizonUpdate){
				mpcValFun->updateHorizon();
				horizonCounter = 0;
			} else {
				horizonCounter += 1;
			}
			inputAct_.inputVec[0] = mpcValFun->uPred[0];
			inputAct_.inputVec[1] = mpcValFun->uPred[1];

			stateNominal_.x        = mpcValFun->xPred[0]; // This is xPred[0,0]
			stateNominal_.y        = mpcValFun->xPred[1];
			stateNominal_.theta    = mpcValFun->xPred[2];
			stateNominal_.v        = mpcValFun->xPred[3];
			stateNominal_.thetaDot = mpcValFun->xPred[4];
			stateNominal_.psi      = mpcValFun->xPred[5] + xeq[5];
			stateNominal_.psiDot   = mpcValFun->xPred[6];	
			stateNominal_.time = stateCurrent_.time;	


			if (delay_ms_ > 0){
				inputBuffer_u1_[0] = inputBuffer_u1_[int(dt_/(delay_ms_/1000))-1];
				inputBuffer_u2_[0] = inputBuffer_u2_[int(dt_/(delay_ms_/1000))-1];
				for (int i = 1; i < int( dt_/(delay_ms_/1000) ); ++i){
					inputBuffer_u1_[i] = inputAct_.inputVec[0];
					inputBuffer_u2_[i] = inputAct_.inputVec[1];
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
			optSol_.time = stateCurrent_.time;	
			pub_optSol_.publish(optSol_);
			
			for (int i = 0; i < nx*nx; i++){
				linearMatrices_.Alinear[i] = mpcValFun->AlinearOut[i];
			}
			for (int i = 0; i < nx*nu; i++){
				linearMatrices_.Blinear[i] = mpcValFun->BlinearOut[i];
			}
			for (int i = 0; i < nx; i++){
				linearMatrices_.Clinear[i] = mpcValFun->ClinearOut[i];
			}
			pub_linearMat.publish(linearMatrices_);			

			if (delay_ms_ < -0.5){
				sendToSegway();
			}

		}
		//Wait for tick
		rate.sleep();
	}
}