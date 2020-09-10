#include "ros/ros.h"
#include "segway_sim/state.h"
#include "segway_sim/sensor.h"
#include "segway_sim/input.h"
#include "segway_sim/ekf.h"

using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::ServiceServer srv_ui_;
ros::Subscriber sub_sensor_;
ros::Subscriber sub_input_;
ros::Publisher pub_state_;
EKF *ekf_ = nullptr;
segway_sim::input inputCurrent_;
segway_sim::sensor sensorCurrent_;

double Rw_ = 0.195;
double L_ = 0.5;
double dt_ = 0;

void inputCallback(const segway_sim::input::ConstPtr msg)
{
	inputCurrent_ = *msg;
}

void sensorCallback(const segway_sim::sensor::ConstPtr msg)
{
	dt_ = (msg->time-sensorCurrent_.time)*1E-6;
	sensorCurrent_ = *msg;
	double v = Rw_*(msg->data[2]+msg->data[3])/2;
	double thetaDot = msg->data[4];
	double psi = msg->data[5];
	double psiDot = msg->data[6];
	if (ekf_->initialized == 0) {
		std::vector<double> state_ekf_ = {0,0,0,v,thetaDot,psi,psiDot};
		std::vector<double> P_ekf_ = {0.,0.,0.,0.01,0.01,0.01,0.01};
		ekf_->init(state_ekf_,P_ekf_);
	}
	else if (ekf_->initialized == 1){
		ekf_->update(std::vector<double> {inputCurrent_.inputVec[0],inputCurrent_.inputVec[1]},
								std::vector<double> {v,thetaDot,psi,psiDot},
								dt_);
	}
	segway_sim::state state_msg;
	state_msg.time = sensorCurrent_.time;
	state_msg.x = ekf_->x_t_[0];
	state_msg.y = ekf_->x_t_[1];
	state_msg.theta = ekf_->x_t_[2];
	state_msg.v = ekf_->x_t_[3];
	state_msg.thetaDot = ekf_->x_t_[4];
	state_msg.psi = ekf_->x_t_[5];
	state_msg.psiDot = ekf_->x_t_[6];

	for (int i = 0 ; i < 7; i ++) {
		state_msg.stateVec[i] = ekf_->x_t_[i];
	}

	pub_state_.publish(state_msg);
}

int main (int argc, char *argv[])
{
	ekf_ = new EKF(std::vector<double> {.01,.01,.01,.01,.01,.01,.01},
								 std::vector<double> {.001,.001,.001,.001});
	// Init ros
	ros::init(argc,argv,"ekf_node");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_input_ = nh_->subscribe<segway_sim::input>("input", 1,inputCallback);
	sub_sensor_ = nh_->subscribe<segway_sim::sensor>("sensor", 1,sensorCallback);
	pub_state_ = nh_->advertise<segway_sim::state>("state", 1);

	// Retreive params
	// nhParams_->param<double>("enc_v_variance",enc_v_variance_,0.001);

	// Display node info
	ROS_INFO("EKF node successfuly started!");

	ros::spin();

	return 0;
}
