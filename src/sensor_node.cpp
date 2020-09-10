#include "segway_sim/sensor_node.hpp"


using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::ServiceServer srv_ui_;
ros::Subscriber sub_state_;
ros::Publisher pub_sensor_;

double enc_v_variance_,psi_variance_,psi_dot_variance_,theta_dot_variance_;
segway_sim::state state_current_;
double dt_,v_last_,theta_last_;
double encL_pos_,encR_pos_;
double Rw_ = 0.195;
double L_ = 0.5;

void stateCallback(const segway_sim::state::ConstPtr msg)
{
	//compute sensor values (no noise)
	state_current_ = *msg;
	double encoderL_vel = (state_current_.v - state_current_.thetaDot*L_)/Rw_;
	double encoderR_vel = (state_current_.v + state_current_.thetaDot*L_)/Rw_;

	double thetaDot = state_current_.thetaDot;

	double psi = state_current_.psi;
	double psiDot = state_current_.psiDot;

	double steering = 0.;
	double battV = 60.;

	// add in random noise
	encoderL_vel += ( ((double)rand()/(double)RAND_MAX) *2-1)*enc_v_variance_;
	encoderR_vel += ( ((double)rand()/(double)RAND_MAX) *2-1)*enc_v_variance_;
	psi += ( ((double)rand()/(double)RAND_MAX) *2-1)*psi_variance_;
	psiDot += ( ((double)rand()/(double)RAND_MAX) *2-1)*psi_dot_variance_;
	thetaDot += ( ((double)rand()/(double)RAND_MAX) *2-1)*theta_dot_variance_;

	//update encoder positions;
	encL_pos_ += encoderL_vel*dt_;
	encR_pos_ += encoderR_vel*dt_;

	segway_sim::sensor sensor_msg;

	sensor_msg.time = (uint32_t) (state_current_.time*1E6);
	sensor_msg.data.push_back(encL_pos_);
	sensor_msg.data.push_back(encR_pos_);
	sensor_msg.data.push_back(encoderL_vel);
	sensor_msg.data.push_back(encoderR_vel);
	sensor_msg.data.push_back(thetaDot);
	sensor_msg.data.push_back(psi);
	sensor_msg.data.push_back(psiDot);
	sensor_msg.data.push_back(steering);
	sensor_msg.data.push_back(battV);

	pub_sensor_.publish(sensor_msg);
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"sensor");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_state_ = nh_->subscribe<segway_sim::state>("state_true", 1,stateCallback);
	pub_sensor_ = nh_->advertise<segway_sim::sensor>("sensor", 1);

	// Retreive params
	nhParams_->param<double>("enc_v_variance",enc_v_variance_,0.001);
	nhParams_->param<double>("psi_variance",psi_variance_,0.001);
	nhParams_->param<double>("psi_dot_variance",psi_dot_variance_,0.001);
	nhParams_->param<double>("theta_dot_variance",theta_dot_variance_,0.001);

	// Display node info
	ROS_INFO("Sensor node successfuly started!");

	ros::spin();

	return 0;
}
