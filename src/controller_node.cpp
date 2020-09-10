#include "segway_sim/controller_node.hpp" 

using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::Subscriber sub_state_;
ros::Subscriber sub_cmd_;
ros::Publisher pub_input_;

segway_sim::input input_;
segway_sim::state stateCurrent_;
segway_sim::cmd cmdCurrent_;

// Global variables
uint32_t iter_;
VectorXd gainsVec_(STATE_LENGTH);
VectorXd stateCurrentVec_(STATE_LENGTH);
double offset_angle_;

// Controller gains
std::vector<double> gains_(STATE_LENGTH,0.0);


void computeControlAction(void)
{
	// Prepare Header
	iter_++;
	input_.header.seq = iter_;
	input_.header.stamp = ros::Time::now();
	input_.header.frame_id = std::string("stateSeq=") + std::to_string(stateCurrent_.header.seq) + std::string(", cmdSeq=") + std::to_string(cmdCurrent_.header.seq);
	input_.status = static_cast<uint8_t>(STATUS::RUNNING);

	VectorXd stateDes(STATE_LENGTH);
	stateDes(5)=offset_angle_;
	stateDes(3)=cmdCurrent_.cmd[0];

	const double u = gainsVec_.dot(stateCurrentVec_ - stateDes);

	input_.inputVec[0] = u;
	input_.inputVec[1] = u;
}

void controlCallback(const segway_sim::state::ConstPtr msg)
{
	stateCurrent_ = *msg;
	for(uint32_t i=0; i<STATE_LENGTH; i++)
		stateCurrentVec_(i) = stateCurrent_.stateVec[i];

	computeControlAction();

	pub_input_.publish(input_);
}

void cmdCallback(const segway_sim::cmd::ConstPtr msg)
{
	cmdCurrent_ = *msg;
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"controller");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Initialize variables
	iter_ = 0;
	cmdCurrent_.cmd.resize(2,0.0);

	// Init pubs, subs and srvs
	sub_state_ = nh_->subscribe<segway_sim::state>("state", 1, controlCallback);
	sub_cmd_ = nh_->subscribe<segway_sim::cmd>("cmd", 1, cmdCallback);
	pub_input_ = nh_->advertise<segway_sim::input>("inputDes", 1);

	// Retreive params
	nhParams_->param<double>("offset_angle",offset_angle_,0.);
	nhParams_->param<std::vector<double>>("gains",gains_,gains_);
	if(gains_.size()!=STATE_LENGTH)
	{
		gains_ = std::vector<double>(STATE_LENGTH,0.0);
		ROS_WARN("gains must be of length %u",STATE_LENGTH);
	}
	for(uint32_t i=0; i<STATE_LENGTH; i++)
		gainsVec_(i) = gains_[i];

	// Display node infor
	ROS_INFO("Controller node successfuly started with:");
	ROS_INFO("___offset_angle=%f",offset_angle_);
	ROS_INFO("___gains=");
	for(uint8_t i = 0; i<STATE_LENGTH; i++)
	{
		ROS_INFO("      %.3f",gains_[i]);
	}

	// Take it for a spin
	while(ros::ok())
		ros::spinOnce();

	return 0;
}