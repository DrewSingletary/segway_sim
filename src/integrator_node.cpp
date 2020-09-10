#include "segway_sim/integrator_node.hpp"

using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::ServiceServer srv_ui_;
ros::Subscriber sub_input_;
ros::Publisher pub_state_;

double t_;
double dt_;
double umax_;
uint32_t iter_;
STATUS status_;
stepper_t odeStepper_;
state_t odeState_;
segway_sim::state stateCurrent_;
segway_sim::input inputCurrent_;
state_t initialConditions_(STATE_LENGTH,0.0);
double input_delay_ms_ = 0.;
std::vector<segway_sim::input> inputBuffer_;

void resetOdeState(void)
{
	odeState_ = initialConditions_;
	inputCurrent_.inputVec[0] = 0.0;
	inputCurrent_.inputVec[1] = 0.0;
}

void dynamicsCL(const state_t &x, state_t &xDot, const double t)
{
	xDot = x;
	dynamics(t,x.data(),inputCurrent_.inputVec.data(),xDot.data());
}

void inputCallback(const segway_sim::input::ConstPtr msg)
{
	if(isnan(msg->inputVec[0]) || isnan(msg->inputVec[1]))
	{
		ROS_WARN_THROTTLE(1,"Input is NaN");
		return;
	}
	inputBuffer_.push_back(*msg);
	inputBuffer_.erase(inputBuffer_.begin());
	// inputCurrent_ = *msg; // inputBuffer_[inputBuffer_.size()-1-(int)(input_delay_ms_/1000/dt_)];
	inputCurrent_ = inputBuffer_[inputBuffer_.size()-1-(int)(input_delay_ms_/1000/dt_)];
	saturateInPlace(inputCurrent_.inputVec.data(),-umax_,umax_,INPUT_LENGTH);
}

void updateStateCurrent(void)
{
	stateCurrent_.status = static_cast<uint8_t>(status_);
	stateCurrent_.time = t_;
	stateCurrent_.x = odeState_[0];
	stateCurrent_.y = odeState_[1];
	stateCurrent_.theta = odeState_[2];
	stateCurrent_.v = odeState_[3];
	stateCurrent_.thetaDot = odeState_[4];
	stateCurrent_.psi = odeState_[5];
	stateCurrent_.psiDot = odeState_[6];
	std::copy(odeState_.begin(),odeState_.end(),stateCurrent_.stateVec.begin());
}

void sendStateCurrent(void)
{
	stateCurrent_.header.seq = iter_;
	stateCurrent_.header.stamp = ros::Time::now();
	stateCurrent_.header.frame_id = std::string("inputSeq=") + std::to_string(inputCurrent_.header.seq);

	pub_state_.publish(stateCurrent_);
}

void sendTransformCurrent(void)
{
	static tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.seq = iter_;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "world";
	odom_trans.child_frame_id = "cyberpod/base_link";

	odom_trans.transform.translation.x = stateCurrent_.x;
	odom_trans.transform.translation.y = stateCurrent_.y;
	odom_trans.transform.translation.z = 0.195;

	Quaterniond cyberpod_q;
	Vector3d cyberpod_eul(0.0,stateCurrent_.psi,stateCurrent_.theta);
	eul2quatZYX(cyberpod_eul,cyberpod_q);

	geometry_msgs::Quaternion odom_quat;
	odom_quat.w = cyberpod_q.w();
	odom_quat.x = cyberpod_q.x();
	odom_quat.y = cyberpod_q.y();
	odom_quat.z = cyberpod_q.z();
	odom_trans.transform.rotation = odom_quat;

	odom_broadcaster.sendTransform(odom_trans);
}


bool interpretRequestCmd(const uint8_t &cmdRaw,
                               CMD &cmd)
{
	if(cmdRaw<5)
	{
		cmd = static_cast<CMD>(cmdRaw);
		return true;
	}
	else
		return false;
}

bool uiCallback(segway_sim::ui::Request &req,
                segway_sim::ui::Response &res)
{
	CMD cmd;
	if(!interpretRequestCmd(req.cmd,cmd))
	{
		ROS_WARN("Unkown request command: %i",req.cmd);
		res.result = false;
		return false;
	}

	switch(cmd)
	{
		case CMD::STOP:
		{
			t_ = 0.0;
			iter_ = 0;
			resetOdeState();
			status_ = STATUS::STOPPED;
			ROS_INFO("Stopping simulation");
			break;
		}
		case CMD::START:
		{
			status_ = STATUS::RUNNING;
			ROS_INFO("Starting simulation");
			break;
		}
		case CMD::PAUSE:
		{
			status_ = STATUS::STOPPED;
			ROS_INFO("Pausing simulation");
			break;
		}
		case CMD::REPOSITION:
		{
			if(req.data.size()==STATE_LENGTH)
			{
				ROS_INFO("State reseted to custom value");
				odeState_ = state_t(req.data.begin(),req.data.end());
			}
			else
			{
				ROS_INFO("State reseted to Initial Conditions");
				resetOdeState();
			}
			break;
		}
		case CMD::RESET:
		{
			t_ = 0.0;
			iter_ = 0;
			resetOdeState();
			ROS_INFO("Resetting simulation");
			break;
		}
	}
	updateStateCurrent();
	sendStateCurrent();
	sendTransformCurrent();

	res.result = true;
	return true;
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"integrator");

	// Create useless broadcaster so that transforms get published
	tf::TransformBroadcaster useless_broadcaster;

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_input_ = nh_->subscribe<segway_sim::input>("input", 1,inputCallback);
	pub_state_ = nh_->advertise<segway_sim::state>("state_true", 1);
	srv_ui_ = nh_->advertiseService("integrator/ui", uiCallback);

	// Retreive params
	nhParams_->param<double>("dt",dt_,0.001);
	nhParams_->param<double>("umax",umax_,20.);
	nhParams_->param<double>("input_delay_ms",input_delay_ms_,1.);
	nhParams_->param<state_t>("IC",initialConditions_,initialConditions_);

	if(dt_<=0.0)
	{
		dt_ = 0.001;
		ROS_WARN("dt must be strictly positive. Will be set to %f",dt_);
	}
	if(umax_<=0.0)
	{
		umax_ = 40.;
		ROS_WARN("umax must be strictly positive. Will be set to %f",umax_);
	}

	if(input_delay_ms_<=1.0)
	{
		input_delay_ms_ = 1.0;
		ROS_WARN("input_delay_ms must be greater or equal to 1.0. Will be set to %f",input_delay_ms_);
	}
	ROS_WARN("input_delay_ms is equal to %f",input_delay_ms_);

	if(initialConditions_.size()!=STATE_LENGTH)
	{
		initialConditions_ = state_t(STATE_LENGTH,0.0);
		ROS_WARN("Initial conditions must be of length %u",STATE_LENGTH);
	}
	// Initialize variables
	t_ = 0.0;
	iter_ = 0;
	status_ = STATUS::STOPPED;
	resetOdeState();
	inputBuffer_.resize((int)(2*input_delay_ms_/1000/dt_));
	for (int i = 0; i < (int)(2*input_delay_ms_/1000/dt_); i++) {
		inputBuffer_[i].inputVec[0] = 0.;
		inputBuffer_[i].inputVec[1] = 0.;
	}
	ros::Rate rate(1/dt_);

	// Display node info
	ROS_INFO("Integrator node successfuly started with:");
	ROS_INFO("___dt=%.4fs",dt_);
	ROS_INFO("___input_delay_ms=%.4fs",input_delay_ms_);
	ROS_INFO("___IC=");
	for(uint8_t i = 0; i<STATE_LENGTH; i++)
	{
		ROS_INFO("      %.3f",initialConditions_[i]);
	}

	// Take it for a spin
	while(ros::ok())
	{

		//Get latest input
		ros::spinOnce();

		//Integrate dynamics
		if(status_==STATUS::RUNNING)
		{
			odeStepper_.do_step(dynamicsCL,odeState_,t_,dt_);
			updateStateCurrent();
			iter_++;
			t_+=dt_;

			//Publish state
			sendStateCurrent();

			//Publish tranform
			sendTransformCurrent();
		}

		//Wait for tick
		rate.sleep();
	}

	return 0;
}
