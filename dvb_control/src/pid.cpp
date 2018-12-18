#include "dvb_control/pid.h"

Pid::Pid() :
	Dvb_Spinner(),
	target_(0),
    output_(0),
	kp_(0.0),
	ki_(0.0),
	kd_(0.0),
	integral_(0.0),
    error_(0.0),
    derivative_(0.0),
    previous_error_(0.0)
{   
	topic_sub_setpoint_name_ = nh_.getNamespace();
	topic_pub_output_name_ = nh_.getNamespace();

	topic_sub_setpoint_name_.append("/pid/setpoint");
	topic_pub_output_name_.append("/pid/output");

	prev_time_ = ros::Time::now();

	ros::param::get("~topic_sub_sensor_name", topic_sub_sensor_name_);

	/*
		Get all params for ros server
	*/
	//PID params
	if (
			nh_.hasParam("/control/pid/Kp") ||
			nh_.hasParam("/control/pid//Ki") ||
			nh_.hasParam("/control/pid//Kd")
	   )
	{
		nh_.getParam("/control/pid/Kp", kp_);
		nh_.getParam("/control/pid/Ki", ki_);
		nh_.getParam("/control/pid//Kd", kd_);

		ROS_INFO("PID : DEBUG_MODE(%d), FREQUENCY(%f), TOPIC_SENSOR(%s), KP(%g), KI(%g), KD(%g)", debug_mode_, freq_, topic_sub_sensor_name_.c_str(), kp_, ki_, kd_);

		spinner_startable_ = true;
		spinner_enable_ = true;
	}
	else
	{
		ROS_INFO("Please check if PID parameters are set in the ROS Parameter Server !\n");
	}	

	//Subcriber
	sub_setpoint_ = nh_.subscribe(topic_sub_setpoint_name_, 1, &Pid::onSetPointEvent, this );
	sub_sensor_ = nh_.subscribe(topic_sub_sensor_name_.c_str(), 1, &Pid::onSensorEvent, this);

	//Publisher
	pub_ = nh_.advertise<std_msgs::Int16>(topic_pub_output_name_.c_str(), 1);
}

Pid::~Pid()
{
	
}

void Pid::spinOnce()
{
	if(spinner_enable_ && spinner_startable_){
        updateOutput();
	}
	else{
        ROS_INFO_COND(debug_mode_,"PID disabled");
	}
}

void Pid::onSetPointEvent(const std_msgs::Int16::ConstPtr& setpoint_msg)
{
	target_ = setpoint_msg->data;

	ROS_INFO_COND(debug_mode_, "%s : %d", topic_sub_setpoint_name_.c_str(), target_);
}

void Pid::onSensorEvent(const std_msgs::Int16::ConstPtr& sensor_msg)
{
	sensor_val_ = sensor_msg->data;

	ROS_INFO_COND(debug_mode_, "%s : %d", topic_sub_sensor_name_.c_str(), sensor_val_);
}

void Pid::updateOutput()
{
	calcul_pid();

	std_msgs::Int16 output;
	output.data = output_;
	pub_.publish(output);

	ROS_INFO_COND(debug_mode_, "%s : %d", topic_pub_output_name_.c_str(), output_);
}

void Pid::calcul_pid()
{
	//Initialize PID stuffs
	float_t P_val = 0;
	float_t I_val = 0;
	float_t D_val = 0;
	
	previous_error_ = 0;
	integral_ = 0;
	error_ = 0;
	derivative_ = 0;

	//Starting PID calculation
	error_ = target_ - sensor_val_;

	P_val = kp_ * error_;

	//Update dt
	dt_ = ros::Time::now() - prev_time_;
	prev_time_ = ros::Time::now();

	integral_ += error_ * (float_t)dt_.toSec();
	I_val = ki_ * integral_;

	derivative_ = (error_ - previous_error_) / dt_.toSec();
	previous_error_ = error_;

	output_ = P_val + I_val + D_val;
}

int Pid::getOutput()
{
	return output_;
}