#include "dvb_control/simple_pid.h"

Simple_Pid::Simple_Pid() :
	target_(0),
	output_(0),
	kp_(0),
	ki_(0),
	kd_(0),
	integral_(0),
	error_(0),
	derivative_(0),
	previous_error_(0)
{
	//Initialize pid variables
	prev_time_ = ros::Time::now();
}

int Simple_Pid::calcul_pid(int value)
{
	//Initialize PID stuffs
	float P_val = 0;
	float I_val = 0;
	float D_val = 0;
	
	previous_error_ = 0;
	integral_ = 0;
	error_ = 0;
	derivative_ = 0;

	//Starting PID calculation
	error_ = target_ - value;

	P_val = kp_ * error_;

	//Update dt
	dt_ = ros::Time::now() - prev_time_;
	prev_time_ = ros::Time::now();

	integral_ += error_ * (float)dt_.toSec();
	I_val = ki_ * integral_;

	derivative_ = (error_ - previous_error_) / dt_.toSec();
	previous_error_ = error_;

	output_ = P_val + I_val + D_val;

	return output_;
}