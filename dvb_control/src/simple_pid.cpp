#include "simple_pid.h"

Simple_Pid::Simple_Pid(std::string topic_motor_name, std::string topic_encoder_name, bool debug_mode) :
	node_name_(""),
	pid_startable_(false),
	pid_enable_(false),
	target_(0),
	kp_(0),
	ki_(0),
	kd_(0),
	motor_out_min_(0),
	motor_out_max_(0),
	freq_(0.0)
{
	//Node name
	node_name_ = ros::this_node::getName();
	ROS_INFO("Starting ROS Node %s", node_name_.c_str());

	//Topic name for motor and encoder
	topic_motor_name_ = topic_motor_name;
	topic_encoder_name_ = topic_encoder_name;

	//Debug mode
	debug_mode_ = debug_mode;

	//Initialize pid variables
	prev_time_ = ros::Time::now();

	integral_ = 0;
	error_ = 0;
	derivative_ = 0;
	previous_error_ = 0;

	/*
		Get all params for ros server
	*/
	//PID params
	if (
			nh_.hasParam("/Kp") ||
			nh_.hasParam("/Ki") ||
			nh_.hasParam("/Kd") ||
			nh_.hasParam("/motor_out_min") ||
			nh_.hasParam("/motor_out_max") ||
			nh_.hasParam("/frequency")
	   )
	{
		nh_.param<float_t>("/Kp", kp_);
		nh_.param<float_t>("/Ki", ki_);
		nh_.param<float_t>("/Kd", kd_);

		nh_.param<float_t>("/motor_out_min", motor_out_min_);
		nh_.param<float_t>("/motor_out_max", motor_out_max_);
		nh_.param<double_t>("/frequency", freq_, 10);

		pid_startable_ = true;
	}
	else{
		ROS_INFO("Please check if PID parameters are set in the ROS Parameter Server !\n");
	}

	/*
		Subscribers
	*/
	sub_encoder_ = nh_.subscribe(topic_encoder_name_.c_str(), 10, &Simple_Pid::spinOnce, this);
	/*
		Publishers
	*/
	pub_motor_ = nh_.advertise<std_msgs::Float32>(topic_motor_name_.c_str(), 10);	
}



void Simple_Pid::calcule_pid(float_t encoder_val){
	//Initialize PID stuffs
	float_t P_val = 0;
	float_t I_val = 0;
	float_t D_val = 0;
previous_error_ = 0;
	integral_ = 0;
	error_ = 0;
	derivative_ = 0;

	//Starting PID calculation
	error_ = target_ - encoder_val;

	P_val = kp_ * error_;

	//Update dt
	dt_ = ros::Time::now() - prev_time_;
	prev_time_ = ros::Time::now();

	integral_ += error_ * (float_t)dt_.toSec();
	I_val = ki_ * integral_;

	derivative_ = (error_ - previous_error_) / dt_.toSec();
	previous_error_ = error_;

	output_ = P_val + I_val + D_val;

	//Check limits
	if ( output_ > motor_out_max_ ) output_ = motor_out_max_;
	else if ( output_ < motor_out_min_ ) output_ = motor_out_min_;

	ROS_INFO_COND(debug_mode_, "Setting motor output to %f", output_);
}

void Simple_Pid::spinOnce(const std_msgs::Float32::ConstPtr& encoder_msg){

	ROS_INFO_COND(debug_mode_, "%s : %f", topic_encoder_name_.c_str(), encoder_msg->data);

	if(pid_enable_ && pid_startable_){
		Simple_Pid::calcule_pid(encoder_msg->data);

		ros::Duration(1).sleep();
	}
	else{
		ROS_INFO_COND(debug_mode_,"PID Controller disabled");
	}
}

void Simple_Pid::spin(){
	ROS_INFO_COND(debug_mode_, "Starting PID control loop for %s", topic_motor_name_.c_str());
	
	//Wait for all stuffs connection
	ros::Duration(0.5).sleep();

	while(ros::ok()){
		ros::Duration(0.001).sleep();
		ros::spinOnce();
	}
}