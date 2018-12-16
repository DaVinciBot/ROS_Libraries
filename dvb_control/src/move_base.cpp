#include "dvb_control/move_base.h"

Move_Base::Move_Base() :
	Dvb_Spinner(),
	motor_left_setpoint_(0.0),
	motor_right_setpoint_(0.0),
	wheel_radius_(0.0),
	centreline_dist_(0.0)
{
	topic_sub_twist_name_ = nh_.getNamespace();
	topic_pub_motor_left_setpoint_name_ = nh_.getNamespace();
	topic_pub_motor_right_setpoint_name_ = nh_.getNamespace();

	topic_sub_twist_name_.append("/cmd_vel");
	topic_pub_motor_left_setpoint_name_.append("/motor/left/pid/setpoint");
	topic_pub_motor_right_setpoint_name_.append("/motor/right/pid/setpoint");

	/*
		Get all params from ros server
	*/
	//PID params
	if (
			nh_.hasParam("/robot/base/centreline_dist") ||
			nh_.hasParam("/robot/base/wheel_radius")
	   )
	{
		nh_.getParam("/robot/base/centreline_dist", centreline_dist_);
		nh_.getParam("/robot/base/wheel_radius", wheel_radius_);

		ROS_INFO("MOVE BASE : DEBUG_MODE(%d), FREQUENCY(%f), CENTRELINE(%f), WHEEL_RADIUS(%f)", debug_mode_, freq_, centreline_dist_, wheel_radius_);

		spinner_startable_ = true;
		spinner_enable_ = true;
	}
	else
	{
		ROS_INFO("Please check if ROBOT MODEL parameters are set in the ROS Parameter Server !\n");
	}	

	//Subscriber
	sub_twist_ = nh_.subscribe(topic_sub_twist_name_, 1, &Move_Base::onTwistEvent, this);

	//Publisher
	pub_motor_left_setpoint_ = nh_.advertise<std_msgs::Int16>(topic_pub_motor_left_setpoint_name_, 1);
	pub_motor_right_setpoint_ = nh_.advertise<std_msgs::Int16>(topic_pub_motor_right_setpoint_name_, 1);
}

Move_Base::~Move_Base()
{

}

void Move_Base::spinOnce()
{
	if(spinner_enable_ && spinner_startable_)
	{

	}
	else
	{
        ROS_INFO_COND(debug_mode_,"Move base disabled");
	}
}

void Move_Base::onTwistEvent(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
	//Please see Kinematic theory for more details
	motor_right_setpoint_ = twist_msg->linear.x + ( centreline_dist_ * twist_msg->angular.z / 2);
	motor_left_setpoint_ = twist_msg->linear.x - ( centreline_dist_ * twist_msg->angular.z / 2);

	motor_right_setpoint_ /= wheel_radius_;
	motor_left_setpoint_ /= wheel_radius_;

	std_msgs::Int16 motor_right_setpoint;
	std_msgs::Int16 motor_left_setpoint;

	motor_right_setpoint.data = motor_right_setpoint_;
	motor_left_setpoint.data = motor_left_setpoint_;

	pub_motor_right_setpoint_.publish(motor_right_setpoint);
	pub_motor_left_setpoint_.publish(motor_left_setpoint);

	ROS_INFO("V(%f), O(%f), Mr(%f), Ml(%f)", twist_msg->linear.x, twist_msg->angular.z, motor_right_setpoint_, motor_left_setpoint_);
}