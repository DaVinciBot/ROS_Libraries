#include "dvb_navigation/diff_robot_odometry.h"

Odometry::Odometry() :
    Dvb_Spinner(),
    meter_per_tick_left_(0.0),
    meter_per_tick_right_(0.0),
    prev_encoder_left_(0),
    prev_encoder_right_(0),
    encoder_left_(0),
    encoder_right_(0)
{
    topic_pub_odom_name_ = nh_.getNamespace();

    topic_pub_odom_name_.append("/odom");

    /*
		Get all params from ros server
	*/
	//PID params
	if (
            nh_.hasParam("/robot/base/centreline_dist") ||
			nh_.hasParam("topic_encoder_left") ||
			nh_.hasParam("topic_encoder_right") ||
            nh_.hasParam("/robot/base/encoder/left/meter_per_tick") ||
            nh_.hasParam("/robot/base/encoder/right/meter_per_tick")
	   )
	{
        nh_.getParam("/robot/base/centreline_dist", centreline_dist_);
		nh_.getParam("topic_encoder_left", topic_encoder_left_name_);
		nh_.getParam("topic_encoder_right", topic_encoder_right_name_);
        nh_.getParam("/robot/base/encoder/left/meter_per_tick", meter_per_tick_left_);
        nh_.getParam("/robot/base/encoder/right/meter_per_tick", meter_per_tick_right_);

		ROS_INFO("ODOM : DEBUG_MODE(%d), FREQUENCY(%f)", debug_mode_, freq_);

		spinner_startable_ = true;
		spinner_enable_ = true;
	}
	else
	{
		ROS_INFO("Please check if ROBOT MODEL parameters are set in the ROS Parameter Server !\n");
	}	

    //Subscriber
    sub_encoder_left_ = nh_.subscribe(topic_encoder_left_name_.c_str(), 1, &Odometry::onEncoderLeftEvent, this);
    sub_encoder_right_ = nh_.subscribe(topic_encoder_right_name_.c_str(), 1, &Odometry::onEncoderRightEvent, this);

    //Publisher
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(topic_pub_odom_name_, 1);

    last_time_ = ros::Time::now();
}

Odometry::~Odometry()
{

}

void Odometry::spinOnce()
{

}

void Odometry::onEncoderLeftEvent(const std_msgs::Int16::ConstPtr& encoder_msg)
{
    encoder_left_ = encoder_msg->data;
}

void Odometry::onEncoderRightEvent(const std_msgs::Int16::ConstPtr& encoder_msg)
{
    encoder_right_ = encoder_msg->data;
}

void Odometry::updateOdom()
{
    float_t delta_enc_left = encoder_left_ - prev_encoder_left_;
    float_t delta_enc_right = encoder_right_ - prev_encoder_right_;

    float_t dist_enc_left = meter_per_tick_left_ * delta_enc_left;
    float_t dist_enc_right = meter_per_tick_right_ * delta_enc_right;

    float_t dist_robot = (dist_enc_left + dist_enc_right) / 2;

    float_t dt = now_.toSec() - last_time_.toSec();

    robot_last_pose_.x += dist_robot * cos(robot_last_pose_.theta);
    robot_last_pose_.y += dist_robot * sin(robot_last_pose_.theta);
    robot_last_pose_.theta += ( dist_enc_right - dist_enc_left ) / centreline_dist_;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_last_pose_.theta);



    robot_odom_.header.frame_id = "odom";
    robot_odom_.header.stamp = now_;

    //Robot velocity
    robot_odom_.twist.twist.linear.x = dist_robot / dt;
    robot_odom_.twist.twist.angular.z = (dist_enc_left - dist_enc_right) / ( centreline_dist_ * dt );

    //Robot current pose
    robot_odom_.pose.pose.position.x = robot_last_pose_.x;
    robot_odom_.pose.pose.position.y = robot_last_pose_.y;
    //robot_odom_.pose.pose.orientation = 
}