#include "dvb_control/track_point.h"

Track_Point::Track_Point() :
    Dvb_Spinner()
{
    topic_sub_point_name_ = nh_.getNamespace();
    
    topic_sub_point_name_.append("/track_point/point");

    
    /*
		Get all params for ros server
	*/
	if (
            ros::param::get("~topic_pub_control_x_name", topic_pub_control_x_name_) &&
            ros::param::get("~topic_pub_control_y_name", topic_pub_control_y_name_)

	   )
	{
		ROS_INFO("TRACK POINT : DEBUG_MODE(%d), FREQUENCY(%f)", debug_mode_, freq_);

		spinner_startable_ = true;
		spinner_enable_ = true;
	}
	else
    {
		ROS_INFO("Please check if Tracking point parameters are set in the ROS Parameter Server !\n");
	}	

    //Subcriber
    sub_point_ = nh_.subscribe(topic_sub_point_name_, 1, &Track_Point::onPointMsgEvent, this);

    //Publisher
    pub_control_x_ = nh_.advertise<std_msgs::Int16>(topic_pub_control_x_name_, 1);
    pub_control_y_ = nh_.advertise<std_msgs::Int16>(topic_pub_control_y_name_, 1);
}

Track_Point::~Track_Point()
{

}

void Track_Point::spinOnce()
{

}

void Track_Point::onPointMsgEvent(const geometry_msgs::Point::ConstPtr& point_msg)
{
    std_msgs::Int16 control_x;
    std_msgs::Int16 control_y;

    control_x.data = point_msg->x;
    control_y.data = point_msg->y;

    pub_control_x_.publish(control_x);
    pub_control_y_.publish(control_y);
}