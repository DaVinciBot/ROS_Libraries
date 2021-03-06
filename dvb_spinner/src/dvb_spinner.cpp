#include "dvb_spinner/dvb_spinner.h"

Dvb_Spinner::Dvb_Spinner() :
    debug_mode_(false),
    spinner_enable_(false),
    spinner_startable_(false)
{
    nh_ = ros::NodeHandle();

    topic_timer_name = ros::this_node::getName();

    topic_timer_name.append("/diagnostics/spin_timer");

    /*
		Get all params for ros server
	*/
    if( nh_.hasParam("/frequency") ||
        nh_.hasParam("/debug_mode")
    )
    {
        nh_.getParam("/frequency", freq_);
        nh_.getParam("/debug_mode", debug_mode_);
    }
    else
    {
		ROS_WARN("Please check if FREQUENCY | DEBUG_MODE parameters is set in the ROS Parameter Server !\n");
	}

    pub_ = nh_.advertise<std_msgs::Float32>(topic_timer_name, 1);
}

Dvb_Spinner::~Dvb_Spinner()
{
    
}

void Dvb_Spinner::spin(int times_freq)
{
    //Wait for initialization
    ros::Duration(1).sleep();

    ROS_INFO_COND(debug_mode_, "Spinner start !");

	while(ros::ok()){
        ros::Time before = ros::Time::now();
        
        spinOnce();

        spinOnce_timer_ = ros::Time::now().toSec() - before.toSec();

		ros::Duration(times_freq * freq_).sleep();

        std_msgs::Float32 spin_timer;
        spin_timer.data = spinOnce_timer_;

        pub_.publish(spin_timer);

        ROS_INFO_COND(debug_mode_, "%s : %f", topic_timer_name.c_str(), spinOnce_timer_);

		ros::spinOnce();

        now_ = ros::Time::now();
	}
}

void Dvb_Spinner::spinOnce()
{

}

void Dvb_Spinner::setEnable(bool state)
{
    spinner_enable_ = state;
}

void Dvb_Spinner::setStartable(bool state)
{
    spinner_startable_ = state;
}

bool Dvb_Spinner::getEnable()
{
    return spinner_enable_;
}

bool Dvb_Spinner::getStartable()
{
    return spinner_startable_;
}

ros::NodeHandle Dvb_Spinner::getNodeHandler()
{
    return nh_;
}

float_t Dvb_Spinner::getSpinOnceTimer()
{
    return spinOnce_timer_;
}