#include "dvb_hardware/servo.h"

Servo::Servo() :
    Dvb_Spinner(),
    pinPWM_(0),
    angle_(0)
{
    //Topic name
    topic_servo_name_ = nh_.getNamespace().c_str();

    topic_servo_name_.append("/angle");

    //Get servo PIN params
    std::string paramPinPWM = nh_.getNamespace().c_str();

    paramPinPWM.append("/pinPWM");

    if (
			nh_.hasParam(paramPinPWM)
    )
    {
        nh_.getParam(paramPinPWM, pinPWM_);

        ROS_INFO("SERVO : DEBUG_MODE(%d), FREQUENCY(%f), PINPWM(%d)", debug_mode_, freq_, pinPWM_);

        spinner_startable_ = true;
        spinner_enable_ = true;
    }
    else
    {
		ROS_WARN("Please check if SERVO PIN parameters are set in the ROS Parameter Server !\n");
	}

    //Subscriber
    sub_servo_ = nh_.subscribe(topic_servo_name_, 1, &Servo::onAngleMsgEvent, this);
}

Servo::~Servo()
{

}

void Servo::spinOnce()
{
    if(spinner_enable_ && spinner_startable_)
    {
        updateAngle();
	}
	else
    {
        ROS_INFO_COND(debug_mode_,"Spinner disabled");
	}
}

void Servo::onAngleMsgEvent(const std_msgs::Int16::ConstPtr& angle_msg)
{
    angle_ = angle_msg->data;
}

void Servo::updateAngle()
{
    //digitalWrite angle_ ...
}

int Servo::getAngle()
{
    return angle_;
}

void Servo::setAngle(int angle)
{
    angle_ = angle;
}

