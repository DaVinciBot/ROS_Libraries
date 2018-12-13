#include "dvb_hardware/motor.h"
#include <wiringPi.h>

Motor::Motor() :
    Dvb_Spinner(),
    pwm_(0)
{
    topic_motor_name_ = ros::this_node::getName();

    //Get motor PIN params
    std::string paramPinPWM = ros::this_node::getName();
    std::string paramPinDir1 = ros::this_node::getName();
    std::string paramPinDir2 = ros::this_node::getName();

    paramPinPWM.append("/pinPWM");
    paramPinDir1.append("/pinDir1");
    paramPinDir2.append("/pinDir2");

    //wiringPiSetup();

    if (
			nh_.hasParam(paramPinPWM)  ||
            nh_.hasParam(paramPinDir1)  ||
            nh_.hasParam(paramPinDir2) ||
            nh_.hasParam("/motor/output/min") ||
            nh_.hasParam("/motor/output/max")
    )
    {
        nh_.getParam(paramPinPWM, pinPWM_);
        nh_.getParam(paramPinDir1, pinDirection);
        nh_.getParam(paramPinDir2, pinDirection2);
        nh_.getParam("/motor/output/min", output_min_);
        nh_.getParam("/motor/output/max", output_max_);
 	    
        /*
        pinMode(pinPWM_,PWM_OUTPUT);   //ENA
 	    pinMode(pinDirection,OUTPUT); //IN1
        pinMode(pinDirection2,OUTPUT);//IN2
        */

        ROS_INFO("MOTOR : DEBUG_MODE(%d), FREQUENCY(%f), PINPWM(%d), PINDIR1(%d), PINDIR2(%d)", debug_mode_, freq_, pinPWM_, pinDirection, pinDirection2);
	
        spinner_startable_ = true;
        spinner_enable_ = false;
    }
    else{
		ROS_WARN("Please check if motor PIN parameters are set in the ROS Parameter Server !\n");
	}

    /*
		Subscribers
	*/
	sub_motor_ = nh_.subscribe(topic_motor_name_.c_str(), 1, &Motor::control_callback, this);
}

Motor::~Motor(){

}

void Motor::spinOnce(){
}

void Motor::control_motor(int32_t pwm, bool trigo_dir)
{
    ROS_INFO("Start %d",pwm);   
    /*pwmWrite(pinPWM_, pwm); //speed 0 - 255 
  
    if (trigo_dir == false) {
     digitalWrite(pinDirection, LOW);
     digitalWrite(pinDirection2, HIGH);
    }
    else {
    digitalWrite(pinDirection, HIGH);
    digitalWrite(pinDirection2,LOW);   
    }*/
}

void Motor::control_callback(const std_msgs::Int32::ConstPtr& control_msg)
{
    /*
        TODO : Check if exists ros msg to control pwm and direction
    */

    if(spinner_enable_ && spinner_startable_){
        ROS_INFO_COND(TRUE, "%s : %d", topic_motor_name_.c_str(), control_msg->data);

		Motor::control_motor(control_msg->data, true);
	}
	else{
		ROS_WARN_COND(debug_mode_,"PID Controller disabled");
	}
}