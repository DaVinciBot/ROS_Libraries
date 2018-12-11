#ifndef DEF_MOTOR_H
#define DEF_MOTOR_H

#include "dvb_hardware/hardware.h"

class Motor : public Hardware
{
    public:
        Motor();
        virtual ~Motor();

        virtual void spinOnce();

    private:
        //Topics name
        std::string topic_motor_name_;

        void control_callback(const std_msgs::Int32::ConstPtr& control_msg);

        //Set motor speed
        void control_motor(int32_t pwm, bool trigo_dir);

        //PIN Number
        int32_t pinPWM_;
        int32_t pinDirection;
	    int32_t pinDirection2; 
        //Control stuffs
        int32_t pwm_;
        bool trigo_dir_;

        //Motor parameters
        float_t output_min_;
        float_t output_max_;

        /*
		    Subscribers
	    */
        ros::Subscriber sub_motor_;
};

#endif