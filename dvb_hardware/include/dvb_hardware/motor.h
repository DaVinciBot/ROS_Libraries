#ifndef DEF_MOTOR_H
#define DEF_MOTOR_H

#include "dvb_spinner/dvb_spinner.h"

#include "std_msgs/Int16.h"

class Motor : public Dvb_Spinner
{
    public:
        Motor();
        virtual ~Motor();

        virtual void spinOnce();

    private:
        //Topics name
        std::string topic_motor_name_;

        void control_callback(const std_msgs::Int16::ConstPtr& control_msg);

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