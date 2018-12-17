#ifndef DEF_SERVO_H
#define DEF_SERVO_H

#include "dvb_spinner/dvb_spinner.h"

#include "std_msgs/Int16.h"

class Servo : public Dvb_Spinner
{
    public:
        Servo();

        virtual ~Servo();

        virtual void spinOnce();

        void onAngleMsgEvent(const std_msgs::Int16::ConstPtr& angle_msg);

        void updateAngle();

        int getAngle();
        void setAngle(int angle);

    private:
        //Topics name
        std::string topic_servo_name_;

        //PIN Number
        int pinPWM_;

        //Servo's angle in degree
        int angle_;

        //Subscriber
        ros::Subscriber sub_servo_;
};

#endif