#ifndef DEF_PID
#define DEF_PID

#include "dvb_spinner/dvb_spinner.h"

#include "std_msgs/Int16.h"

class Pid : public Dvb_Spinner
{
    public:
        Pid();

        ~Pid();

        virtual void spinOnce();

        void onSetPointEvent(const std_msgs::Int16::ConstPtr& setpoint_msg);
        void onSensorEvent(const std_msgs::Int16::ConstPtr& sensor_msg);
        void updateOutput();

        void calcul_pid();

        int getOutput();

     private:
        std::string topic_sub_setpoint_name_;
        std::string topic_sub_sensor_name_;
        std::string topic_pub_output_name_;

        int sensor_val_;
        int target_;
        int output_;

         //PID
        float_t kp_;
        float_t ki_;
        float_t kd_;

        float_t integral_;
        float_t error_;
        float_t derivative_;
        float_t previous_error_;

         //Time
        ros::Time prev_time_;
        ros::Duration dt_;

        //Subscriber
        ros::Subscriber sub_setpoint_;
        ros::Subscriber sub_sensor_;

        //Publisher
        ros::Publisher pub_;
};
 #endif 