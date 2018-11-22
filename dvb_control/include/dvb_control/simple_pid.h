#ifndef DEF_SIMPLE_PID
#define DEF_SIMPLE_PID

#include <memory>
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"

#include "std_msgs/Float32.h"

class Simple_Pid
{
    public:
        Simple_Pid(std::string topic_motor_name, std::string topic_encoder_name, bool debug_mode);

        bool setParams();

        void calcule_pid(float_t encoder_val);

        void spinOnce(const std_msgs::Float32::ConstPtr& encoder_msg);
        void spin();

    private:
    	ros::NodeHandle nh_;
        
        std::string node_name_;

        //Topics name
        std::string topic_motor_name_;
        std::string topic_encoder_name_;

        //State
        bool debug_mode_;
        bool pid_startable_;
        bool pid_enable_;

        float_t target_;

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

        float_t output_;

        //Motor settings
        float_t motor_out_min_; //output min
        float_t motor_out_max_; //output max

        //Frequency rate
        double_t freq_;

        /*
            Subscribers
        */
        ros::Subscriber sub_encoder_;
        /*
            Publishers
        */
        ros::Publisher pub_motor_;
};

#endif