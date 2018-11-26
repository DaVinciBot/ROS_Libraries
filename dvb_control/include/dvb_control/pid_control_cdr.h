/*
    TO DO :

        - Implement tolerance error for if statement in spinOnce
*/

#ifndef DEF_PID_CONTROL_CDR_H
#define DEF_PID_CONTROL_CDR_H

#include "dvb_control/simple_pid.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"

#include "std_msgs/Int32.h"

#include "actionlib/server/simple_action_server.h"
#include "dvb_control/SetPIDSetpointAction.h"

class Pid_Control_CDR
{
    public:
        Pid_Control_CDR(std::string topic_motor_name, std::string topic_encoder_name, bool debug_mode);
        ~Pid_Control_CDR();

        std::string getControlVelTopic();

        void setGoal();

        void onEncoderEvent(const std_msgs::Int32::ConstPtr& encoder_msg);

        void spinOnce();
        void spin();
    
    private:
        ros::NodeHandle nh_;

        std::string node_name_;

        //Topics names
        std::string topic_control_vel_name_;

        std::string topic_motor_name_;
        std::string topic_encoder_name_;

        //Action stuffs
        dvb_control::SetPIDSetpointFeedback current_pos_;
        dvb_control::SetPIDSetpointResult result_;

        //State
        bool debug_mode_;
        bool pid_control_startable_;
        bool pid_control_enable_;

        //Motor settings
        float motor_out_min_; //output min
        float motor_out_max_; //output max

        //Frequency rate
        float freq_;

        /*
            Subscribers
        */
        ros::Subscriber sub_encoder_;
        /*
            Publishers
        */
        ros::Publisher pub_motor_;

        /*
            Action Server
        */
        actionlib::SimpleActionServer<dvb_control::SetPIDSetpointAction> server_;
        
        //PID
        Simple_Pid simple_pid_;

        //Goal
        int goal_;

        //Output
        int output_;
};

#endif