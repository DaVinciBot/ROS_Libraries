#ifndef DEF_SIMPLE_PID
#define DEF_SIMPLE_PID

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"

class Simple_Pid
{
    public:
        Simple_Pid();
        ~Simple_Pid();

        int calcul_pid(int value);

    private:
        int target_;
        int output_;

        //PID
        float kp_;
        float ki_;
        float kd_;

        float integral_;
        float error_;
        float derivative_;
        float previous_error_;

        //Time
        ros::Time prev_time_;
        ros::Duration dt_;
};

#endif