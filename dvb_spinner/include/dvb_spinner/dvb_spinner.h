#ifndef DEF_DVB_SPINNER_H
#define DEF_DVB_SPINNER_H

#include "ros/ros.h"

#include "std_msgs/Float32.h"

class Dvb_Spinner
{
    public:
        Dvb_Spinner();

        virtual ~Dvb_Spinner();

        void spin(int times_freq); // spin frequency = times_freq * freq
        virtual void spinOnce();

        void setEnable(bool state);
        void setStartable(bool state);

        bool getEnable();
        bool getStartable();

        ros::NodeHandle getNodeHandler();

        float_t getSpinOnceTimer();

    protected:
        ros::NodeHandle nh_;

        std::string topic_timer_name;

        //State
        bool debug_mode_;
        bool spinner_startable_;
        bool spinner_enable_;

        //Frequency
        float_t freq_;

        //Time
        ros::Time now_;
        float_t spinOnce_timer_;

        //Spin timer pub
        ros::Publisher pub_;
};

#endif