#ifndef DEF_MOVE_BASE_H
#define DEF_MOVE_BASE_H

#include "dvb_spinner/dvb_spinner.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"

class Move_Base : public Dvb_Spinner
{
    public:
        Move_Base();

        virtual ~Move_Base();

        virtual void spinOnce();

        void onTwistEvent(const geometry_msgs::Twist::ConstPtr& twist_msg);

    private:
        std::string topic_sub_twist_name_;
        std::string topic_pub_motor_right_setpoint_name_;
        std::string topic_pub_motor_left_setpoint_name_;

        geometry_msgs::Twist cmd_vel_;

        //Setpoints
        float_t motor_left_setpoint_;
        float_t motor_right_setpoint_;

        //Robot model
        float_t wheel_radius_;
        float_t centreline_dist_;

        //Subscriber
        ros::Subscriber sub_twist_;

        //Publisher
        ros::Publisher pub_motor_left_setpoint_;
        ros::Publisher pub_motor_right_setpoint_;
};

#endif