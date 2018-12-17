#ifndef DEF_DIFF_ROBOT_ODOMETRY_H
#define DEF_DIFF_ROBOT_ODOMETRY_H

#include "dvb_spinner/dvb_spinner.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include "std_msgs/Int16.h"

class Odometry : public Dvb_Spinner
{
    public:
        Odometry();

        virtual ~Odometry();

        virtual void spinOnce();

        void onEncoderLeftEvent(const std_msgs::Int16::ConstPtr& encoder_msg);
        void onEncoderRightEvent(const std_msgs::Int16::ConstPtr& encoder_msg);

        void updateOdom();

    private:
        std::string topic_pub_odom_name_; 
        std::string topic_encoder_left_name_;
        std::string topic_encoder_right_name_;

        geometry_msgs::Pose2D robot_last_pose_;

        //Robot model
        float_t centreline_dist_;
        float_t meter_per_tick_left_;
        float_t meter_per_tick_right_;

        //Encoder values
        int16_t prev_encoder_left_;
        int16_t prev_encoder_right_;
        int16_t encoder_left_;
        int16_t encoder_right_;

        //Odometry
        nav_msgs::Odometry robot_odom_;

        //Time
        ros::Time last_time_;

        //Subscriber
        ros::Subscriber sub_encoder_left_;
        ros::Subscriber sub_encoder_right_;

        //Publisher
        ros::Publisher pub_odom_;
};

#endif