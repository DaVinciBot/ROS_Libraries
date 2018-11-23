#ifndef DEF_MOVE_TO_POINT_H
#define DEF_MOVE_TO_POINT_H

#include <string>

#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

#include "actionlib/server/simple_action_server.h"
#include "dvb_control/SetMoveToPointAction.h"

class Move_To_Point
{
    public:
        Move_To_Point(geometry_msgs::Pose2D initial_pose, geometry_msgs::Pose2D target_pose);
        ~Move_To_Point();

        void moveForward();

        void setGoal();

        void spinOnce();
        void spin();

    private:
        ros::NodeHandle nh_;

        //"/robot/control/wheel/"
        std::string node_name_;

        //Topics names
        std::string topic_pid_left_name_;
        std::string topic_pid_right_name_;

        //Action stuffs
        dvb_control::SetMoveToPointFeedback current_pose_;
        dvb_control::SetMoveToPointResult final_pose_;

        //State
        bool debug_mode_;
        bool move_startable_;
        bool move_enable_;

        //Frequency rate
        float freq_;

        /*
            Publishers
        */
        ros::Publisher pub_wheel_left_control;
        ros::Publisher pub_wheel_right_control;
     
        /*
            Action Server
        */
        actionlib::SimpleActionServer<dvb_control::SetMoveToPointAction> server_;
        
        //Control Velocity
        geometry_msgs::Twist control_vel_left_msg_;
        geometry_msgs::Twist control_vel_right_msg_;

        //Poses
        geometry_msgs::Pose2D initial_pose_;
        geometry_msgs::Pose2D target_pose_;
};

#endif