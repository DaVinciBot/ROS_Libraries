#include "dvb_control/move_to_point.h"

Move_To_Point::Move_To_Point(geometry_msgs::Pose2D initial_pose, geometry_msgs::Pose2D target_pose) :
    node_name_(""),
    topic_pid_left_name_(""),
    topic_pid_right_name_(""),
    debug_mode_(false),
    move_startable_(false),
    move_enable_(false),
    freq_(0.001),
    //server_(nh_, node_name_, false),
    initial_pose_(initial_pose)
{
    node_name_ = ros::this_node::getName();

    topic_pid_left_name_ = "/robot/wheel/left/control";
    topic_pid_right_name_ = "/robot/wheel/right/control";

    //Load params
    nh_.getParam("/debug_mode", debug_mode_);
    nh_.getParam("/frequency", freq_);

    move_startable_ = true;
}

Move_To_Point::~Move_To_Point()
{

}

void Move_To_Point::moveForward()
{
    
}

void Move_To_Point::setGoal()
{
    //target_pose_ = server_.acceptNewGoal()->target_pose;
}

void Move_To_Point::spinOnce()
{

}

void Move_To_Point::spin()
{

}