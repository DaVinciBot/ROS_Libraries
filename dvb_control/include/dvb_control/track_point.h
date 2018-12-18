#ifndef DEF_TRACK_POINT_H
#define DEF_TRACK_POINT_H

#include "dvb_spinner/dvb_spinner.h"

#include "geometry_msgs/Point.h"
#include "std_msgs/Int16.h"

/*
    Track point in local frame
*/

class Track_Point : public Dvb_Spinner
{
    public:
        Track_Point();

        ~Track_Point();

        virtual void spinOnce();

        void onPointMsgEvent(const geometry_msgs::Point::ConstPtr& point_msg);

    private:
        std::string topic_sub_point_name_;
        std::string topic_pub_control_x_name_;
        std::string topic_pub_control_y_name_;

        //Subscriber
        ros::Subscriber sub_point_;
        
        //Publisher
        ros::Publisher pub_control_x_;
        ros::Publisher pub_control_y_;
};

#endif