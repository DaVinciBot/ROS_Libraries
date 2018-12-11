#include "dvb_hardware/encoder.h"

Encoder::Encoder() :
    Hardware(),
    pos_(0)
{
    //Topic name for motor and encoder
	topic_encoder_name_ = ros::this_node::getName();

    //Get Encoder PIN params
    std::string paramPinA = ros::this_node::getName();
    std::string paramPinB = ros::this_node::getName();

    paramPinA.append("/pinA");
    paramPinB.append("/pinB");

    if (
			nh_.hasParam(paramPinA) ||
            nh_.hasParam(paramPinB) //||
            //wiringPiSetup() < 0
    )
    {
        nh_.getParam(paramPinA, pinA_);
        nh_.getParam(paramPinB, pinB_);

        ROS_INFO("ENCODER : DEBUG_MODE(%d), FREQUENCY(%f), PINA(%d), PINB(%d)", debug_mode_, freq_,pinA_, pinB_);

        hardware_startable_ = true;
        hardware_enable_ = true;
    }
    else{
		ROS_WARN("Please check if encoder PIN parameters are set in the ROS Parameter Server !\n");
	}

    /*
		Publishers
	*/
	pub_encoder_ = nh_.advertise<std_msgs::Int32>(topic_encoder_name_.c_str(), 1);

    //WIRING PI Setup
    /*
    wiringPiISR(encoder->pinA, INT_EDGE_RISING, &increment)
    pinMode(encoder->pinB, _IOS_IMPUT);
    */
}

Encoder::~Encoder(){
    
}

int32_t Encoder::getPos()
{
    return pos_;
}

void Encoder::increment()
{
    /*
    int32_t signalA = digitalRead(pinA_);
    int32_t signalB = digitalRead(pinB_);

    if (signalB == HIGH)
    {
        --pos_;
    }

    else
    {
        ++pos_;
    }
    */

   std_msgs::Int32 encoder_pos;
   encoder_pos.data = pos_;

   pub_encoder_.publish(encoder_pos);

   ROS_INFO_COND(debug_mode_, "%s : %d", topic_encoder_name_.c_str(), pos_);
}

void Encoder::spinOnce(){
    if(hardware_enable_ && hardware_startable_){
        increment();
	}
	else{
        ROS_INFO_COND(debug_mode_,"Encoder disabled");
	}
}