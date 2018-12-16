#include "dvb_hardware/encoder.h"

Encoder::Encoder() :
    Dvb_Spinner(),
    pos_(0)
{
    //Topic name for motor and encoder
	topic_encoder_name_ = nh_.getNamespace().c_str();

    //Get Encoder PIN params
    std::string paramPinA = nh_.getNamespace().c_str();
    std::string paramPinB = nh_.getNamespace().c_str();

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

        spinner_startable_ = true;
        spinner_enable_ = true;
    }
    else{
		ROS_WARN("Please check if encoder PIN parameters are set in the ROS Parameter Server !\n");
	}

    /*
		Publishers
	*/
	pub_encoder_ = nh_.advertise<std_msgs::Int16>(topic_encoder_name_.c_str(), 1);

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

   std_msgs::Int16 encoder_pos;
   encoder_pos.data = 5;//pos_;

   pub_encoder_.publish(encoder_pos);

   ROS_INFO_COND(debug_mode_, "%s : %d", topic_encoder_name_.c_str(), pos_);
}

void Encoder::spinOnce(){
    if(spinner_enable_ && spinner_startable_){
        increment();
	}
	else{
        ROS_INFO_COND(debug_mode_,"Encoder disabled");
	}
}