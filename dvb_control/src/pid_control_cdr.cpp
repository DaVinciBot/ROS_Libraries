#include "dvb_control/pid_control_cdr.h"

Pid_Control_CDR::Pid_Control_CDR(std::string topic_motor_name, std::string topic_encoder_name, bool debug_mode) :
    node_name_(""),
    topic_motor_name_(topic_motor_name),
    topic_encoder_name_(topic_encoder_name),
    debug_mode_(debug_mode),
	pid_control_startable_(false),
	pid_control_enable_(false),
    motor_out_min_(0),
	motor_out_max_(0),
    freq_(0.0),
	server_(nh_, node_name_, false),
    goal_(0)
{
    //Node name
	node_name_ = ros::this_node::getName();
	ROS_INFO("Starting ROS Node %s", node_name_.c_str());

    /*
		Get all params for ros server
	*/
	//PID params
	if (
			nh_.hasParam("/motor_out_min") ||
			nh_.hasParam("/motor_out_max") ||
			nh_.hasParam("/frequency")
	   )
	{
		nh_.param<float_t>("/motor_out_min", motor_out_min_);
		nh_.param<float_t>("/motor_out_max", motor_out_max_);
		nh_.param<double_t>("/frequency", freq_);

		pid_control_startable_ = true;
	}
	else{
		ROS_INFO("Please check if PID parameters are set in the ROS Parameter Server !\n");
	}

    /*
		Subscribers
	*/
	sub_encoder_ = nh_.subscribe(topic_encoder_name_.c_str(), 1, &Pid_Control_CDR::onEncoderEvent, this);
	/*
		Publishers
	*/
	pub_motor_ = nh_.advertise<std_msgs::Int32>(topic_motor_name_.c_str(), 10);	

	/*
		Actions
	*/
	server_.registerGoalCallback(boost::bind(&Pid_Control_CDR::setGoal, this));
}

Pid_Control_CDR::~Pid_Control_CDR()
{

}

void Pid_Control_CDR::setGoal()
{
    goal_ = server_.acceptNewGoal()->setpoint;
}

void Pid_Control_CDR::onEncoderEvent(const std_msgs::Int32::ConstPtr& encoder_msg)
{
    //Check control states
    if (pid_control_enable_ && pid_control_startable_ && server_.isActive())
    {
        //Get PID result
        output_ = simple_pid_.calcul_pid(encoder_msg->data);

        //Check limits
	    if ( output_ > motor_out_max_ ) output_ = motor_out_max_;
	    else if ( output_ < motor_out_min_ ) output_ = motor_out_min_;

        //Pub to motor
        std_msgs::Int32 tmp;
	    tmp.data = output_;
	    pub_motor_.publish(tmp);

        //Refresh feedback
        current_pos_.current_value = output_;
        server_.publishFeedback(current_pos_);

        //Debug
        ROS_INFO_COND(debug_mode_, "%s : %d", topic_motor_name_.c_str(), output_);
    }
    else
    {
        ROS_INFO_COND(debug_mode_, "%s is waiting for goal", node_name_.c_str());
    }
}

void Pid_Control_CDR::spinOnce()
{
    if(output_ >= goal_)
    {
        server_.setSucceeded(result_);
    }

    ros::Duration(freq_).sleep();
	ros::spinOnce();
}

void Pid_Control_CDR::spin()
{
    ROS_INFO_COND(debug_mode_, "Starting PID control loop for %s", topic_motor_name_.c_str());
	
	//Wait for all stuffs connection
	ros::Duration(0.5).sleep();

	while(ros::ok())
    {
		spinOnce();
	}
}

/*void Simple_Pid::spinOnce(const std_msgs::Float32::ConstPtr& encoder_msg){

	

	if(pid_enable_ && pid_startable_ && server_.isActive()){
		Simple_Pid::calcule_pid(encoder_msg->data);

		std_msgs::Float32 tmp;
		tmp.data = output__;
		pub_motor_.publish(tmp);
	}
	else{
		ROS_INFO_COND(debug_mode_,"PID Controller disabled");
	}
}

void Simple_Pid::spin(){
	ROS_INFO_COND(debug_mode_, "Starting PID control loop for %s", topic_motor_name_.c_str());
	
	//Wait for all stuffs connection
	ros::Duration(0.5).sleep();

	while(ros::ok()){
		ros::Duration(freq_).sleep();
		ros::spinOnce();
	}
}*/