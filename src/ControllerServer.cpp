#include <ros/ros.h>
#include <tutorial_controller/TutorialAction.h>
#include <actionlib/server/simple_action_server.h>

#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"

#define PI 3.14159265

//Class for containing the server
class ControllerServer{
public:
	
	ControllerServer(std::string name):

	as(n, "pid_control", boost::bind(&ControllerServer::executeCB, this, _1), false),
	action_name(name)
	{
		as.registerPreemptCallback(boost::bind(&ControllerServer::preemptCB, this));

		//Start the server
		as.start();	  
		
		//Subscriber current positon of servo
		positionservosub = n2.subscribe("/sensor/encoder/servo", 1, &ControllerServer::SensorCallBack, this);
		
		//Publisher setpoint, current position and error of control
		error_controlpub = n2.advertise<geometry_msgs::Vector3>("/control/error", 1);		
		
		//Publisher PID output in servo
		positionservopub = n2.advertise<std_msgs::Float64>("/motor/servo", 1);
		
		//Max e Min Output PID Controller
		float max = PI;
		float min = -PI;
		
		//Initializing PID Controller
		Initialize(min,max);
  }


//Callback for handling preemption. Reset your helpers here.
//Note that you still have to check for preemption in your work method to break it off
void preemptCB()
{
	ROS_INFO("%s got preempted!", action_name.c_str());
	result.ok = 0;
	as.setPreempted(result, "I got Preempted!");
}

//Callback for processing a goal
void executeCB(const tutorial_controller::TutorialGoalConstPtr& goal)
{
  prevTime = ros::Time::now();
	
	//If the server has been killed, don't process
	if(!as.isActive()||as.isPreemptRequested()) return;

	//Run the processing at 100Hz
	ros::Rate rate(100);

	//Setup some local variables
	bool success = true;	
	
	//Loop control
	while(1)
	{
		std_msgs::Float64 msg_pos;
		
		//PID Controller
		msg_pos.data = PIDController(goal->position, position_encoder);
		
		//Publishing PID output in servo
		positionservopub.publish(msg_pos);
		
		//Auxiliary Message
		geometry_msgs::Vector3 msg_error;
		
		msg_error.x = goal->position;
		msg_error.y = position_encoder;
		msg_error.z = goal->position - position_encoder;
		
		//Publishing setpoint, feedback and error control
		error_controlpub.publish(msg_error);
		
		feedback.position = position_encoder;
    
    //Publish feedback to action client
    as.publishFeedback(feedback);
		
		//Check for ROS kill
		if(!ros::ok())
		{
			success = false;
			ROS_INFO("%s Shutting Down", action_name.c_str());
			break;
		}

		//If the server has been killed/preempted, stop processing
		if(!as.isActive()||as.isPreemptRequested()) return;
		
		//Sleep for rate time
		rate.sleep();
	}
	
	//Publish the result if the goal wasn't preempted
	if(success)
	{
		result.ok = 1;
		as.setSucceeded(result);
	}
	else
	{
		result.ok = 0;
		as.setAborted(result,"I Failed!");
	}
}

void Initialize( float min, float max)
{
	setOutputLimits(min, max);
  lastError = 0;
  errSum = 0;
    
	kp = 1.5;
	ki = 0.1;
	kd = 0;
	
//		kp = 1;
//	ki = 2.3;
//	kd = 0;   
}

void setOutputLimits(float min, float max)
{
	if (min > max) return;
    
	minLimit = min;
	maxLimit = max;
}

float PIDController(float setpoint, float PV)
{
	ros::Time now = ros::Time::now();
	ros::Duration change = now - prevTime;

	float error = setpoint - PV;
  
	float dErr = error - lastError;
	
	errSum += error*change.toSec();
	errSum = std::min(errSum, maxLimit);
	errSum = std::max(errSum, minLimit);	
	
  dErr = (error - lastError)/change.toSec();
	
	//Do the full calculation
	float output = (kp*error) + (ki*errSum) + (kd*dErr);
   
	//Clamp output to bounds
	output = std::min(output, maxLimit);
	output = std::max(output, minLimit);  

	//Required values for next round
	lastError = error;
	
	return output;
}

void SensorCallBack(const sensor_msgs::JointState& msg)
{
	position_encoder = msg.position[0];
}

protected:
	ros::NodeHandle n;
	ros::NodeHandle n2;
	
	//Subscriber
	ros::Subscriber positionservosub;
	
	//Publishers
	ros::Publisher positionservopub;
	ros::Publisher error_controlpub;
	
	//Actionlib variables
	actionlib::SimpleActionServer<tutorial_controller::TutorialAction> as;
	tutorial_controller::TutorialFeedback feedback;
	tutorial_controller::TutorialResult result;
	std::string action_name;
	
	//Control variables
	float position_encoder;
	float errSum;
	float lastError;
	float minLimit, maxLimit;
	ros::Time prevTime;
	float kp;
	float ki;
	float kd;	
};

//Used by ROS to actually create the node. Could theoretically spawn more than one server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pid_server");

 	//Just a check to make sure the usage was correct
	if(argc != 1)
	{
		ROS_INFO("Usage: pid_server");
		return 1;
	}
	
	//Spawn the server
	ControllerServer server(ros::this_node::getName());
  
	ros::spin();

	return 0;
}
