#include <ros/ros.h>
#include <tutorial_controller/TutorialAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64.h"

//class containing the client
class ControllerClient{

  public:

    ControllerClient(std::string name):

	    //Set up the client. It's publishing to topic "pid_control", and is set to auto-spin
	    ac("pid_control", true),

	    //Stores the name
	    action_name(name)
	    {
	      //Get connection to a server
	      ROS_INFO("%s Waiting For Server...", action_name.c_str());

	      //Wait for the connection to be valid
	      ac.waitForServer();

	      ROS_INFO("%s Got a Server...", action_name.c_str());
	
	      goalsub = n.subscribe("/cmd_pos", 100, &ControllerClient::GoalCallback, this);
      }

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const tutorial_controller::TutorialResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());

	ROS_INFO("Result: %i", result->ok);
};

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Goal just went active...");
};

// Called every time feedback is received for the goal
void feedbackCb(const tutorial_controller::TutorialFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback of Progress to Goal: position: %f", feedback->position);
};

void GoalCallback(const std_msgs::Float64& msg)
{	
	goal.position = msg.data;
	
	ac.sendGoal(goal, boost::bind(&ControllerClient::doneCb, this, _1, _2),
		 boost::bind(&ControllerClient::activeCb, this),
		 boost::bind(&ControllerClient::feedbackCb, this, _1));
};

private:
	actionlib::SimpleActionClient<tutorial_controller::TutorialAction> ac;
	std::string action_name;	
	tutorial_controller::TutorialGoal goal;	
	ros::Subscriber goalsub;
	ros::NodeHandle n;
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "pid_client");
	
	// create the action client
	// true causes the client to spin its own thread
	ControllerClient client(ros::this_node::getName());

	ros::spin();
	
	//exit
	return 0;
}
