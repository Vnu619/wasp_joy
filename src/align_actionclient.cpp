#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <wasp_joy/TrolleyAlignAction.h>
#include <wasp_joy/LogisticsAction.h>

double distance,trolley_gap;
void tafeedbackCb(const wasp_joy::TrolleyAlignFeedbackConstPtr& feedback);
wasp_joy::TrolleyAlignFeedback feedback_;
void tadoneCb(const actionlib::SimpleClientGoalState& state,
            const wasp_joy::TrolleyAlignResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: ");
  //ros::shutdown();
}

// Called once when the goal becomes active
void taactiveCb()
{
  ROS_INFO("Goal just went active");
return;
}
void tafeedbackCb(const wasp_joy::TrolleyAlignFeedbackConstPtr& feedback1)
{
  //ROS_INFO("Got Feedback: current distance %f", feedback->align_distance);
  distance=feedback1->align_distance;
  //ROS_INFO("Got Feedback:distance %f", distance);
  //return;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_align_action");
  ros::NodeHandle nh;
  ros::Time::init();
  actionlib::SimpleActionClient<wasp_joy::TrolleyAlignAction> ta("trolleyalign_action", true);
  actionlib::SimpleActionClient<wasp_joy::LogisticsAction> lg("logi_action", true);

  ROS_INFO("Waiting for action server to start.");
  ta.waitForServer();
  /*while(!ta.waitForServer(ros::Duration(5.0)))
  {
	//ROS_INFO("Sending goal[%ld]", (long int)point_.x.size());
	ROS_INFO("Waiting for the trolley_align action server");
  }
  */
  wasp_joy::TrolleyAlignGoal goal_ta;
  wasp_joy::LogisticsGoal goal_lg;
  wasp_joy::LogisticsFeedback feedback;
  bool finished_align = false;
  goal_ta.align =true;
  int sequence =1;
  bool logistic_control=false;

  ta.sendGoal(goal_ta, &tadoneCb, &taactiveCb, &tafeedbackCb);  
  finished_align = ta.waitForResult();

  while(finished_align)
	{
		ROS_INFO("aligned");
		ROS_INFO("feedback_distance_inside %f", distance);
	switch (sequence)
	{
		case 0:
			ROS_INFO("trolley not detected");
			goal_lg.active = true;
			goal_lg.liftup =0;
			goal_lg.liftdown =0;
			lg.sendGoal(goal_lg);//,  &doneCb, &activeCb, &feedbackCb);
			
			if (trolley_gap >=1.5)
			{
				sequence =1;
			}
			break;
		case 1:
			ROS_INFO("trolley detected & sending commands to lift up");
			goal_lg.active = true;
			goal_lg.liftup =1;
			goal_lg.liftdown =0;
			goal_lg.distance =distance;
			lg.sendGoal(goal_lg);
			break;
	}

	logistic_control = lg.waitForResult();
		if (logistic_control)
	{
		actionlib::SimpleClientGoalState state = lg.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());
	finished_align = false;
	break;
	}
	else
	ROS_INFO("Action did not finish before the time out.");

	}


return 0;
}
