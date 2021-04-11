#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <wasp_joy/LogisticsAction.h>
#include <wasp_joy/TrolleyAlignAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <boost/thread.hpp>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <iostream>
#include <fstream>
#include <vector>
#include<sstream>

double distance;
struct goal_msgs
{
	std::vector<float> x;
	std::vector<float> y;
	std::vector<float> theta_z;
	std::vector<float> theta_w;
};

goal_msgs ReadFile(std::string file_location)
	{
		  //std::ofstream myfile(file_location);
		  std::ifstream myfile;
		  myfile.open(file_location);
		  std::string line;
		  //std::vector<goal_msgs> point_vec;
		  goal_msgs data,point;
		  float valx,valy,valtheta_z, valtheta_w;
		  int i=0;
		  while(std::getline(myfile,line))
		  {
		  	std::string token;
		  	std::stringstream ss(line);
		  	//ss>> valx >> valy >> valtheta;
		  	std::getline(ss,token,',');
		  	valx=std::stod(token);
		  	std::getline(ss,token,',');
		  	valy=std::stod(token);
		  	std::getline(ss,token,',');
		  	valtheta_z=std::stod(token);
		  	std::getline(ss,token,',');
		  	valtheta_w=std::stod(token);
		  	point.x.push_back(valx);
		  	point.y.push_back(valy);
		  	point.theta_z.push_back(valtheta_z);
		  	point.theta_w.push_back(valtheta_w);
		  	
		  }
		  myfile.close();
		  return point;		  	  
	}

void spinThread()
{
  ros::spin();
}
void doneCb(const actionlib::SimpleClientGoalState& state,
            const wasp_joy::LogisticsResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: ");
  //ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

void feedbackCb(const wasp_joy::LogisticsFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback: current angle is");
  //distance=feedback->distance;
  return;
}

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
}
void tafeedbackCb(const wasp_joy::TrolleyAlignFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback: current angle is %f", feedback->align_distance);
  distance=feedback->align_distance;
  return;
}
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_actionclient");
  ros::NodeHandle nh;
  ros::Time::init();
  ros::Time begin;
  actionlib::SimpleActionClient<wasp_joy::LogisticsAction> lg("logi_action", true);
  actionlib::SimpleActionClient<wasp_joy::TrolleyAlignAction> ta("trolleyalign_action", true);
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  //boost::thread spin_thread(&spinThread);
  goal_msgs point_;
  wasp_joy::LogisticsGoal goal_lg;
  wasp_joy::TrolleyAlignGoal goal_ta;
  wasp_joy::LogisticsFeedback feedback;
  int trolley_flag =0;
  int sequence =1;
  int i=0;
  bool finished_waypoint=false;
  bool logistic_control=false;
  bool finished_align = false;

  ROS_INFO("navi_waypoint");
  point_=ReadFile("/home/vinu/nav_points.csv");
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
	//ROS_INFO("Sending goal[%ld]", (long int)point_.x.size());
	ROS_INFO("Waiting for the move_base action server");
  }
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	switch(i)
	{
		case 0:
		    goal.target_pose.pose.position.x = point_.x[i];
	    	goal.target_pose.pose.position.y = point_.y[i];
	    	goal.target_pose.pose.orientation.z = point_.theta_z[i];

	    	goal.target_pose.pose.orientation.w = point_.theta_w[i];

		  	ROS_INFO("Sending goal[%ld]", (long int)i);
		  	ac.sendGoal(goal);
	  		finished_waypoint = ac.waitForResult();
	  		while (finished_waypoint)
	  		{
	  			  ROS_INFO("Waiting for trolleyalign_action server to start.");
  				ta.waitForServer();
	  			ROS_INFO("%f:feedbackdistance",feedback.distance);
	  			//feedback.distance = 1.7;
	  			goal_ta.align =true;
	  			ta.sendGoal(goal_ta,&tadoneCb, &taactiveCb, &tafeedbackCb);
	  			finished_align = ta.waitForResult();
	  			while(finished_align)
	  			{
	  				switch (sequence)
		  			{
		  				case 0:
		  					ROS_INFO("trolley not detected");
		  					goal_lg.active = true;
		  					goal_lg.liftup =0;
		  					goal_lg.liftdown =0;
		  					lg.sendGoal(goal_lg,  &doneCb, &activeCb, &feedbackCb);
		  					ROS_INFO("%f:feedback_distance_inside",feedback.distance);
		  					if (distance >=1.5)
				  			{
				  				sequence =1;
				  			}
		  					break;
		  				case 1:
		  					ROS_INFO("trolley detected & sending commands to lift up");
		  					goal_lg.active = true;
		  					goal_lg.liftup =1;
		  					goal_lg.liftdown =0;
		  					goal_lg.distance =0.5;
		  					lg.sendGoal(goal_lg);
		  					break;
		  			}
		  			
		  			logistic_control = lg.waitForResult();
		  			if (logistic_control)
		  			{
		  				i=1;
		  				//finished_waypoint = false;
		  				break;
		  			}

	  			}
		  			
	  		}

		case 1:
			finished_align =false;
			finished_waypoint = false;
			logistic_control = false;
		    goal.target_pose.pose.position.x = point_.x[i];
	    	goal.target_pose.pose.position.y = point_.y[i];
	    	goal.target_pose.pose.orientation.z = point_.theta_z[i];

	    	goal.target_pose.pose.orientation.w = point_.theta_w[i];
	    	sequence =1;
		  	ROS_INFO("Sending goal[%ld]", (long int)i);
		  	ac.sendGoal(goal);
	  		finished_waypoint = ac.waitForResult();
	  		while (finished_waypoint)
	  		{
	  			/*if (feedback.distance >=1.5)
	  			{
	  				sequence =1;
	  			}*/
	  			switch (sequence)
	  			{
	  				case 0:
	  					ROS_INFO("trolley not detected[%ld]", (long int)i);
	  					goal_lg.liftup =0;
	  					goal_lg.liftdown =0;
	  					break;
	  				case 1:
	  					ROS_INFO("trolley detected & sending commands to lift down[%ld]", (long int)i);
	  					goal_lg.liftup =0;
	  					goal_lg.liftdown =1;
	  					lg.sendGoal(goal_lg);
	  					break;
	  			}
	  			
	  			logistic_control = lg.waitForResult();
	  			if (logistic_control)
	  			{
	  				actionlib::SimpleClientGoalState state = lg.getState();
    				ROS_INFO("Action finished: %s",state.toString().c_str());
    				finished_waypoint = false;
    				break;
	  			}
	  			else
    				ROS_INFO("Action did not finish before the time out.");
	  			
	  		}

	  	//ac.waitForResult();
/*
	  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    	ROS_INFO("Robot moving");
	  	else
	    	ROS_INFO("The robot failed to move for some reason");
	    	*/

	}

  //ROS_INFO("Waiting for action server to start.");
  //lg.waitForServer();

/*
  bool finished_before_timeout = lg.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = lg.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
    */

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  //spin_thread.join();

  //exit
  return 0;
}

