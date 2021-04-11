#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "wasp_joy/GetPose.h"
#include <cstdlib>
#include <stdio.h>


void RunWaypointService();
void ResetWaypointService();
void NaviWaypointService();
void WallfollowerService();
class RobotState
{
public:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	//void RunWaypointService();
	int RobotButton(int ch);
	
	//void run();
private:
	double start_button,clear_button,clear_mode,waypoint_store,mode_button,mode2_button;
	double previous_start_button=0;
	double previous_clear_mode=0;
	double previous_waypoint_store=0;
	int button_state=1;
	int waypoint_state = 0;
	int mode =4;
	int a=0;
	enum Operation_Modes
	{
		WAYPOINT = -1,
		WAYPOINT_NAVI = -2,
		LOGISTICS = 1,
		WALLFOLLOWER = 2,
		DEFAULT =4

	};
	
}; 
int RobotState::RobotButton(int ch)
{	
	RobotState monitor;

	ROS_INFO("  sending back response: [%ld]", (long int)ch);
	/*switch(ch)
	{
	case 1 :
		ROS_INFO(" Teleop mode ON");
		a=1;
		break;
	case 0 :
		ROS_INFO("Autonomous Navigation");
		a=2;
		break;
	}
	*/
	if (ch==1 && a==0)
	{
		a=1;
	}
	else if (ch==0 && a==0)
	{
		a=0;
	}
	else if (ch==0 && a==1)
	{
		a=1;
	}
	else if (ch==1 && a==1)
	{
		a=0;
	}
	return a;

}

void RobotState::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	start_button= joy->buttons[9];
	//RobotState _monitor;
	//int ch=1*joy->buttons[8];
	clear_mode=joy->buttons[10];
	waypoint_store= joy->buttons[13];
	mode_button = 1*joy->axes[2];
	mode2_button = 2*joy->axes[3];
	mode = mode_button + mode2_button;
	if(clear_mode!=previous_clear_mode)
	{
		mode = 4;
		previous_clear_mode = clear_mode;
	}

	switch (mode)
	{
		case WAYPOINT:
			waypoint_state= 0;
			ROS_INFO(" waypoint");
			break;
		case WAYPOINT_NAVI:
			waypoint_state= 4;
			NaviWaypointService();
			ROS_INFO(" WAYPOINT_NAVI");
			break;
		case LOGISTICS:
			waypoint_state= 4;
			ROS_INFO(" LOGISTICS");
			break;
		case WALLFOLLOWER:
			waypoint_state= 4;
			WallfollowerService();
			ROS_INFO(" WALLFOLLOWER");
			break;
		case DEFAULT:
			ROS_INFO("DEFAULT");
			break;
	}
	switch(waypoint_state)
	{
		case 0 :
			//ROS_INFO(" Teleop mode ON");	
			if(start_button!=previous_start_button)
			{
				waypoint_state=1;
				previous_start_button= start_button;
				ROS_INFO("case0");

			}

			break;
		case 1 :
			
			if(start_button!=previous_start_button)
			{
				waypoint_state=2;
				previous_start_button= start_button;
			}
				ROS_INFO("case1");
			if(waypoint_store!= previous_waypoint_store)
			{
				ResetWaypointService();
				previous_waypoint_store=waypoint_store;
			}
			//TeleopVelocity(2);
			break;
		
		case 2 :
			
			if(waypoint_store!= previous_waypoint_store)
			{
				RunWaypointService();
				previous_waypoint_store=waypoint_store;
			}
			if(start_button!=previous_start_button)
			{
				waypoint_state=3;
				previous_start_button= start_button;
				//ROS_INFO(" going to case 0");
			}


			ROS_INFO("case2");
			break;
		
		case 3 :
			
			if(start_button!=previous_start_button)
			{
				waypoint_state=0;
				previous_start_button= start_button;
				ROS_INFO("  going to case 0");
			}
			if(waypoint_store!= previous_waypoint_store)
			{
				ResetWaypointService();
				previous_waypoint_store=waypoint_store;
			}
			ROS_INFO("  case3");
			//TeleopVelocity(2);
			break;

		case 4 :
			
			
			ROS_INFO("  case4 DEFAULT");
			//TeleopVelocity(2);
			break;

	}
	//return 0;
}

void RunWaypointService()
{	
	ros::NodeHandle n;
	ros::service::waitForService("get_pose");
	ros::ServiceClient client = n.serviceClient<wasp_joy::GetPose>("/get_pose");
	wasp_joy::GetPose srv;
	client.call(srv);

}
void ResetWaypointService()
{	
	ros::NodeHandle n;
	ros::service::waitForService("reset_pose");
	ros::ServiceClient client = n.serviceClient<wasp_joy::GetPose>("/reset_pose");
	wasp_joy::GetPose srv;
	client.call(srv);

}
void NaviWaypointService()
{	
	ros::NodeHandle n;
	ros::service::waitForService("navi_pose");
	ros::ServiceClient client = n.serviceClient<wasp_joy::GetPose>("/navi_pose");
	wasp_joy::GetPose srv;
	client.call(srv);

}
void WallfollowerService()
{	
	ros::NodeHandle n;
	ros::service::waitForService("wall_follow");
	ros::ServiceClient client = n.serviceClient<wasp_joy::GetPose>("/wall_follow");
	wasp_joy::GetPose srv;
	client.call(srv);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "activate_waypoint_read");
	ros::NodeHandle n;
	RobotState monitor;
	ros::Subscriber joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &RobotState::joyCallback, &monitor);
	ros::spin();
	//srv.request.name = atoll(argv[1]);
	return 0;
}
