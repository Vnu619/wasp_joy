#include "wasp_joy/joynode.h"

void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //previous_start_button=0;
  sensor_msgs::Joy newjoy;
  newjoy.buttons =joy->buttons;
  newjoy.axes =joy->axes;
  //ROS_INFO("  newjoy: [%ld]", (long int)newjoy.buttons[9]);
  start_button= joy->buttons[9];
  waypointstart_button= joy->buttons[8];
  command_velocity.linear.x  = 0.25*joy->axes[1] + 0.18*joy->axes[10];
  logistics_command.liftup  = 1*joy->buttons[1];
  command_velocity.linear.y  = 0.4*joy->axes[2];
  logistics_command.liftdown = 1*joy->buttons[3];
  logistics_command.extrude  = 1*joy->buttons[0];
  logistics_command.intrude  = 1*joy->buttons[2];
  command_velocity.angular.z = 0.1*joy->axes[0]+ 0.1*joy->axes[9];
  TeleopVelocity(2);
/*
switch(teleop_state)
	{
	case 0 :
		//ROS_INFO(" Teleop mode ON");	
		if(start_button!=previous_start_button)
		{
			teleop_state=1;
			previous_start_button= start_button;
			ROS_INFO("  case0");

		}
		break;
	case 1 :
		
		if(start_button!=previous_start_button)
		{
			teleop_state=2;
			previous_start_button= start_button;

		}
			ROS_INFO("  case1");

		//TeleopVelocity(2);
		break;
	
	case 2 :
		
		if(start_button!=previous_start_button)
		{
			teleop_state=3;
			previous_start_button= start_button;
			//ROS_INFO(" going to case 0");

		}
		ROS_INFO("  case2");
		TeleopVelocity(2);
		break;
	
	case 3 :
		
		if(start_button!=previous_start_button)
		{
			teleop_state=0;
			previous_start_button= start_button;
			ROS_INFO("  going to case 0");

		}
		ROS_INFO("  case3");
		//TeleopVelocity(2);
		break;

	}
	*/
	
   
}

void TeleopVelocity(int flag)
{
	ROS_INFO("  flag: [%ld]", (long int)flag);
	if (abs(command_velocity.linear.x)>=0.00000001 || abs(command_velocity.linear.y)>=0.00000001 || abs(command_velocity.angular.z)>=0.00000001)
{	
	//ros::Rate loop_rate(3.5);
	//while (ros::ok())
	//{
	teleop_cmd.publish(command_velocity);  
	//ros::spinOnce();
	//loop_rate.sleep();
	//}
	
}
	teleop_cmd2.publish(logistics_command);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "activate_waypoint_read");
	ros::NodeHandle n;
	ros::Subscriber joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &JoyCallback);
	teleop_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	teleop_cmd2 = n.advertise<wasp_joy::logistics>("/cmd_logi", 10);
	ros::spin();
	//srv.request.name = atoll(argv[1]);
	return 0;
}