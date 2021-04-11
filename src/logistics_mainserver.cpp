#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <wasp_joy/LogisticsAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <wasp_joy/testConfig.h>
#include <wasp_joy/logistics.h>
#include <math.h>  
#include <cmath>

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser);
void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
int LogisticsController(int liftup, int liftdown);

sensor_msgs::LaserScan laser_msg;
float dist=0,right_min=0,left_min=0,x_r,x_l,y_r,y_l;
float pi =3.14;
float odom_posex,odom_start_pose;
int liftup , liftdown, state=0, odom_flag=0;
bool success =false;
//std::string footprint_string = "[[1.5, 1.5], [1.5, -1.5], [-1.5, -1.5], [-1.5, 1.5]]";
geometry_msgs::Twist command_velocity;
wasp_joy::logistics logistics_command;
ros::Publisher  teleop_cmd, teleop_cmd2;


class LogiAction
{
protected:

	ros::NodeHandle nh;
	actionlib::SimpleActionServer<wasp_joy::LogisticsAction> lg;
	std::string action_name;
	wasp_joy::LogisticsFeedback feedback;
	wasp_joy::LogisticsResult result;
	ros::Subscriber sub_;
	int goal_;

public:
	LogiAction(std::string name) :lg(nh,name, boost::bind(&LogiAction::execteCB, this, _1), false), action_name(name)
	{
		//lg.registerGoalCallback(boost::bind(&LogiAction::goalCB, this));
    	//lg.registerPreemptCallback(boost::bind(&LogiAction::preemptCB, this));

    //subscribe to the data topic of interest
    	//sub_ = nh_.subscribe("/random_number", 1, &LogiAction::analysisCB, this);
		lg.start();

	}
	~LogiAction(void)
	{

	}
	/*void goalCB()
  {

    goal_ = lg.acceptNewGoal()->payload;

  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name.c_str());
    // set the action state to preempted
    lg.setPreempted();
  }
  */
	void execteCB(const wasp_joy::LogisticsGoalConstPtr &goal)
	{
		ros::Rate r(1);
		//bool success =true;
		feedback.distance = 1.6;
		feedback.liftup = goal->liftup;
		feedback.liftdown = goal->liftdown;
		liftup = goal->liftup;
		liftdown = goal->liftdown;
		ros::Time::init();
		ros::Time begin;
		ROS_INFO("%f: feedback", feedback.distance);
		int logi_state=LogisticsController(liftup, liftdown);

switch(logi_state)
	{
		case 0:
			ROS_INFO("do nothing");
			break;
		case 1:
			//ROS_INFO("liftup the trolley");
			
			
			ROS_INFO("trolley detected odom_start_pose: %f",odom_start_pose );
			ROS_INFO(" odom_posex: %f",odom_posex );
			if((odom_posex-odom_start_pose)>0.5)
			{
				command_velocity.linear.x = 0.0;
				state = 3;
				//logistics_command.liftup = 1;
			}
			else
			{
				command_velocity.linear.x = 0.5;
				logistics_command.liftup = 0;

			}
			teleop_cmd.publish(command_velocity);
			break;
		case 2:
			//ROS_INFO("no trolley_detected");
			ROS_INFO("no trolley_detected odom_start_pose: %f",odom_start_pose );
			ROS_INFO(" odom_posex: %f",odom_posex );
			if((odom_posex-odom_start_pose)>0.5)
			{
				command_velocity.linear.x = 0.0;
				state = 4;
				//logistics_command.liftup = 1;
			}
			else
			{
				command_velocity.linear.x = 0.5;
				logistics_command.liftup = 0;

			}
			teleop_cmd.publish(command_velocity);
			break;
		case 3:
			begin = ros::Time::now();
			if (ros::Time::now()- begin > ros::Duration(15.0))
			{
				logistics_command.liftup = 0;
				success = true;
			}
			else
			{
				logistics_command.liftup = 1;
			}
			teleop_cmd2.publish(logistics_command);
			break;
		case 4:
			begin = ros::Time::now();
			if (ros::Time::now()- begin > ros::Duration(15.0))
			{
				logistics_command.liftdown = 0;
				success = true;
			}
			else
			{
				logistics_command.liftdown = 1;
			}
			teleop_cmd2.publish(logistics_command);
			break;


	}
		if (lg.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name.c_str());
				lg.setPreempted();
				success = false;
				
			}
		lg.publishFeedback(feedback);
		r.sleep();

		if(success)
		{
			
			result.liftup_ = feedback.liftup;
			result.liftdown_ = feedback.liftdown;
			ROS_INFO("%s: Susscceeded", action_name.c_str());
			lg.setSucceeded(result);
		}	
		//lg.publishFeedback(feedback);
		//r.sleep();

	}

};
int LogisticsController(int liftup, int liftdown)
{

	float odom_start_pose=0;

	if (liftup==1)
	{
		if (odom_flag==0)
		{
			odom_start_pose= odom_posex;
			odom_flag = 1;
		}
		
		state = 1;
		ROS_INFO("%d: state", state);
	}
	else if (liftdown==1)
	{
		if (odom_flag==1)
		{
			odom_start_pose= odom_posex;
			odom_flag = 2;
		}
		state = 2;
		ROS_INFO("%d: state", state);
	}
	else if (liftup==0 && liftdown ==0)
	{
		state = 0;
		ROS_INFO("%d: state", state);
	}
	return state;

}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_posex=msg->pose.pose.position.x;
}
void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	//ROS_INFO(": Succeeded");
	laser_msg = *laser;
    std::vector<float> laser_ranges;
    std::vector<float>::iterator iter_minL,iter_minR;
    laser_ranges = laser_msg.ranges;
    float range_right_min = laser_ranges[3];
    size_t range_size = laser_ranges.size();
    float laser_right,laser_left,laser_front;
    float range_min = laser_msg.range_max, range_max = laser_msg.range_min;
    iter_minL= std::min_element(laser_ranges.begin(), laser_ranges.begin()+275);
    iter_minR= std::min_element(laser_ranges.begin()+276, laser_ranges.begin()+550);
    int indexL= std::distance(laser_ranges.begin(), iter_minL);
    int indexR = std::distance(laser_ranges.begin(), iter_minR);
    right_min= *iter_minR;
    left_min= *iter_minL;
    x_r=right_min*cos(indexR * 0.5*180/pi);
    y_r=right_min*sin(indexR * 0.5*180/pi);
    x_l=right_min*cos(indexL * 0.5*180/pi);
    y_l=right_min*sin(indexL * 0.5*180/pi);
    dist=sqrt(pow((x_r-x_l),2)+pow((y_r-y_l),2));
    //std::cout << "\nright= " << right_min;
    //std::cout << "\nleft " << left_min;
    // std::cout << "itermin4 " << left_min;
    // std::cout << "itermin5 " << fleft_min;
//angle_right= acos(right_min/fright_min)*(180/pi);
//angle_left= acos(left_min/fleft_min)*(180/pi);

}
int main(int argc, char** argv)
{
   ros::init(argc, argv, "logistics");
   ros::NodeHandle nh;
   ros::Subscriber laserscan = nh.subscribe("/scan",10,ScanCallback);
   ros::Subscriber sub = nh.subscribe("/odometry/filtered", 1000, OdomCallback);
   teleop_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
   teleop_cmd2 = nh.advertise<wasp_joy::logistics>("/cmd_logi", 10);
   //teleop_cmd.publish(command_velocity);
   //teleop_cmd2.publish(logistics_command);
   LogiAction logistics("logistics");
   ros::spin();
 
   return 0;
}