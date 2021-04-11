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
void LocalController(int flag_trolleydetected);
void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void callback(wasp_joy::testConfig &config, uint32_t level);
void NaviWaypoint(double pointx, double pointy, double pointz, double pointw);
sensor_msgs::LaserScan laser_msg;
float dist=0,right_min=0,left_min=0,x_r,x_l,y_r,y_l;
float pi =3.14;
float odom_posex,odom_start_pose;
//std::string footprint_string = "[[1.5, 1.5], [1.5, -1.5], [-1.5, -1.5], [-1.5, 1.5]]";
geometry_msgs::Twist command_velocity;
wasp_joy::logistics logistics_command;
/*dynamic_reconfigure::ReconfigureRequest srv_req;
dynamic_reconfigure::ReconfigureResponse srv_resp;
dynamic_reconfigure::StrParameter str_param;
dynamic_reconfigure::Config config;
str_param.name = "/move_base_node/local_costmap/footprint";
str_param.value = footprint_string;
config.strs.push_back(str_param);
*/

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
		lg.registerGoalCallback(boost::bind(&LogiAction::goalCB, this));
    	lg.registerPreemptCallback(boost::bind(&LogiAction::preemptCB, this));

    //subscribe to the data topic of interest
    	//sub_ = nh_.subscribe("/random_number", 1, &LogiAction::analysisCB, this);
		lg.start();
	}
	~LogiAction(void)
	{

	}
	void goalCB()
  {

    goal_ = lg.acceptNewGoal()->startpoint_x;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name.c_str());
    // set the action state to preempted
    lg.setPreempted();
  }
	void execteCB(const wasp_joy::LogisticsGoalConstPtr &goal)
	{
		ros::Rate r(1);
		bool success =true;

		feedback.sequence.clear();
		feedback.sequence.push_back(0);
		feedback.sequence.push_back(1);

		NaviWaypoint(goal->startpoint_x, goal->startpoint_y, goal->startpoint_z, goal->startpoint_w);
    	/*LocalController(0);
    	

    	dynamic_reconfigure::Server<wasp_joy::testConfig> server;
  		dynamic_reconfigure::Server<wasp_joy::testConfig>::CallbackType f;
  		f = boost::bind(&callback, _1, _2);
  		server.setCallback(f);

  		NaviWaypoint(goal->endpoint_x, goal->endpoint_y, goal->endpoint_z,goal->endpoint_w);
    	LocalController(3);
*/
    	//ros::Timer timer = n.createTimer(ros::Duration(15), timerliftCallback);
    	
       //}
		/*for(int i=1; i<=goal->order; i++)
		{
			if (lg.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name.c_str());
				lg.setPreempted();
				success = false;
				break;
			}
			feedback.sequence.push_back(feedback.sequence[i] + feedback.sequence[i-1]);
			lg.publishFeedback(feedback);
			r.sleep();

		}
		if(success)
		{
			result.sequence = feedback.sequence;
			ROS_INFO("%s: Succeeded", action_name.c_str());
			lg.setSucceeded(result);
		}
*/
	}

};

void NaviWaypoint(double pointx, double pointy, double pointz, double pointw)
{
		MoveBaseClient ac("move_base", true);

      	while(!ac.waitForServer(ros::Duration(5.0)))
      	{
        	ROS_INFO("Waiting for the move_base action server to come up");
      	}

      	
      	move_base_msgs::MoveBaseGoal local_goal;
      	local_goal.target_pose.header.frame_id = "base_link";
      	local_goal.target_pose.header.stamp = ros::Time::now();
      	//for( int i = 0; i < lenght0f_Array; i = i + 1 ) 
      	//{
    	local_goal.target_pose.pose.position.x = pointx;
    	local_goal.target_pose.pose.position.y = pointy;
    	local_goal.target_pose.pose.orientation.z = pointz;
    	local_goal.target_pose.pose.orientation.w = pointw;
  		ROS_INFO("Sending goal");
  		ac.sendGoal(local_goal);
  		ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        	ROS_INFO("reached startpoint switching to local controller state");

      else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
    
}

void callback(wasp_joy::testConfig &config, uint32_t level) 
{
  ROS_INFO("Reconfigure Request: %s", config.footprint.c_str());
}

void LocalController(int flag_trolleydetected)
{
	ros::Time::init();
	ros::Time begin;
	float odom_start_pose=0;

	ROS_INFO(": Sceeded");
		//if (dist>=1.5)
		if(flag_trolleydetected==0)
		{
			flag_trolleydetected=1;
		}
		else
		{
			flag_trolleydetected=0;
		}
	switch(flag_trolleydetected)
	{
		case 0:
			ROS_INFO("trolley not found");
			break;
		case 1:
			ROS_INFO("trolley detected");
			odom_start_pose= odom_posex;
			if((odom_posex-odom_start_pose)>0.5)
			{
				command_velocity.linear.x = 0.0;
				flag_trolleydetected = 2;
				//logistics_command.liftup = 1;
			}
			else
			{
				command_velocity.linear.x = 0.1;
				logistics_command.liftup = 0;

			}
			
			break;
		case 2:
			begin = ros::Time::now();
			if (ros::Time::now()- begin > ros::Duration(15.0))
			{
				logistics_command.liftup = 0;
			}
			else
			{
				logistics_command.liftup = 1;
			}
			break;
		
		case 3:
			ROS_INFO("trolley detected");
			odom_start_pose= odom_posex;
			if((odom_posex-odom_start_pose)>0.5)
			{
				command_velocity.linear.x = 0.0;
				flag_trolleydetected = 4;
				//logistics_command.liftup = 1;
			}
			else
			{
				command_velocity.linear.x = 0.1;
				logistics_command.liftdown = 0;

			}
			
			break;
			
		case 4:
			begin = ros::Time::now();
			if (ros::Time::now()- begin > ros::Duration(15.0))
			{
				logistics_command.liftdown = 0;
			}
			else
			{
				logistics_command.liftdown = 1;
			}
			break;
	}

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
    std::cout << "\nright= " << right_min;
    std::cout << "\nleft " << left_min;
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
   LogiAction logistics("logistics");
   ros::spin();
 
   return 0;
}