#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <wasp_joy/LogisticsAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <wasp_joy/logistics.h>
#include "nav_msgs/Odometry.h"
#include <math.h>  
#include <cmath>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include<thread>
#include <tf/tf.h>

sensor_msgs::LaserScan laser_msg;
float dist=0,right_min=0,left_min=0,x_r,x_l,y_r,y_l, angle=1.5707;
float pi =3.14;
float odom_posex,odom_start_pose, odom_orientationz,odom_start_orientation;
bool output,m_ReturnValue;
//ros::NodeHandle nh;
wasp_joy::logistics logistics_command;
//logistics_command.liftup=0;
float begin;
float set;
int timer_flag=0;
ros::Time current_time, last_time;


//void pub_cmd_logi();
void TimerCallback(bool flag);
//void RosTimerCb();
class LogiAction
{
protected:

	ros::NodeHandle nh;
	actionlib::SimpleActionServer<wasp_joy::LogisticsAction> lg;
	std::string action_name;
	wasp_joy::LogisticsFeedback feedback;
	wasp_joy::LogisticsResult result;
	geometry_msgs::Twist command_velocity;
	ros::Subscriber sub_;
	ros::Publisher  teleop_cmd, teleop_cmd2;
	//geometry_msgs::Pose2D current_pose_;
	int goal_;
	int progress=1;
	int liftup , liftdown, state=0, odom_flag=0;
	bool linear_dist =false;


	//ros::Timer timer = nh.createTimer(ros::Duration(5), TimerCallback);

	//bool success =false;

public:
	double odom_posex, odom_orientationz;
	LogiAction(std::string name) :lg(nh,name, boost::bind(&LogiAction::execteCB, this, _1), false), action_name(name)
	{
		//lg.registerGoalCallback(boost::bind(&LogiAction::goalCB, this));
 	//ros::AsyncSpinner spinner(4);
    	
    	lg.registerPreemptCallback(boost::bind(&LogiAction::preemptCB, this));

    //subscribe to the data topic of interest
    	//sub_ = nh_.subscribe("/random_number", 1, &LogiAction::analysisCB, this);
		lg.start();
		
		ros::Subscriber laserscan = nh.subscribe("/scan",10,&LogiAction::ScanCallback, this);
	   	ros::Subscriber sub = nh.subscribe("/odom_ekf", 1000, &LogiAction::OdomCallback, this);
	   	teleop_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	   	teleop_cmd2 = nh.advertise<wasp_joy::logistics>("/cmd_logi", 10);
	   	ros::spin();
		//spinner.start();
	}
	~LogiAction(void)
	{

	}
	void preemptCB();
	void execteCB(const wasp_joy::LogisticsGoalConstPtr &goal);
	void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser);
	

};
/*
 void RosTimerCb()
 {
 	//ros::AsyncSpinner spinner(4);
 	ROS_INFO("ros_timer_started");
 	
	
	begin = ros::Time::now().toSec();
	std::cout<<"**************timer_began:************"<<begin;
	//ros::spin();
 	//return output;
	//spinner.start();
 }
*/
 void TimerCallback(bool flag)
 {
 	ROS_INFO("Timer Callback triggered");
 	if (flag)
 	{
 		
 		//logistics_command.liftup = 1.0;
 		std::cout<<"delay = "<<output;
 		ros::Duration(15).sleep();

		output =true;

 	}
 	else if (!flag)
 	{

		output =false;
 	}
 	 m_ReturnValue = output;
 	//return output;

 }
 /*
void pub_cmd_logi()
{
	ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
	teleop_cmd2 = nh.advertise<wasp_joy::logistics>("/cmd_logi", 10);
	ros::Rate loop(5);
	while (ros::ok())
	{
		teleop_cmd2.publish(logistics_command);
		loop.sleep();
	}	
}
*/
void LogiAction::preemptCB()
  {
    ROS_WARN("%s: Preempted", action_name.c_str());
    // set the action state to preempted
    lg.setPreempted();
  }


void LogiAction::execteCB(const wasp_joy::LogisticsGoalConstPtr &goal)
	{
		if(!lg.isActive() || lg.isPreemptRequested()) return;
		ros::Rate rate(5);
		//boost::thread thrd1(RosTimerCb);
		//thrd1.join();
		m_ReturnValue =false;
		
		bool time_set=false;
		feedback.distance = 1.6;
		bool success =false;
		//bool a =lg.setAccepted();
		ROS_INFO("%s is processing the goal %d & %d", action_name.c_str(), goal->liftup, goal->liftdown);
		
		while(goal->active)
		{

		feedback.liftup = goal->liftup;
		feedback.liftdown = goal->liftdown;
		dist = goal->distance +0.72;
		if(!ros::ok())
		{
	       result.final = progress;
	       lg.setAborted(result,"I failed !");
	       ROS_INFO("%s Shutting down",action_name.c_str());
	       break;

		}
		if(!lg.isActive() || lg.isPreemptRequested())
		{
       		return;
     		}
     	if (goal->liftup==1 && goal-> liftdown == 0)
     	{
     		
     		//ros::Duration(2).sleep();

 			
 			if (odom_flag==0)
     		{
     			ROS_INFO("odom_start_pose= %f", odom_start_pose);
     			odom_start_pose=this->odom_posex;

     			odom_flag=1;
     			

     		}

 			//ROS_INFO("Duration in seconds: %lf", begin.toSec());
 			ROS_INFO("trolley begine= %lf", odom_start_pose);
 			ROS_INFO("trolley now= %lf", this->odom_posex);
			if (abs(this->odom_posex- odom_start_pose) > dist)
			{
				command_velocity.linear.x = 0.0;
				//boost::thread TimerCallback(true);
				
				linear_dist =true;
			}
 			ROS_INFO("angle begin= %lf", odom_start_orientation);
 			ROS_INFO("angle now= %lf", this->odom_orientationz);
			if (linear_dist && (abs(this->odom_orientationz- odom_start_orientation) > angle) )
			{
				command_velocity.angular.z = 0.0;
				logistics_command.liftup = 1.0;
				if(timer_flag == 0)
				{
					last_time = ros::Time::now();
					timer_flag = 1;
				}
				std::cout<<"\nlast_time:"<<last_time.toSec();
				if(abs(ros::Time::now().toSec()-last_time.toSec())<10)
				{
					logistics_command.liftup = 1.0;
					std::cout<<"\ntimerstarted for 10 seconds";
					success =false;
				}
				else if(abs(ros::Time::now().toSec()-last_time.toSec())>=10)
				{
					logistics_command.liftup = 0.0;
					std::cout<<"\nlifting up finished 10 seconds";
					success =true;
					break;
				}


			//logistics_command.liftup = 1;
				
			}
			else if(linear_dist && (abs(this->odom_orientationz- odom_start_orientation) <= angle) )
			{
				command_velocity.angular.z = 0.04;
				logistics_command.liftup = 0;
			}
			else if(abs(this->odom_posex- odom_start_pose) <= dist)
			{
				command_velocity.linear.x = 0.015;
				logistics_command.liftup = 0;
				odom_start_orientation=this->odom_orientationz;

			}
			

     	}
     	if (goal->liftup==0 && goal->liftdown == 1)
     	{
     		ROS_INFO("trolley liftdown");
     		if (odom_flag==0)
     		{
     			ROS_INFO("odom_start_pose= %f", odom_start_pose);
     			odom_start_pose=this->odom_posex;

     			odom_flag=1;
     			

     		}

 			//ROS_INFO("Duration in seconds: %lf", begin.toSec());
 			//ROS_INFO("trolley begine= %lf", odom_start_pose);
 			//ROS_INFO("trolley now= %lf", this->odom_posex);
			if (abs(this->odom_posex- odom_start_pose) > 0.5)
			{
				command_velocity.linear.x = 0.0;
				//boost::thread thrd(TimerCallback(true));
				boost::thread thrd(TimerCallback, true);
				thrd.join();
				time_set = m_ReturnValue;
				if (!time_set)
				{
					logistics_command.liftdown = 1;
					success =false;
				}
				else if (time_set)
				{
					logistics_command.liftdown = 0;
					success =true;
				}

			//logistics_command.liftup = 1;
				break;
			}
			else if(abs(this->odom_posex- odom_start_pose) <= 0.5)
			{
				command_velocity.linear.x = 0.15;
				logistics_command.liftup = 0;

			}
     	}
     	else if (goal->liftup ==0 && goal->liftdown==0)
     	{
     		ROS_INFO("waiting to perform action");
     	}

     	lg.publishFeedback(feedback);
     	teleop_cmd.publish(command_velocity);
     	teleop_cmd2.publish(logistics_command);
   		//teleop_cmd2.publish(logistics_command);
     	rate.sleep();
     }
     if(success)
		{
			
			result.liftup_ = feedback.liftup;
			result.liftdown_ = feedback.liftdown;
			ROS_INFO("%s: Susscceeded", action_name.c_str());
			logistics_command.liftup = 0.0;
			logistics_command.liftdown = 0.0;
			teleop_cmd2.publish(logistics_command);
			lg.setSucceeded(result);
			success=false;
			time_set = false;
			m_ReturnValue = false;
			odom_flag = 0;
			timer_flag = 0;

		}

	}
void LogiAction::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	this->odom_posex=msg->pose.pose.position.x;
	
	tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll,pitch,yaw);
	this->odom_orientationz=yaw;

}
void LogiAction::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser)
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
    //dist =1.6;
    //dist=sqrt(pow((x_r-x_l),2)+pow((y_r-y_l),2));
	//ROS_INFO("right= %f", right_min);
    //std::cout << "\nright= " << right_min;
    //std::cout << "\nleft " << left_min;

//angle_right= acos(right_min/fright_min)*(180/pi);
//angle_left= acos(left_min/fleft_min)*(180/pi);

}
	int main(int argc, char** argv)
	{
	 ros::init(argc, argv, "logi_action");
	 ros::Time::init();	
	 //ros::NodeHandle nh;
	 //ros::MultiThreadedSpinner spinner(3);
    //ros::AsyncSpinner spinner(4);
	
	
	 ROS_INFO("Startinglogi Action Server");
	 //boost::thread thread_b(pub_cmd_logi);
	 current_time = ros::Time::now();
last_time = ros::Time::now();
	 LogiAction demo_action_obj(ros::this_node::getName());

	 //spinner.spin();
	 //spinner.start();
	 ros::spin();
	 //thread_b.join();
	 return 0;
	}

		//bool success =true;
		//feedback.distance = 1.6;

