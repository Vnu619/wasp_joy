#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <wasp_joy/TrolleyAlignAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>  
#include <cmath>
#include <stdio.h>

sensor_msgs::LaserScan laser_msg;

float pi =3.14;
geometry_msgs::Twist command_velocity;

class LaserTest
{
protected:
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<wasp_joy::TrolleyAlignAction> ta;
	std::string action_name;
	wasp_joy::TrolleyAlignFeedback feedback;
	wasp_joy::TrolleyAlignResult result;
	ros::Publisher  teleop_cmd;
	//geometry_msgs::Twist command_velocity;

public:
//ros::NodeHandle nh;
	LaserTest(std::string name) :ta(nh,name,boost::bind(&LaserTest::execteCB, this, _1), false), action_name(name)
	{
		//ta.registerGoalCallback(boost::bind(&LaserTest::goalCB, this));
 	//ros::AsyncSpinner spinner(4);
    	
    	ta.registerPreemptCallback(boost::bind(&LaserTest::preemptCB, this));

    //subscribe to the data topic of interest
    	//sub_ = nh_.subscribe("/random_number", 1, &LogiAction::analysisCB, this);
		ta.start();
		ros::Subscriber laserscan = nh.subscribe("/scan",10,&LaserTest::ScanCallback, this);
	 	teleop_cmd=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		//ros::spin();
		//ros::Rate loop_rate(2.5);
		//while(ros::ok())
	 //{
		//testnode.Processing();
		
	 	ros::spin();
		//loop_rate.sleep();
	 //}
	}
	~LaserTest(void)
	{

	}
	void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser);
	void execteCB(const wasp_joy::TrolleyAlignGoalConstPtr &goal);
	void preemptCB();
//ros::spin();
//private:
float dist=0,right_min=0,left_min=0,x_r,x_l,y_r,y_l;
int goal_;
bool ActivateHolonomic = false;
bool positioned = false;
};


void LaserTest::preemptCB()
  {
    ROS_WARN("%s: Preempted", action_name.c_str());
    // set the action state to preempted
    ta.setPreempted();
  }
void LaserTest::execteCB(const wasp_joy::TrolleyAlignGoalConstPtr &goal)
	{
		if(!ta.isActive() || ta.isPreemptRequested()) return;
		
		ros::Rate rate(5);
		ROS_INFO("%s is processing the goal ", action_name.c_str());
		while(goal->align)
		{
			if(!ros::ok())
			{
		       result.success = false;
		       ta.setAborted(result,"I failed !");
		       ROS_INFO("%s Shutting down",action_name.c_str());
		       break;
			}
			if(!ta.isActive() || ta.isPreemptRequested())
			{
	       		return;
	     	}
	     	if (this->dist>=0.74)
			{

				ActivateHolonomic = true;
			}
			if(ActivateHolonomic && !positioned)
			{
				if((this->x_l + this->x_r)>0.05)
				{
					std::cout<<"\n move right";
					command_velocity.linear.y = -0.015;
					feedback.align_distance = dist;
					ta.publishFeedback(feedback);
					//
					//if (dist>0.5)
					//{
					//ActivateHolonomic = false;
					//result.success = true;
					//positioned = true;
					//break;
					//}
				}
				else if((this->x_l + this->x_r)<0.0)
				{
					std::cout<<"\n move left";
					command_velocity.linear.y = 0.015;
				}
				else if((this->x_l + this->x_r)<=0.05 || (this->x_l + this->x_r)>=0.0)
				{
					std::cout<<"\n reached centre";
					command_velocity.linear.y = 0.00;
					ActivateHolonomic = false;
					result.success = true;
					positioned = true;
					break;
				}
			}
			teleop_cmd.publish(command_velocity);
			//ta.publishFeedback(feedback);
		}
	}
void LaserTest::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	//LaserTest lt;
	//bool ActivateHolonomic = false;
	//bool positioned = false;
	//ROS_INFO(": Succeeded");
	laser_msg = *laser;
    std::vector<float> laser_ranges,laser_filtered;
    std::vector<float>::iterator iter_minL,iter_minR, iter_minR_filter;
    laser_ranges = laser_msg.ranges;
    float range_right_min = laser_ranges[3];
    size_t range_size = laser_ranges.size();
    float laser_right,laser_left,laser_front;
    float range_min = laser_msg.range_max, range_max = laser_msg.range_min;

	std::replace (laser_ranges.begin()+45, laser_ranges.begin()+255,0.0020000000949949026, 99.0);

    iter_minR= std::min_element(laser_ranges.begin()+45, laser_ranges.begin()+154);

    int indexR = std::distance(laser_ranges.begin(), iter_minR);

    iter_minL= std::min_element(laser_ranges.begin()+175, laser_ranges.begin()+255);
	
    int indexL= std::distance(laser_ranges.begin(), iter_minL);

    right_min= *iter_minR;
    left_min= *iter_minL;
   // std::cout<<"\nsize"<<range_size;

    float angle_r =((indexR-45));
    float angle_l = ((indexL-45));
    /*
    std::cout<<"\nangler"<<angle_r;
    std::cout<<"\nanglel"<<angle_l;
    std::cout<<"\niter_minl"<<left_min;    
    std::cout<<"\niter_minR"<<right_min;
*/
    x_r=right_min*cos((angle_r-35)*pi/180)+0.255;
    y_r=right_min*sin((angle_r-35)*pi/180);
    x_l=-left_min*cos((180-angle_l+35)*pi/180)+0.255;
    y_l=left_min*sin((180-angle_l+35)*pi/180);
    //std::cout<<"\nx_r"<<x_r;
  //  std::cout<<"\ny_r"<<y_r;
    //std::cout<<"\nx_l"<<x_l;    
  //  std::cout<<"\ny_l"<<y_l;

    //dist =1.6;
    dist=pow((x_r-x_l),2)+pow((y_r-y_l),2);
    dist = sqrt(dist);
	//ROS_INFO("right= %f", right_min);
    //std::cout << "\nright= " << right_min;
    //std::cout << "\nleft " << left_min;
    //std::cout << "\ndistance" << dist;

}

int main(int argc, char** argv)
	{
	 ros::init(argc, argv, "trolleyalign_action");
	 //ros::Time::init();	
	 ros::NodeHandle nh;
	 
	 ROS_INFO("laser_node_test");
	 LaserTest testnode("trolleyalign_action");
	 ros::spin();
	 //thread_b.join();
	 return 0;
	}
