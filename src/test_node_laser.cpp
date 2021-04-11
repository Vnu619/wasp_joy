#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>  
#include <cmath>
#include<stdio.h>

sensor_msgs::LaserScan laser_msg;

float pi =3.14;
geometry_msgs::Twist command_velocity;

class LaserTest
{
public:
//ros::NodeHandle nh;
	
	void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser);
	void Processing(float distance);
//ros::spin();
private:
float dist=0,right_min=0,left_min=0,x_r,x_l,y_r,y_l;
};

void LaserTest::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	LaserTest lt;
bool ActivateHolonomic = false,  ActivateRotation =false;
bool positioned = false;
	//ROS_INFO(": Succeeded");
	laser_msg = *laser;
    std::vector<float> laser_ranges,laser_filtered;
    std::vector<float>::iterator iter_minL,iter_minR, iter_minR_filter;
    laser_ranges = laser_msg.ranges;
    float range_right_min = laser_ranges[3];
    size_t range_size = laser_ranges.size();
    float laser_right,laser_left,laser_front;
    float range_min = laser_msg.range_max, range_max = laser_msg.range_min;
/*
    iter_minL= std::min_element(laser_ranges.begin()+180, laser_ranges.begin()+359);
    iter_minR= std::min_element(laser_ranges.begin()+360, laser_ranges.begin()+540);
    int indexL= std::distance(laser_ranges.begin(), iter_minL);
    int indexR = std::distance(laser_ranges.begin(), iter_minR);
    right_min= *iter_minR;
    left_min= *iter_minL;
   // std::cout<<"\nsize"<<range_size;

    float angle_r =((indexR-180) * 0.5);
    float angle_l = ((indexL-90)* 0.5);
    std::cout<<"\nangler"<<indexR;
    std::cout<<"\nanglel"<<indexL;
    std::cout<<"\niter_minl"<<left_min;    
    std::cout<<"\niter_minR"<<right_min;

    x_r=right_min*cos((180-angle_r)*180/pi);
    y_r=right_min*sin((180-angle_r)*180/pi);
    x_l=left_min*cos(angle_l*180/pi);
    y_l=left_min*sin(angle_l*180/pi);
    std::cout<<"\nx_r"<<x_r;
    std::cout<<"\ny_r"<<y_r;
    std::cout<<"\nx_l"<<x_l;    
    std::cout<<"\ny_l"<<y_l;
    //dist =1.6;
    dist=pow((x_r-x_l),2)+pow((y_r-y_l),2);
    dist = sqrt(dist);
	//ROS_INFO("right= %f", right_min);
    //std::cout << "\nright= " << right_min;
    //std::cout << "\nleft " << left_min;
    std::cout << "\ndistance" << dist;

*/
//angle_right= acos(right_min/fright_min)*(180/pi);
//angle_left= acos(left_min/fleft_min)*(180/pi);

//v.erase(std::remove(v.begin(), v.end(), 5), v.end());


/*
std::replace (laser_ranges.begin()+45, laser_ranges.begin()+200,0.0020000000949949026, 99.0);

    iter_minR= std::min_element(laser_ranges.begin()+45, laser_ranges.begin()+134);
	//if (*iter_minR_
    iter_minR_filter= std::lower_bound(iter_minR, laser_ranges.begin()+134, 0.01);

    int indexR = std::distance(laser_ranges.begin(), iter_minR_filter);

    iter_minL= std::min_element(laser_ranges.begin()+135, laser_ranges.begin()+200);
	
    int indexL= std::distance(laser_ranges.begin(), iter_minL);
*/

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
    std::cout<<"\nangler"<<angle_r;
    std::cout<<"\nanglel"<<angle_l;
    std::cout<<"\niter_minl"<<left_min;    
    std::cout<<"\niter_minR"<<right_min;

    x_r=right_min*cos((angle_r-35)*pi/180)+0.255;
    y_r=right_min*sin((angle_r-35)*pi/180);
    x_l=-left_min*cos((180-angle_l+35)*pi/180)+0.255;
    y_l=left_min*sin((180-angle_l+35)*pi/180);
    std::cout<<"\nx_r"<<x_r;
    std::cout<<"\ny_r"<<y_r;
    std::cout<<"\nx_l"<<x_l;    
    std::cout<<"\ny_l"<<y_l;

    //dist =1.6;
    dist=pow((x_r-x_l),2)+pow((y_r-y_l),2);
    dist = sqrt(dist);
	//ROS_INFO("right= %f", right_min);
    //std::cout << "\nright= " << right_min;
    //std::cout << "\nleft " << left_min;
    std::cout << "\ndistance" << dist;
    if (dist>=0.74)
{
ActivateHolonomic = false;
ActivateRotation = true;
}
if(ActivateRotation && !positioned && !ActivateHolonomic)
			{
				//ActivateHolonomic = true;
				if((y_l-y_r)>0.025)
				{
					std::cout<<"\n rotate right";
					//command_velocity.linear.y = -0.015;
					//feedback.align_distance = this->y_r;

				}
				else if((y_l - y_r)<-0.025)
				{
					std::cout<<"\n rotate left";
					//command_velocity.linear.y = 0.015;
					//feedback.align_distance = this->y_r;
				}
				else if((y_l - y_r)>=-0.025 && (y_l-y_r)<=0.025)
				{
					std::cout<<"\n orientation aligned";
					//command_velocity.linear.y = 0.015;
					//feedback.align_distance = this->y_r;
					ActivateHolonomic = true;
				}
				
			}
else if(ActivateHolonomic && !positioned)
{
		if((x_l + x_r)>0.05)
		{
			std::cout<<"\n move right";
			//command_velocity.linear.y = -0.015;
		}
		else if((x_l + x_r)<0.0)
		{
			std::cout<<"\n move left";
			//command_velocity.linear.y = 0.015;
		}
		else if((x_l + x_r)<=0.05 || (x_l + x_r)>=0.0)
		{
			std::cout<<"\n reached centre";
			command_velocity.linear.y = 0.00;
			ActivateHolonomic = false;
			positioned = true;
		}
}


}
void LaserTest::Processing(float distance)
{
	bool ActivateHolonomic = false;
	
	LaserTest test;
ROS_INFO("testing: %f",test.dist);
	if (distance>=0.74)
	{
		ActivateHolonomic = true;
	}
	std::cout<<"\nholonomic"<<ActivateHolonomic;
	while(ActivateHolonomic)
	{
		if((test.x_l - test.y_r)>0.10)
		{
			std::cout<<"\n move left";
			command_velocity.linear.y = 0.01;
		}
		else if((test.x_l - test.y_r)<-0.10)
		{
			std::cout<<"\n move left";
			command_velocity.linear.y = -0.01;
		}
		else if((test.x_l - test.y_r)<=0.10 || (test.x_l - test.y_r)>=-0.10)
		{
			std::cout<<"\n reached centre";
			command_velocity.linear.y = 0.0;
			ActivateHolonomic = false;
			break;
		}
	}

	//std::cout<<"hi";
//ros:spin();
}
int main(int argc, char** argv)
	{
	 ros::init(argc, argv, "logi_action");
	 //ros::Time::init();	
	 ros::NodeHandle nh;
ros::Rate loop_rate(2.5);
	 ROS_INFO("laser_node_test");
	 LaserTest testnode;
	 ros::Subscriber laserscan = nh.subscribe("/scan",10,&LaserTest::ScanCallback, &testnode);
	 ros::Publisher  teleop_cmd=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	 
	 
	 while(ros::ok())
	 {
		//testnode.Processing();
		teleop_cmd.publish(command_velocity);
	 	ros::spinOnce();
		loop_rate.sleep();
	 }
	 
	 //thread_b.join();
	 return 0;
	}
