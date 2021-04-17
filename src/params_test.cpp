#include <ros/ros.h>
#include <ros/service.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <dynamic_reconfigure/server.h>
         dynamic_reconfigure::ReconfigureRequest srv_req;
         dynamic_reconfigure::ReconfigureResponse srv_resp;
         dynamic_reconfigure::StrParameter str_param;
         dynamic_reconfigure::Config conf;
std::string footprint_string = "[[-0.78, -0.38], [-0.78, 0.38], [0.78, 0.38], [0.78, -0.38]]";
//std::string footprint_string =
std::string min_range ="0.45";

int main(int argc, char** argv)
	{
	 ros::init(argc, argv, "logi_param");


	 costmap_2d::Costmap2DConfig cfg;
	 cfg.footprint = footprint_string;
	 str_param.name = "footprint";
	 str_param.value = footprint_string;
	 conf.strs.push_back(str_param);
	 srv_req.config = conf;
	 ros::service::waitForService("/move_base/global_costmap/set_parameters");
	 ros::service::call("/move_base/global_costmap/set_parameters", srv_req, srv_resp);
	 //ros::param::set("/move_base/local_costmap/footprint", footprint_string);
	 //ros::param::set("/move_base/global_costmap/footprint", footprint_string);
	 //ros::param::set("/laserscan_multi_merger/range_min", min_range);
 	 return 0;
	}




