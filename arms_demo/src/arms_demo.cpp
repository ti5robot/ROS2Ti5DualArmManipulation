#include <rclcpp/rclcpp.hpp>
#include "Ti5_Arms/Ti5_Arms.h"


int main(int argc,char** argv)
{
	rclcpp::init(argc,argv);
	rclcpp::Rate loop_rate(500);
	rclcpp::Rate loop_rate_slow(2);
	
	auto node = std::make_shared<arms::ArmsDemo>("arms_demo","arm_L","arm_R");

	RCLCPP_INFO(node->get_logger(), "Hello, ROS 2!");
	node->move_up();
	//std::vector<double> LL = {0.5, 0.1, 0.3, 0.5, 0.3, 0.2, 0.3};
	// std::vector<double> LL = {0.0689,0.6335,0.1913,0.9706,0.3198,0.3198};
	// node->move_L_by_joint(LL);
	// node->move_L_by_pos(LL);
	// node->get_L_joint();
	// node->move_R_by_pos({0.1196,-0.626,-0.0539,0.8728,-1.156,-0.0639});
	// node->get_R_joint();
	 while(rclcpp::ok())
	{
    	//RCLCPP_INFO_STREAM(node->get_logger(), "aaaaaaaaaaaaa");
    	loop_rate.sleep();     
    }

}

