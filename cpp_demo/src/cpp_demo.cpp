#include <rclcpp/rclcpp.hpp>
#include "Ti5robot/Ti5robot.h"

int main()
{
	rclcpp::init(argc,argv);
	auto node = std::make_shared<Ti5robot::Ti5robot>("cpp_demo","armgroup");

	node->move_by_joint({0.2, 0.3, 0.4, 0.5, 0.6, 0.7});
	node->get_pos();

	rclcpp::spin(node);
	rclcpp::shutdown();
}
