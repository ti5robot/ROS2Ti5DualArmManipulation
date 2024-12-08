cmake_minimum_required(VERSION 3.8)
project(arms_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Ti5_Arms REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)  


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${moveit_msgs_INCLUDE_DIRS}
    ${moveit_ros_planning_interface_INCLUDE_DIRS}
    )



add_executable(arms_demo src/arms_demo.cpp)

ament_target_dependencies(arms_demo rclcpp Ti5_Arms std_msgs moveit_ros_planning_interface tf2_geometry_msgs moveit_msgs)

target_link_libraries(arms_demo 
	${catkin_LIBRARIES}
	controlcan
)

install(TARGETS arms_demo
	DESTINATION lib/${PROJECT_NAME})

