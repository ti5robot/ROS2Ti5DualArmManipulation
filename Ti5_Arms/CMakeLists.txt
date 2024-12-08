cmake_minimum_required(VERSION 3.8)
project(Ti5_Arms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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


include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${moveit_msgs_INCLUDE_DIRS}
    ${moveit_ros_planning_interface_INCLUDE_DIRS}
    )



add_library(Ti5_Arms SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/Ti5_Arms.cpp)

target_include_directories(Ti5_Arms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) ###


ament_target_dependencies(Ti5_Arms rclcpp std_msgs moveit_ros_planning_interface tf2_geometry_msgs moveit_msgs)


	
target_link_libraries(Ti5_Arms
	  ${catkin_LIBRARIES}
	  controlcan
	)


# 安装可执行文件
install(TARGETS 
      Ti5_Arms 
      LIBRARY DESTINATION lib)

install(DIRECTORY include/
	DESTINATION include)
	
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})


ament_package()
