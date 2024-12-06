#ifndef TI5ROBOT_H
#define TI5ROBOT_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>  
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include "controlcan.h"

namespace Ti5robot
{
    class Ti5robot : public rclcpp::Node
    {
    public:
        Ti5robot(const std::string &name, const std::string &planning_group);

        double findMax(double arr[], int size);
        bool init_can();
        int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4]);
        void toIntArray(int number, int *res, int size);
        void sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList);
        void sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList);
        void callback(const moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr &msg);
        
        bool move_by_joint(const std::vector<double> &joint_group_positions);
        bool move_by_pos(const std::vector<double> &pose);
        void get_pos();
        void get_joint();
        

    private:
        moveit::planning_interface::MoveGroupInterface move_group_;

        std::string planning_group_;
        std::string end_effector_link;
        rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_;
    };
}

#endif
