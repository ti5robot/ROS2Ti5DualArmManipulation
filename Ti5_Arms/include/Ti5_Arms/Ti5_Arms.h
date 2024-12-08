#ifndef ARMS_DEMO_H
#define ARMS_DEMO_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h> 
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include "controlcan.h"

namespace arms
{
    class ArmsDemo : public rclcpp::Node
    {
    public:
        ArmsDemo(const std::string &name, const std::string &planning_group_1,const std::string &planning_group_2);

        double findMax(double arr[], int size);
        bool init_can();
        int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4]);
        void toIntArray(int number, int *res, int size);
        void sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList,int CANInd);
        void sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList,int CANInd);
        void callback(const moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr &msg);
        bool move_L_by_joint(const std::vector<double> &joint_group_positions);
        bool move_R_by_joint(const std::vector<double> &joint_group_positions);
        bool move_L_by_pos(const std::vector<double> &pose);
        bool move_R_by_pos(const std::vector<double> &pose);
        void get_L_pos();
        void get_R_pos();
        void get_L_joint();
        void get_R_joint();
        void move_up();
        

    private:
        moveit::planning_interface::MoveGroupInterface move_group_1_;
        moveit::planning_interface::MoveGroupInterface move_group_2_;

	std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // Single-threaded executor
	std::thread executor_thread_;  // Thread to run the executor


        std::string planning_group_;
        std::string end_effector_link;
        rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_;
    };
}

#endif
