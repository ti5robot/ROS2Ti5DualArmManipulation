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

#include <string>
#include <termio.h>
#include <csignal>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <serial/serial.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <typeinfo>

namespace Ti5robot
{
    class Ti5robot : public rclcpp::Node
    {
    public:
        Ti5robot(const std::string &name, const std::string &planning_group);

        double findMax(double arr[], int size);
        bool init_can();
        bool init_serial(const std::string &port, int baudrate);
        void read_ser();
        int init_udp(int port);
        bool udp_read(int sock);
        bool write_ser(std::string &data);
        uint16_t calc_crc(const std::vector<uint8_t> &data, size_t length);

        int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4]);
        void toIntArray(int number, int *res, int size);
        std::vector<int> sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList);
        std::vector<int> sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList);
        void callback(const moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr &msg);

        bool move_by_joint(const std::vector<double> &joint_group_positions);
        bool move_by_pos(const std::vector<double> &pose);
        void get_pos();
        void get_joint();

        char get_key();
        std::vector<int> get_elec_pos();
        void clean_error();
        std::vector<int> get_error();
        std::vector<int> get_electric();
        void change_v(int v_);


        bool move_joint(const std::vector<double> &joint_group_positions);
        bool test_joint(const std::vector<double> &pose);
        void move_up();
        void move_joint_in_time(const std::vector<double> &joint,int time);
        void move_pos_in_time(const std::vector<double> &pose,int time);

    private:
        std::atomic<bool> running{true};
        int v = 130;
        bool time_flag = false;
        int t = 0;
        serial::Serial ser;

        moveit::planning_interface::MoveGroupInterface move_group_;

        std::string planning_group_;
        std::string end_effector_link;
        rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_;
    };
}

#endif
