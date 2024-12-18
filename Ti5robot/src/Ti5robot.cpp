#include <rclcpp/rclcpp.hpp>
#include "Ti5robot.h"

double next_position[20] = {0};
double ori_position[20] = {0};
int now_position[20] = {0};

static const uint16_t crc16tab[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

using std::placeholders::_1;

namespace Ti5robot
{

    Ti5robot::Ti5robot(const std::string &name, const std::string &planning_group)
        : Node(name),
          move_group_(std::shared_ptr<rclcpp::Node>(this), planning_group)
    {

        // 初始化订阅者，监听规划路径的显示
        subscription_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "display_planned_path", 1000,
            std::bind(&Ti5robot::callback, this, std::placeholders::_1));

        move_group_.setGoalPositionTolerance(0.001);
        move_group_.setGoalOrientationTolerance(0.01);
        move_group_.setGoalJointTolerance(0.001);
        move_group_.setMaxAccelerationScalingFactor(0.5);
        move_group_.setMaxVelocityScalingFactor(0.5);
        move_group_.allowReplanning(true);
        move_group_.setPlanningTime(5.0);

        RCLCPP_INFO(this->get_logger(), "ArmsDemo initialized with planning group: %s", planning_group.c_str());
    }

    double Ti5robot::findMax(double arr[], int size)
    {
        double max = abs(arr[0]);
        for (int i = 1; i < size - 2; i++)
        {
            if (abs(arr[i]) > abs(max))
                max = abs(arr[i]);
        }
        return max;
    }

    bool Ti5robot::init_can()
    {
        int nDeviceType = 4;
        int nDeviceInd = 0;
        int nCANInd = 0;

        if (VCI_OpenDevice(nDeviceType, nDeviceInd, 0) != 1)
        {
            std::cout << "Open device failed!" << std::endl;
            return false;
        }
        else
            std::cout << "Open device success!" << std::endl;

        VCI_INIT_CONFIG vic;
        vic.AccCode = 0x80000008;
        vic.AccMask = 0xFFFFFFFF;
        vic.Filter = 0;
        vic.Timing0 = 0x00;
        vic.Timing1 = 0x14;
        vic.Mode = 0;

        if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &vic) != 1)
        {
            std::cout << "Init can failed!" << std::endl;
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            return false;
        }
        else
            std::cout << "Init can success!" << std::endl;

        if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
        {
            std::cout << "Start can failed!" << std::endl;
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            return false;
        }
        else
            std::cout << "Start can success!" << std::endl;

        return true;
    }

    int Ti5robot::init_udp(int port)
    {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock == -1)
            std::cerr << "Failed to create socket" << std::endl;

        sockaddr_in server_addr;
        std::memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(port);

        if (bind(sock, reinterpret_cast<struct sockaddr *>(&server_addr), sizeof(server_addr)) < 0)
        {
            std::cerr << "Failed to bind socket" << std::endl;
            close(sock);
        }
        std::cout << "UDP socketbound to port: " << port << std::endl;
        return sock;
    }

    bool Ti5robot::udp_read(int sock)
    {
        while (true)
        {
            std::string message;
            char buffer[1024];
            sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);

            ssize_t bytes_received = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addr_len);
            if (bytes_received == -1)
                std::cerr << "Failed to receive data" << std::endl;

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
            int client_port = ntohs(client_addr.sin_port);

            message.assign(buffer, bytes_received);
            std::cout << "receive message from  " << client_ip << " : " << client_port << " : " << message << std::endl;

            // for(int i=0;i<32;i++)
            //	message += buffer[i];
            // message = buffer;
            std::cout << "message:  " << message << std::endl;

            std::vector<uint8_t> bytes;
            for (size_t i = 0; i < message.size(); i += 2)
            {
                std::istringstream iss(message.substr(i, 2));
                int byte;
                iss >> std::hex >> byte;
                bytes.push_back(byte);
            }
            if (bytes[0] != 0xAA)
                std::cerr << "error occur!!!!!!" << std::endl;

            if (bytes[1] == 0xFF || bytes[1] == 0xff)
                break;

            if (bytes.size() != 16)
                std::cerr << "receive data length is not valid." << std::endl;

            int16_t z = static_cast<int16_t>((bytes[2] << 8) | bytes[3]);
            int16_t x = static_cast<int16_t>((bytes[4] << 8) | bytes[5]);
            int16_t c = static_cast<int16_t>((bytes[6] << 8) | bytes[7]);
            int16_t v = static_cast<int16_t>((bytes[8] << 8) | bytes[9]);
            int16_t b = static_cast<int16_t>((bytes[10] << 8) | bytes[11]);
            int16_t n = static_cast<int16_t>((bytes[12] << 8) | bytes[13]);
            uint16_t crc = static_cast<uint16_t>((bytes[14] << 8) | bytes[15]);

            if (crc == calc_crc(bytes, 14))
            {
                std::vector<double> res = {z / 10000.0, x / 10000.0, c / 10000.0, v / 10000.0, b / 10000.0, n / 10000.0};
                if (bytes[1] == 0x01)
                    move_by_pos(res);
                else if (bytes[1] == 0x00)
                    move_by_joint(res);
                else if (bytes[1] == 0xFF || bytes[1] == 0xff)
                    break;
                std::vector<int> r = get_error();
                std::string data = "";
                for (int num : r)
                    data += std::to_string(num);
                data += "  ";

                ssize_t bytes_send = sendto(sock, data.c_str(), data.length(), 0,
                                            reinterpret_cast<struct sockaddr *>(&client_addr), sizeof(client_addr));
                if (bytes_send == -1)
                    std::cerr << "Failed to send reply message" << std::endl;
                get_joint();
                get_pos();
            }
            else
                std::cout << "crc jiaoyan error,please check and send again aaaaaaaa" << std::endl;
        }
    }

    bool Ti5robot::init_serial(const std::string &port, int baudrate)
    {

        try
        {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);

            ser.open();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to open serial port: " << e.what() << std::endl;
        }

        if (ser.isOpen())
            std::cout << "serial port initialized." << std::endl;
        else
            std::cerr << "Failed to open serial port." << std::endl;
    }

    void Ti5robot::read_ser()
    {
        while (true)
        {
            std::string result;
            int cnt = 0;
            while (cnt < 16)
            {
                if (ser.available())
                {
                    result += ser.read(2);
                    cnt++;
                }
            }
            std::cout << "result:  " << result << std::endl;

            std::vector<uint8_t> bytes;
            for (size_t i = 0; i < result.size(); i += 2)
            {
                std::istringstream iss(result.substr(i, 2));
                int byte;
                iss >> std::hex >> byte;
                bytes.push_back(byte);
            }

            if (bytes[0] != 0xAA)
                std::cerr << "error occur!!!!!!" << std::endl;

            if (bytes[1] == 0xFF || bytes[1] == 0xff)
                break;

            if (bytes.size() != 16)
                std::cerr << "receive data length is not valid." << std::endl;

            int16_t z = static_cast<int16_t>((bytes[2] << 8) | bytes[3]);
            int16_t x = static_cast<int16_t>((bytes[4] << 8) | bytes[5]);
            int16_t c = static_cast<int16_t>((bytes[6] << 8) | bytes[7]);
            int16_t v = static_cast<int16_t>((bytes[8] << 8) | bytes[9]);
            int16_t b = static_cast<int16_t>((bytes[10] << 8) | bytes[11]);
            int16_t n = static_cast<int16_t>((bytes[12] << 8) | bytes[13]);
            uint16_t crc = static_cast<uint16_t>((bytes[14] << 8) | bytes[15]);

            if (crc == calc_crc(bytes, 14))
            {
                std::vector<double> res = {z / 10000.0, x / 10000.0, c / 10000.0, v / 10000.0, b / 10000.0, n / 10000.0};
                if (bytes[1] == 0x01)
                    move_by_pos(res);
                else if (bytes[1] == 0x00)
                    move_by_joint(res);
                else if (bytes[1] == 0xFF || bytes[1] == 0xff)
                    break;
                std::vector<int> r = get_error();
                std::string data = "";
                for (int num : r)
                    data += std::to_string(num);
                data += "  ";
                write_ser(data);
                get_joint();
                get_pos();
            }
            else
                std::cout << "crc jiaoyan error,please check and send again aaaaaaaa" << std::endl;
        }
    }

    bool Ti5robot::write_ser(std::string &data)
    {
        try
        {
            ser.write(data);
        }
        catch (const serial::SerialException &e)
        {
            std::cerr << "Failed write to port: " << e.what() << std::endl;
            return 0;
        }
        return 1;
    }

    uint16_t Ti5robot::calc_crc(const std::vector<uint8_t> &data, size_t length)
    {

        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; i++)
            crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ data[i]) & 0xFF];
        return crc;
    }

    int32_t Ti5robot::convertHexArrayToDecimal(const std::uint8_t hexArray[4])
    {
        std::int32_t result = 0;
        for (int i = 0; i < 4; i++)
            result = (result << 8) | hexArray[i];
        if (result > 0x7FFFFFFF)
            result -= 0x100000000;

        return result;
    }

    void Ti5robot::toIntArray(int number, int *res, int size)
    {
        unsigned int unsignedNumber = static_cast<unsigned int>(number);

        for (int i = 0; i < size; ++i)
        {
            res[i] = unsignedNumber & 0xFF;
            unsignedNumber >>= 8;
        }
    }

    std::vector<int> Ti5robot::sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList)
    {
        std::vector<int> res;
        VCI_CAN_OBJ send[1];
        send[0].SendType = 0;
        send[0].RemoteFlag = 0;
        send[0].ExternFlag = 0;
        send[0].DataLen = 1;
        for (int i = 0; i < numOfActuator; i++)
        {
            send[0].ID = i + 1;
            send[0].Data[0] = commandList[i];
            pthread_t threadid;
            if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
            {
                int cnt = 5;
                int reclen = 0, ind = 0;
                VCI_CAN_OBJ rec[3000];

                while ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) <= 0 && cnt)
                    cnt--;
                if (cnt == 0)
                    std::cout << "ops! ID " << send[0].ID << " failed after try 5 times.";
                else
                {
                    for (int j = 0; j < reclen; j++)
                    {
                        std::uint8_t hexArray[4] = {rec[j].Data[4], rec[j].Data[3], rec[j].Data[2], rec[j].Data[1]};
                        std::int32_t decimal = convertHexArrayToDecimal(hexArray);
                        res.push_back(decimal);
                        std::cout << "ID: " << send[0].ID << "       data: " << decimal << std::endl;
                    }
                }
            }
            else
                break;
        }
        return res;
    }

    std::vector<int> Ti5robot::sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList)
    {
        std::vector<int> res;
        VCI_CAN_OBJ send[1];
        send[0].SendType = 0;
        send[0].RemoteFlag = 0;
        send[0].ExternFlag = 0;
        send[0].DataLen = 5;
        for (int i = 0; i < numOfActuator; i++)
        {
            send[0].ID = i + 1;
            send[0].Data[0] = commandList[i];
            int res[4];
            toIntArray(parameterList[i], res, 4);
            for (int j = 1; j < 5; j++)
                send[0].Data[j] = res[j - 1];

            if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
            {
                for (int cnt = 0; cnt < send[0].DataLen; cnt++)
                    printf("");
                // printf(" %02X",send[0].Data[cnt]);
                // std::cout<<std::endl;
            }
            else
                break;
        }
        return res;
    }

    void Ti5robot::callback(const moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr &msg)
    {
        int n = msg->trajectory[0].joint_trajectory.points.size();
        double cn[10] = {0}, ori_position[10] = {0}, next_position[10] = {0}, now_position[10] = {0}, maxVal = 0, status = 1;
        int maxSpeed[10] = {0};
        uint8_t canidlist[10] = {1, 2, 3, 4, 5, 6}, cmd_pos[10] = {30, 30, 30, 30, 30, 30};
        uint8_t cmd_v_p[10] = {36, 36, 36, 36, 36, 36}, cmd_v_n[10] = {37, 37, 37, 37, 37, 37};
        uint32_t para_pos[10] = {0};

        int temp_p = 100, temp_n = -100;

        uint32_t para_v_p[10] = {temp_p, temp_p, temp_p, temp_p, temp_p, temp_p}, para_v_n[10] = {temp_n, temp_n, temp_n, temp_n, temp_n, temp_n};
        sendCanCommand(6, canidlist, cmd_v_p, para_v_p);
        usleep(100);
        sendCanCommand(6, canidlist, cmd_v_n, para_v_n);
        usleep(100);

        for (int cnt = 0; cnt < 6; cnt++)
        {
            ori_position[cnt] = msg->trajectory[0].joint_trajectory.points[0].positions[cnt] * 57.3;
            next_position[cnt] = msg->trajectory[0].joint_trajectory.points[n - 1].positions[cnt] * 57.3;
            now_position[cnt] = msg->trajectory[0].joint_trajectory.points[n - 1].positions[cnt] * 57.3 * 101 * 65536 / 360;

            cn[cnt] = next_position[cnt] - ori_position[cnt];
            maxVal = maxVal < fabs(cn[cnt]) ? fabs(cn[cnt]) : maxVal;
        }
        para_pos[0] = static_cast<uint32_t>(int(now_position[0]));
        para_pos[1] = static_cast<uint32_t>(int(now_position[1]));
        para_pos[2] = static_cast<uint32_t>(int(now_position[2]));
        para_pos[3] = static_cast<uint32_t>(int(now_position[3]));
        para_pos[4] = static_cast<uint32_t>(int(now_position[4]));
        para_pos[5] = static_cast<uint32_t>(int(now_position[5]));

        double cala_t = maxVal / 5 / 35.6;
        if (t <= cala_t)
            time_flag = false;

        double periodTime;
        if (time_flag == true)
        {
            for (int cnt = 0; cnt < 6; cnt++)
                // maxSpeed[cnt] = abs((cn[cnt] / double(t)*0.8) * 10100 / 360);
                maxSpeed[cnt] = abs((cn[cnt] / t) * 10100 / 360);
            periodTime = t;
        }
        else
        {
            double periodTime = maxVal / v;
            if (periodTime < 0.2)
                periodTime *= 3;
            else if (periodTime < 0.6)
                periodTime *= 2;

            for (int cnt = 0; cnt < 6; cnt++)
                maxSpeed[cnt] = abs((cn[cnt] / periodTime) * 10100 / 360);
        }

        sendCanCommand(6, canidlist, cmd_pos, para_pos);

        for (int j = 10; j < 105; j++)
        {
            double acceleration_ratio = (j - 5) / 100.0;
            int tmp02[10] = {0};
            for (int cnt = 0; cnt < 6; cnt++)
                tmp02[cnt] = acceleration_ratio * maxSpeed[cnt];

            int tmp12[6] = {-tmp02[0], -tmp02[1], -tmp02[2], -tmp02[3], -tmp02[4], -tmp02[5]};

            uint32_t para_v_p[10] = {tmp02[0], tmp02[1], tmp02[2], tmp02[3], tmp02[4], tmp02[5]};
            uint32_t para_v_n[10] = {tmp12[0], tmp12[1], tmp12[2], tmp12[3], tmp12[4], tmp12[5]};

            sendCanCommand(6, canidlist, cmd_v_p, para_v_p);
            usleep(10);
            sendCanCommand(6, canidlist, cmd_v_n, para_v_n);
            usleep(10);
            usleep(5000 * status);
        }

        if (periodTime >= 0.55)
            usleep(periodTime * 1000000 - 550000 * status);
        //	maxspeed go
        // usleep(300000);

        for (int j = 105; j > 20; j--)
        {
            double deceleration_ratio = (105 - j) / 100.0;
            int tmp02[10] = {0};
            for (int cnt = 0; cnt < 6; cnt++)
                tmp02[cnt] = j * maxSpeed[cnt] / 100;

            int tmp12[6] = {-tmp02[0], -tmp02[1], -tmp02[2], -tmp02[3], -tmp02[4], -tmp02[5]};

            uint32_t para_v_p[10] = {tmp02[0], tmp02[1], tmp02[2], tmp02[3], tmp02[4], tmp02[5]};
            uint32_t para_v_n[10] = {tmp12[0], tmp12[1], tmp12[2], tmp12[3], tmp12[4], tmp12[5]};

            sendCanCommand(6, canidlist, cmd_v_p, para_v_p);
            usleep(10);
            sendCanCommand(6, canidlist, cmd_v_n, para_v_n);
            usleep(10);
            usleep(5000 * status * deceleration_ratio);
        }
    }

    bool Ti5robot::move_by_joint(const std::vector<double> &joint_group_positions)
    {
        std::vector<double> current_joint_values = this->move_group_.getCurrentJointValues();
        for (int i = 0; i < joint_group_positions.size(); i++)
            current_joint_values[i] += joint_group_positions[i];

        move_group_.setJointValueTarget(current_joint_values);
        move_group_.move();
        // sleep(3);
        return true;
    }

    bool Ti5robot::move_by_pos(const std::vector<double> &pose)
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = pose[0];
        target_pose.position.y = pose[1];
        target_pose.position.z = pose[2];

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(pose[3], pose[4], pose[5]);
        target_pose.orientation.x = myQuaternion.getX();
        target_pose.orientation.y = myQuaternion.getY();
        target_pose.orientation.z = myQuaternion.getZ();
        target_pose.orientation.w = myQuaternion.getW();

        move_group_.setStartStateToCurrentState();
        move_group_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = move_group_.plan(plan);
        RCLCPP_INFO(this->get_logger(), "move_p:%s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            move_group_.execute(plan);
            // sleep(3);
            return true;
        }
        return false;
    }

    void Ti5robot::get_pos()
    {
        geometry_msgs::msg::PoseStamped current_pose = this->move_group_.getCurrentPose(this->end_effector_link);

        RCLCPP_INFO(this->get_logger(), "current pose:x:%f,y:%f,z:%f,Roll:%f,Pitch:%f,Yaw:%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);

        std::vector<double> rpy = this->move_group_.getCurrentRPY(this->end_effector_link);
        RCLCPP_INFO(this->get_logger(), "current rpy:%f,%f,%f", rpy[0], rpy[1], rpy[2]);
    }

    void Ti5robot::get_joint()
    {
        std::vector<double> current_joint_values = this->move_group_.getCurrentJointValues();
        RCLCPP_INFO(this->get_logger(), "current joint values:%f,%f,%f,%f,%f,%f", current_joint_values[0], current_joint_values[1], current_joint_values[2],
                    current_joint_values[3], current_joint_values[4], current_joint_values[5]);
    }

    char Ti5robot::get_key()
    {
        struct termios stored_settings, new_settings;
        tcgetattr(0, &stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VMIN] = 1;
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(0, &stored_settings);
        tcsetattr(0, TCSANOW, &new_settings);
        char in = getchar();
        tcsetattr(0, TCSANOW, &stored_settings);
        return in;
    }

    std::vector<int> Ti5robot::get_elec_pos()
    {
        uint8_t canidlist[10] = {1, 2, 3, 4, 5, 6}, cmd[10] = {8, 8, 8, 8, 8, 8};
        std::vector<int> res;
        res = sendCanCommand(6, canidlist, cmd);
        return res;
    }

    void Ti5robot::clean_error()
    {
        uint8_t canidlist[10] = {1, 2, 3, 4, 5, 6}, cmd[10] = {11, 11, 11, 11, 11, 11};
        sendCanCommand(6, canidlist, cmd);
    }

    std::vector<int> Ti5robot::get_error()
    {
        std::vector<int> res;
        uint8_t canidlist[10] = {1, 2, 3, 4, 5, 6}, cmd[10] = {10, 10, 10, 10, 10, 10};
        res = sendCanCommand(6, canidlist, cmd);
        return res;
    }

    std::vector<int> Ti5robot::get_electric()
    {
        uint8_t canidlist[10] = {1, 2, 3, 4, 5, 6}, cmd[10] = {4, 4, 4, 4, 4, 4};
        sendCanCommand(6, canidlist, cmd);
    }

    void Ti5robot::change_v(int v_)
    {
        this->v = v_ / 1000 * 35.6;
    }

    bool Ti5robot::move_joint(const std::vector<double> &joint_group_positions)
    {
        std::vector<double> current_joint_values = this->move_group_.getCurrentJointValues();
        for (int i = 0; i < joint_group_positions.size(); i++)
            current_joint_values[i] += joint_group_positions[i];

        move_group_.setJointValueTarget(current_joint_values);
        move_group_.move();
        // sleep(3);
        return true;
    }

    bool Ti5robot::test_joint(const std::vector<double> &pose)
    {
        move_by_pos(pose);

        std::vector<int> res;
        res = get_elec_pos();

        std::vector<double> current_joint_values = this->move_group_.getCurrentJointValues();
        std::vector<double> jj;

        bool f = 1;
        for (int i = 0; i < res.size(); i++)
        {
            jj.push_back((double)res[i] / 100 / 65536 * 2 * M_PI);
            std::cout << "i: " << i << "  elec_joint: " << jj[i] << "  rviz_joint:  " << current_joint_values[i] << "           " << fabs(jj[i] - current_joint_values[i]) << std::endl;
            if (fabs(jj[i] - current_joint_values[i]) > 0.1)
                f = 0;
        }
        return f;
    }

    void Ti5robot::move_up()
    {
        int tmp0 = 500, tmp1 = -500;
        uint32_t para_v_p[6] = {tmp0, tmp0, tmp0, tmp0, tmp0, tmp0}, para_v_n[6] = {tmp1, tmp1, tmp1, tmp1, tmp1, tmp1};
        uint8_t canidlist[6] = {1, 2, 3, 4, 5, 6}, cmd_v_p[6] = {36, 36, 36, 36, 36, 36}, cmd_v_n[6] = {37, 37, 37, 37, 37, 37}, cmd_pos[6] = {30, 30, 30, 30, 30, 30};

        sendCanCommand(6, canidlist, cmd_v_p, para_v_p);
        usleep(50);
        sendCanCommand(6, canidlist, cmd_v_n, para_v_n);
        usleep(50);

        uint32_t para_pos[6] = {0, 0, 0, 0, 0, 0};
        sendCanCommand(6, canidlist, cmd_pos, para_pos);
    }

    void Ti5robot::move_joint_in_time(const std::vector<double> &joint, int time)
    {
        this->time_flag = true;
        this->t = time;
        move_group_.setJointValueTarget(joint);
        move_group_.move();
        this->time_flag = false;
    }

    void Ti5robot::move_pos_in_time(const std::vector<double> &pose, int time)
    {
        this->time_flag = true;
        this->t = time;
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = pose[0];
        target_pose.position.y = pose[1];
        target_pose.position.z = pose[2];

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(pose[3], pose[4], pose[5]);
        target_pose.orientation.x = myQuaternion.getX();
        target_pose.orientation.y = myQuaternion.getY();
        target_pose.orientation.z = myQuaternion.getZ();
        target_pose.orientation.w = myQuaternion.getW();

        move_group_.setStartStateToCurrentState();
        move_group_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = move_group_.plan(plan);
        RCLCPP_INFO(this->get_logger(), "move_p:%s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            move_group_.execute(plan);
            // sleep(3);
        }
        this->time_flag = false;
    }

}
