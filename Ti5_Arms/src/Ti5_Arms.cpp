#include <rclcpp/rclcpp.hpp>
#include "Ti5_Arms.h"

double next_position[20] = {0};
double ori_position[20] = {0};
int now_position[20] = {0};

using std::placeholders::_1;

namespace arms
{

    ArmsDemo::ArmsDemo(const std::string &name, const std::string &planning_group_1,const std::string &planning_group_2)
        : Node(name), 
        move_group_1_(std::shared_ptr<rclcpp::Node>(this), planning_group_1),
        move_group_2_(std::shared_ptr<rclcpp::Node>(this), planning_group_2),
        executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {
    
    	// Add the node to the executor and start the executor thread
    executor_->add_node(this->shared_from_this());
    executor_thread_ = std::thread([this]() {
    RCLCPP_INFO(this->get_logger(), "Starting executor thread"); // Log message indicating the thread start
    executor_->spin(); // Run the executor to process callbacks
    });

  

        // 初始化订阅者，监听规划路径的显示
        subscription_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "display_planned_path", 1000,
            std::bind(&ArmsDemo::callback, this, std::placeholders::_1));
        
        move_group_1_.setGoalPositionTolerance(0.001);
        move_group_1_.setGoalOrientationTolerance(0.01);
        move_group_1_.setGoalJointTolerance(0.001);
        move_group_1_.setMaxAccelerationScalingFactor(0.5);
        move_group_1_.setMaxVelocityScalingFactor(0.5);
        move_group_1_.allowReplanning(true);
        move_group_1_.setPlanningTime(5.0);
        
        move_group_2_.setGoalPositionTolerance(0.001);
        move_group_2_.setGoalOrientationTolerance(0.01);
        move_group_2_.setGoalJointTolerance(0.001);
        move_group_2_.setMaxAccelerationScalingFactor(0.5);
        move_group_2_.setMaxVelocityScalingFactor(0.5);
        move_group_2_.allowReplanning(true);
        move_group_2_.setPlanningTime(5.0);

        RCLCPP_INFO(this->get_logger(), "ArmsDemo initialized with planning group: %s", planning_group_1.c_str());
        RCLCPP_INFO(this->get_logger(), "ArmsDemo initialized with planning group: %s", planning_group_2.c_str());

        init_can();
    }

    double ArmsDemo::findMax(double arr[], int size)
    {
        double max = abs(arr[0]);
        for (int i = 1; i < size - 2; i++)
        {
            if (abs(arr[i]) > abs(max))
                max = abs(arr[i]);
        }
        return max;
    }

    bool ArmsDemo::init_can()
    {
        int nDeviceType = 4;
        int nDeviceInd = 0;
        int nCANInd_1 = 0, nCANInd_2 = 1;

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

        // tongdao 1
        if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd_1, &vic) != 1)
        {
            std::cout << "Init can 1 failed!" << std::endl;
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            return false;
        }
        else
            std::cout << "Init can 1 success!" << std::endl;

        if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
        {
            std::cout << "Start can 1 failed!" << std::endl;
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            return false;
        }
        else
            std::cout << "Start can 1 success!" << std::endl;
        
        // tongdao 2
        if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd_2, &vic) != 1)
        {
            std::cout << "Init can 2 failed!" << std::endl;
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            return false;
        }
        else
            std::cout << "Init can 2 success!" << std::endl;

        if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1)
        {
            std::cout << "Start can 2 failed!" << std::endl;
            VCI_CloseDevice(nDeviceType, nDeviceInd);
            return false;
        }
        else
            std::cout << "Start can 2 success!" << std::endl;

        return true;
    }

    int32_t ArmsDemo::convertHexArrayToDecimal(const std::uint8_t hexArray[4])
    {
        std::int32_t result = 0;
        for (int i = 0; i < 4; i++)
            result = (result << 8) | hexArray[i];
        if (result > 0x7FFFFFFF)
            result -= 0x100000000;

        return result;
    }

    void ArmsDemo::toIntArray(int number, int *res, int size)
    {
        unsigned int unsignedNumber = static_cast<unsigned int>(number);

        for (int i = 0; i < size; ++i)
        {
            res[i] = unsignedNumber & 0xFF;
            unsignedNumber >>= 8;
        }
    }

    void ArmsDemo::sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, int CANInd)
    {
        VCI_CAN_OBJ send[1];
        send[0].SendType = 0;
        send[0].RemoteFlag = 0;
        send[0].ExternFlag = 0;
        send[0].DataLen = 1;

        for (int i = 0; i < numOfActuator; i++)
        {
            send[0].ID = canIdList[i];
            send[0].Data[0] = commandList[i];

            if (VCI_Transmit(VCI_USBCAN2, 0, CANInd, send, 1) == 1)
            {
                VCI_CAN_OBJ rec[3000];
                int reclen = 0, cnt = 2;

                while (VCI_Receive(VCI_USBCAN2, 0, CANInd, rec, 3000, 100)<= 0 && cnt)
                    cnt--;
                if (cnt == 0)
                    std::cout << "ops! ID " << send[0].ID << " receive failed!" << std::endl;
                else
                {
                    for (int j = 0; j < reclen; j++)
                    {
                        std::uint8_t hexArray[4] = {rec[j].Data[4], rec[j].Data[3], rec[j].Data[2], rec[j].Data[1]};
                        std::int32_t decimal = convertHexArrayToDecimal(hexArray);
                        //std::cout << "ID: " << send[0].ID << " Data: " << decimal << std::endl;
                    }
                }
            }
            else
                break;
        }
    }

    void ArmsDemo::sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList,int CANInd)
    {
        VCI_CAN_OBJ send[1];
        send[0].SendType = 0;
        send[0].RemoteFlag = 0;
        send[0].ExternFlag = 0;
        send[0].DataLen = 5;

        for (int i = 0; i < numOfActuator; i++)
        {
            send[0].ID = canIdList[i];
            send[0].Data[0] = commandList[i];
            int res[4], cnt = 2, reclen = 0;
            toIntArray(parameterList[i], res, 4);

            for (int j = 1; j < 5; j++)
                send[0].Data[j] = res[j - 1];

            while (VCI_Transmit(VCI_USBCAN2, 0, CANInd, send, 1) <= 0 && cnt)
                cnt--;
            if (cnt == 0)
                std::cout << "ops! ID " << send[0].ID << " transmit failed!" << std::endl;
            else
            {
                //std::cout << "ID: " << send[0].ID << std::endl;
                //for (int c = 0; c < send[0].DataLen; c++)
                   // printf("  %02X ", send[0].Data[c]);
                //std::cout << std::endl;
            }
        }
    }

    void ArmsDemo::callback(const moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr &msg)
    {
        std::vector<std::string> joints = {"L_SHOULDER_P","L_SHOULDER_R","L_SHOULDER_Y","L_ELBOW_R","L_WRIST_P","L_WRIST_Y","L_WRIST_R",
                                           "R_SHOULDER_P","R_SHOULDER_R","R_SHOULDER_Y","R_ELBOW_R","R_WRIST_P","R_WRIST_Y","R_WRIST_R"};

        uint8_t canidList[20];
        int CANInd = 0;
        uint8_t List[20] = {23, 24, 25, 26, 27, 28, 29, 16, 17, 18, 19, 20, 21, 22};
        uint8_t cmd_pos[20] = {30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30},
                cmd_v_p[20] = {36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36},
                cmd_v_n[20] = {37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37};
        uint32_t para_v_p[20], para_v_n[20], para_pos[20];

        int n = msg->trajectory[0].joint_trajectory.points.size();
        double cn[20];

        int tmp0 = 100, tmp1 = -100;
        for (int i = 0; i < 14; i++)
        {
            para_v_p[i] = tmp0;
            para_v_n[i] = tmp1;
        }

        sendCanCommand(14, canidList, cmd_v_p, para_v_p,CANInd);
        usleep(100);
        sendCanCommand(14, canidList, cmd_v_n, para_v_n,CANInd);
        usleep(100);

        for (int i = 0; i<7; i++)
        {
            auto it = std::find(joints.begin(), joints.end(), msg->trajectory[0].joint_trajectory.joint_names[i]);
            int cnt = std::distance(joints.begin(), it);
            canidList[i] = List[cnt];
            CANInd = cnt/7;
            ori_position[i] = msg->trajectory[0].joint_trajectory.points[0].positions[i] * 57.3;
            next_position[i] = msg->trajectory[0].joint_trajectory.points[n - 1].positions[i] * 57.3;
            now_position[i] = msg->trajectory[0].joint_trajectory.points[n - 1].positions[i] * 57.3 * 101 * 65536 / 360;
            cn[i] = next_position[cnt] - ori_position[cnt];            
            para_pos[i] = static_cast<uint32_t>(now_position[i]);
            //std::cout<<"trajectory[0].joint_trajectory.joint_names[i]: "<<msg->trajectory[0].joint_trajectory.joint_names[i]
            //    <<"trajectory[0].joint_trajectory.points[0].positions[i]: "<<msg->trajectory[0].joint_trajectory.points[0].positions[i]
            //    <<" cnt:  "<<cnt<<std::endl;
        }
        //std::cout<<std::endl;

        double maxVal = findMax(cn, 7);
        double periodTime = maxVal / 100;
        double status = 1;
        if (0.3 < periodTime && periodTime < 0.5)
            periodTime *= 2;
        else if (0.001 < periodTime && periodTime < 0.3)
            periodTime *= 3;

        int maxSpeed[20];
        for (int i = 0; i < 7; i++)
            maxSpeed[i] = abs((cn[i] / periodTime) * 10100 / 360);

        sendCanCommand(7, canidList, cmd_pos, para_pos,CANInd);

        for (int j = 55; j < 105; j++)
        {
            double acceleration_ratio = (j - 5) / 100.0;
            for (int i = 0; i < 7; i++)
            {
                para_v_p[i] = acceleration_ratio * maxSpeed[i];
                para_v_n[i] = -1 * para_v_p[i];
            }
            sendCanCommand(14, canidList, cmd_v_p, para_v_p,CANInd);
            usleep(10);
            sendCanCommand(14, canidList, cmd_v_n, para_v_n,CANInd);
            usleep(10);
            usleep(5000 * status);
        }
        if (periodTime >= 0.55)
            usleep(periodTime * 1000000 - 5000 * status);

        for (int j = 105; j > 55; j--)
        {
            double acceleration_ratio = (j - 5) / 100.0;
            for (int i = 0; i < 14; i++)
            {
                para_v_p[i] = j * maxSpeed[i] / 100.0;
                para_v_n[i] = -1 * para_v_p[i];
            }
            sendCanCommand(14, canidList, cmd_v_p, para_v_p,CANInd);
            usleep(10);
            sendCanCommand(14, canidList, cmd_v_n, para_v_n,CANInd);
            usleep(10);
            usleep(5000 * status * acceleration_ratio);
        }
    }

    bool ArmsDemo::move_L_by_joint(const std::vector<double> &joint_group_positions)
    {
        std::vector<double> current_joint_values = move_group_1_.getCurrentJointValues();
        for (int i = 0; i < joint_group_positions.size(); i++)
            current_joint_values[i] += joint_group_positions[i];

        move_group_1_.setJointValueTarget(current_joint_values);
        move_group_1_.move();
        // sleep(3);
        return true;
    }

    bool ArmsDemo::move_R_by_joint(const std::vector<double> &joint_group_positions)
    {
        std::vector<double> current_joint_values = this->move_group_2_.getCurrentJointValues();
        for (int i = 0; i < joint_group_positions.size(); i++)
            current_joint_values[i] += joint_group_positions[i];

        move_group_2_.setJointValueTarget(current_joint_values);
        move_group_2_.move();
        // sleep(3);
        return true;
    }

    bool ArmsDemo::move_L_by_pos(const std::vector<double> &pose)
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

        move_group_1_.setStartStateToCurrentState();
        move_group_1_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = move_group_1_.plan(plan);
        RCLCPP_INFO(this->get_logger(), "move_p:%s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            move_group_1_.execute(plan);
            // sleep(3);
            return true;
        }
        return false;
    }


    bool ArmsDemo::move_R_by_pos(const std::vector<double> &pose)
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

        move_group_2_.setStartStateToCurrentState();
        move_group_2_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = move_group_2_.plan(plan);
        RCLCPP_INFO(this->get_logger(), "move_p:%s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            move_group_2_.execute(plan);
            // sleep(3);
            return true;
        }
        return false;
    }



    void ArmsDemo::get_L_pos()
    {
        geometry_msgs::msg::PoseStamped current_pose = this->move_group_1_.getCurrentPose(this->end_effector_link);

        RCLCPP_INFO(this->get_logger(), "current pose:x:%f,y:%f,z:%f,Roll:%f,Pitch:%f,Yaw:%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);

        std::vector<double> rpy = this->move_group_1_.getCurrentRPY(this->end_effector_link);
        RCLCPP_INFO(this->get_logger(), "current rpy:%f,%f,%f", rpy[0], rpy[1], rpy[2]);
    }

    void ArmsDemo::get_R_pos()
    {
        geometry_msgs::msg::PoseStamped current_pose = this->move_group_2_.getCurrentPose(this->end_effector_link);

        RCLCPP_INFO(this->get_logger(), "current pose:x:%f,y:%f,z:%f,Roll:%f,Pitch:%f,Yaw:%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);

        std::vector<double> rpy = this->move_group_2_.getCurrentRPY(this->end_effector_link);
        RCLCPP_INFO(this->get_logger(), "current rpy:%f,%f,%f", rpy[0], rpy[1], rpy[2]);
    }
    
    void ArmsDemo::get_L_joint()
    {
        std::vector<double> current_joint_values = this->move_group_1_.getCurrentJointValues();
        RCLCPP_INFO(this->get_logger(), "current joint values:%f,%f,%f,%f,%f,%f", current_joint_values[0], current_joint_values[1], current_joint_values[2],
                    current_joint_values[3], current_joint_values[4], current_joint_values[5]);
    }

    void ArmsDemo::get_R_joint()
    {
        std::vector<double> current_joint_values = this->move_group_2_.getCurrentJointValues();
        RCLCPP_INFO(this->get_logger(), "current joint values:%f,%f,%f,%f,%f,%f", current_joint_values[0], current_joint_values[1], current_joint_values[2],
                    current_joint_values[3], current_joint_values[4], current_joint_values[5]);
    }

    void ArmsDemo::move_up()
    {
        int tmp0 = 500, tmp1 = -500;
        uint32_t para_v_p[20] = {tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0, tmp0};
        uint32_t para_v_n[20] = {tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1, tmp1};
        uint8_t canidList_L[20] = {23, 24, 25, 26, 27, 28, 29};
        uint8_t canidList_R[20] = {16, 17, 18, 19, 20, 21, 22};
        uint8_t cmd_pos[20] = {30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30};
        uint8_t cmd_v_p[20] = {36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36};
        uint8_t cmd_v_n[20] = {37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37};
        
        sendCanCommand(7, canidList_L, cmd_v_p, para_v_p, 0);
        sendCanCommand(7, canidList_R, cmd_v_p, para_v_p, 1);
        usleep(50);
        sendCanCommand(7, canidList_L, cmd_v_n, para_v_n,0);
        sendCanCommand(7, canidList_R, cmd_v_n, para_v_n,1);
        usleep(50);

        uint32_t para_pos[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        sendCanCommand(7, canidList_L, cmd_pos, para_pos,0);
        sendCanCommand(7, canidList_R, cmd_pos, para_pos,1);
    }

}
