
﻿

# ROS2双臂接口文档
运行环境：ununtu 2204
ROS2版本：humble，moveit2



### 关于Ti5_Arms功能包介绍
Ti5_Arms功能包中主要实现了ArmsDemo类，功能包中集成了函数具体实现的Ti5_Arms.cpp和相应头文件Ti5_Arms.h以及相应的ROS2头文件。
使用时前将Ti5_Arms文件夹下载到本地文件中。

 **Ti5_Arms.h**


#### 相应函数功能和参数说明

> **ArmsDemo(const std::string &name, const std::string &planning_group_1,const std::string &planning_group_2);**
> 
> 该函数为ArmsDemo类的构造函数，用于创建并初始化一个ArmsDemo类的对象。const std::string &name用于指定当前类实例的名字。const std::string &planning_group_1 是一个常量引用，表示第一个规划组的名称。const std::string &planning_group_2表示第二个规划组的名称。将节点名和规划组名称关联起来，并创建 MoveIt 的运动规划接口实例。

> **bool move_L_by_joint(const std::vector<double> &joint_group_positions);**
> 
> 该函数实现了左臂的关节运动，const vector<double> &joint_group_positions为一个类型为vector的变长数组，里面定义的数值为左臂每个轴需要达到的角度数值。

> **bool move_R_by_joint(const std::vector<double> &joint_group_positions);**
> 
> 该函数实现了右臂的关节运动，const vector<double> &joint_group_positions为一个类型为vector的变长数组，里面定义的数值为右臂每个轴需要达到的角度数值。





> **bool move_L_by_pos(const std::vector<double> &pose);**
> 
> 该函数实现了左臂的位姿运动，const vector<double> &pose 为一个类型为vector的变长数组，里面定义了左臂末端位姿的x、y、z、roll、pitch、yaw的具体数值。x、y、z分别是左臂末端在基坐标系下的位置坐标，描述了左臂末端在三个空间方向上的位置，roll、pitch、yaw分别代表绕x、y、z轴的旋转角度，描述了左臂末端的姿态或方向。在某些位置，由于左臂结构的限制或特定的姿态导致给出x、y、z、roll、pitch、yaw并不能解算出对应的关节角度，如果不能解算出相应的关节角度，该函数则会给出move_p:FAILED的提示。


> **bool move_R_by_pos(const std::vector<double> &pose);**
> 
> 该函数实现了右臂的位姿运动，const vector<double> &pose 为一个类型为vector的变长数组，里面定义了右臂末端位姿的x、y、z、roll、pitch、yaw的具体数值。x、y、z分别是右臂末端在基坐标系下的位置坐标，描述了右臂末端在三个空间方向上的位置，roll、pitch、yaw分别代表绕x、y、z轴的旋转角度，描述了右臂末端的姿态或方向。在某些位置，由于右臂结构的限制或特定的姿态导致给出x、y、z、roll、pitch、yaw并不能解算出对应的关节角度，如果不能解算出相应的关节角度，该函数则会给出move_p:FAILED的提示。




> **void get_L_joint();**
> 
> 该函数实现了得到当前左臂各关节的角度并显示在终端窗口中。

> **void get_R_joint();**
> 
> 该函数实现了得到当前右臂各关节的角度并显示在终端窗口中。


> **void get_L_pos();**
> 
> 该函数实现了得到当前左臂末端位姿x、y、z、roll、pitch、yaw并显示在终端窗口中。


> **void get_R_pos();**
> 
> 该函数实现了得到当前右臂末端位姿x、y、z、roll、pitch、yaw并显示在终端窗口中。

> **void move_up();**
> 
> 该函数实现将双臂移动到初始零位，双臂的初始零位为T_pose，即双臂张开。




 
#### 具体使用步骤说明：



1 .  将Ti5_Arms功能包和arms_demo功能包和arms功能包下载到ros的工作空间的src目录下。
arms功能包是根据双臂的urdf配置生成的启动RVIZ的launch文件，Ti5_Arms功能包主要实现了ArmsDemo类和相关函数，arms_demo功能包调用了Ti5_Arms功能包里的函数接口。arms_demo文件夹下src里的arms_demo.cpp是一个实例程序，可以按照给出的方式来让双臂运动到指定位置，运动过程中注意人员和财产安全。


2 .  下载完成之后在工作空间下编译，编译命令为`colcon build`，编译成功之后执行`source install/setup.bash`。

3 .  打开一个终端，进入工作空间，执行`ros2 launch arms demo.launch.py`，会启动RVIZ，会在RVIZ显示出双臂的模型。

4 .  再打开一个终端，进入工作空间，输入`sudo chmod -R 777 /dev`，会提示输入密码，然后再输入`ros2 run arms_demo arms_demo`，双臂会移动回到零位。

5 .  RVIZ界面左下角的Planning Group选项中可以选择arm_L或者arm_R，即选择控制左臂或者右臂。也可以拖动RVIZ中的末端的球来移动到别的地方，然后点击Plan&Execute来让双臂移动到指定位置。



