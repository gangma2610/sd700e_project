/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "RobotController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    RobotController controller;

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    // 控制机械臂先回到初始化位置
    //arm.setNamedTarget("home");
    //arm.move();
    //sleep(1);

    // record 3
    double targetPose[6] = {-0.08866324960008753, -0.33132457855234354, -0.01910088333382594,
0.5796151179410568, 0.14123291573600716, 1.57};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    sleep(1);
    
    //实体机器人运动控制
    ROS_INFO("Real robot controlling...");
    //joint_group_positions[0] = -joint_group_positions[0];
    //joint_group_positions[1] = -joint_group_positions[1];
    //joint_group_positions[5] = -joint_group_positions[5];
    //controller.MoveByAxis(joint_group_positions);
     // 获取关节坐标
    ROS_INFO("get current axis...");
    std::vector<double> axispos = controller.GetCurrentAxisPos();
    
    for (int i = 0; i < 6; ++i) {
        std::cout << axispos[i] << " ";
    }
    std::cout << std::endl;
     

    // 控制机械臂先回到初始化位置
    //arm.setNamedTarget("home");
    //arm.move();
    //sleep(1);

    ros::shutdown(); 

    return 0;
}
