/*************************************************************************
 @ File Name: kinematic_model_tutorial.cpp
 @ Author: Louis Ma
 @ Mail: lynn_mg_study@163.com 
 @ Created Time: Fri 08 Nov 2019 10:46:54 AM CST
 @ Description:  学习RobotModel和RobotState 类
 @ 参考代码：https://github.com/ros-planning/moveit_pr2/tree/hydro-devel/pr2_moveit_tutorials/kinematics
 ************************************************************************/

#include <iostream>
#include <ros/ros.h>
// Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "manipulator");
    ros::AsynSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    return 0;
}
