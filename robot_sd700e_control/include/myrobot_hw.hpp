#ifndef _MYROBOT_HW_HPP_
#define _MYROBOT_HW_HPP_


#include <iostream>
#include <vector>
#include <string>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
//#include "myrobotcpp/myrobotcpp.hpp"

namespace myrobots_hardware_interface
{

class MyRobotInterface: public hardware_interface::RobotHW
{
public:
    MyRobotInterface();
    ~MyRobotInterface();
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

protected:
    ros::NodeHandle nh_;
    
    //interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;

    int num_joints;
    std::vector<std::string> joint_name;

    //actual states
    std::vector<double> joint_position_state;
    std::vector<double> joint_velocity_state;
    std::vector<double> joint_effort_state;

    //given setpoints
    std::vector<double> joint_effort_command;

    //MyRobotCPP* robot;
};
}

#endif // _MYROBOT_HW_HPP_
