/*************************************************************************
 @ File Name: mrobot_joint_state_listen.cpp
 @ Author: Louis Ma
 @ Mail: lynn_mg_study@163.com 
 @ Created Time: Thu 31 Oct 2019 06:57:57 PM CST
 @ Description: 
 ************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace std;
///////////////////////////////////////////////////////////////////////////

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
    ROS_INFO("Joint state: %lf %lf %lf %lf %lf %lf", 
            msg->position[0], 
            msg->position[1], 
            msg->position[2],
            msg->position[3],
            msg->position[4],
            msg->position[5]);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joint_state_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("mrobot_joint_states",1000, jointStateCallback);
    ros::spin();
    return 0;
}
