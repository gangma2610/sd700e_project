/*************************************************************************
 @ File Name: mrobot_trajectory.cpp
 @ Author: Louis Ma
 @ Mail: lynn_mg_study@163.com 
 @ Created Time: Wed 13 Nov 2019 08:54:46 PM CST
 @ Description: control the robot move my joint trajectory.
 ************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "mrobot_msgs/DoJointTrajectoryAction.h"

typedef actionlib::SimpleActionServer<mrobot_msgs::DoJointTrajectory> Server;

// 收到action的goal后调用该回调函数
void execute(const mrobot_msgs::DoJointTrajectoryConstPtr& goal, Server* as)
{
    

}

int main(int argc, char** argv){
    ros::init(argc, argv, "do_joint_trajectory_server");
    ros::NodeHandle nh;
    
    // 定义一个服务器
    Server server(nh, "do_joint_trajectory", boost::bind(&execute, _1, &server), false);
    
    // 服务器开始运行
    server.start();
    ros::spin();
    return 0;
}
