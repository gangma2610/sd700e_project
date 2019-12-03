/*************************************************************************
 @ File Name: mrobot_joint_state.cpp
 @ Author: Louis Ma
 @ Mail: lynn_mg_study@163.com 
 @ Created Time: Thu 31 Oct 2019 04:43:17 PM CST
 @ Description: 
 ************************************************************************/

#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <unistd.h>
#include <math.h>

using namespace std;
////////////////////////////////////////////////////////////////

char buffer[256] = {0};

// socket send data.
void Send(int sockfd, const string& s)
{
    write(sockfd, s.c_str(), s.length());
}

// socket receive data.
string Receive(int sockfd, int n) 
{
    memset((void*)buffer, 0, sizeof(buffer));
    read(sockfd, (void*)buffer, n);
    //printf("m_buffer = %s\n", m_buffer);
    return string(buffer);
}

// convert string to double.
double String2Double(const string& s)
{
    istringstream is(s.c_str());
    double data;
    is >> data;
    return data;
}

int main(int argc, char** argv){
    const double c_pi = acos(-1.0);
    const double degree = c_pi / 180;
    string joint_name[6] = {"joint1", "joint2", "joint3",
        "joint4", "joint5", "joint6"};
    // socket 链接
    int sockfd = socket(PF_INET, SOCK_STREAM, 0);
    if(sockfd == -1) // Socket failed!
    {
        ROS_INFO("Socket Error!");
        exit(-1);
    }

    const string ip = "192.168.39.220";
    const int port = 9875;
    struct sockaddr_in addr;
    addr.sin_family = PF_INET;
    addr.sin_port = htons(port); //Port

    ROS_INFO("Connect to robot: ip= %s, port= %d ...", ip.c_str(), port);
    addr.sin_addr.s_addr = inet_addr(ip.c_str()); //Ip

    int res = connect(sockfd, (struct sockaddr*)&addr, sizeof(addr));
    if(res == -1)  // connect failed!
    {
        ROS_INFO("Connect Error!");
        exit(-2);
    }

    // 节点名称
    ros::init(argc, argv, "mrobot_joint_state_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("mrobot_joint_states", 1);
    ros::Rate loop_rate(10); //
    
    sensor_msgs::JointState joint_state;
    //bool is_running = true;
    while(ros::ok()) 
    {
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position.resize(6);
        // receive data.
        for (int i = 0; i < 6; ++i) {
            string data = Receive(sockfd, 7);
            //std::cout << data << std::endl;
            joint_state.name[i] = joint_name[i];
            joint_state.position[i] = String2Double(data) * degree;
        }
        ROS_INFO("Joint state: %lf %lf %lf %lf %lf %lf", 
                joint_state.position[0], 
                joint_state.position[1],
                joint_state.position[2],
                joint_state.position[3],
                joint_state.position[4],
                joint_state.position[5]);

        joint_pub.publish(joint_state);

        loop_rate.sleep();
    }

    return 0;
}
