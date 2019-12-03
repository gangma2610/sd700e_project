#include <iostream>
#include <ros/ros.h>
#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "combo_control_node");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    combined_robot_hw::CombinedRobotHW hw;
    bool init_success = hw.init(nh,nh);

    controller_manager::ControllerManager cm(&hw,nh);

    ros::Duration period(1.0/200); // 200Hz update rate

    ROS_INFO("combo_control started");
    while(ros::ok()){
        hw.read(ros::Time::now(), period);
        cm.update(ros::Time::now(), period);
        hw.write(ros::Time::now(), period);
        period.sleep();
    }

    spinner.stop();
    //period.stop();
    return 0;
}
