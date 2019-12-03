// Octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <string>

#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
//#include <octomap_msgs.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "octomap_example");
    ros::NodeHandle nh;

    ros::Publisher octomap_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    ROS_INFO("Load 1.bt ...");
    octomap::OcTree* octree = new octomap::OcTree("/home/louis/Desktop/1.bt");
    static octomap_msgs::Octomap octomap;
    ROS_INFO("Transfer the format ...");
    octomap_msgs::binaryMapToMsg(*octree, octomap);//转换成消息格式

    ROS_INFO("create a planning scene and publish it ...");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.octomap.header.frame_id = "odom_combined";
    planning_scene.world.octomap.header.stamp = ros::Time::now();
    planning_scene.world.octomap.octomap.header.frame_id = "odom_combined";
    planning_scene.world.octomap.octomap.header.stamp = ros::Time::now();
    planning_scene.world.octomap.octomap.binary = true;
    planning_scene.world.octomap.octomap.id = "OcTree";
    planning_scene.world.octomap.octomap.data = octomap.data;

    ROS_INFO("Publish ...");
    octomap_pub.publish(planning_scene);
    ROS_INFO("Published.");
    while (octomap_pub.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    ROS_INFO("more than one subscriber, start publishing msgs on and on...");

    ros::spin();

    return 0;
}