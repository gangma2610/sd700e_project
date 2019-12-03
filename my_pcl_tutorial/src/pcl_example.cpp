/*************************************************************************
 @ File Name: pcl_example.cpp
 @ Author: Louis Ma
 @ Mail: lynn_mg_study@163.com 
 @ Created Time: Wed 27 Nov 2019 11:40:21 PM CST
 @ Description: 
 ************************************************************************/
#include <ros/ros.h>
// PCL 相关的头文件
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 滤波的头文件
#include <pcl/filters/voxel_grid.h>

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{
    // 声明存储原始数据与滤波后的数据点云的格式
    // Container for original & filtered data
    pcl::PCLPointCloud2 cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered; // 存储滤波后的数据格式

    // Convert to PCL data type 转化为PCL中的点云的数据格式
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering  进行一个滤波处理
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor; 	// 创建滤波对象
    sor.setInputCloud(cloudPtr); 		// 设置输入的滤波，将需要过滤的点云给滤波对象
    sor.setLeafSize(0.1, 0.1, 0.1);		// 设置滤波时穿件的体素大小1cm立方体
    sor.filter(cloud_filtered); 		// 执行滤波处理，存储输入cloud_filtered

    // Convert to ROS data type. 		// 再将滤波后的点云数据格式转换成ROS下的数据格式发布出去
    sensor_msgs::PointCloud2 output;		// 声明的输出的点云的格式
    pcl_conversions::moveFromPCL(cloud_filtered, output); 	// 第一个参数是输入，后面是输出

    // Publish the data
    pub.publish(output);

}


int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial"); // 声明节点的名称
    ROS::NodeHandle nh;
   
    // Create a ROS subscriber for the inpute point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);
    
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // 回调
    ros::spin(); 
    return 0;
}
