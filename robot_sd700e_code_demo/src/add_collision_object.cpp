/*************************************************************************
 @ File Name: add_collision_object.cpp
 @ Author: Louis Ma
 @ Mail: lynn_mg_study@163.com 
 @ Created Time: Fri 01 Nov 2019 08:50:56 AM CST
 @ Description: 测试陆点接口 
 ************************************************************************/

#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "RobotController.h"

//#define ROBOT_CONTROLLER_ON


using namespace std;

////////////////////////////////////////////////////////////////////////////

void getJointPoints( const moveit_msgs::DisplayTrajectory &display_trajectory);
void getJointPointsFromPlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan);

#ifdef ROBOT_CONTROLLER_ON
// Robot运动控制类
RobotController controller;
#endif // ROBOT_CONTROLLER_ON

int main(int argc, char** argv){
    ros::init(argc, argv, "add_collision_object");
    ros::NodeHandle nh;


    ros::AsyncSpinner spin(1);

    spin.start();

    moveit::planning_interface::MoveGroupInterface group("manipulator");
    
    // 创建运动规划的情景，等待创建完成
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 获取终端link的名称
    string end_effector_link = group.getEndEffectorLink();
    // 设置目标位置所使用的参考坐标系
    string reference_frame = "base_link";
    group.setPoseReferenceFrame(reference_frame);

    // 当运动规划失败后允许重新规划
    group.allowReplanning(true);

    // 设置位置（单位：米）和姿态（单位：弧度）的允许误差
    group.setGoalPositionTolerance(0.0005);
    group.setGoalOrientationTolerance(0.01);

    // 设置允许的最大速度和加速度 
    group.setMaxAccelerationScalingFactor(0.2);
    group.setMaxVelocityScalingFactor(0.2);

    // 控制机械臂先回到初始化位置
    group.setNamedTarget("home");
    group.move();
    sleep(1);

    // 设置机械臂当前的状态作为初始运动状态
    group.setStartStateToCurrentState();

    // 设置机器人终端目标位置

    //group.setPoseTarget();

    // 获取基本信息
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame - end effector link: %s", group.getEndEffectorLink().c_str());

    // 计划运动到制定姿态， 这里是笛卡尔坐标
    // pose_01: joints= [-2.37413012475 -0.362712233756 -3.30842914776 0.0891402481348 -1.07037546857 -4.08564328731]
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = 0.301206172091;
    target_pose1.position.y = 0.298737764642;
    target_pose1.position.z = 0.339192475207;
    target_pose1.orientation.x = -0.671488959638;
    target_pose1.orientation.y = -0.739855922235;
    target_pose1.orientation.z = 0.0110674564227;
    target_pose1.orientation.w = 0.0399161975336;
    
    group.setPoseTarget(target_pose1);

    // pose_02: joints = joints = [-0.560043554869 0.302201155354 0.218951920005 3.13278702575 -1.13188586352 0.918372519597]
    geometry_msgs::Pose target_pose2;
    target_pose2.position.x = 0.339922108072;
    target_pose2.position.y = -0.213831761871;
    target_pose2.position.z = 0.348633733074;

    target_pose2.orientation.x = -0.671622577034;
    target_pose2.orientation.y = -0.73974166926;
    target_pose2.orientation.z = 0.011169080917;
    target_pose2.orientation.w = 0.0397571177159;


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // plan 返回值需要注意类型问题
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1(pose goal) %s", success.val? "":"FAILED");
    sleep(1.0);
    #ifndef ROBOT_CONTROLLER_ON
    sleep(10.0);
    #endif // ROBOT_CONTROLLER_ON
    group.move();
    // 获取并显示路点
    getJointPointsFromPlan(my_plan);
    

    // 可视化规划点
    // 创建一个pubulisher来将规划结果可视化到Rviz上.
    ros::Publisher display_publisher = 
        nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;


    /*
    // 发布规划结果
    ROS_INFO("Visualizing plan 1(again)");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    ROS_INFO("trajectory points: %d", (int)display_trajectory.trajectory.size());
    sleep(5.0);
    */

    // 规划一个关节空间的目标, 关节运动
    ROS_INFO("Planning to a joint-space goal...");
    vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),group_variable_values);
    group_variable_values[0] = -1.0;
    group.setJointValueTarget(group_variable_values);
    success = group.plan(my_plan);
    ROS_INFO("Visualizing plan 2(joint space goal) %s", success.val ? "": "FAILD");
    sleep(5.0);
    #ifndef ROBOT_CONTROLLER_ON
    sleep(5.0);
    #endif // ROBOT_CONTROLLER_ON
    group.move();
    // 获取并显示路点
    getJointPointsFromPlan(my_plan);
    

    #if 0
    // Planning with path constraints
    // Path constraints can easily be specitied for a link on the robot. 
    // Let's specify a path constraint and a pose goal for out group.
    // First define the path constraint.
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "link6";
    ocm.header.frame_id = "base_link";
    ocm.orientation.x = -0.67;
    ocm.orientation.y = -0.74;
    ocm.orientation.z = 0.01;
    ocm.orientation.w =  0.04;
    ocm.absolute_x_axis_tolerance = 0.1; 
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    // Now, set it as the path constraint for the group
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(test_constraints);

    // We will reuse the old goal that we had and plan to it. 
    // Note that this will only work if the current state already satisties the path constraints.
    // So, we need to set the start state to a new pose;
    /*
    robot_state::RobotSconditiontate start_state(*group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w =  0.03398;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;

    const robot_state::JointModelGroup * joint_model_group = 
        start_state.getJointModelGroup( group.getName());
    
    start_state.setFromIK(joint_model_group, start_pose2);
    group.setStartState(start_state);

    // Now, we will plan to the earlier pose target from the new start state that we have just created.
    group.setPoseTarget(target_pose1);
    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan3 (constraints) %s", success.val ? "" : "FAILD");
    // sleep to give rviz time to visualize the plan.
    sleep(10);
    */
#endif

    // Adding/Removing Objects and Attaching/Detaching Objects
    // First, we will define the collision object message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id= group.getPlanningFrame();
    // The id of the object is used to identify it.
    collision_object.id = "box1";

    // Define  a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;

    // A pose for the boconditionx (specified relative to frame_id).
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0;
    box_pose.position.z = 0.3;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    // Now, let's add the collsion object into the world
    ROS_INFO("Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    sleep(2.0);
    // Planning with collsion detection can be slow. Lets set the planning time to be sure the planner
    // has enough time tocondition plan around the box. 10 seconds should be plenty.
    group.setPlanningTime(10.0);
    // Now when we plan a trajectory it will avoid the obstacle.
    group.setStartState(*group.getCurrentState());
    group.setPoseTarget(target_pose1);
    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 5 (pose goal move around box) %s", success ? "" : "FAILED");
    sleep(10.0);
    #ifndef ROBOT_CONTROLLER_ON
    sleep(10.0);
    #endif // ROBOT_CONTROLLER_ON
    group.move();
    getJointPointsFromPlan(my_plan);
    
    // Sleep to give rviz time to visualize the plan.
    

    /*
    ROS_INFO("Visualizing plan 1(again)");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    */


    /*
    ROS_INFO("sleep for 5s...");
    sleep(5.0);
    
    // 声明一个障碍物的实例，并且为其设置一个Id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject cylinder;
    cylinder.id = "arm_cylinder";

    // 设置障碍物的外形，尺寸等信息
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3); 
    primitive.dimensions[0] = 0.6;in
    primitive.dimensions[1] = 0.2;

    // 设置障碍物的位置信息
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = -0.4;
    pose.position.z = 0.4;

    // 将障碍物的属性、位置加入到障碍物实例中
    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(pose);
    cylinder.operation = cylinder.ADD;

    // 创建一个障碍物的列表，把之前创建的障碍物实例加入其中
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(cylinder);
    ROS_INFO("add collision object into the scene...");
    // 所有障碍物加入列表后（这里只有一个障碍物）,再把障碍物加入到当前情景中，
    // 如果删除障碍物,使用removeCollisionObjects(collision_objects)
    planning_scene_interface.addCollisionObjects(collision_objects);
    sleep(10);
    ros::shutdown();
    */
    return 0;
}

void getJointPoints( const moveit_msgs::DisplayTrajectory &display_trajectory)
{
    ROS_INFO("trajectory size: %d", (int)display_trajectory.trajectory.size());
    ROS_INFO("trajectory points: %d", (int)display_trajectory.trajectory[0].joint_trajectory.points.size());
    sleep(5.0);

    vector< vector<double> > joint_points; 
    joint_points.push_back(display_trajectory.trajectory_start.joint_state.position);

    for (size_t i = 0; i < display_trajectory.trajectory[0].joint_trajectory.points.size(); i++)
    {
        joint_points.push_back(display_trajectory.trajectory[0].joint_trajectory.points[i].positions);
    }
         
    ROS_INFO("joint points: %d", (int)joint_points.size());
        
    for (size_t j = 0; j < joint_points.size(); j++)
    {
        /* code for loop body */
        ROS_INFO("point %d : %f %f %f %f %f %f", (int)j + 1, joint_points[j][0], joint_points[j][1], joint_points[j][2], 
        joint_points[j][3], joint_points[j][4], joint_points[j][5]);
    }
}

// 获取规划中的路点
void getJointPointsFromPlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    vector< vector<double> > joint_points; 
    // 将起始点加入路点容器中
    joint_points.push_back(plan.start_state_.joint_state.position);
    //int n_joints = plan.trajectory_.joint_trajectory.joint_names.size();
    for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        /* code for loop body */
        joint_points.push_back(plan.trajectory_.joint_trajectory.points[i].positions);
    }


    ROS_INFO("joint points: %d", (int)joint_points.size());
        
    for (size_t j = 0; j < joint_points.size(); j++)
    {
        /* code for loop body */
        ROS_INFO("point %d : %f %f %f %f %f %f", (int)j + 1, joint_points[j][0], joint_points[j][1], joint_points[j][2], 
        joint_points[j][3], joint_points[j][4], joint_points[j][5]);
    }

    ROS_INFO("Control the robot to move by trajectory...");

    #ifdef ROBOT_CONTROLLER_ON
    controller.MoveByJointTrajectory(joint_points);
    #endif // ROBOT_CONTROLLER_ON

    //sleep(20.0);

}

