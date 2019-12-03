#include <myrobot_hw.hpp>
#include <algorithm>

namespace myrobots_hardware_interface
{

MyRobotInterface::MyRobotInterface(){

}

MyRobotInterface::~MyRobotInterface(){

}

bool MyRobotInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh){
    //init base
    //robot = myrobotcpp::initRobot();

    //get joint names and num of joint
    robot_hw_nh.getParam("joints", joint_name);
    num_joints = joint_name.size();

    //resize vectors
    joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
    joint_effort_command.resize(num_joints);
 
    //Register handles
    for(int i=0; i<num_joints; i++){
        //State
        hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(jointStateHandle);
        

        //Effort
        hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command[i]);
        effort_joint_interface.registerHandle(jointEffortHandle);
    }

    //Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&effort_joint_interface);
    
    //return true for successful init or ComboRobotHW initialisation will fail
    return true;
}

void MyRobotInterface::read(const ros::Time& time, const ros::Duration& period){
        for(int i=0;i < num_joints;i++){
            //joint_position_state[i] = robot.readJoints(i);
            joint_position_state[i] = 2.7;
        }
}

void MyRobotInterface::write(const ros::Time& time, const ros::Duration& period){
   for(int i=0;i < num_joints;i++){
       //robot.writeJoints(joint_effort_command[i]);
       joint_effort_command[i] = 2;
   }
}

}
PLUGINLIB_EXPORT_CLASS(myrobots_hardware_interface::MyRobotInterface, hardware_interface::RobotHW)