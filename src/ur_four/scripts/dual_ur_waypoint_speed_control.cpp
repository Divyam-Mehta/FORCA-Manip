#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Geometry>

#include <iostream>
#include <cmath>
#include <vector>

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ORCA_MoveIt_Control", node_options);

  
  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();


  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit_UR");

  // Create the MoveIt MoveGroup Interface for the combined group
  using moveit::planning_interface::MoveGroupInterface;
  auto combined_group_interface = MoveGroupInterface(node, "UR_manipulator");
  auto UR1_group_interface = MoveGroupInterface(node, "UR1_manipulator");
  auto UR2_group_interface = MoveGroupInterface(node, "UR2_manipulator");

  // // Set velocity and acceleration scaling factors
  // UR1_group_interface.setMaxVelocityScalingFactor(1.0);
  // UR1_group_interface.setMaxAccelerationScalingFactor(1.0);
  // UR2_group_interface.setMaxVelocityScalingFactor(1.0);
  // UR2_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Start the state monitor
  combined_group_interface.startStateMonitor();
  UR1_group_interface.startStateMonitor();
  UR2_group_interface.startStateMonitor();

  combined_group_interface.setPlanningTime(10);

  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

  // moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

  // Get Current Robot EE Pose
  moveit::core::RobotStatePtr combined_group_state = combined_group_interface.getCurrentState();
  moveit::core::RobotStatePtr ur1_current_state = UR1_group_interface.getCurrentState();
  moveit::core::RobotStatePtr ur2_current_state = UR2_group_interface.getCurrentState();

  const moveit::core::JointModelGroup* UR1_joint_model_group = kinematic_model->getJointModelGroup("UR1_manipulator");
  const moveit::core::JointModelGroup* UR2_joint_model_group = kinematic_model->getJointModelGroup("UR2_manipulator");

  const std::vector<std::string>& UR1_joint_names = UR1_joint_model_group->getVariableNames();
  const std::vector<std::string>& UR2_joint_names = UR2_joint_model_group->getVariableNames();

  std::vector<double> UR1_joint_values;
  std::vector<double> UR2_joint_values;

  geometry_msgs::msg::Pose UR1_pose;
  geometry_msgs::msg::Pose UR2_pose;

  UR1_pose.position.x = 0.63;
  UR1_pose.position.y = -0.35;   // -0.35
  UR1_pose.position.z = 1.0;

  UR2_pose.position.x = 0.87;
  UR2_pose.position.y = 0.35;   //  0.35 
  UR2_pose.position.z = 1.0;

  // Define roll, pitch, and yaw
  double ur1_roll = -3.14; 
  double ur1_pitch = 0; 
  double ur1_yaw = -3.14; 

  // Define roll, pitch, and yaw
  double ur2_roll = -3.14; 
  double ur2_pitch = 0; 
  double ur2_yaw = -3.14; 

  // Convert roll, pitch, and yaw to quaternion
  tf2::Quaternion ur1_quaternion;
  ur1_quaternion.setRPY(ur1_roll, ur1_pitch, ur1_yaw);
  
  tf2::Quaternion ur2_quaternion;
  ur2_quaternion.setRPY(ur2_roll, ur2_pitch, ur2_yaw);

  UR1_pose.orientation.w = ur1_quaternion.w();
  UR1_pose.orientation.x = ur1_quaternion.x();
  UR1_pose.orientation.y = ur1_quaternion.y();
  UR1_pose.orientation.z = ur1_quaternion.z();

  UR2_pose.orientation.w = ur2_quaternion.w();
  UR2_pose.orientation.x = ur2_quaternion.x();
  UR2_pose.orientation.y = ur2_quaternion.y();
  UR2_pose.orientation.z = ur2_quaternion.z();

  // UR1_pose.orientation.x = 0;
  // UR1_pose.orientation.y = 1;
  // UR1_pose.orientation.z = 0;
  // UR1_pose.orientation.w = 0;

  // UR2_pose.orientation.x = 0;
  // UR2_pose.orientation.y = 1;
  // UR2_pose.orientation.z = 0;
  // UR2_pose.orientation.w = 0;

  double timeout = 0.1;
  bool UR1_found_ik = combined_group_state->setFromIK(UR1_joint_model_group, UR1_pose, timeout);
  bool UR2_found_ik = combined_group_state->setFromIK(UR2_joint_model_group, UR2_pose, timeout);

  if (UR1_found_ik && UR2_found_ik)
  {
    combined_group_state->copyJointGroupPositions(UR1_joint_model_group, UR1_joint_values);
    combined_group_state->copyJointGroupPositions(UR2_joint_model_group, UR2_joint_values);

    for (std::size_t i = 0; i < UR1_joint_names.size(); ++i)
    {
      RCLCPP_INFO(node->get_logger(), "Joint %s: %f", UR1_joint_names[i].c_str(), UR1_joint_values[i]);
      RCLCPP_INFO(node->get_logger(), "Joint %s: %f", UR2_joint_names[i].c_str(), UR2_joint_values[i]);
    }
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Did not find IK solution");
  }


  combined_group_interface.setJointValueTarget(UR1_joint_names, UR1_joint_values);
  combined_group_interface.setJointValueTarget(UR2_joint_names, UR2_joint_values);


  // Construct Plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (combined_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if(!success){
    RCLCPP_INFO(node->get_logger(), "Plan did not succeed");
  }
  combined_group_interface.execute(my_plan);


  rclcpp::shutdown();
  return 0;
}