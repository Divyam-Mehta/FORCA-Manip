#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Geometry>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "UR2_manipulator_control",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("UR2_manipulator_control");

  // Create the MoveIt MoveGroup Interface for the combined group
  using moveit::planning_interface::MoveGroupInterface;

  auto UR2_group_interface = MoveGroupInterface(node, "UR2_manipulator");

  UR2_group_interface.setMaxVelocityScalingFactor(0.1);
  UR2_group_interface.setMaxAccelerationScalingFactor(0.1);

  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

  const moveit::core::JointModelGroup* UR2_joint_model_group = kinematic_model->getJointModelGroup("UR2_manipulator");

  const std::vector<std::string>& UR2_joint_names = UR2_joint_model_group->getVariableNames();

  std::vector<double> UR2_joint_values;

  geometry_msgs::msg::Pose UR2_pose;

  UR2_pose.position.x = 0.75;
  UR2_pose.position.y = -0.05;
  UR2_pose.position.z = 1.0;

  // Define roll, pitch, and yaw
  double roll = 0.0; 
  double pitch = 3.14; 
  double yaw = 0.0; 

  // Convert roll, pitch, and yaw to quaternion
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);

  UR2_pose.orientation.w = quaternion.w();
  UR2_pose.orientation.x = quaternion.x();
  UR2_pose.orientation.y = quaternion.y();
  UR2_pose.orientation.z = quaternion.z();

  double timeout = 5.0;
  bool UR2_found_ik = kinematic_state->setFromIK(UR2_joint_model_group, UR2_pose, timeout);

  if (UR2_found_ik)
  {
    kinematic_state->copyJointGroupPositions(UR2_joint_model_group, UR2_joint_values);

    for (std::size_t i = 0; i < UR2_joint_names.size(); ++i)
    {
      RCLCPP_INFO(node->get_logger(), "Joint %s: %f", UR2_joint_names[i].c_str(), UR2_joint_values[i]);
    }
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Did not find IK solution");
  }

  UR2_group_interface.setJointValueTarget(UR2_joint_names, UR2_joint_values);

  // Construct Plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (UR2_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // bool success = (UR2_group_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);
  if(!success){
    RCLCPP_INFO(node->get_logger(), "Plan did not succeed");
  }
  UR2_group_interface.execute(my_plan);

  rclcpp::shutdown();
  return 0;
}