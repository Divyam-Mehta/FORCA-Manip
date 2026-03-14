#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Geometry>



void EE_speed_control(const std::shared_ptr<rclcpp::Node> &node, moveit::core::RobotStatePtr &kinematic_state, moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string &end_effector, const double speed)
{
    // Get the number of waypoints and joint names
    int num_waypoints = plan.trajectory.joint_trajectory.points.size();
    const std::vector<std::string> joint_names = plan.trajectory.joint_trajectory.joint_names;

    // Set the joint positions of the first waypoint
    kinematic_state->setVariablePositions(joint_names, plan.trajectory.joint_trajectory.points[0].positions);

    Eigen::Affine3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
    Eigen::Affine3d next_end_effector_state;
    double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
    trajectory_msgs::msg::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;

    // Start iterating over the waypoints
    for (int i = 0; i < num_waypoints - 1; i++)
    {
        curr_waypoint = &plan.trajectory.joint_trajectory.points[i];
        next_waypoint = &plan.trajectory.joint_trajectory.points[i + 1];

        // Set joints for the next waypoint
        kinematic_state->setVariablePositions(joint_names, next_waypoint->positions);

        // Do forward kinematics to get cartesian positions of end effector for the next waypoint
        next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);

        // Get euclidean distance between the two waypoints
        euclidean_distance = (next_end_effector_state.translation() - current_end_effector_state.translation()).norm();

        // Updated for ROS 2 time handling
        new_timestamp = rclcpp::Duration(curr_waypoint->time_from_start).seconds() + (euclidean_distance / speed);
        old_timestamp = rclcpp::Duration(next_waypoint->time_from_start).seconds();

        // Update next waypoint timestamp if constraints allow
        if (new_timestamp > old_timestamp)
        {
            next_waypoint->time_from_start = rclcpp::Duration::from_seconds(new_timestamp);
        }
        else
        {
            // RCLCPP_WARN(rclcpp::get_logger("EE_speed_control"), "Average speed is too fast. Moving as fast as joint constraints allow.");
            next_waypoint->time_from_start = rclcpp::Duration::from_seconds(new_timestamp);
            RCLCPP_WARN(node->get_logger(), "%d -> %d :    New Timestamp: %f   |    Old Timestamp: %f", i, i + 1, new_timestamp, old_timestamp);
        }

        // Update the current_end_effector_state for the next iteration
        current_end_effector_state = next_end_effector_state;
    }

    // Update joint velocities and accelerations
    for (int i = 0; i < num_waypoints; i++)
    {
        curr_waypoint = &plan.trajectory.joint_trajectory.points[i];

        if (i > 0)
        {
            prev_waypoint = &plan.trajectory.joint_trajectory.points[i - 1];
        }
        if (i < num_waypoints - 1)
        {
            next_waypoint = &plan.trajectory.joint_trajectory.points[i + 1];
        }

        // Handle timestamp differences for ROS 2
        if (i == 0)
        {
            dt1 = dt2 = rclcpp::Duration(next_waypoint->time_from_start).seconds() - rclcpp::Duration(curr_waypoint->time_from_start).seconds();
        }
        else if (i < num_waypoints - 1)
        {
            dt1 = rclcpp::Duration(curr_waypoint->time_from_start).seconds() - rclcpp::Duration(prev_waypoint->time_from_start).seconds();
            dt2 = rclcpp::Duration(next_waypoint->time_from_start).seconds() - rclcpp::Duration(curr_waypoint->time_from_start).seconds();
        }
        else
        {
            dt1 = dt2 = rclcpp::Duration(curr_waypoint->time_from_start).seconds() - rclcpp::Duration(prev_waypoint->time_from_start).seconds();
        }

        // Iterate over all joints in waypoint
        for (size_t j = 0; j < joint_names.size(); j++)
        {
            if (i == 0)
            {
                q1 = next_waypoint->positions[j];
                q2 = curr_waypoint->positions[j];
                q3 = q1;
            }
            else if (i < num_waypoints - 1)
            {
                q1 = prev_waypoint->positions[j];
                q2 = curr_waypoint->positions[j];
                q3 = next_waypoint->positions[j];
            }
            else
            {
                q1 = prev_waypoint->positions[j];
                q2 = curr_waypoint->positions[j];
                q3 = q1;
            }

            if (dt1 == 0.0 || dt2 == 0.0)
            {
                v1 = v2 = a = 0.0;
            }
            else
            {
                v1 = (q2 - q1) / dt1;
                v2 = (q3 - q2) / dt2;
                a = 2.0 * (v2 - v1) / (dt1 + dt2);
            }

            // Set the velocity and acceleration
            curr_waypoint->velocities[j] = (v1 + v2) / 2.0;
            curr_waypoint->accelerations[j] = a;
        }
    }
}



int main(int argc, char * argv[])
{

  const std::string& ur1_ee = "UR1_wrist_3_link";
  const std::string& ur1_planning_group = "UR1_manipulator";

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
  auto const logger = rclcpp::get_logger("hello_moveit_UR1");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto ur1_move_group_interface = MoveGroupInterface(node, ur1_planning_group);

  // Start the state monitor
  ur1_move_group_interface.startStateMonitor();

//   ur1_move_group_interface.setPlanningTime(10.0);  // Set a higher planning time, e.g., 10 seconds.
//   ur1_move_group_interface.setGoalTolerance(0.1);  // Set a reasonable goal tolerance.

  // Get and print the planning frame ID
  std::string planning_frame = ur1_move_group_interface.getPlanningFrame();
  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());

  // Wait for a while to ensure that the current state is properly updated
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Get Current Robot EE Pose
  moveit::core::RobotStatePtr ur1_current_state = ur1_move_group_interface.getCurrentState();

  Eigen::Affine3d ur1_current_EE_state = ur1_current_state->getGlobalLinkTransform(ur1_ee);

  Eigen::Vector3d ur1_current_EE_position = ur1_current_EE_state.translation();

  RCLCPP_INFO(logger, "UR - 1 Current pose position: [x: %f, y: %f, z: %f]", 
    ur1_current_EE_position(0), 
    ur1_current_EE_position(1), 
    ur1_current_EE_position(2));
  
  //   std::vector<double> current_positions;
  //   moveit::core::RobotStatePtr current_state = ur1_move_group_interface.getCurrentState();
  //   current_state->copyJointGroupPositions("UR1_manipulator", current_positions);

  // Set a target Pose
  auto const ur1_target_pose = []{
    geometry_msgs::msg::Pose ur1_msg;
    ur1_msg.position.x = 0.73;
    ur1_msg.position.y = 0.3;
    ur1_msg.position.z = 1.05;

    // Define roll, pitch, and yaw
    double ur1_roll = 0.0; 
    double ur1_pitch = 3.14; 
    double ur1_yaw = 0.0; 

    // Convert roll, pitch, and yaw to quaternion
    tf2::Quaternion ur1_quaternion;
    ur1_quaternion.setRPY(ur1_roll, ur1_pitch, ur1_yaw);
    ur1_msg.orientation.x = ur1_quaternion.x();
    ur1_msg.orientation.y = ur1_quaternion.y();
    ur1_msg.orientation.z = ur1_quaternion.z();
    ur1_msg.orientation.w = ur1_quaternion.w();

    return ur1_msg;
  }();

  // Set the joint value target with approximation
  //   std::string end_effector_link = "UR1_wrist_3_link";  

  ur1_move_group_interface.setPoseTarget(ur1_target_pose);
  //   move_group_interface.setJointValueTarget(target_pose, end_effector_link);

  // Create a plan to that target pose
  auto const [ur1_success, ur1_plan] = [&ur1_move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan ur1_msg;
    auto const ur1_ok = static_cast<bool>(ur1_move_group_interface.plan(ur1_msg));
    return std::make_pair(ur1_ok, ur1_msg);
  }();

  if(ur1_success) {
    auto ur1_plan_copy = ur1_plan; // Copy the plan to avoid const issues
    EE_speed_control(node, ur1_current_state, ur1_plan_copy, ur1_ee, 0.4); // Pass the copy of the plan

    // Execute the plan after modifying it
    ur1_move_group_interface.execute(ur1_plan_copy);
    RCLCPP_INFO(logger, "UR - 1 has executed the planned motion.");


    // ur1_move_group_interface.asyncExecute(ur1_plan_copy);
    // RCLCPP_INFO(logger, "UR - 1 has executed the planned motion.");


  } else {
    RCLCPP_ERROR(logger, "UR - 1 Planning failed!");
  }

  // Wait for the motion to complete
//   rclcpp::sleep_for(std::chrono::seconds(2));

  ur1_current_state = ur1_move_group_interface.getCurrentState();

  ur1_current_state->update();

  ur1_current_EE_state = ur1_current_state->getGlobalLinkTransform(ur1_ee);

  ur1_current_EE_position = ur1_current_EE_state.translation();

  RCLCPP_INFO(logger, "UR - 1 New pose position: [x: %f, y: %f, z: %f]", 
    ur1_current_EE_position(0), 
    ur1_current_EE_position(1), 
    ur1_current_EE_position(2));

  // Keep the node alive while the motion is being executed
//   rclcpp::spin(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}