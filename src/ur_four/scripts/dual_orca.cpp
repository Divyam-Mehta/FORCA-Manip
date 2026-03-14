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
#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <nav_msgs/msg/odometry.hpp>
#include <future>
#include <limits>
#include <utility>
#include <cstdlib>
#include <unordered_map>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "RVOSimulator.h"
#include "Vector2.h"

// Alias for the action type.
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;



// Global dictionary holding onion poses, populated once using getAllOnionPoses.
std::unordered_map<std::string, Eigen::Vector3d> global_onion_pose_map;

std::vector<std::string> onion_ids;

Eigen::Vector3d ur1_current_EE_vel(0, 0, 0);
Eigen::Vector3d ur2_current_EE_vel(0, 0, 0);

Eigen::Vector3d ur1_prev_EE_position; 
Eigen::Vector3d ur2_prev_EE_position; 

double ur1_speed, ur2_speed;


double min(double a, double b)
{
    if (a > b)
    {
        return b;
    }
    else
    {
        return a;
    }
}


double distance(const Eigen::Vector3d v1, const Eigen::Vector3d v2) 
{
    return (v1 - v2).norm();
}


bool goal_reached(const Eigen::Vector3d current_state, const Eigen::Vector3d goal_state, const double tolerance)
{
    if (distance(current_state, goal_state) < tolerance){
        return true;
    }

    else{
        return false;
    }
}


// Helper function to get one odometry message from a topic within a timeout period.
std::shared_ptr<nav_msgs::msg::Odometry> getOdomMessage(const rclcpp::Node::SharedPtr &node, const std::string &topic, double timeout_sec = 1.0)
{
  // Use a shared_ptr for the promise so it can be copied in the lambda.
  auto prom_ptr = std::make_shared<std::promise<std::shared_ptr<nav_msgs::msg::Odometry>>>();
  auto fut = prom_ptr->get_future();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  // Create a temporary subscription.
  auto subscription = node->create_subscription<nav_msgs::msg::Odometry>(
      topic, qos,
      // Capture prom_ptr by value.
      [prom_ptr](const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Try to set the promise value.
        // If it’s already set, this will throw an exception, which we catch.
        try {
          prom_ptr->set_value(msg);
        } catch (const std::future_error & e) {
          // The promise was already satisfied.
        }
      });

  // Wait until a message is received or timeout.
  auto start = std::chrono::steady_clock::now();
  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    auto status = fut.wait_for(std::chrono::milliseconds(100));
    if (status == std::future_status::ready)
    {
      return fut.get();
    }
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now() - start);
    if (elapsed.count() > timeout_sec)
    {
      RCLCPP_WARN(node->get_logger(), "Timeout while waiting for message on topic: %s", topic.c_str());
      break;
    }
    rate.sleep();
  }
  return nullptr;
}



// Returns a vector of topic names starting with "/model"
std::vector<std::string> getModelTopics(const rclcpp::Node::SharedPtr & node)
{
  // Wait for 5 seconds to allow all topics to be discovered
  std::this_thread::sleep_for(std::chrono::seconds(5));

  std::vector<std::string> model_topics;
  // Retrieve all available topics and their types
  auto topics = node->get_topic_names_and_types();
  
  // Iterate over each topic
  for (const auto & topic_pair : topics)
  {
    // Check if the topic name starts with "/model"
    if (topic_pair.first.rfind("/model", 0) == 0)
    {
      model_topics.push_back(topic_pair.first);
    }
  }
  return model_topics;
}



// Function to extract onion IDs from topics formatted as "/model/{onion_id}/odometry"
std::vector<std::string> extractOnionIDs(const std::vector<std::string>& model_topics)
{
    std::vector<std::string> onion_ids;
    const std::string prefix = "/model/";
    
    for (const auto& topic : model_topics)
    {
        // Check if the topic starts with "/model/"
        if (topic.find(prefix) == 0)
        {
            // Remove the "/model/" prefix
            std::string remaining = topic.substr(prefix.size());
            // Find the position of the next "/" which should be before "odometry"
            size_t pos = remaining.find("/");
            if (pos != std::string::npos)
            {
                // Extract the onion id which is the substring from the start to the found "/"
                std::string onion_id = remaining.substr(0, pos);
                onion_ids.push_back(onion_id);
            }
        }
    }
    return onion_ids;
}



// Returns a map of all onions with onion_id as key and their position (x, y, z) as value.
std::unordered_map<std::string, Eigen::Vector3d> getAllOnionPoses(const rclcpp::Node::SharedPtr &node, const std::vector<std::string>& onion_ids)
{
    std::unordered_map<std::string, Eigen::Vector3d> global_onion_pose_map;
    
    // Iterate through each onion id.
    for (const auto &onion_id : onion_ids)
    {
        // Form the topic name.
        std::string topic = "/model/" + onion_id + "/odometry";

        // Retrieve the odometry message (with a 5-second timeout).
        auto odom_msg = getOdomMessage(node, topic, 5.0);
        if (!odom_msg)
        {
            RCLCPP_WARN(node->get_logger(), "No odometry message received for onion id: %s", onion_id.c_str());
            continue;
        }

        // Extract the onion position from the odometry message.
        const auto &p = odom_msg->pose.pose.position;
        Eigen::Vector3d onion_position(p.x, p.y, p.z);

        // Insert the position into the map with the onion_id as the key.
        global_onion_pose_map[onion_id] = onion_position;
    }
    
    return global_onion_pose_map;
}



geometry_msgs::msg::Pose set_pose_target(const Eigen::Vector3d& ur_goal_EE_position, const Eigen::Vector3d& ur_goal_EE_orientation)
{
    geometry_msgs::msg::Pose ur_msg;

        ur_msg.position.x = ur_goal_EE_position(0); 
        ur_msg.position.y = ur_goal_EE_position(1);
        ur_msg.position.z = ur_goal_EE_position(2); 
        

        // Define roll, pitch, and yaw
        double ur_roll = ur_goal_EE_orientation(0); 
        double ur_pitch = ur_goal_EE_orientation(1); 
        double ur_yaw = ur_goal_EE_orientation(2); 


        // Convert roll, pitch, and yaw to quaternion
        tf2::Quaternion ur_quaternion;
        ur_quaternion.setRPY(ur_roll, ur_pitch, ur_yaw);
        ur_msg.orientation.x = ur_quaternion.x();
        ur_msg.orientation.y = ur_quaternion.y();
        ur_msg.orientation.z = ur_quaternion.z();
        ur_msg.orientation.w = ur_quaternion.w();

        return ur_msg;
}



std::pair<std::string, Eigen::Vector3d> findClosestOnionQuick(
    const Eigen::Vector3d &robot_ee_position,
    std::vector<std::string> &onion_ids,
    const std::string &robot_id,
    moveit::core::RobotStatePtr &current_state,
    const moveit::core::JointModelGroup* joint_model_group)
{
    double min_distance = std::numeric_limits<double>::max();
    std::string closest_onion_id = "";
    Eigen::Vector3d closest_onion_position(0.0, 0.0, 0.0);

    // Iterate over the provided onion IDs.
    for (const auto &onion_id : onion_ids)
    {
        // Look up the onion's pose.
        auto it = global_onion_pose_map.find(onion_id);
        if (it == global_onion_pose_map.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("findClosestOnionQuick"),
                        "Onion ID '%s' not found in global pose map", onion_id.c_str());
            continue;
        }
        const Eigen::Vector3d &onion_position = it->second;

        // Filter based on the robot_id criteria.
        if (robot_id == "ur1")
        {
            if (onion_position.x() >= 0.80 || onion_position.y() < -0.5 || onion_position.y() > 0.5)
                continue;
        }
        else if (robot_id == "ur2")
        {
            if (onion_position.x() <= 0.70 || onion_position.y() < -0.5 || onion_position.y() > 0.5)
                continue;
        }

        // Create the goal pose: x and y from the onion, fixed z of 0.96.
        Eigen::Vector3d goal_position_1(onion_position.x(), onion_position.y(), 0.96);
        Eigen::Vector3d goal_position_2(onion_position.x(), onion_position.y(), 1.0);
        
        // Fixed orientation: roll=3.14, pitch=0, yaw=3.14.
        Eigen::Vector3d goal_orientation(3.14, 0, 3.14);
        
        geometry_msgs::msg::Pose target_pose_1 = set_pose_target(goal_position_1, goal_orientation);
        geometry_msgs::msg::Pose target_pose_2 = set_pose_target(goal_position_2, goal_orientation);

        // Attempt to find an IK solution for this target pose.
        double timeout = 1.0;

        bool ik_found_1 = current_state->setFromIK(joint_model_group, target_pose_1, timeout);
        bool ik_found_2 = current_state->setFromIK(joint_model_group, target_pose_2, timeout);
        
        if (!ik_found_1 || !ik_found_2)
        {
            // IK not found for this onion; skip it.
            continue;
        }

        // Compute distance from the robot's end-effector position to the onion.
        double distance = (robot_ee_position - onion_position).norm();
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_onion_id = onion_id;
            closest_onion_position = onion_position;
        }
    }

    // Remove the closest onion from the list if one was found.
    if (!closest_onion_id.empty())
    {
        auto it = std::remove(onion_ids.begin(), onion_ids.end(), closest_onion_id);
        onion_ids.erase(it, onion_ids.end());
    }

    return std::make_pair(closest_onion_id, closest_onion_position);
}



void publishToTopic(const std::string &service, const std::string &ur_id, const std::string &onion_id)
{
  // Construct the topic name in the format: /{service}_{ur_id}_{onion_id}
  std::string topic = "/" + service + "_" + ur_id + "_" + onion_id;

  // Construct the full command string.
  // The command: ros2 topic pub /<topic> std_msgs/msg/Empty "{}" -1
  std::ostringstream command_stream;
  command_stream << "ros2 topic pub " << topic << " std_msgs/msg/Empty \"{}\" -1";

  std::string command = command_stream.str();
  
  // Optionally print the command for debugging.
  std::cout << "Executing command: " << command << std::endl;

  // Publish multiple messages with a small delay in between.
  for (int i = 0; i < 3; ++i) {
    int ret = std::system(command.c_str());
    if(ret != 0) {
      std::cerr << "Command failed with return code: " << ret << std::endl;
    }
    // Sleep for 100 milliseconds between messages.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}



geometry_msgs::msg::Pose set_vel_pose_target(const Eigen::Vector3d& ur_current_EE_position, const Eigen::Vector3d& ur_target_unit_vec, double len, const Eigen::Vector3d& ur_goal_EE_orientation, bool ur_goal_reached)
{
    geometry_msgs::msg::Pose ur_msg;

    // Set position based on current state and target vector
    if (!ur_goal_reached)
    {
        ur_msg.position.x = ur_current_EE_position(0) + ur_target_unit_vec(0) * len;
        ur_msg.position.y = ur_current_EE_position(1) + ur_target_unit_vec(1) * len;
        ur_msg.position.z = ur_current_EE_position(2) + ur_target_unit_vec(2) * len;
    }
    else
    {
        ur_msg.position.x = ur_current_EE_position(0);
        ur_msg.position.y = ur_current_EE_position(1);
        ur_msg.position.z = ur_current_EE_position(2);
    }

    // Define roll, pitch, and yaw
    double ur_roll = ur_goal_EE_orientation(0);
    double ur_pitch = ur_goal_EE_orientation(1);
    double ur_yaw = ur_goal_EE_orientation(2);

    // Convert roll, pitch, and yaw to quaternion
    tf2::Quaternion ur_quaternion;
    ur_quaternion.setRPY(ur_roll, ur_pitch, ur_yaw);
    ur_msg.orientation.x = ur_quaternion.x();
    ur_msg.orientation.y = ur_quaternion.y();
    ur_msg.orientation.z = ur_quaternion.z();
    ur_msg.orientation.w = ur_quaternion.w();

    return ur_msg;
}



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
            // RCLCPP_WARN(node->get_logger(), "%d -> %d :    New Timestamp: %f   |    Old Timestamp: %f", i, i + 1, new_timestamp, old_timestamp);
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



// Create a struct to wrap simulator data per robot
struct RVO2Wrapper
{
  std::shared_ptr<RVO::RVOSimulator> sim;
  size_t self_agent;
  size_t other_agent;
  bool initialized = false;
};

// Returns the suggested velocity from RVO2
Eigen::Vector3d getRVO2Velocity(
  RVO2Wrapper & rvo,
  const Eigen::Vector3d & current_EE_position,
  const Eigen::Vector3d & current_EE_velocity,
  const Eigen::Vector3d & other_robot_EE_position,
  const Eigen::Vector3d & other_robot_EE_velocity,
  const Eigen::Vector3d & goal_EE_position,
  double pref_speed,
  rclcpp::Logger logger,
  const std::string & robot_id)
{
  const double interaction_threshold = 0.15;

  // Lazy initialize RVO2 simulator and agents
  if (!rvo.initialized) {
    rvo.sim = std::make_shared<RVO::RVOSimulator>();
    rvo.sim->setTimeStep(0.1f);
    rvo.sim->setAgentDefaults(2.0f, 2, 1.0f, 1.0f, 0.1f, 0.5f);

    rvo.self_agent = rvo.sim->addAgent(RVO::Vector2(current_EE_position.x(), current_EE_position.y()));
    rvo.other_agent = rvo.sim->addAgent(RVO::Vector2(other_robot_EE_position.x(), other_robot_EE_position.y()));
    rvo.initialized = true;
  }

  // Update agent positions
  rvo.sim->setAgentPosition(rvo.self_agent, RVO::Vector2(current_EE_position.x(), current_EE_position.y()));
  rvo.sim->setAgentPosition(rvo.other_agent, RVO::Vector2(other_robot_EE_position.x(), other_robot_EE_position.y()));

  // Set preferred velocities
  Eigen::Vector3d to_goal = goal_EE_position - current_EE_position;
  to_goal.z() = 0.0;
  Eigen::Vector2d to_goal_2D(to_goal.x(), to_goal.y());

  RVO::Vector2 pref_vel(0.0f, 0.0f);
  if (to_goal_2D.norm() > 1e-3) {
    pref_vel = RVO::Vector2(to_goal_2D.x(), to_goal_2D.y());
    pref_vel = pref_speed * RVO::normalize(pref_vel);
  }

  rvo.sim->setAgentPrefVelocity(rvo.self_agent, pref_vel);
  rvo.sim->setAgentPrefVelocity(rvo.other_agent, RVO::Vector2(other_robot_EE_velocity.x(), other_robot_EE_velocity.y()));

  // Compute distance between agents
  double distance = (current_EE_position.head<2>() - other_robot_EE_position.head<2>()).norm();

  
  // Run RVO step only when agents are too close
  rvo.sim->doStep();
  RVO::Vector2 new_vel = rvo.sim->getAgentVelocity(rvo.self_agent);
  RCLCPP_INFO(logger, "[%s] RVO2 ACTIVE: vel = (%.2f, %.2f),   |   distance = %.3f", robot_id.c_str(), new_vel.x(), new_vel.y(), distance);
  return Eigen::Vector3d(new_vel.x(), new_vel.y(), 0.0);
}



RVO2Wrapper ur1_rvo, ur2_rvo;



// It sends the trajectory to the action server and waits for execution to finish.
void executeTrajectoryAndWait(
  const rclcpp::Node::SharedPtr & node,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::string & action_name,
  const std::string & robot_id)
{
  auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(node, action_name);

  if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "[%s] Action server not available on '%s'",
                 robot_id.c_str(), action_name.c_str());
    return;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = trajectory;

  // Create a promise to block until the execution completes.
  auto promise_ptr = std::make_shared<std::promise<void>>();
  auto future = promise_ptr->get_future();

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =
    [node, robot_id, promise_ptr](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "[%s] Execution succeeded", robot_id.c_str());
      } else {
        RCLCPP_ERROR(node->get_logger(), "[%s] Execution failed", robot_id.c_str());
      }
      promise_ptr->set_value();
    };

  RCLCPP_INFO(node->get_logger(), "[%s] Sending plan for asynchronous execution...", robot_id.c_str());
  action_client->async_send_goal(goal, send_goal_options);

  // Wait until the trajectory execution has finished.
  future.wait();
}



// RViz Marker Publish Function
void publish_marker(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & marker_pub,
                    const Eigen::Vector3d & position,
                    const std::string & robot_id,
                    int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";  // or your base frame
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = robot_id + "_marker";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.a = 1.0;
  marker.color.r = (robot_id == "ur1") ? 1.0 : 0.0;
  marker.color.g = 0.0;
  marker.color.b = (robot_id == "ur2") ? 1.0 : 0.0;

  marker_pub->publish(marker);
}



// Continuous planning and execution loop for one robot.
void runRobot(const rclcpp::Node::SharedPtr & node,
              const std::string & planning_group,
              const moveit::core::JointModelGroup* joint_model_group,
              const std::string & robot_id,
              const std::string & action_name,
              const std::string & robot_ee,
              const std::string & other_robot_ee,
              const std::vector<std::vector<double>> gz_poses,
              moveit::planning_interface::MoveGroupInterface & move_group,
              moveit::planning_interface::MoveGroupInterface & other_robot_move_group,
              const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & marker_pub)
{
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> goal_joint_values;


  // Gazebo World Poses
  std::vector<double> inspect_pos = gz_poses[0];
  std::vector<double> inspect_approach = gz_poses[1];
  std::vector<double> bin_approach = gz_poses[2];
  std::vector<double> bin_pos = gz_poses[3];

 

//   // Create a MoveGroupInterface instance for this robot.
//   moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
//   move_group.setMaxVelocityScalingFactor(1.0);
//   move_group.setMaxAccelerationScalingFactor(1.0);
//   move_group.setPlanningTime(20);

//   move_group.startStateMonitor();


  

  // Get Current Robot End-Effector State
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  Eigen::Affine3d current_EE_state = current_state->getGlobalLinkTransform(robot_ee);
  Eigen::Vector3d current_EE_position = current_EE_state.translation();

  move_group.setStartStateToCurrentState();

  moveit::core::RobotStatePtr other_robot_current_state = other_robot_move_group.getCurrentState();
  Eigen::Affine3d other_robot_current_EE_state = other_robot_current_state->getGlobalLinkTransform(other_robot_ee);
  Eigen::Vector3d other_robot_current_EE_position = other_robot_current_EE_state.translation();





  Eigen::MatrixXd jacobian = current_state->getJacobian(joint_model_group);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  double cond_num = svd.singularValues()(0) / svd.singularValues().tail(1)(0);
  RCLCPP_INFO(node->get_logger(), "[%s] Condition number: %f", robot_id.c_str(), cond_num);

  if (cond_num > 1000.0) {
    RCLCPP_WARN(node->get_logger(), "[%s] Near-singular configuration!", robot_id.c_str());
  }





  // Define Robot End-Effector Goal Pose
  Eigen::Vector3d goal_EE_position, goal_EE_orientation;  

  std::string target_id;

  bool robot_goal_reached;

  // ORCA Params

  double epsilon = 0.02;
  double max_len = 0.2;
  double alpha = 0.5;
  double robot_speed = 0.2;
  double pref_speed = 0.2; 
  double max_speed = 1.5;
  double stagnant_count = 0;
  
  double len;

  Eigen::Vector3d current_EE_vel, other_robot_current_EE_vel;


  // Initialize variables

  if (robot_id == "ur1") 
  {
    current_EE_vel = ur1_current_EE_vel;
    ur1_speed = pref_speed;
    other_robot_current_EE_vel = ur2_current_EE_vel;
    ur2_speed = pref_speed;

    ur1_prev_EE_position = current_EE_position;
    ur2_prev_EE_position = other_robot_current_EE_position; 
  } 
  else 
  {
    current_EE_vel = ur2_current_EE_vel;
    ur2_speed = pref_speed;
    other_robot_current_EE_vel = ur1_current_EE_vel;
    ur1_speed = pref_speed;

    ur2_prev_EE_position = current_EE_position;
    ur1_prev_EE_position = other_robot_current_EE_position;
  }



  bool safe_psort_start = true;

  while (rclcpp::ok())
  { 

    // Assign Closest Onion Pose to the Robot
    auto closest_onion = findClosestOnionQuick(current_EE_position, onion_ids, robot_id, current_state, joint_model_group);
    RCLCPP_INFO(node->get_logger(), "Closest onion to [%s] is [%s] at position (%f, %f, %f)",
                robot_id.c_str(),
                closest_onion.first.c_str(),
                closest_onion.second.x(), closest_onion.second.y(), closest_onion.second.z());

    
    robot_goal_reached = closest_onion.first.empty();

    if(robot_goal_reached)
    {
        break;
    }



    // PSort Status Info
    bool onion_target_reached = false;
    bool onion_target_approach = false;
    bool onion_target_retreat = false;
    bool inspection_pose_reached = false;
    bool inspection_pose_approach = false;
    bool inspection_pose_retreat = false;
    bool bin_pose_approach = false;
    bool bin_reached = false;

    
    
    while(!robot_goal_reached)   // One whole Pass of PSort
    {

        RCLCPP_INFO(node->get_logger(), " ");
        RCLCPP_INFO(node->get_logger(), "[%s]   CHECKPOINT", robot_id.c_str());
        RCLCPP_INFO(node->get_logger(), " ");

        // Get Current Robot End-Effector State

        current_state = move_group.getCurrentState();
        current_EE_state = current_state->getGlobalLinkTransform(robot_ee);
        current_EE_position = current_EE_state.translation();

        publish_marker(marker_pub, current_EE_position, robot_id, 0);



        // -------------------------------- Safety Mechanism -------------------------------- //

        if(!safe_psort_start)
        {
            if((robot_id == "ur1") && (closest_onion.second.y() > 0))
            {
                if(!safe_psort_start)
                {
                    goal_EE_position[0] = bin_approach[0];
                    goal_EE_position[1] = bin_approach[1];
                    goal_EE_position[2] = bin_approach[2];

                    goal_EE_orientation[0] = bin_approach[3];
                    goal_EE_orientation[1] = bin_approach[4];
                    goal_EE_orientation[2] = bin_approach[5];

                    RCLCPP_INFO(node->get_logger(), " ");
                    RCLCPP_INFO(node->get_logger(), "[%s] Safety Mechanism Activation Case", robot_id.c_str());
                    RCLCPP_INFO(node->get_logger(), " ");


                    robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
                    if (robot_goal_reached)
                    {
                        safe_psort_start = true;
                        robot_goal_reached = false;
                        continue;
                    }

                    len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
                }
            }

            else if((robot_id == "ur2") && (closest_onion.second.y() < 0))
            {
                if(!safe_psort_start)
                {
                    goal_EE_position[0] = bin_approach[0];
                    goal_EE_position[1] = bin_approach[1];
                    goal_EE_position[2] = bin_approach[2];

                    goal_EE_orientation[0] = bin_approach[3];
                    goal_EE_orientation[1] = bin_approach[4];
                    goal_EE_orientation[2] = bin_approach[5];

                    RCLCPP_INFO(node->get_logger(), " ");
                    RCLCPP_INFO(node->get_logger(), "[%s] Safety Mechanism Activation Case", robot_id.c_str());
                    RCLCPP_INFO(node->get_logger(), " ");


                    robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
                    if (robot_goal_reached)
                    {
                        safe_psort_start = true;
                        robot_goal_reached = false;
                        continue;
                    }

                    len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
                }
            }

            else
            {
                safe_psort_start = true;
                continue;
            }
        }



        // --------------------------------  PSort  Procedure  -------------------------------- //

        else
        {
        
        // Set Target Pose to the closest onion available
        if(!onion_target_reached)        // Change "if" to "else if" in case you uncomment the Safety Mechanism
        {
            target_id = closest_onion.first;
            goal_EE_position[0] = closest_onion.second.x();
            goal_EE_position[1] = closest_onion.second.y();
            goal_EE_position[2] = 1.0;

            goal_EE_orientation[0] = 3.14;
            goal_EE_orientation[1] = 0.0;
            goal_EE_orientation[2] = 3.14;

            RCLCPP_INFO(node->get_logger(), " ");
            RCLCPP_INFO(node->get_logger(), "[%s] Onion Target Approach Case", robot_id.c_str());
            RCLCPP_INFO(node->get_logger(), " ");

            robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
            if (robot_goal_reached)
            {
                onion_target_reached = true;
            }

            len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
        }

        // Gripper Down
        else if(!onion_target_approach)
        {
            goal_EE_position[0] = closest_onion.second.x();
            goal_EE_position[1] = closest_onion.second.y();
            goal_EE_position[2] = 0.95;

            goal_EE_orientation[0] = 3.14;
            goal_EE_orientation[1] = 0.0;
            goal_EE_orientation[2] = 3.14;

            RCLCPP_INFO(node->get_logger(), " ");
            RCLCPP_INFO(node->get_logger(), "[%s] Gripper Moving Down Case", robot_id.c_str());
            RCLCPP_INFO(node->get_logger(), " ");


            robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
            if (robot_goal_reached)
            {
                onion_target_approach = true;

                // publishToTopic("attach", robot_id, target_id);
            }

            len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
        }

        // Gripper Up
        else if (!onion_target_retreat)
        {
            goal_EE_position[0] = closest_onion.second.x();
            goal_EE_position[1] = closest_onion.second.y();
            goal_EE_position[2] = 1.0;

            goal_EE_orientation[0] = 3.14;
            goal_EE_orientation[1] = 0.0;
            goal_EE_orientation[2] = 3.14;

            RCLCPP_INFO(node->get_logger(), " ");
            RCLCPP_INFO(node->get_logger(), "[%s] Gripper Moving Up Case", robot_id.c_str());
            RCLCPP_INFO(node->get_logger(), " ");


            robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
            if (robot_goal_reached)
            {
                onion_target_retreat = true;
            }

            len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
        }        

        // Inspection Position Reached
        else if (!inspection_pose_reached)
        {
            goal_EE_position[0] = inspect_pos[0];
            goal_EE_position[1] = inspect_pos[1];
            goal_EE_position[2] = inspect_pos[2];

            goal_EE_orientation[0] = inspect_pos[3];
            goal_EE_orientation[1] = inspect_pos[4];
            goal_EE_orientation[2] = inspect_pos[5];

            RCLCPP_INFO(node->get_logger(), " ");
            RCLCPP_INFO(node->get_logger(), "[%s] Inspection Pose Location Approach Case", robot_id.c_str());
            RCLCPP_INFO(node->get_logger(), " ");


            robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
            if (robot_goal_reached)
            {
                inspection_pose_reached = true;
            }

            len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
        }

        // Inspection Orientation Approach
        else if (!inspection_pose_approach)
        {
            goal_EE_position[0] = inspect_approach[0];
            goal_EE_position[1] = inspect_approach[1];
            goal_EE_position[2] = inspect_approach[2];

            goal_EE_orientation[0] = inspect_approach[3];
            goal_EE_orientation[1] = inspect_approach[4];
            goal_EE_orientation[2] = inspect_approach[5];

            RCLCPP_INFO(node->get_logger(), " ");
            RCLCPP_INFO(node->get_logger(), "[%s] Camera Inspection Orientation Approach Case", robot_id.c_str());
            RCLCPP_INFO(node->get_logger(), " ");


            robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
            if (robot_goal_reached)
            {
                inspection_pose_approach = true;
            }

            len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
        }

        // Inspection Position Retreat
        else if (!inspection_pose_retreat)
        {
            goal_EE_position[0] = inspect_pos[0];
            goal_EE_position[1] = inspect_pos[1];
            goal_EE_position[2] = inspect_pos[2];

            goal_EE_orientation[0] = inspect_pos[3];
            goal_EE_orientation[1] = inspect_pos[4];
            goal_EE_orientation[2] = inspect_pos[5];

            RCLCPP_INFO(node->get_logger(), " ");
            RCLCPP_INFO(node->get_logger(), "[%s] Inspection Pose Location Retreat Case", robot_id.c_str());
            RCLCPP_INFO(node->get_logger(), " ");


            robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
            if (robot_goal_reached)
            {
                inspection_pose_retreat = true;
            }

            len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
        }

        // Bin Approach Position
        else if (!bin_pose_approach)
        {
            goal_EE_position[0] = bin_approach[0];
            goal_EE_position[1] = bin_approach[1];
            goal_EE_position[2] = bin_approach[2];

            goal_EE_orientation[0] = bin_approach[3];
            goal_EE_orientation[1] = bin_approach[4];
            goal_EE_orientation[2] = bin_approach[5];

            RCLCPP_INFO(node->get_logger(), " ");
            RCLCPP_INFO(node->get_logger(), "[%s] Near Bin Pose Location Approach Case", robot_id.c_str());
            RCLCPP_INFO(node->get_logger(), " ");


            robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
            if (robot_goal_reached)
            {
                bin_pose_approach = true;
            }

            len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
        }

        // Bin Reached
        else if (!bin_reached)
        {
            goal_EE_position[0] = bin_pos[0];
            goal_EE_position[1] = bin_pos[1];
            goal_EE_position[2] = bin_pos[2];

            goal_EE_orientation[0] = bin_pos[3];
            goal_EE_orientation[1] = bin_pos[4];
            goal_EE_orientation[2] = bin_pos[5];

            RCLCPP_INFO(node->get_logger(), " ");
            RCLCPP_INFO(node->get_logger(), "[%s] Exact Bin Pose Location Approach Case", robot_id.c_str());
            RCLCPP_INFO(node->get_logger(), " ");


            robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
            if (robot_goal_reached)
            {
                bin_reached = true;

                // publishToTopic("detach", robot_id, target_id);

                len = max_len;

                safe_psort_start = false;
            }

            len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
        }

        }



        // ----------------------------------------------------------------------------------------------------------------------------------------- //



        // Plan for those Goal Position using ORCA

        other_robot_current_state = other_robot_move_group.getCurrentState();
        other_robot_current_EE_state = other_robot_current_state->getGlobalLinkTransform(other_robot_ee);
        other_robot_current_EE_position = other_robot_current_EE_state.translation();

        Eigen::Vector3d next_EE_vel;
        Eigen::Vector3d target_unit_vec;
        
        const double interaction_threshold = 0.12;

        // Compute distance between agents
        double robot_distance = (current_EE_position.head<2>() - other_robot_current_EE_position.head<2>()).norm();


        if (robot_distance < interaction_threshold)
        {
          if (robot_id == "ur1") 
          {
              next_EE_vel = getRVO2Velocity(ur1_rvo, current_EE_position, ur1_current_EE_vel,
                                              other_robot_current_EE_position, ur2_current_EE_vel,
                                              goal_EE_position, pref_speed,
                                              node->get_logger(), robot_id);
              ur1_current_EE_vel = next_EE_vel;
              ur1_prev_EE_position = current_EE_position;
              ur1_speed = next_EE_vel.norm();

              target_unit_vec = ur1_current_EE_vel/ur1_speed;
          } 
          
          else 
          {
              next_EE_vel = getRVO2Velocity(ur2_rvo, current_EE_position, ur2_current_EE_vel,
                                              other_robot_current_EE_position, ur1_current_EE_vel,
                                              goal_EE_position, pref_speed,
                                              node->get_logger(), robot_id);
              ur2_current_EE_vel = next_EE_vel;
              ur2_prev_EE_position = current_EE_position;
              ur2_speed = next_EE_vel.norm();

              target_unit_vec = ur2_current_EE_vel/ur2_speed;
          }
        }



        else
        {
          if (robot_id == "ur1")
          {
            target_unit_vec = (goal_EE_position - current_EE_position)/distance(current_EE_position, goal_EE_position); 
            
            ur1_speed = pref_speed;
            ur1_current_EE_vel = target_unit_vec*pref_speed;
            ur1_prev_EE_position = current_EE_position;
          }

          else
          {
            target_unit_vec = (goal_EE_position - current_EE_position)/distance(current_EE_position, goal_EE_position); 
            
            ur2_speed = pref_speed;
            ur2_current_EE_vel = target_unit_vec*pref_speed;
            ur2_prev_EE_position = current_EE_position;
          }
        }


        // ----------------------------------------------------------------------------------------------------------------------------------------- //



        // Set the Pose Target
        geometry_msgs::msg::Pose target_pose = set_vel_pose_target(current_EE_position, target_unit_vec, len, goal_EE_orientation, robot_goal_reached);



        // Solve for the IK Solution of the Target Pose
        double timeout = 1.0;
        bool ik_found = current_state->setFromIK(joint_model_group, target_pose, timeout);

        if (ik_found)
        {
            current_state->copyJointGroupPositions(joint_model_group, goal_joint_values);
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Did not find IK solution for [%s]", robot_id.c_str());
        }
        


        // Set the target joint values.
        move_group.setJointValueTarget(joint_names, goal_joint_values);
        


        // Plan the motion.
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success)
        {
        // Create a copy of the plan to modify.
            auto plan_copy = plan;

            // Adjust the trajectory based on the desired robot speed.
            EE_speed_control(node, current_state, plan_copy, robot_ee, robot_speed);

            trajectory_msgs::msg::JointTrajectory trajectory = plan_copy.trajectory.joint_trajectory;
            // Execute the trajectory and wait for its completion.
            executeTrajectoryAndWait(node, trajectory, action_name, robot_id);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "[%s] Planning failed!", robot_id.c_str());
            // Optionally, wait a short time before retrying.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!bin_reached)
        {
            robot_goal_reached = false;
        }
    }
  }
}






int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create one node to be used by both MoveGroupInterface instances.
  auto node = rclcpp::Node::make_shared("dual_ur_async_moveit_control_node");
  
  // Create a MultiThreadedExecutor so that callbacks for each robot can run concurrently.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  // Spin the executor in a separate thread.
  std::thread spin_thread([&executor]() { executor.spin(); });



  const std::string& ur1_ee = "UR1_wrist_3_link";
  const std::string& ur2_ee = "UR2_wrist_3_link";

  const std::string& ur1_planning_group = "UR1_manipulator";
  const std::string& ur2_planning_group = "UR2_manipulator";



  // Get Robot Joint Information

  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

  const moveit::core::JointModelGroup* ur1_joint_model_group = kinematic_model->getJointModelGroup(ur1_planning_group);
  const moveit::core::JointModelGroup* ur2_joint_model_group = kinematic_model->getJointModelGroup(ur2_planning_group);



  // Process Onion Data

  // ------------------------------------------------------------------------------------- //
    
  // Get and log topics starting with "/model"
  std::vector<std::string> model_topics = getModelTopics(node);

  if (model_topics.empty())
  {
      RCLCPP_INFO(node->get_logger(), "No topics starting with '/model' found.");
  }
  else
  {
      std::stringstream ss;
      ss << "Found " << model_topics.size() << " topics starting with '/model':";
      for (const auto & topic : model_topics)
      {
      ss << "\n  " << topic;
      }
      RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
  }


  // ------------------------------------------------------------------------------------- //


  // Extract onion IDs from the model_topics and log them
  onion_ids = extractOnionIDs(model_topics);
  if (onion_ids.empty())
  {
      RCLCPP_INFO(node->get_logger(), "No onion IDs found in the topics.");
  }
  else
  {
      std::stringstream ss;
      ss << "Extracted onion IDs:";
      for (const auto & id : onion_ids)
      {
      ss << "\n  " << id;
      }
      RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
  }


  // ------------------------------------------------------------------------------------- //


  // Retrieve all onion positions as a dictionary (unordered_map).
  global_onion_pose_map = getAllOnionPoses(node, onion_ids);

  if (global_onion_pose_map.empty())
  {
      RCLCPP_INFO(node->get_logger(), "No onion locations found.");
  }
  else
  {
      // Log each onion location.
      for (const auto& pair : global_onion_pose_map)
      {
          const std::string& onion_id = pair.first;
          const Eigen::Vector3d& position = pair.second;
          RCLCPP_INFO(node->get_logger(), "Onion ID: %s, Position: [x: %.3f, y: %.3f, z: %.3f]",
                      onion_id.c_str(), position.x(), position.y(), position.z());
      }
  }


  // ------------------------------------------------------------------------------------- //


  // Poses in Gazebo World

  std::vector<double> ur1_bin = {0.35, -0.49, 1.0, 3.14, 0.0, 3.14};
  std::vector<double> ur2_bin = {1.16, 0.49, 1.0, 3.14, 0.0, 3.14};

  std::vector<double> ur1_bin_approach = {0.63, -0.35, 1.0, 3.14, 0.0, 3.14};
  std::vector<double> ur2_bin_approach = {0.87, 0.35, 1.0, 3.14, 0.0, 3.14};

  std::vector<double> ur1_inspect_pos = {0.6, 0.28, 1.0, 3.14, 0, 3.14};
  std::vector<double> ur2_inspect_pos = {0.9, 0.28, 1.0, 3.14, 0, 3.14};

  std::vector<double> ur1_inspect_approach = {0.6, 0.28, 1.0, -3.14, -1.3, -3.14};
  std::vector<double> ur2_inspect_approach = {0.9, 0.28, 1.0, -3.14, 1.3, -3.14};

  std::vector<std::vector<double>> ur1_gz_poses = {
    ur1_inspect_pos,
    ur1_inspect_approach,
    ur1_bin_approach,
    ur1_bin
  };

  std::vector<std::vector<double>> ur2_gz_poses = {
    ur2_inspect_pos,
    ur2_inspect_approach,
    ur2_bin_approach,
    ur2_bin
  };


  // Create a MoveGroupInterface instance for both the robots.
  moveit::planning_interface::MoveGroupInterface ur1_move_group(node, ur1_planning_group);
  ur1_move_group.setMaxVelocityScalingFactor(1.0);
  ur1_move_group.setMaxAccelerationScalingFactor(1.0);
  ur1_move_group.setPlanningTime(20);

  ur1_move_group.startStateMonitor();
  
  ur1_move_group.setStartStateToCurrentState();


  moveit::planning_interface::MoveGroupInterface ur2_move_group(node, ur2_planning_group);
  ur2_move_group.setMaxVelocityScalingFactor(1.0);
  ur2_move_group.setMaxAccelerationScalingFactor(1.0);
  ur2_move_group.setPlanningTime(20);

  ur2_move_group.startStateMonitor();

  ur2_move_group.setStartStateToCurrentState();

  
  // Define the action server names for each robot.
  std::string ur1_action_name = "joint_trajectory_controller_ur1/follow_joint_trajectory";
  std::string ur2_action_name = "joint_trajectory_controller_ur2/follow_joint_trajectory";
  
  // Create to Markers for visualizing in RViz
  auto ur1_marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("ur1_marker", 10);
  auto ur2_marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("ur2_marker", 10);

  // Start continuous planning/execution for each robot in separate threads.
  std::thread ur1_thread(runRobot, node, ur1_planning_group, ur1_joint_model_group, "ur1", ur1_action_name, ur1_ee, ur2_ee, ur1_gz_poses, std::ref(ur1_move_group), std::ref(ur2_move_group), ur1_marker_pub);
  std::thread ur2_thread(runRobot, node, ur2_planning_group, ur2_joint_model_group, "ur2", ur2_action_name, ur2_ee, ur1_ee, ur2_gz_poses, std::ref(ur2_move_group), std::ref(ur1_move_group), ur2_marker_pub);
  
  // Wait for both threads to finish.
  ur1_thread.join();
  ur2_thread.join();
  
  // Shutdown the executor and ROS.
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  
  return 0;
}