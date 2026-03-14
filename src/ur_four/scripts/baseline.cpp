#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

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
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/dynamic_detachable_joint.pb.h>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/transport/MessageInfo.hh>

#include <atomic>

#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>



namespace gztr = gz::transport;
namespace gzmsgs = gz::msgs;


// Alias for the action type.
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;



// Global dictionary holding onion poses, populated once using getAllOnionPoses.
std::unordered_map<std::string, Eigen::Vector3d> global_onion_pose_map;

std::vector<std::string> onion_ids;

Eigen::Vector3d ur1_current_EE_vel(0, 0, 0);
Eigen::Vector3d ur2_current_EE_vel(0, 0, 0);
Eigen::Vector3d ur3_current_EE_vel(0, 0, 0);
Eigen::Vector3d ur4_current_EE_vel(0, 0, 0);

Eigen::Vector3d ur1_prev_EE_position; 
Eigen::Vector3d ur2_prev_EE_position;
Eigen::Vector3d ur3_prev_EE_position;
Eigen::Vector3d ur4_prev_EE_position; 

// double ur1_speed, ur2_speed, ur3_speed, ur4_speed;


struct Agent {
    Eigen::Vector3d position;
    double radius;
    Eigen::Vector3d velocity;
};


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
    // Create a vector of onion IDs and distances
    std::vector<std::pair<std::string, double>> sorted_onions;

    double Y_MIN_UR;
    double Y_MAX_UR;
    double Z_MIN_UR;


    // Define Workspace Bounds for all the Robots

    if (robot_id == "ur1")
    {
        Y_MIN_UR = -0.7;
        Y_MAX_UR =  0.3;
    }

    else if (robot_id == "ur2")
    {
        Y_MIN_UR = -1.2;
        Y_MAX_UR = -0.2;
    }

    else if (robot_id == "ur3")
    {
        Y_MIN_UR = 0.3;
        Y_MAX_UR = 1.3;
    }

    else
    {
        Y_MIN_UR = -0.2;  // -0.2
        Y_MAX_UR = 0.8;   // 0.8
    }

    Z_MIN_UR = 0.75;


    for (const auto &onion_id : onion_ids)
    {
        auto it = global_onion_pose_map.find(onion_id);
        if (it == global_onion_pose_map.end())
            continue;


        const double y = it->second.y();
        const double z = it->second.z();
        if (y < Y_MIN_UR || y > Y_MAX_UR  || z < Z_MIN_UR)
            continue;


        double dist = (robot_ee_position - it->second).norm();
        sorted_onions.emplace_back(onion_id, dist);
    }

    // Sort onions by distance (ascending)
    std::sort(sorted_onions.begin(), sorted_onions.end(),
              [](const auto &a, const auto &b) {
                  return a.second < b.second;
              });

    // Iterate through sorted onions, and check for IK
    for (const auto &entry : sorted_onions)
    {
        const std::string &onion_id = entry.first;

        const Eigen::Vector3d &onion_position = global_onion_pose_map[onion_id];

        Eigen::Vector3d goal_position_1(onion_position.x(), onion_position.y(), 0.95);
        Eigen::Vector3d goal_position_2(onion_position.x(), onion_position.y(), 1.0);
        Eigen::Vector3d goal_orientation(3.14, 0, 3.14);

        geometry_msgs::msg::Pose target_pose_1 = set_pose_target(goal_position_1, goal_orientation);
        geometry_msgs::msg::Pose target_pose_2 = set_pose_target(goal_position_2, goal_orientation);

        double timeout = 1.0;
        bool ik_found_1 = current_state->setFromIK(joint_model_group, target_pose_1, timeout);
        bool ik_found_2 = current_state->setFromIK(joint_model_group, target_pose_2, timeout);

        if (ik_found_1 && ik_found_2)
        {
            // Remove from onion_ids list
            auto it = std::remove(onion_ids.begin(), onion_ids.end(), onion_id);
            onion_ids.erase(it, onion_ids.end());

            return std::make_pair(onion_id, onion_position);
        }
    }

    // No valid IK solution found
    return std::make_pair("", Eigen::Vector3d(0, 0, 0));
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





std::vector<Eigen::Vector3d> Baseline(const std::shared_ptr<rclcpp::Node> &node,
                                  const Eigen::Vector3d &goal_EE_position,
                                  const Eigen::Vector3d &current_EE_position,
                                  double pref_speed,
                                  const std::string & robot_id) 
{
    
    Eigen::Vector3d goal_velocity = (goal_EE_position - current_EE_position);
    if (goal_velocity.norm() > 1e-3) {
        goal_velocity = goal_velocity.normalized() * pref_speed;
    } else {
        goal_velocity = Eigen::Vector3d::Zero();
    }

    std::cout << std::string(100, '-') << std::endl;
    // std::cout << "Non - Collision Case " << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    return {goal_velocity};  // Return as vector of one element

}




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




// Uppercases ASCII letters
static std::string Upper(const std::string &s) {
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c){ return static_cast<char>(std::toupper(c)); });
  return out;
}




static bool CallDynamicDetachableJointService(const std::string &robot_id,
                                              const std::string &onion_id,
                                              const std::string &service_type,
                                              unsigned int timeout_ms = 3000)
{
  static gz::transport::Node node;

  // /ddj/UR1_wrist_3_link, /ddj/UR2_wrist_3_link, ...
  const std::string srv = "/ddj/" + Upper(robot_id) + "_wrist_3_link";

  gz::msgs::DynamicDetachableJointRequest req;
  req.set_child_model_name(onion_id);
  req.set_child_link_name("base_link");

  const std::string cmd = Upper(service_type);
  if (cmd == "ATTACH") {
    req.set_command(gz::msgs::DynamicDetachableJointRequest::ATTACH);
  } else if (cmd == "DETACH") {
    req.set_command(gz::msgs::DynamicDetachableJointRequest::DETACH);
  } else {
    std::cerr << "[DDJ] Invalid service_type '" << service_type
              << "'. Use ATTACH or DETACH.\n";
    return false;
  }

  gz::msgs::DynamicDetachableJointResponse rep;
  bool transport_ok = false;  // “service call completed and replied”
  const bool request_ok = node.Request(srv, req, timeout_ms, rep, transport_ok);

  if (!request_ok) {
    std::cerr << "[DDJ] Transport error or timeout calling '" << srv << "'.\n";
    return false;
  }
  if (!transport_ok) {
    std::cerr << "[DDJ] Service '" << srv << "' returned no/invalid reply.\n";
    return false;
  }
  if (!rep.success()) {
    std::cerr << "[DDJ] Plugin reported failure: " << rep.message() << "\n";
    return false;
  }

  std::cout << "[DDJ] " << cmd << " accepted by '" << srv
            << "': " << rep.message() << "\n";
  return true;
}









// Collect ALL links by name prefix (e.g., "UR1_")
static std::vector<std::string>
linksByPrefix(const moveit::core::RobotModelPtr& model, const std::string& prefix) {
  std::vector<std::string> out;
  for (const auto& ln : model->getLinkModelNames())
    if (ln.rfind(prefix, 0) == 0) out.push_back(ln);   // starts_with
  return out;
}

static void allowInterRobotByPrefix(const rclcpp::Node::SharedPtr& node,
                                    const moveit::core::RobotModelPtr& model,
                                    const std::vector<std::string>& prefixes) {
  // fetch current ACM (preserve SRDF)
  moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  if (client->wait_for_service(std::chrono::seconds(5))) {
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    if (auto fut = client->async_send_request(req);
        fut.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
      acm_msg = fut.get()->scene.allowed_collision_matrix;
  }

  collision_detection::AllowedCollisionMatrix acm =
      acm_msg.entry_names.empty()
        ? collision_detection::AllowedCollisionMatrix(model->getLinkModelNames())
        : collision_detection::AllowedCollisionMatrix(acm_msg);

  // link sets per robot prefix
  std::vector<std::vector<std::string>> sets;
  sets.reserve(prefixes.size());
  for (const auto& p : prefixes) sets.push_back(linksByPrefix(model, p));

  // allow all cross-robot pairs
  for (size_t i = 0; i < sets.size(); ++i)
    for (size_t j = i + 1; j < sets.size(); ++j)
      for (const auto& a : sets[i])
        for (const auto& b : sets[j])
          if (a != b) acm.setEntry(a, b, true);

  // publish diff
  moveit_msgs::msg::PlanningScene ps; ps.is_diff = true;
  acm.getMessage(ps.allowed_collision_matrix);
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyPlanningScene(ps);
}









// Continuous planning and execution loop for one robot.
void runRobot(const rclcpp::Node::SharedPtr & node,
              const std::string & planning_group,
              const moveit::core::JointModelGroup* joint_model_group,
              const std::string & robot_id,
              const std::string & action_name,
              const std::string & robot_ee,
              const std::vector<std::string> other_robot_ee,
              const std::vector<std::vector<double>> gz_poses,
              moveit::planning_interface::MoveGroupInterface & move_group,
              const std::vector<std::reference_wrapper<moveit::planning_interface::MoveGroupInterface>> & other_move_groups,
              gztr::Node & gz_node,
              const std::string & GZ_WORLD)
{
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> goal_joint_values;


  // Gazebo World Poses
  std::vector<double> inspect_pos = gz_poses[0];
  std::vector<double> inspect_approach = gz_poses[1];
  std::vector<double> safe_pos = gz_poses[2];
  std::vector<double> bin_approach = gz_poses[3];
  std::vector<double> bin_pos = gz_poses[4];

 

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



  auto &other_robot_1_move_group = other_move_groups[0].get();

  moveit::core::RobotStatePtr other_robot_1_current_state = other_robot_1_move_group.getCurrentState();
  Eigen::Affine3d other_robot_1_current_EE_state = other_robot_1_current_state->getGlobalLinkTransform(other_robot_ee[0]);
  Eigen::Vector3d other_robot_1_current_EE_position = other_robot_1_current_EE_state.translation();

  auto &other_robot_2_move_group = other_move_groups[1].get();

  moveit::core::RobotStatePtr other_robot_2_current_state = other_robot_2_move_group.getCurrentState();
  Eigen::Affine3d other_robot_2_current_EE_state = other_robot_2_current_state->getGlobalLinkTransform(other_robot_ee[1]);
  Eigen::Vector3d other_robot_2_current_EE_position = other_robot_2_current_EE_state.translation();

  auto &other_robot_3_move_group = other_move_groups[2].get();

  moveit::core::RobotStatePtr other_robot_3_current_state = other_robot_3_move_group.getCurrentState();
  Eigen::Affine3d other_robot_3_current_EE_state = other_robot_3_current_state->getGlobalLinkTransform(other_robot_ee[2]);
  Eigen::Vector3d other_robot_3_current_EE_position = other_robot_3_current_EE_state.translation();



  double d1 = (current_EE_position - other_robot_1_current_EE_position).norm();
  double d2 = (current_EE_position - other_robot_2_current_EE_position).norm();
  double d3 = (current_EE_position - other_robot_3_current_EE_position).norm();

  double robot_distance = std::min({d1, d2, d3});



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

  double epsilon = 0.05;
  double max_len = 0.2;
  double min_len = 0.065;  // 0.05
  double alpha = 0.50;
  double robot_speed = 0.2;
  double pref_speed = 0.05; 
  double max_speed = 0.1;
  double stagnant_count = 0;
  
  double len;

  Eigen::Vector3d current_EE_vel;


  // Initialize variables

  if (robot_id == "ur1") 
  {
    current_EE_vel = ur1_current_EE_vel;
    // ur1_speed = pref_speed;
    ur1_prev_EE_position = current_EE_position; 
  } 

  else if (robot_id == "ur2")
  {
    current_EE_vel = ur2_current_EE_vel;
    // ur2_speed = pref_speed;
    ur2_prev_EE_position = current_EE_position; 
  }

  else if (robot_id == "ur3")
  {
    current_EE_vel = ur3_current_EE_vel;
    // ur3_speed = pref_speed;
    ur3_prev_EE_position = current_EE_position; 
  }

  else 
  {
    current_EE_vel = ur4_current_EE_vel;
    // ur4_speed = pref_speed;
    ur4_prev_EE_position = current_EE_position; 
  }



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

        Eigen::Vector3d ur_end_pose;
        Eigen::Vector3d ur_end_orientation;

        if (robot_id == "ur1")
        {
            ur_end_pose << 0.50, -0.07, 1.0;
            ur_end_orientation << 3.14, 0, 3.14;
        }

        else if (robot_id == "ur2")
        {
            ur_end_pose << 1.15, -0.51, 1.0;
            ur_end_orientation << 3.14, 0, 3.14;
        }

        else if (robot_id == "ur3")
        {
            ur_end_pose << 0.48, 0.65, 1.0;
            ur_end_orientation << 3.14, 0, 3.14;
        }

        else
        {
            ur_end_pose << 1.24, 0.2, 1.0;
            ur_end_orientation << 3.14, 0, 3.14;
        }



        // Move back slightly
        geometry_msgs::msg::Pose end_pose = set_pose_target(ur_end_pose, ur_end_orientation);

        // Solve for the IK Solution of the Target Pose
        double timeout = 1.0;
        bool ik_found = current_state->setFromIK(joint_model_group, end_pose, timeout);

        if (ik_found)
        {
            current_state->copyJointGroupPositions(joint_model_group, goal_joint_values);
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Did not find End Position IK solution for [%s]", robot_id.c_str());
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
            // EE_speed_control(node, current_state, plan_copy, robot_ee, robot_speed);

            trajectory_msgs::msg::JointTrajectory trajectory = plan_copy.trajectory.joint_trajectory;
            // Execute the trajectory and wait for its completion.
            executeTrajectoryAndWait(node, trajectory, action_name, robot_id);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "[%s] End Position Planning failed!", robot_id.c_str());
            // Optionally, wait a short time before retrying.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        break;  
        // End of Program
    }



    target_id = closest_onion.first.c_str();



    // PSort Status Info
    bool onion_target_reached = false;
    bool onion_target_approach = false;
    bool onion_target_retreat = false;
    bool safe_pos_one_reached = false;
    bool inspection_pose_reached = false;
    bool inspection_pose_approach = false;
    bool inspection_pose_retreat = false;
    bool safe_pos_two_reached = false;
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



        // -------------------------------- Safety Mechanism -------------------------------- //

        double robot_distance_threshold = 0.3;    // 0.5


        // --------------------------------  PSort  Procedure  -------------------------------- //

        
        // Set Target Pose to the closest onion available
        if(!onion_target_reached)        // Change "if" to "else if" in case you uncomment the Safety Mechanism
        {
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

            if (robot_distance > robot_distance_threshold){
                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
            
            else {
                len = min_len;
            }
        }

        // Gripper Down
        else if(!onion_target_approach)
        {
            goal_EE_position[0] = closest_onion.second.x();
            goal_EE_position[1] = closest_onion.second.y();
            goal_EE_position[2] = 0.925;

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

                // Attach the Onion

                CallDynamicDetachableJointService(robot_id, target_id, "ATTACH");
            }

            if (robot_distance > robot_distance_threshold){
                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
            
            else {
                len = min_len;
            }
        }

        // Gripper Up
        else if (!onion_target_retreat)
        {
            goal_EE_position[0] = closest_onion.second.x();
            goal_EE_position[1] = closest_onion.second.y();
            goal_EE_position[2] = 1.05;

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

            if (robot_distance > robot_distance_threshold){
                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
            
            else {
                len = min_len;
            }
        }       

        // Safety Position
        else if (!safe_pos_one_reached)
        {
            goal_EE_position[0] = safe_pos[0];
            goal_EE_position[1] = safe_pos[1];
            goal_EE_position[2] = safe_pos[2];

            goal_EE_orientation[0] = safe_pos[3];
            goal_EE_orientation[1] = safe_pos[4];
            goal_EE_orientation[2] = safe_pos[5];

            RCLCPP_INFO(node->get_logger(), " ");
            RCLCPP_INFO(node->get_logger(), "[%s] Safe Position Approach Case", robot_id.c_str());
            RCLCPP_INFO(node->get_logger(), " ");


            robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
            if (robot_goal_reached)
            {
                safe_pos_one_reached = true;
            }

            if (robot_distance > robot_distance_threshold){
                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
            
            else {
                len = min_len;
            }
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

            if (robot_distance > robot_distance_threshold){
                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
            
            else {
                len = min_len;
            }
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

            if (robot_distance > robot_distance_threshold){
                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
            
            else {
                len = min_len;
            }
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

            if (robot_distance > robot_distance_threshold){
                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
            
            else {
                len = min_len;
            }
        }

        // // Safety Position
        // else if (!safe_pos_two_reached)
        // {
        //     goal_EE_position[0] = safe_pos[0];
        //     goal_EE_position[1] = safe_pos[1];
        //     goal_EE_position[2] = safe_pos[2];

        //     goal_EE_orientation[0] = safe_pos[3];
        //     goal_EE_orientation[1] = safe_pos[4];
        //     goal_EE_orientation[2] = safe_pos[5];

        //     RCLCPP_INFO(node->get_logger(), " ");
        //     RCLCPP_INFO(node->get_logger(), "[%s] Safe Position Approach Case", robot_id.c_str());
        //     RCLCPP_INFO(node->get_logger(), " ");


        //     robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
        //     if (robot_goal_reached)
        //     {
        //         safe_pos_two_reached = true;
        //     }

        //     if (robot_distance > robot_distance_threshold){
        //         len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
        //     }
            
        //     else {
        //         len = min_len;
        //     }
        // }

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

            if (robot_distance > robot_distance_threshold){
                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
            
            else {
                len = min_len;
            }
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

                // Detach the Onion
                
                CallDynamicDetachableJointService(robot_id, target_id, "DETACH");

                len = max_len;
            }

            if (robot_distance > robot_distance_threshold){
                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
            
            else {
                len = min_len;
            }
        }



        // ----------------------------------------------------------------------------------------------------------------------------------------- //



        // Plan for those Goal Position using ORCA

        // double threshold_distance = 0.25;

        other_robot_1_current_state = other_robot_1_move_group.getCurrentState();
        other_robot_1_current_EE_state = other_robot_1_current_state->getGlobalLinkTransform(other_robot_ee[0]);
        other_robot_1_current_EE_position = other_robot_1_current_EE_state.translation();

        other_robot_2_current_state = other_robot_2_move_group.getCurrentState();
        other_robot_2_current_EE_state = other_robot_2_current_state->getGlobalLinkTransform(other_robot_ee[1]);
        other_robot_2_current_EE_position = other_robot_2_current_EE_state.translation();

        other_robot_3_current_state = other_robot_3_move_group.getCurrentState();
        other_robot_3_current_EE_state = other_robot_3_current_state->getGlobalLinkTransform(other_robot_ee[2]);
        other_robot_3_current_EE_position = other_robot_3_current_EE_state.translation();
        
        Eigen::Vector3d next_EE_vel;

        if (robot_id == "ur1") 
        {

            // Compute distances to the three other robots' EE positions 
            d1 = (current_EE_position - other_robot_1_current_EE_position).norm();
            d2 = (current_EE_position - other_robot_2_current_EE_position).norm();
            d3 = (current_EE_position - other_robot_3_current_EE_position).norm();

            // stash them in an array
            std::array<double,3> ds = { d1, d2, d3 };

            // find the iterator to the smallest element
            auto it = std::min_element(ds.begin(), ds.end());

            // compute both the min value and the 1-based index
            double min_dist = *it;
            int min_idx = std::distance(ds.begin(), it) + 1;

            // Implement   Baseline

            if (min_dist > robot_distance_threshold)
            {
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "[%s] -> Using Baseline)", robot_id.c_str());
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");
                next_EE_vel = Baseline(node, goal_EE_position, current_EE_position, pref_speed, robot_id)[0];
            }

            // Implement   ORCA

            else
            {
                Agent main_agent;
                main_agent.position = current_EE_position;
                main_agent.radius = 0.1;       // 0.085    
                main_agent.velocity = ur1_current_EE_vel;

                Agent other_agent;

                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");

                if (min_idx == 1)
                {
                    other_agent.position = other_robot_1_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur2_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-2");
                }

                else if (min_idx == 2)
                {
                    other_agent.position = other_robot_2_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur3_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-3");
                }

                else
                {
                    other_agent.position = other_robot_3_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur4_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-4");
                }

                next_EE_vel = Baseline(node, goal_EE_position, current_EE_position, pref_speed, robot_id)[0];

                RCLCPP_INFO(node->get_logger(), "");

                // RCLCPP_INFO(node->get_logger(), "[%s]   ->   Next Velocity is ([%f], [%f], [%f])", robot_id.c_str(), next_EE_vel.x(), next_EE_vel.y(), next_EE_vel.z());

                // RCLCPP_INFO(node->get_logger(), "");
                // RCLCPP_INFO(node->get_logger(), "");
            }

            ur1_current_EE_vel = next_EE_vel;
            ur1_prev_EE_position = current_EE_position;
            // ur1_speed = next_EE_vel.norm();

        } 
        


        else if (robot_id == "ur2")
        {
            
            // Compute distances to the three other robots' EE positions 
            d1 = (current_EE_position - other_robot_1_current_EE_position).norm();
            d2 = (current_EE_position - other_robot_2_current_EE_position).norm();
            d3 = (current_EE_position - other_robot_3_current_EE_position).norm();

            // stash them in an array
            std::array<double,3> ds = { d1, d2, d3 };

            // find the iterator to the smallest element
            auto it = std::min_element(ds.begin(), ds.end());

            // compute both the min value and the 1-based index
            double min_dist = *it;
            int min_idx = std::distance(ds.begin(), it) + 1;

            // Implement   Baseline

            if (min_dist > robot_distance_threshold)
            {
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "[%s] -> Using Baseline)", robot_id.c_str());
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");
                next_EE_vel = Baseline(node, goal_EE_position, current_EE_position, pref_speed, robot_id)[0];
            }

            // Implement   ORCA

            else
            {
                Agent main_agent;
                main_agent.position = current_EE_position;
                main_agent.radius = 0.1;       // 0.085    
                main_agent.velocity = ur2_current_EE_vel;

                Agent other_agent;

                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");

                if (min_idx == 1)
                {
                    other_agent.position = other_robot_1_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur1_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-1");
                }

                else if (min_idx == 2)
                {
                    other_agent.position = other_robot_2_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur3_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-3");
                }

                else
                {
                    other_agent.position = other_robot_3_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur4_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-4");
                }

                next_EE_vel = Baseline(node, goal_EE_position, current_EE_position, pref_speed, robot_id)[0];

                RCLCPP_INFO(node->get_logger(), "");

                // RCLCPP_INFO(node->get_logger(), "[%s]   ->   Next Velocity is ([%f], [%f], [%f])", robot_id.c_str(), next_EE_vel.x(), next_EE_vel.y(), next_EE_vel.z());

                // RCLCPP_INFO(node->get_logger(), "");
                // RCLCPP_INFO(node->get_logger(), "");
            }

            ur2_current_EE_vel = next_EE_vel;
            ur2_prev_EE_position = current_EE_position;
            // ur2_speed = next_EE_vel.norm();

        }



        else if (robot_id == "ur3")
        {
            
            // Compute distances to the three other robots' EE positions 
            d1 = (current_EE_position - other_robot_1_current_EE_position).norm();
            d2 = (current_EE_position - other_robot_2_current_EE_position).norm();
            d3 = (current_EE_position - other_robot_3_current_EE_position).norm();

            // stash them in an array
            std::array<double,3> ds = { d1, d2, d3 };

            // find the iterator to the smallest element
            auto it = std::min_element(ds.begin(), ds.end());

            // compute both the min value and the 1-based index
            double min_dist = *it;
            int min_idx = std::distance(ds.begin(), it) + 1;

            // Implement   Baseline

            if (min_dist > robot_distance_threshold)
            {
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "[%s] -> Using Baseline)", robot_id.c_str());
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");
                next_EE_vel = Baseline(node, goal_EE_position, current_EE_position, pref_speed, robot_id)[0];
            }

            // Implement   ORCA

            else
            {
                Agent main_agent;
                main_agent.position = current_EE_position;
                main_agent.radius = 0.1;       // 0.085    
                main_agent.velocity = ur3_current_EE_vel;

                Agent other_agent;

                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");

                if (min_idx == 1)
                {
                    other_agent.position = other_robot_1_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur1_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-1");
                }

                else if (min_idx == 2)
                {
                    other_agent.position = other_robot_2_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur2_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-2");
                }

                else
                {
                    other_agent.position = other_robot_3_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur4_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-4");
                }

                next_EE_vel = Baseline(node, goal_EE_position, current_EE_position, pref_speed, robot_id)[0];

                RCLCPP_INFO(node->get_logger(), "");

                // RCLCPP_INFO(node->get_logger(), "[%s]   ->   Next Velocity is ([%f], [%f], [%f])", robot_id.c_str(), next_EE_vel.x(), next_EE_vel.y(), next_EE_vel.z());

                // RCLCPP_INFO(node->get_logger(), "");
                // RCLCPP_INFO(node->get_logger(), "");
            }

            ur3_current_EE_vel = next_EE_vel;
            ur3_prev_EE_position = current_EE_position;
            // ur3_speed = next_EE_vel.norm();

        }



        else if (robot_id == "ur4")
        {
            
            // Compute distances to the three other robots' EE positions 
            d1 = (current_EE_position - other_robot_1_current_EE_position).norm();
            d2 = (current_EE_position - other_robot_2_current_EE_position).norm();
            d3 = (current_EE_position - other_robot_3_current_EE_position).norm();

            // stash them in an array
            std::array<double,3> ds = { d1, d2, d3 };

            // find the iterator to the smallest element
            auto it = std::min_element(ds.begin(), ds.end());

            // compute both the min value and the 1-based index
            double min_dist = *it;
            int min_idx = std::distance(ds.begin(), it) + 1;

            // Implement   Baseline

            if (min_dist > robot_distance_threshold)
            {
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "[%s] -> Using Baseline)", robot_id.c_str());
                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");
                next_EE_vel = Baseline(node, goal_EE_position, current_EE_position, pref_speed, robot_id)[0];
            }

            // Implement   ORCA

            else
            {
                Agent main_agent;
                main_agent.position = current_EE_position;
                main_agent.radius = 0.1;       // 0.085    
                main_agent.velocity = ur4_current_EE_vel;

                Agent other_agent;

                RCLCPP_INFO(node->get_logger(), "");
                RCLCPP_INFO(node->get_logger(), "");

                if (min_idx == 1)
                {
                    other_agent.position = other_robot_1_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur1_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-1");
                }

                else if (min_idx == 2)
                {
                    other_agent.position = other_robot_2_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur2_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-2");
                }

                else
                {
                    other_agent.position = other_robot_3_current_EE_position;
                    other_agent.radius = 0.1;    
                    other_agent.velocity = ur3_current_EE_vel;

                    RCLCPP_INFO(node->get_logger(), "Collision with UR-3");
                }

                next_EE_vel = Baseline(node, goal_EE_position, current_EE_position, pref_speed, robot_id)[0];
                RCLCPP_INFO(node->get_logger(), "");

                // RCLCPP_INFO(node->get_logger(), "[%s]   ->   Next Velocity is ([%f], [%f], [%f])", robot_id.c_str(), next_EE_vel.x(), next_EE_vel.y(), next_EE_vel.z());

                // RCLCPP_INFO(node->get_logger(), "");
                // RCLCPP_INFO(node->get_logger(), "");
            }

            ur4_current_EE_vel = next_EE_vel;
            ur4_prev_EE_position = current_EE_position;
            // ur4_speed = next_EE_vel.norm();

        }


        Eigen::Vector3d target_unit_vec;

        target_unit_vec = next_EE_vel/next_EE_vel.norm();



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
            // EE_speed_control(node, current_state, plan_copy, robot_ee, robot_speed);

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
  setenv("GZ_DESCRIPTOR_PATH", "", 1);   // prevent duplicate dynamic load
  setenv("IGN_DESCRIPTOR_PATH", "", 1);  // legacy var

  rclcpp::init(argc, argv);
  
  // Create one node to be used by both MoveGroupInterface instances.
  auto node = rclcpp::Node::make_shared("baseline");
  
  // Create a MultiThreadedExecutor so that callbacks for each robot can run concurrently.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  // Spin the executor in a separate thread.
  std::thread spin_thread([&executor]() { executor.spin(); });


  // Gazebo transport node (one is enough; thread-safe requests)
  gztr::Node gz_node;

  // Your world is "empty":
  const std::string GZ_WORLD = "empty";


  const std::string& ur1_ee = "UR1_wrist_3_link";
  const std::string& ur2_ee = "UR2_wrist_3_link";
  const std::string& ur3_ee = "UR3_wrist_3_link";
  const std::string& ur4_ee = "UR4_wrist_3_link";

  const std::string& ur1_planning_group = "UR1_manipulator";
  const std::string& ur2_planning_group = "UR2_manipulator";
  const std::string& ur3_planning_group = "UR3_manipulator";
  const std::string& ur4_planning_group = "UR4_manipulator";



  // Get Robot Joint Information

  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

  const moveit::core::JointModelGroup* ur1_joint_model_group = kinematic_model->getJointModelGroup(ur1_planning_group);
  const moveit::core::JointModelGroup* ur2_joint_model_group = kinematic_model->getJointModelGroup(ur2_planning_group);
  const moveit::core::JointModelGroup* ur3_joint_model_group = kinematic_model->getJointModelGroup(ur3_planning_group);
  const moveit::core::JointModelGroup* ur4_joint_model_group = kinematic_model->getJointModelGroup(ur4_planning_group);



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




//   // Inspection Pose - Camera FAR Away from Bin

//   std::vector<double> ur1_bin = {0.46, 0.30, 0.98, 3.14, 0.0, 3.14};
//   std::vector<double> ur2_bin = {1.04, -0.20, 0.98, 3.14, 0.0, 3.14};
//   std::vector<double> ur3_bin = {0.46, 0.30, 0.98, 3.14, 0.0, 3.14};
//   std::vector<double> ur4_bin = {1.04, -0.20, 0.98, 3.14, 0.0, 3.14};

//   std::vector<double> ur1_bin_approach = {0.585, 0.1, 1.0, 3.14, 0.0, 3.14};
//   std::vector<double> ur2_bin_approach = {0.9136, -0.374, 1.0, 3.14, 0.0, 3.14};
//   std::vector<double> ur3_bin_approach = {0.5854, 0.5, 1.0, 3.14, 0.0, 3.14};
//   std::vector<double> ur4_bin_approach = {0.9, -0.01, 1.0, 3.14, 0.0, 3.14};

//   std::vector<double> ur1_inspect_pos = {0.43, -0.64, 1.0, 3.14, 0, 3.14};
//   std::vector<double> ur2_inspect_pos = {1.05, -1.07, 1.0, 3.14, 0, 3.14};
//   std::vector<double> ur3_inspect_pos = {0.43, 1.15, 1.0, 3.14, 0, 3.14};
//   std::vector<double> ur4_inspect_pos = {1.05, 0.74, 1.0, 3.14, 0, 3.14};

//   std::vector<double> ur1_inspect_approach = {0.43, -0.64, 1.0, -1.55, 0, -3.14};
//   std::vector<double> ur2_inspect_approach = {1.05, -1.07, 1.0, -1.59, 0, 3.14};
//   std::vector<double> ur3_inspect_approach = {0.43, 1.15, 1.0, 1.56, 0, -3.14};
//   std::vector<double> ur4_inspect_approach = {1.05, 0.74, 1.0, 1.57, 0, 3.14};

//   std::vector<double> ur1_safe_pos = {0.61, -0.33, 1.0, 3.14, 0, 3.14};
//   std::vector<double> ur2_safe_pos = {0.89, -0.8, 1.0, 3.14, 0, 3.14};
//   std::vector<double> ur3_safe_pos = {0.57, 0.9, 1.0, 3.14, 0, 3.14};
//   std::vector<double> ur4_safe_pos = {0.92, 0.39, 1.0, 3.14, 0, 3.14};



  // Inspection Case - Camera NEAR the Bin  

  std::vector<double> ur1_bin = {0.46, 0.30, 0.98, 3.14, 0.0, 3.14};
  std::vector<double> ur2_bin = {1.04, -0.20, 0.98, 3.14, 0.0, 3.14};
  std::vector<double> ur3_bin = {0.46, 0.30, 0.98, 3.14, 0.0, 3.14};
  std::vector<double> ur4_bin = {1.04, -0.20, 0.98, 3.14, 0.0, 3.14};

  std::vector<double> ur1_bin_approach = {0.4137, 0.0624, 1.0, -3.14, 0, 3.14};
  std::vector<double> ur2_bin_approach = {1.0672, -0.4491, 1.0, 3.14, 0, 3.14};
  std::vector<double> ur3_bin_approach = {0.4419, 0.5246, 1.0, 3.14, 0, -3.14};
  std::vector<double> ur4_bin_approach = {1.0520, 0.0243, 1.0, 3.14, 0, 3.14};

  std::vector<double> ur1_inspect_pos = {0.4137, 0.0624, 1.0, -3.14, 0, 3.14};
  std::vector<double> ur2_inspect_pos = {1.0672, -0.4491, 1.0, 3.14, 0, 3.14};
  std::vector<double> ur3_inspect_pos = {0.4419, 0.5246, 1.0, 3.14, 0, -3.14};
  std::vector<double> ur4_inspect_pos = {1.0520, 0.0243, 1.0, 3.14, 0, 3.14};

  std::vector<double> ur1_inspect_approach = {0.4137, 0.0624, 1.0, -3.14, -1.3374, 3.14};
  std::vector<double> ur2_inspect_approach = {1.0672, -0.4491, 1.0, 3.14, 1.5303, 3.14};
  std::vector<double> ur3_inspect_approach = {0.4419, 0.5246, 1.0, 3.14, -1.3641, -3.14};
  std::vector<double> ur4_inspect_approach = {1.0520, 0.0243, 1.0, 3.14, 1.4424, 3.14};

  std::vector<double> ur1_safe_pos = {0.5398, 0.0624, 1.0, -3.14, 0, 3.14};
  std::vector<double> ur2_safe_pos = {0.9898, -0.4491, 1.0, 3.14, 0, 3.14};
  std::vector<double> ur3_safe_pos = {0.5301, 0.5246, 1.0, 3.14, 0, -3.14};
  std::vector<double> ur4_safe_pos = {0.9727, 0.0291, 1.0, 3.14, 0, 3.14};


  std::vector<std::vector<double>> ur1_gz_poses = {
    ur1_inspect_pos,
    ur1_inspect_approach,
    ur1_safe_pos,
    ur1_bin_approach,
    ur1_bin
  };

  std::vector<std::vector<double>> ur2_gz_poses = {
    ur2_inspect_pos,
    ur2_inspect_approach,
    ur2_safe_pos,
    ur2_bin_approach,
    ur2_bin
  };

  std::vector<std::vector<double>> ur3_gz_poses = {
    ur3_inspect_pos,
    ur3_inspect_approach,
    ur3_safe_pos,
    ur3_bin_approach,
    ur3_bin
  };

  std::vector<std::vector<double>> ur4_gz_poses = {
    ur4_inspect_pos,
    ur4_inspect_approach,
    ur4_safe_pos,
    ur4_bin_approach,
    ur4_bin
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


  moveit::planning_interface::MoveGroupInterface ur3_move_group(node, ur3_planning_group);
  ur3_move_group.setMaxVelocityScalingFactor(1.0);
  ur3_move_group.setMaxAccelerationScalingFactor(1.0);
  ur3_move_group.setPlanningTime(20);

  ur3_move_group.startStateMonitor();

  ur3_move_group.setStartStateToCurrentState();


  moveit::planning_interface::MoveGroupInterface ur4_move_group(node, ur4_planning_group);
  ur4_move_group.setMaxVelocityScalingFactor(1.0);
  ur4_move_group.setMaxAccelerationScalingFactor(1.0);
  ur4_move_group.setPlanningTime(20);

  ur4_move_group.startStateMonitor();

  ur4_move_group.setStartStateToCurrentState();




    allowInterRobotByPrefix(
    node, kinematic_model,
    {"UR1_", "UR2_", "UR3_", "UR4_"}
  );
  
  rclcpp::sleep_for(std::chrono::milliseconds(300)); // let the monitors ingest the diff




  auto ur1_other_move_groups = std::vector<std::reference_wrapper<moveit::planning_interface::MoveGroupInterface>>{
    std::ref(ur2_move_group),
    std::ref(ur3_move_group),
    std::ref(ur4_move_group)
  };

  auto ur2_other_move_groups = std::vector<std::reference_wrapper<moveit::planning_interface::MoveGroupInterface>>{
    std::ref(ur1_move_group),
    std::ref(ur3_move_group),
    std::ref(ur4_move_group)
  };

  auto ur3_other_move_groups = std::vector<std::reference_wrapper<moveit::planning_interface::MoveGroupInterface>>{
    std::ref(ur1_move_group),
    std::ref(ur2_move_group),
    std::ref(ur4_move_group)
  };
  
  auto ur4_other_move_groups = std::vector<std::reference_wrapper<moveit::planning_interface::MoveGroupInterface>>{
    std::ref(ur1_move_group),
    std::ref(ur2_move_group),
    std::ref(ur3_move_group)
  };



  
  std::vector<std::string> ur1_other_ee{ ur2_ee, ur3_ee, ur4_ee };
  std::vector<std::string> ur2_other_ee{ ur1_ee, ur3_ee, ur4_ee };
  std::vector<std::string> ur3_other_ee{ ur1_ee, ur2_ee, ur4_ee };
  std::vector<std::string> ur4_other_ee{ ur1_ee, ur2_ee, ur3_ee };



  
  // Define the action server names for each robot.
  std::string ur1_action_name = "joint_trajectory_controller_ur1/follow_joint_trajectory";
  std::string ur2_action_name = "joint_trajectory_controller_ur2/follow_joint_trajectory";
  std::string ur3_action_name = "joint_trajectory_controller_ur3/follow_joint_trajectory";
  std::string ur4_action_name = "joint_trajectory_controller_ur4/follow_joint_trajectory";
  
  // Start continuous planning/execution for each robot in separate threads.
  std::thread ur1_thread(runRobot, node, ur1_planning_group, ur1_joint_model_group, "ur1", ur1_action_name, ur1_ee, ur1_other_ee, ur1_gz_poses, std::ref(ur1_move_group), ur1_other_move_groups, std::ref(gz_node), std::cref(GZ_WORLD));
  std::thread ur2_thread(runRobot, node, ur2_planning_group, ur2_joint_model_group, "ur2", ur2_action_name, ur2_ee, ur2_other_ee, ur2_gz_poses, std::ref(ur2_move_group), ur2_other_move_groups, std::ref(gz_node), std::cref(GZ_WORLD));
  std::thread ur3_thread(runRobot, node, ur3_planning_group, ur3_joint_model_group, "ur3", ur3_action_name, ur3_ee, ur3_other_ee, ur3_gz_poses, std::ref(ur3_move_group), ur3_other_move_groups, std::ref(gz_node), std::cref(GZ_WORLD));
  std::thread ur4_thread(runRobot, node, ur4_planning_group, ur4_joint_model_group, "ur4", ur4_action_name, ur4_ee, ur4_other_ee, ur4_gz_poses, std::ref(ur4_move_group), ur4_other_move_groups, std::ref(gz_node), std::cref(GZ_WORLD));
  
  // Wait for both threads to finish.
  ur1_thread.join();
  ur2_thread.join();
  ur3_thread.join();
  ur4_thread.join();
  
  // Shutdown the executor and ROS.
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  
  return 0;
}