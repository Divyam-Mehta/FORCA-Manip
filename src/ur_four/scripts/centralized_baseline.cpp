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





Eigen::Vector3d Baseline(const std::shared_ptr<rclcpp::Node> &node,
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

    return goal_velocity;  // Return as vector of one element

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
























int main(int argc, char * argv[])
{

    const std::string& ur1_ee = "UR1_wrist_3_link";
    const std::string& ur2_ee = "UR2_wrist_3_link";
    const std::string& ur3_ee = "UR3_wrist_3_link";
    const std::string& ur4_ee = "UR4_wrist_3_link";


    const std::string& ur1_planning_group = "UR1_manipulator";
    const std::string& ur2_planning_group = "UR2_manipulator";
    const std::string& ur3_planning_group = "UR3_manipulator";
    const std::string& ur4_planning_group = "UR4_manipulator";

    const std::string& ur_combined_planning_group = "UR_manipulator";

    Eigen::Vector3d ur1_current_EE_vel(0, 0, 0);
    Eigen::Vector3d ur2_current_EE_vel(0, 0, 0);
    Eigen::Vector3d ur3_current_EE_vel(0, 0, 0);
    Eigen::Vector3d ur4_current_EE_vel(0, 0, 0);

    double epsilon = 0.03;
    double len = 0.05;
    double max_len = 0.2;
    double alpha = 0.50;

    double ur1_speed = 1.0;
    double ur1_pref_speed = 1.0; //0.05
    double ur1_max_speed = 0.2;

    double ur2_speed = 0.0;
    double ur2_pref_speed = 0.05;
    double ur2_max_speed = 0.2;

    double ur3_speed = 0.0;
    double ur3_pref_speed = 0.05;
    double ur3_max_speed = 0.2;

    double ur4_speed = 0.0;
    double ur4_pref_speed = 0.05;
    double ur4_max_speed = 0.2;

    double stagnant_count = 0;


    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("Centralized_MoveIt_Control", node_options);


    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Sleep for 5 seconds to allow time for topic discovery.
    std::this_thread::sleep_for(std::chrono::seconds(5));


    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit_URs");


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
    std::vector<std::string> onion_ids = extractOnionIDs(model_topics);
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



    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto ur_move_group_interface = MoveGroupInterface(node, ur_combined_planning_group);

    auto ur1_move_group_interface = MoveGroupInterface(node, ur1_planning_group);
    auto ur2_move_group_interface = MoveGroupInterface(node, ur2_planning_group);
    auto ur3_move_group_interface = MoveGroupInterface(node, ur3_planning_group);
    auto ur4_move_group_interface = MoveGroupInterface(node, ur4_planning_group);

    ur_move_group_interface.setMaxVelocityScalingFactor(1.0);
    ur_move_group_interface.setMaxAccelerationScalingFactor(1.0);
    ur_move_group_interface.setPlanningTime(20);

    //   ur1_move_group_interface.setPlanningTime(10.0);  // Set a higher planning time, e.g., 10 seconds.
    //   ur1_move_group_interface.setGoalTolerance(0.1);  // Set a reasonable goal tolerance.

    // Start the state monitors
    ur_move_group_interface.startStateMonitor();

    ur1_move_group_interface.startStateMonitor();
    ur2_move_group_interface.startStateMonitor();
    ur3_move_group_interface.startStateMonitor();
    ur4_move_group_interface.startStateMonitor();

    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    const moveit::core::JointModelGroup* ur1_joint_model_group = kinematic_model->getJointModelGroup("UR1_manipulator");
    const moveit::core::JointModelGroup* ur2_joint_model_group = kinematic_model->getJointModelGroup("UR2_manipulator");
    const moveit::core::JointModelGroup* ur3_joint_model_group = kinematic_model->getJointModelGroup("UR3_manipulator");
    const moveit::core::JointModelGroup* ur4_joint_model_group = kinematic_model->getJointModelGroup("UR4_manipulator");

    const std::vector<std::string>& ur1_joint_names = ur1_joint_model_group->getVariableNames();
    const std::vector<std::string>& ur2_joint_names = ur2_joint_model_group->getVariableNames();
    const std::vector<std::string>& ur3_joint_names = ur3_joint_model_group->getVariableNames();
    const std::vector<std::string>& ur4_joint_names = ur4_joint_model_group->getVariableNames();

    std::vector<double> ur1_joint_values;
    std::vector<double> ur2_joint_values;
    std::vector<double> ur3_joint_values;
    std::vector<double> ur4_joint_values;


    // Get Current Robot EE Pose
    moveit::core::RobotStatePtr ur_combined_state = ur_move_group_interface.getCurrentState();

    moveit::core::RobotStatePtr ur1_current_state = ur1_move_group_interface.getCurrentState();
    moveit::core::RobotStatePtr ur2_current_state = ur2_move_group_interface.getCurrentState();
    moveit::core::RobotStatePtr ur3_current_state = ur3_move_group_interface.getCurrentState();
    moveit::core::RobotStatePtr ur4_current_state = ur4_move_group_interface.getCurrentState();

    Eigen::Affine3d ur1_current_EE_state = ur1_current_state->getGlobalLinkTransform(ur1_ee);
    Eigen::Affine3d ur2_current_EE_state = ur2_current_state->getGlobalLinkTransform(ur2_ee);
    Eigen::Affine3d ur3_current_EE_state = ur3_current_state->getGlobalLinkTransform(ur3_ee);
    Eigen::Affine3d ur4_current_EE_state = ur4_current_state->getGlobalLinkTransform(ur4_ee);

    Eigen::Vector3d ur1_current_EE_position = ur1_current_EE_state.translation();
    Eigen::Vector3d ur2_current_EE_position = ur2_current_EE_state.translation();
    Eigen::Vector3d ur3_current_EE_position = ur3_current_EE_state.translation();
    Eigen::Vector3d ur4_current_EE_position = ur4_current_EE_state.translation();

    Eigen::Quaterniond ur1_current_EE_orientation(ur1_current_EE_state.rotation());
    Eigen::Quaterniond ur2_current_EE_orientation(ur2_current_EE_state.rotation());
    Eigen::Quaterniond ur3_current_EE_orientation(ur3_current_EE_state.rotation());
    Eigen::Quaterniond ur4_current_EE_orientation(ur4_current_EE_state.rotation());

    // Convert quaternion to Euler angles
    Eigen::Vector3d ur1_euler_angles = ur1_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
    Eigen::Vector3d ur2_euler_angles = ur2_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
    Eigen::Vector3d ur3_euler_angles = ur3_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
    Eigen::Vector3d ur4_euler_angles = ur4_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw

    // Store Euler angles in std::vector for compatibility
    std::vector<double> ur1_current_EE_orientation_euler = {ur1_euler_angles[0], ur1_euler_angles[1], ur1_euler_angles[2]};
    std::vector<double> ur2_current_EE_orientation_euler = {ur2_euler_angles[0], ur2_euler_angles[1], ur2_euler_angles[2]};
    std::vector<double> ur3_current_EE_orientation_euler = {ur3_euler_angles[0], ur3_euler_angles[1], ur3_euler_angles[2]};
    std::vector<double> ur4_current_EE_orientation_euler = {ur4_euler_angles[0], ur4_euler_angles[1], ur4_euler_angles[2]};

    auto ur1_closest = findClosestOnionQuick(ur1_current_EE_position, onion_ids, "ur1", ur1_current_state, ur1_joint_model_group);
    RCLCPP_INFO(node->get_logger(), "Closest onion to UR-1 is [%s] at position (%f, %f, %f)",
                ur1_closest.first.c_str(),
                ur1_closest.second.x(), ur1_closest.second.y(), ur1_closest.second.z());

    auto ur2_closest = findClosestOnionQuick(ur2_current_EE_position, onion_ids, "ur2", ur2_current_state, ur2_joint_model_group);
    RCLCPP_INFO(node->get_logger(), "Closest onion to UR-2 is [%s] at position (%f, %f, %f)",
                ur2_closest.first.c_str(),
                ur2_closest.second.x(), ur2_closest.second.y(), ur2_closest.second.z());

    auto ur3_closest = findClosestOnionQuick(ur3_current_EE_position, onion_ids, "ur3", ur3_current_state, ur3_joint_model_group);
    RCLCPP_INFO(node->get_logger(), "Closest onion to UR-3 is [%s] at position (%f, %f, %f)",
                ur3_closest.first.c_str(),
                ur3_closest.second.x(), ur3_closest.second.y(), ur3_closest.second.z());

    auto ur4_closest = findClosestOnionQuick(ur4_current_EE_position, onion_ids, "ur4", ur4_current_state, ur4_joint_model_group);
    RCLCPP_INFO(node->get_logger(), "Closest onion to UR-4 is [%s] at position (%f, %f, %f)",
                ur4_closest.first.c_str(),
                ur4_closest.second.x(), ur4_closest.second.y(), ur4_closest.second.z());



    // // Poses in Gazebo World

    // std::vector<double> ur1_bin = {0.35, -0.49, 0.96, 3.14, 0.0, -3.14};
    // std::vector<double> ur2_bin = {1.16, 0.49, 0.96, 3.14, 0.0, 3.14};

    // std::vector<double> ur1_bin_approach = {0.63, -0.35, 1.0, 3.14, 0.0, 3.14};
    // std::vector<double> ur2_bin_approach = {0.87, 0.35, 1.0, 3.14, 0.0, 3.14};

    // std::vector<double> ur1_inspect_pos = {0.6, 0.28, 1.0, 3.14, 0, 3.14};
    // std::vector<double> ur2_inspect_pos = {0.9, 0.28, 1.0, 3.14, 0, 3.14};

    // std::vector<double> ur1_inspect_approach = {0.6, 0.28, 1.0, -3.14, -1.3, -3.14};
    // std::vector<double> ur2_inspect_approach = {0.9, 0.28, 1.0, -3.14, 1.3, -3.14};


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

    
    // Define Robot EE Goal Pose

    std::vector<double> ur1_goal_EE_state;
    std::vector<double> ur2_goal_EE_state;
    std::vector<double> ur3_goal_EE_state;
    std::vector<double> ur4_goal_EE_state;

    Eigen::Vector3d ur1_goal_EE_position;
    Eigen::Vector3d ur2_goal_EE_position;
    Eigen::Vector3d ur3_goal_EE_position;
    Eigen::Vector3d ur4_goal_EE_position;

    Eigen::Vector3d ur1_goal_EE_orientation;  // [roll, pitch, yaw]
    Eigen::Vector3d ur2_goal_EE_orientation;  // [roll, pitch, yaw]
    Eigen::Vector3d ur3_goal_EE_orientation;  // [roll, pitch, yaw]
    Eigen::Vector3d ur4_goal_EE_orientation;  // [roll, pitch, yaw]

    bool ur1_start = true;
    bool ur2_start = true;
    bool ur3_start = true;
    bool ur4_start = true;

    
    // Status Info
    bool ur1_goal_reached = false;
    bool ur2_goal_reached = false;
    bool ur3_goal_reached = false;
    bool ur4_goal_reached = false;

    bool ur1_onion_target_reached = false;
    bool ur2_onion_target_reached = false;
    bool ur3_onion_target_reached = false;
    bool ur4_onion_target_reached = false;

    bool ur1_onion_target_approach = false;
    bool ur2_onion_target_approach = false;
    bool ur3_onion_target_approach = false;
    bool ur4_onion_target_approach = false;

    bool ur1_onion_target_retreat = false;
    bool ur2_onion_target_retreat = false;
    bool ur3_onion_target_retreat = false;
    bool ur4_onion_target_retreat = false;

    bool ur1_safe_pos_reached = false;
    bool ur2_safe_pos_reached = false;
    bool ur3_safe_pos_reached = false;
    bool ur4_safe_pos_reached = false;

    bool ur1_inspection_pose_reached = false;
    bool ur2_inspection_pose_reached = false;
    bool ur3_inspection_pose_reached = false;
    bool ur4_inspection_pose_reached = false;

    bool ur1_inspection_pose_approach = false;
    bool ur2_inspection_pose_approach = false;
    bool ur3_inspection_pose_approach = false;
    bool ur4_inspection_pose_approach = false;

    bool ur1_inspection_pose_retreat = false;
    bool ur2_inspection_pose_retreat = false;
    bool ur3_inspection_pose_retreat = false;
    bool ur4_inspection_pose_retreat = false;


    bool ur1_bin_pose_approach = false;
    bool ur2_bin_pose_approach = false;
    bool ur3_bin_pose_approach = false;
    bool ur4_bin_pose_approach = false;

    bool ur1_bin_reached = false;
    bool ur2_bin_reached = false;
    bool ur3_bin_reached = false;
    bool ur4_bin_reached = false;


    std::string ur1_target_id;
    std::string ur2_target_id;
    std::string ur3_target_id;
    std::string ur4_target_id;


    while (!onion_ids.empty()){                   // TODO: Update --> while(ur1_closest is not "" || ur2_closest is not "")

        ur1_goal_reached = false;
        ur2_goal_reached = false;
        ur3_goal_reached = false;
        ur4_goal_reached = false;

        //  Continue till the goal of both URs are reached

        while (!ur1_goal_reached || !ur2_goal_reached || !ur3_goal_reached || !ur4_goal_reached){

            // UR-1 GOAL POSITION CASES 

           
           
            // Check ur1_onion_target_reached status
            if (!ur1_onion_target_reached){
                
                // Set Target Pose to the closest onion available for UR-1 

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_closest.second.x();
                ur1_goal_EE_position[1] = ur1_closest.second.y();
                ur1_goal_EE_position[2] = 1.0;

                ur1_goal_EE_orientation[0] = 3.14;
                ur1_goal_EE_orientation[1] = 0.0;
                ur1_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-1 onion target approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_onion_target_reached = true;
                }

                len = min(max_len, alpha*distance(ur1_current_EE_position, ur1_goal_EE_position));
                
            }

            // UR-1 Gripper Down
            else if (!ur1_onion_target_approach){ 

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_closest.second.x();
                ur1_goal_EE_position[1] = ur1_closest.second.y();
                ur1_goal_EE_position[2] = 0.96;

                ur1_goal_EE_orientation[0] = 3.14;
                ur1_goal_EE_orientation[1] = 0.0;
                ur1_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-1 Gripper Down Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_onion_target_approach = true;

                    CallDynamicDetachableJointService("ur1", ur1_target_id, "ATTACH");

                    // publishToTopic("attach", "ur1", ur1_target_id);
                }

                len = min(max_len, alpha*distance(ur1_current_EE_position, ur1_goal_EE_position));
            }

            // UR-1 Gripper Up
            else if (!ur1_onion_target_retreat){

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_closest.second.x();
                ur1_goal_EE_position[1] = ur1_closest.second.y();
                ur1_goal_EE_position[2] = 1.0;

                ur1_goal_EE_orientation[0] = 3.14;
                ur1_goal_EE_orientation[1] = 0.0;
                ur1_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-1 Gripper Up Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_onion_target_retreat = true;
                }

                len = min(max_len, alpha*distance(ur1_current_EE_position, ur1_goal_EE_position));
            }

            // UR-1 Safety Position
            else if (!ur1_safe_pos_reached)
            {
                ur1_goal_EE_position[0] = ur1_safe_pos[0];
                ur1_goal_EE_position[1] = ur1_safe_pos[1];
                ur1_goal_EE_position[2] = ur1_safe_pos[2];

                ur1_goal_EE_orientation[0] = ur1_safe_pos[3];
                ur1_goal_EE_orientation[1] = ur1_safe_pos[4];
                ur1_goal_EE_orientation[2] = ur1_safe_pos[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-1 Safety Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_safe_pos_reached = true;
                }

                len = min(max_len, alpha*distance(ur1_current_EE_position, ur1_goal_EE_position));
            } 

            // UR-1 Inspection Position Reached
            else if (!ur1_inspection_pose_reached){

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_inspect_pos[0];
                ur1_goal_EE_position[1] = ur1_inspect_pos[1];
                ur1_goal_EE_position[2] = ur1_inspect_pos[2];

                ur1_goal_EE_orientation[0] = 3.14;
                ur1_goal_EE_orientation[1] = 0.0;
                ur1_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-1 Inspection Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_inspection_pose_reached = true;
                }

                len = min(max_len, alpha*distance(ur1_current_EE_position, ur1_goal_EE_position));
            }
            
            // UR-1 Inspection Orientation Approach
            else if (!ur1_inspection_pose_approach){

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_inspect_approach[0];
                ur1_goal_EE_position[1] = ur1_inspect_approach[1];
                ur1_goal_EE_position[2] = ur1_inspect_approach[2];

                ur1_goal_EE_orientation[0] = ur1_inspect_approach[3];
                ur1_goal_EE_orientation[1] = ur1_inspect_approach[4];
                ur1_goal_EE_orientation[2] = ur1_inspect_approach[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-1 Inspection Orientation Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_inspection_pose_approach = true;
                }

                len = min(max_len, alpha*distance(ur1_current_EE_position, ur1_goal_EE_position));
            }

            // UR-1 Inspection Position Retreat
            else if (!ur1_inspection_pose_retreat){

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_inspect_pos[0];
                ur1_goal_EE_position[1] = ur1_inspect_pos[1];
                ur1_goal_EE_position[2] = ur1_inspect_pos[2];

                ur1_goal_EE_orientation[0] = 3.14;
                ur1_goal_EE_orientation[1] = 0.0;
                ur1_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-1 Inspection Pose Retreat Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_inspection_pose_retreat = true;
                }

                len = min(max_len, alpha*distance(ur1_current_EE_position, ur1_goal_EE_position));
            }            

            // UR-1 Bin Approach Position
            else if (!ur1_bin_pose_approach){

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_bin_approach[0];
                ur1_goal_EE_position[1] = ur1_bin_approach[1];
                ur1_goal_EE_position[2] = ur1_bin_approach[2];

                ur1_goal_EE_orientation[0] = ur1_inspect_approach[3];
                ur1_goal_EE_orientation[1] = ur1_inspect_approach[4];
                ur1_goal_EE_orientation[2] = ur1_inspect_approach[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-1 Bin Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_bin_pose_approach = true;
                }

                len = min(max_len, alpha*distance(ur1_current_EE_position, ur1_goal_EE_position));
            }

            // UR-1 Bin Reached
            else if (!ur1_bin_reached){

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_bin[0];
                ur1_goal_EE_position[1] = ur1_bin[1];
                ur1_goal_EE_position[2] = ur1_bin[2];

                ur1_goal_EE_orientation[0] = ur1_bin_approach[3];
                ur1_goal_EE_orientation[1] = ur1_bin_approach[4];
                ur1_goal_EE_orientation[2] = ur1_bin_approach[5];


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_bin_reached = true;

                    CallDynamicDetachableJointService("ur1", ur1_target_id, "DETACH");

                    // publishToTopic("detach", "ur1", ur1_target_id);

                    // ur1_closest = findClosestOnionQuick(ur1_current_EE_position, onion_ids, node);
                }

                len = min(max_len, alpha*distance(ur1_current_EE_position, ur1_goal_EE_position));
            }



            


            // UR-2 GOAL POSITION CASES 

           
           
            // Check ur2_onion_target_reached status
            if (!ur2_onion_target_reached){
                
                // Set Target Pose to the closest onion available for UR-2 

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_closest.second.x();
                ur2_goal_EE_position[1] = ur2_closest.second.y();
                ur2_goal_EE_position[2] = 1.0;

                ur2_goal_EE_orientation[0] = 3.14;
                ur2_goal_EE_orientation[1] = 0.0;
                ur2_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-2 onion target approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_onion_target_reached = true;
                }

                len = min(max_len, alpha*distance(ur2_current_EE_position, ur2_goal_EE_position));
                
            }

            // UR-2 Gripper Down
            else if (!ur2_onion_target_approach){ 

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_closest.second.x();
                ur2_goal_EE_position[1] = ur2_closest.second.y();
                ur2_goal_EE_position[2] = 0.96;

                ur2_goal_EE_orientation[0] = 3.14;
                ur2_goal_EE_orientation[1] = 0.0;
                ur2_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-2 Gripper Down Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_onion_target_approach = true;

                    CallDynamicDetachableJointService("ur2", ur2_target_id, "ATTACH");

                    // publishToTopic("attach", "ur1", ur2_target_id);
                }

                len = min(max_len, alpha*distance(ur2_current_EE_position, ur2_goal_EE_position));
            }

            // UR-2 Gripper Up
            else if (!ur2_onion_target_retreat){

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_closest.second.x();
                ur2_goal_EE_position[1] = ur2_closest.second.y();
                ur2_goal_EE_position[2] = 1.0;

                ur2_goal_EE_orientation[0] = 3.14;
                ur2_goal_EE_orientation[1] = 0.0;
                ur2_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-2 Gripper Up Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_onion_target_retreat = true;
                }

                len = min(max_len, alpha*distance(ur2_current_EE_position, ur2_goal_EE_position));
            }

            // UR-2 Safety Position
            else if (!ur2_safe_pos_reached)
            {
                ur2_goal_EE_position[0] = ur2_safe_pos[0];
                ur2_goal_EE_position[1] = ur2_safe_pos[1];
                ur2_goal_EE_position[2] = ur2_safe_pos[2];

                ur2_goal_EE_orientation[0] = ur2_safe_pos[3];
                ur2_goal_EE_orientation[1] = ur2_safe_pos[4];
                ur2_goal_EE_orientation[2] = ur2_safe_pos[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-2 Safety Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_safe_pos_reached = true;
                }

                len = min(max_len, alpha*distance(ur2_current_EE_position, ur2_goal_EE_position));
            } 

            // UR-2 Inspection Position Reached
            else if (!ur2_inspection_pose_reached){

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_inspect_pos[0];
                ur2_goal_EE_position[1] = ur2_inspect_pos[1];
                ur2_goal_EE_position[2] = ur2_inspect_pos[2];

                ur2_goal_EE_orientation[0] = 3.14;
                ur2_goal_EE_orientation[1] = 0.0;
                ur2_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-2 Inspection Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_inspection_pose_reached = true;
                }

                len = min(max_len, alpha*distance(ur2_current_EE_position, ur2_goal_EE_position));
            }
            
            // UR-2 Inspection Orientation Approach
            else if (!ur2_inspection_pose_approach){

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_inspect_approach[0];
                ur2_goal_EE_position[1] = ur2_inspect_approach[1];
                ur2_goal_EE_position[2] = ur2_inspect_approach[2];

                ur2_goal_EE_orientation[0] = ur2_inspect_approach[3];
                ur2_goal_EE_orientation[1] = ur2_inspect_approach[4];
                ur2_goal_EE_orientation[2] = ur2_inspect_approach[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-2 Inspection Orientation Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_inspection_pose_approach = true;
                }

                len = min(max_len, alpha*distance(ur2_current_EE_position, ur2_goal_EE_position));
            }

            // UR-2 Inspection Position Retreat
            else if (!ur2_inspection_pose_retreat){

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_inspect_pos[0];
                ur2_goal_EE_position[1] = ur2_inspect_pos[1];
                ur2_goal_EE_position[2] = ur2_inspect_pos[2];

                ur2_goal_EE_orientation[0] = 3.14;
                ur2_goal_EE_orientation[1] = 0.0;
                ur2_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-2 Inspection Pose Retreat Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_inspection_pose_retreat = true;
                }

                len = min(max_len, alpha*distance(ur2_current_EE_position, ur2_goal_EE_position));
            }            

            // UR-2 Bin Approach Position
            else if (!ur2_bin_pose_approach){

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_bin_approach[0];
                ur2_goal_EE_position[1] = ur2_bin_approach[1];
                ur2_goal_EE_position[2] = ur2_bin_approach[2];

                ur2_goal_EE_orientation[0] = ur2_inspect_approach[3];
                ur2_goal_EE_orientation[1] = ur2_inspect_approach[4];
                ur2_goal_EE_orientation[2] = ur2_inspect_approach[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-2 Bin Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_bin_pose_approach = true;
                }

                len = min(max_len, alpha*distance(ur2_current_EE_position, ur2_goal_EE_position));
            }

            // UR-2 Bin Reached
            else if (!ur2_bin_reached){

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_bin[0];
                ur2_goal_EE_position[1] = ur2_bin[1];
                ur2_goal_EE_position[2] = ur2_bin[2];

                ur2_goal_EE_orientation[0] = ur2_bin_approach[3];
                ur2_goal_EE_orientation[1] = ur2_bin_approach[4];
                ur2_goal_EE_orientation[2] = ur2_bin_approach[5];


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_bin_reached = true;

                    CallDynamicDetachableJointService("ur2", ur2_target_id, "DETACH");

                    // publishToTopic("detach", "ur1", ur2_target_id);

                    // ur2_closest = findClosestOnionQuick(ur2_current_EE_position, onion_ids, node);
                }

                len = min(max_len, alpha*distance(ur2_current_EE_position, ur2_goal_EE_position));
            }


            



            // UR-3 GOAL POSITION CASES 

           
           
            // Check ur3_onion_target_reached status
            if (!ur3_onion_target_reached){
                
                // Set Target Pose to the closest onion available for UR-3 

                ur3_target_id = ur3_closest.first;
                ur3_goal_EE_position[0] = ur3_closest.second.x();
                ur3_goal_EE_position[1] = ur3_closest.second.y();
                ur3_goal_EE_position[2] = 1.0;

                ur3_goal_EE_orientation[0] = 3.14;
                ur3_goal_EE_orientation[1] = 0.0;
                ur3_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-3 onion target approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur3_goal_reached = goal_reached(ur3_current_EE_position, ur3_goal_EE_position, epsilon);
                if (ur3_goal_reached){
                    ur3_onion_target_reached = true;
                }

                len = min(max_len, alpha*distance(ur3_current_EE_position, ur3_goal_EE_position));
                
            }

            // UR-3 Gripper Down
            else if (!ur3_onion_target_approach){ 

                ur3_target_id = ur3_closest.first;
                ur3_goal_EE_position[0] = ur3_closest.second.x();
                ur3_goal_EE_position[1] = ur3_closest.second.y();
                ur3_goal_EE_position[2] = 0.96;

                ur3_goal_EE_orientation[0] = 3.14;
                ur3_goal_EE_orientation[1] = 0.0;
                ur3_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-3 Gripper Down Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur3_goal_reached = goal_reached(ur3_current_EE_position, ur3_goal_EE_position, epsilon);
                if (ur3_goal_reached){
                    ur3_onion_target_approach = true;

                    CallDynamicDetachableJointService("ur3", ur3_target_id, "ATTACH");

                    // publishToTopic("attach", "ur1", ur3_target_id);
                }

                len = min(max_len, alpha*distance(ur3_current_EE_position, ur3_goal_EE_position));
            }

            // UR-3 Gripper Up
            else if (!ur3_onion_target_retreat){

                ur3_target_id = ur3_closest.first;
                ur3_goal_EE_position[0] = ur3_closest.second.x();
                ur3_goal_EE_position[1] = ur3_closest.second.y();
                ur3_goal_EE_position[2] = 1.0;

                ur3_goal_EE_orientation[0] = 3.14;
                ur3_goal_EE_orientation[1] = 0.0;
                ur3_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-3 Gripper Up Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur3_goal_reached = goal_reached(ur3_current_EE_position, ur3_goal_EE_position, epsilon);
                if (ur3_goal_reached){
                    ur3_onion_target_retreat = true;
                }

                len = min(max_len, alpha*distance(ur3_current_EE_position, ur3_goal_EE_position));
            }

            // UR-3 Safety Position
            else if (!ur3_safe_pos_reached)
            {
                ur3_goal_EE_position[0] = ur3_safe_pos[0];
                ur3_goal_EE_position[1] = ur3_safe_pos[1];
                ur3_goal_EE_position[2] = ur3_safe_pos[2];

                ur3_goal_EE_orientation[0] = ur3_safe_pos[3];
                ur3_goal_EE_orientation[1] = ur3_safe_pos[4];
                ur3_goal_EE_orientation[2] = ur3_safe_pos[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-3 Safety Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur3_goal_reached = goal_reached(ur3_current_EE_position, ur3_goal_EE_position, epsilon);
                if (ur3_goal_reached){
                    ur3_safe_pos_reached = true;
                }

                len = min(max_len, alpha*distance(ur3_current_EE_position, ur3_goal_EE_position));
            } 

            // UR-3 Inspection Position Reached
            else if (!ur3_inspection_pose_reached){

                ur3_target_id = ur3_closest.first;
                ur3_goal_EE_position[0] = ur3_inspect_pos[0];
                ur3_goal_EE_position[1] = ur3_inspect_pos[1];
                ur3_goal_EE_position[2] = ur3_inspect_pos[2];

                ur3_goal_EE_orientation[0] = 3.14;
                ur3_goal_EE_orientation[1] = 0.0;
                ur3_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-3 Inspection Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur3_goal_reached = goal_reached(ur3_current_EE_position, ur3_goal_EE_position, epsilon);
                if (ur3_goal_reached){
                    ur3_inspection_pose_reached = true;
                }

                len = min(max_len, alpha*distance(ur3_current_EE_position, ur3_goal_EE_position));
            }
            
            // UR-3 Inspection Orientation Approach
            else if (!ur3_inspection_pose_approach){

                ur3_target_id = ur3_closest.first;
                ur3_goal_EE_position[0] = ur3_inspect_approach[0];
                ur3_goal_EE_position[1] = ur3_inspect_approach[1];
                ur3_goal_EE_position[2] = ur3_inspect_approach[2];

                ur3_goal_EE_orientation[0] = ur3_inspect_approach[3];
                ur3_goal_EE_orientation[1] = ur3_inspect_approach[4];
                ur3_goal_EE_orientation[2] = ur3_inspect_approach[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-3 Inspection Orientation Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur3_goal_reached = goal_reached(ur3_current_EE_position, ur3_goal_EE_position, epsilon);
                if (ur3_goal_reached){
                    ur3_inspection_pose_approach = true;
                }

                len = min(max_len, alpha*distance(ur3_current_EE_position, ur3_goal_EE_position));
            }

            // UR-3 Inspection Position Retreat
            else if (!ur3_inspection_pose_retreat){

                ur3_target_id = ur3_closest.first;
                ur3_goal_EE_position[0] = ur3_inspect_pos[0];
                ur3_goal_EE_position[1] = ur3_inspect_pos[1];
                ur3_goal_EE_position[2] = ur3_inspect_pos[2];

                ur3_goal_EE_orientation[0] = 3.14;
                ur3_goal_EE_orientation[1] = 0.0;
                ur3_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-3 Inspection Pose Retreat Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur3_goal_reached = goal_reached(ur3_current_EE_position, ur3_goal_EE_position, epsilon);
                if (ur3_goal_reached){
                    ur3_inspection_pose_retreat = true;
                }

                len = min(max_len, alpha*distance(ur3_current_EE_position, ur3_goal_EE_position));
            }            

            // UR-3 Bin Approach Position
            else if (!ur3_bin_pose_approach){

                ur3_target_id = ur3_closest.first;
                ur3_goal_EE_position[0] = ur3_bin_approach[0];
                ur3_goal_EE_position[1] = ur3_bin_approach[1];
                ur3_goal_EE_position[2] = ur3_bin_approach[2];

                ur3_goal_EE_orientation[0] = ur3_inspect_approach[3];
                ur3_goal_EE_orientation[1] = ur3_inspect_approach[4];
                ur3_goal_EE_orientation[2] = ur3_inspect_approach[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-3 Bin Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur3_goal_reached = goal_reached(ur3_current_EE_position, ur3_goal_EE_position, epsilon);
                if (ur3_goal_reached){
                    ur3_bin_pose_approach = true;
                }

                len = min(max_len, alpha*distance(ur3_current_EE_position, ur3_goal_EE_position));
            }

            // UR-3 Bin Reached
            else if (!ur3_bin_reached){

                ur3_target_id = ur3_closest.first;
                ur3_goal_EE_position[0] = ur3_bin[0];
                ur3_goal_EE_position[1] = ur3_bin[1];
                ur3_goal_EE_position[2] = ur3_bin[2];

                ur3_goal_EE_orientation[0] = ur3_bin_approach[3];
                ur3_goal_EE_orientation[1] = ur3_bin_approach[4];
                ur3_goal_EE_orientation[2] = ur3_bin_approach[5];


                ur3_goal_reached = goal_reached(ur3_current_EE_position, ur3_goal_EE_position, epsilon);
                if (ur3_goal_reached){
                    ur3_bin_reached = true;

                    CallDynamicDetachableJointService("ur3", ur3_target_id, "DETACH");

                    // publishToTopic("detach", "ur1", ur3_target_id);

                    // ur3_closest = findClosestOnionQuick(ur3_current_EE_position, onion_ids, node);
                }

                len = min(max_len, alpha*distance(ur3_current_EE_position, ur3_goal_EE_position));
            }

            



            // UR-4 GOAL POSITION CASES 

           
           
            // Check ur4_onion_target_reached status
            if (!ur4_onion_target_reached){
                
                // Set Target Pose to the closest onion available for UR-4 

                ur4_target_id = ur4_closest.first;
                ur4_goal_EE_position[0] = ur4_closest.second.x();
                ur4_goal_EE_position[1] = ur4_closest.second.y();
                ur4_goal_EE_position[2] = 1.0;

                ur4_goal_EE_orientation[0] = 3.14;
                ur4_goal_EE_orientation[1] = 0.0;
                ur4_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-4 onion target approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur4_goal_reached = goal_reached(ur4_current_EE_position, ur4_goal_EE_position, epsilon);
                if (ur4_goal_reached){
                    ur4_onion_target_reached = true;
                }

                len = min(max_len, alpha*distance(ur4_current_EE_position, ur4_goal_EE_position));
                
            }

            // UR-4 Gripper Down
            else if (!ur4_onion_target_approach){ 

                ur4_target_id = ur4_closest.first;
                ur4_goal_EE_position[0] = ur4_closest.second.x();
                ur4_goal_EE_position[1] = ur4_closest.second.y();
                ur4_goal_EE_position[2] = 0.96;

                ur4_goal_EE_orientation[0] = 3.14;
                ur4_goal_EE_orientation[1] = 0.0;
                ur4_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-4 Gripper Down Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur4_goal_reached = goal_reached(ur4_current_EE_position, ur4_goal_EE_position, epsilon);
                if (ur4_goal_reached){
                    ur4_onion_target_approach = true;

                    CallDynamicDetachableJointService("ur4", ur4_target_id, "ATTACH");

                    // publishToTopic("attach", "ur1", ur4_target_id);
                }

                len = min(max_len, alpha*distance(ur4_current_EE_position, ur4_goal_EE_position));
            }

            // UR-4 Gripper Up
            else if (!ur4_onion_target_retreat){

                ur4_target_id = ur4_closest.first;
                ur4_goal_EE_position[0] = ur4_closest.second.x();
                ur4_goal_EE_position[1] = ur4_closest.second.y();
                ur4_goal_EE_position[2] = 1.0;

                ur4_goal_EE_orientation[0] = 3.14;
                ur4_goal_EE_orientation[1] = 0.0;
                ur4_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-4 Gripper Up Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur4_goal_reached = goal_reached(ur4_current_EE_position, ur4_goal_EE_position, epsilon);
                if (ur4_goal_reached){
                    ur4_onion_target_retreat = true;
                }

                len = min(max_len, alpha*distance(ur4_current_EE_position, ur4_goal_EE_position));
            }

            // UR-4 Safety Position
            else if (!ur4_safe_pos_reached)
            {
                ur4_goal_EE_position[0] = ur4_safe_pos[0];
                ur4_goal_EE_position[1] = ur4_safe_pos[1];
                ur4_goal_EE_position[2] = ur4_safe_pos[2];

                ur4_goal_EE_orientation[0] = ur4_safe_pos[3];
                ur4_goal_EE_orientation[1] = ur4_safe_pos[4];
                ur4_goal_EE_orientation[2] = ur4_safe_pos[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-4 Safety Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur4_goal_reached = goal_reached(ur4_current_EE_position, ur4_goal_EE_position, epsilon);
                if (ur4_goal_reached){
                    ur4_safe_pos_reached = true;
                }

                len = min(max_len, alpha*distance(ur4_current_EE_position, ur4_goal_EE_position));
            } 

            // UR-4 Inspection Position Reached
            else if (!ur4_inspection_pose_reached){

                ur4_target_id = ur4_closest.first;
                ur4_goal_EE_position[0] = ur4_inspect_pos[0];
                ur4_goal_EE_position[1] = ur4_inspect_pos[1];
                ur4_goal_EE_position[2] = ur4_inspect_pos[2];

                ur4_goal_EE_orientation[0] = 3.14;
                ur4_goal_EE_orientation[1] = 0.0;
                ur4_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-4 Inspection Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur4_goal_reached = goal_reached(ur4_current_EE_position, ur4_goal_EE_position, epsilon);
                if (ur4_goal_reached){
                    ur4_inspection_pose_reached = true;
                }

                len = min(max_len, alpha*distance(ur4_current_EE_position, ur4_goal_EE_position));
            }
            
            // UR-4 Inspection Orientation Approach
            else if (!ur4_inspection_pose_approach){

                ur4_target_id = ur4_closest.first;
                ur4_goal_EE_position[0] = ur4_inspect_approach[0];
                ur4_goal_EE_position[1] = ur4_inspect_approach[1];
                ur4_goal_EE_position[2] = ur4_inspect_approach[2];

                ur4_goal_EE_orientation[0] = ur4_inspect_approach[3];
                ur4_goal_EE_orientation[1] = ur4_inspect_approach[4];
                ur4_goal_EE_orientation[2] = ur4_inspect_approach[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-4 Inspection Orientation Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur4_goal_reached = goal_reached(ur4_current_EE_position, ur4_goal_EE_position, epsilon);
                if (ur4_goal_reached){
                    ur4_inspection_pose_approach = true;
                }

                len = min(max_len, alpha*distance(ur4_current_EE_position, ur4_goal_EE_position));
            }

            // UR-4 Inspection Position Retreat
            else if (!ur4_inspection_pose_retreat){

                ur4_target_id = ur4_closest.first;
                ur4_goal_EE_position[0] = ur4_inspect_pos[0];
                ur4_goal_EE_position[1] = ur4_inspect_pos[1];
                ur4_goal_EE_position[2] = ur4_inspect_pos[2];

                ur4_goal_EE_orientation[0] = 3.14;
                ur4_goal_EE_orientation[1] = 0.0;
                ur4_goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-4 Inspection Pose Retreat Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur4_goal_reached = goal_reached(ur4_current_EE_position, ur4_goal_EE_position, epsilon);
                if (ur4_goal_reached){
                    ur4_inspection_pose_retreat = true;
                }

                len = min(max_len, alpha*distance(ur4_current_EE_position, ur4_goal_EE_position));
            }            

            // UR-4 Bin Approach Position
            else if (!ur4_bin_pose_approach){

                ur4_target_id = ur4_closest.first;
                ur4_goal_EE_position[0] = ur4_bin_approach[0];
                ur4_goal_EE_position[1] = ur4_bin_approach[1];
                ur4_goal_EE_position[2] = ur4_bin_approach[2];

                ur4_goal_EE_orientation[0] = ur4_inspect_approach[3];
                ur4_goal_EE_orientation[1] = ur4_inspect_approach[4];
                ur4_goal_EE_orientation[2] = ur4_inspect_approach[5];

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "UR-4 Bin Pose Approach Case");
                RCLCPP_INFO(node->get_logger(), " ");


                ur4_goal_reached = goal_reached(ur4_current_EE_position, ur4_goal_EE_position, epsilon);
                if (ur4_goal_reached){
                    ur4_bin_pose_approach = true;
                }

                len = min(max_len, alpha*distance(ur4_current_EE_position, ur4_goal_EE_position));
            }

            // UR-4 Bin Reached
            else if (!ur4_bin_reached){

                ur4_target_id = ur4_closest.first;
                ur4_goal_EE_position[0] = ur4_bin[0];
                ur4_goal_EE_position[1] = ur4_bin[1];
                ur4_goal_EE_position[2] = ur4_bin[2];

                ur4_goal_EE_orientation[0] = ur4_bin_approach[3];
                ur4_goal_EE_orientation[1] = ur4_bin_approach[4];
                ur4_goal_EE_orientation[2] = ur4_bin_approach[5];


                ur4_goal_reached = goal_reached(ur4_current_EE_position, ur4_goal_EE_position, epsilon);
                if (ur4_goal_reached){
                    ur4_bin_reached = true;

                    CallDynamicDetachableJointService("ur4", ur4_target_id, "DETACH");

                    // publishToTopic("detach", "ur1", ur4_target_id);

                    // ur4_closest = findClosestOnionQuick(ur4_current_EE_position, onion_ids, node);
                }

                len = min(max_len, alpha*distance(ur4_current_EE_position, ur4_goal_EE_position));
            }






            // Plan for those Goal Position using Centralized Approach

            Eigen::Vector3d ur1_next_vel;
            Eigen::Vector3d ur2_next_vel;
            Eigen::Vector3d ur3_next_vel;
            Eigen::Vector3d ur4_next_vel;


            ur1_next_vel = Baseline(node, ur1_goal_EE_position, ur1_current_EE_position, ur1_pref_speed, "ur1");
            ur2_next_vel = Baseline(node, ur2_goal_EE_position, ur2_current_EE_position, ur2_pref_speed, "ur2");
            ur3_next_vel = Baseline(node, ur3_goal_EE_position, ur3_current_EE_position, ur3_pref_speed, "ur3");
            ur4_next_vel = Baseline(node, ur4_goal_EE_position, ur4_current_EE_position, ur4_pref_speed, "ur4");



            // ----------------------------------------------------------------------------------------------------------------------------------------- //



            Eigen::Vector3d ur1_target_unit_vec = Eigen::Vector3d::Zero();
            Eigen::Vector3d ur2_target_unit_vec = Eigen::Vector3d::Zero();
            Eigen::Vector3d ur3_target_unit_vec = Eigen::Vector3d::Zero();
            Eigen::Vector3d ur4_target_unit_vec = Eigen::Vector3d::Zero();

            if (ur1_next_vel.norm() > 1e-6) ur1_target_unit_vec = ur1_next_vel.normalized();
            if (ur2_next_vel.norm() > 1e-6) ur2_target_unit_vec = ur2_next_vel.normalized();
            if (ur3_next_vel.norm() > 1e-6) ur3_target_unit_vec = ur3_next_vel.normalized();
            if (ur4_next_vel.norm() > 1e-6) ur4_target_unit_vec = ur4_next_vel.normalized();







            // Set the Pose Target

            geometry_msgs::msg::Pose ur1_target_pose = set_vel_pose_target(ur1_current_EE_position, ur1_target_unit_vec, len, ur1_goal_EE_orientation, ur1_goal_reached);
            geometry_msgs::msg::Pose ur2_target_pose = set_vel_pose_target(ur2_current_EE_position, ur2_target_unit_vec, len, ur2_goal_EE_orientation, ur2_goal_reached);
            geometry_msgs::msg::Pose ur3_target_pose = set_vel_pose_target(ur3_current_EE_position, ur3_target_unit_vec, len, ur3_goal_EE_orientation, ur3_goal_reached);
            geometry_msgs::msg::Pose ur4_target_pose = set_vel_pose_target(ur4_current_EE_position, ur4_target_unit_vec, len, ur4_goal_EE_orientation, ur4_goal_reached);






            // Solve for the IK Solution of the Target Pose

            double timeout = 1.0;
            bool ur1_found_ik = ur_combined_state->setFromIK(ur1_joint_model_group, ur1_target_pose, timeout);
            bool ur2_found_ik = ur_combined_state->setFromIK(ur2_joint_model_group, ur2_target_pose, timeout);
            bool ur3_found_ik = ur_combined_state->setFromIK(ur3_joint_model_group, ur3_target_pose, timeout);
            bool ur4_found_ik = ur_combined_state->setFromIK(ur4_joint_model_group, ur4_target_pose, timeout);


            if (ur1_found_ik && ur2_found_ik && ur3_found_ik && ur4_found_ik)
            {
                ur_combined_state->copyJointGroupPositions(ur1_joint_model_group, ur1_joint_values);
                ur_combined_state->copyJointGroupPositions(ur2_joint_model_group, ur2_joint_values);
                ur_combined_state->copyJointGroupPositions(ur3_joint_model_group, ur3_joint_values);
                ur_combined_state->copyJointGroupPositions(ur4_joint_model_group, ur4_joint_values);

                // for (std::size_t i = 0; i < ur1_joint_names.size(); ++i)
                // {
                // RCLCPP_INFO(node->get_logger(), "UR - 1 Joint %s: %f", ur1_joint_names[i].c_str(), ur1_joint_values[i]);
                // RCLCPP_INFO(node->get_logger(), "UR - 2 Joint %s: %f", ur2_joint_names[i].c_str(), ur2_joint_values[i]);
                // }
            }
            else
            {
                if (!ur1_found_ik){
                    RCLCPP_INFO(node->get_logger(), "Did not find IK solution for UR-1");
                }

                if (!ur2_found_ik){
                    RCLCPP_INFO(node->get_logger(), "Did not find IK solution for UR-2");
                }

                if (!ur3_found_ik){
                    RCLCPP_INFO(node->get_logger(), "Did not find IK solution for UR-3");
                }

                if (!ur4_found_ik){
                    RCLCPP_INFO(node->get_logger(), "Did not find IK solution for UR-4");
                }
            }


            ur_move_group_interface.setJointValueTarget(ur1_joint_names, ur1_joint_values);
            ur_move_group_interface.setJointValueTarget(ur2_joint_names, ur2_joint_values);
            ur_move_group_interface.setJointValueTarget(ur3_joint_names, ur3_joint_values);
            ur_move_group_interface.setJointValueTarget(ur4_joint_names, ur4_joint_values);


            // Construct Plan and Execute It
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (ur_move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if(!success){
                RCLCPP_INFO(node->get_logger(), "Plan did not succeed");
            }
            ur_move_group_interface.execute(my_plan);





            // ----------------------------------------------------------------------------------------------------------------------------------------- //

            // Update Current Pose after Plan Execution

            Eigen::Vector3d ur1_prev_EE_position = ur1_current_EE_position;
            Eigen::Vector3d ur2_prev_EE_position = ur2_current_EE_position;
            Eigen::Vector3d ur3_prev_EE_position = ur3_current_EE_position;
            Eigen::Vector3d ur4_prev_EE_position = ur4_current_EE_position;

            ur1_current_state = ur1_move_group_interface.getCurrentState();
            ur2_current_state = ur2_move_group_interface.getCurrentState();
            ur3_current_state = ur3_move_group_interface.getCurrentState();
            ur4_current_state = ur4_move_group_interface.getCurrentState();

            ur1_current_EE_state = ur1_current_state->getGlobalLinkTransform(ur1_ee);
            ur2_current_EE_state = ur2_current_state->getGlobalLinkTransform(ur2_ee);
            ur3_current_EE_state = ur3_current_state->getGlobalLinkTransform(ur3_ee);
            ur4_current_EE_state = ur4_current_state->getGlobalLinkTransform(ur4_ee);


            ur1_current_EE_position = ur1_current_EE_state.translation();
            ur2_current_EE_position = ur2_current_EE_state.translation();
            ur3_current_EE_position = ur3_current_EE_state.translation();
            ur4_current_EE_position = ur4_current_EE_state.translation();

            ur1_current_EE_orientation = Eigen::Quaterniond(ur1_current_EE_state.rotation());
            ur2_current_EE_orientation = Eigen::Quaterniond(ur2_current_EE_state.rotation());
            ur3_current_EE_orientation = Eigen::Quaterniond(ur3_current_EE_state.rotation());
            ur4_current_EE_orientation = Eigen::Quaterniond(ur4_current_EE_state.rotation());


            // Convert quaternion to Euler angles
            ur1_euler_angles = ur1_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
            ur2_euler_angles = ur2_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
            ur3_euler_angles = ur1_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
            ur4_euler_angles = ur2_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw

            // Store Euler angles in std::vector for compatibility
            ur1_current_EE_orientation_euler = {ur1_euler_angles[0], ur1_euler_angles[1], ur1_euler_angles[2]};
            ur2_current_EE_orientation_euler = {ur2_euler_angles[0], ur2_euler_angles[1], ur2_euler_angles[2]};
            ur3_current_EE_orientation_euler = {ur3_euler_angles[0], ur3_euler_angles[1], ur3_euler_angles[2]};
            ur4_current_EE_orientation_euler = {ur4_euler_angles[0], ur4_euler_angles[1], ur4_euler_angles[2]};


            // RCLCPP_INFO(logger, "************************************************************************");

            // RCLCPP_INFO(logger, "UR1 Current EE Orientation Euler: [roll: %f, pitch: %f, yaw: %f]",
            // ur1_current_EE_orientation_euler[0], 
            // ur1_current_EE_orientation_euler[1], 
            // ur1_current_EE_orientation_euler[2]);

            // RCLCPP_INFO(logger, "UR1 Goal Orientation Euler: [roll: %f, pitch: %f, yaw: %f]",
            // ur1_goal_orientation_euler[0], 
            // ur1_goal_orientation_euler[1], 
            // ur1_goal_orientation_euler[2]);

            // RCLCPP_INFO(logger, "UR2 Current EE Orientation Euler: [roll: %f, pitch: %f, yaw: %f]",
            // ur2_current_EE_orientation_euler[0], 
            // ur2_current_EE_orientation_euler[1], 
            // ur2_current_EE_orientation_euler[2]);

            // RCLCPP_INFO(logger, "UR2 Goal Orientation Euler: [roll: %f, pitch: %f, yaw: %f]",
            // ur2_goal_orientation_euler[0], 
            // ur2_goal_orientation_euler[1], 
            // ur2_goal_orientation_euler[2]);


            RCLCPP_INFO(logger, "************************************************************************");




            RCLCPP_INFO(logger, " ");

            // RCLCPP_INFO(logger, "UR - 1 New pose position: [x: %f, y: %f, z: %f]", 
            // ur1_current_EE_position(0), 
            // ur1_current_EE_position(1), 
            // ur1_current_EE_position(2));

            // RCLCPP_INFO(logger, " ");

            // RCLCPP_INFO(logger, "UR - 2 New pose position: [x: %f, y: %f, z: %f]", 
            // ur2_current_EE_position(0), 
            // ur2_current_EE_position(1), 
            // ur2_current_EE_position(2));

            // RCLCPP_INFO(logger, " ");

            // RCLCPP_INFO(logger, "UR - 3 New pose position: [x: %f, y: %f, z: %f]", 
            // ur3_current_EE_position(0), 
            // ur3_current_EE_position(1), 
            // ur3_current_EE_position(2));

            // RCLCPP_INFO(logger, " ");

            // RCLCPP_INFO(logger, "UR - 4 New pose position: [x: %f, y: %f, z: %f]", 
            // ur4_current_EE_position(0), 
            // ur4_current_EE_position(1), 
            // ur4_current_EE_position(2));

    

            // RCLCPP_INFO(logger, "%s", std::string(50, '=').c_str());

            ur1_current_EE_vel = ur1_target_unit_vec*ur1_speed;
            ur2_current_EE_vel = ur2_target_unit_vec*ur2_speed;
            ur3_current_EE_vel = ur3_target_unit_vec*ur3_speed;
            ur4_current_EE_vel = ur4_target_unit_vec*ur4_speed;






            // Update the ur1_goal_reached and ur2_goal_reached to avoid exiting the loop
            if (!ur1_bin_reached){
                ur1_goal_reached = false;
            }

            else {
                ur1_closest = findClosestOnionQuick(ur1_current_EE_position, onion_ids, "ur1", ur1_current_state, ur1_joint_model_group);

                ur1_goal_reached = false;
                ur1_onion_target_reached = false;
                ur1_onion_target_approach = false;
                ur1_onion_target_retreat = false;
                ur1_safe_pos_reached = false;
                ur1_inspection_pose_reached = false;
                ur1_inspection_pose_approach = false;
                ur1_inspection_pose_retreat = false;
                ur1_bin_pose_approach = false;
                ur1_bin_reached = false;


                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, " ");
                RCLCPP_INFO(node->get_logger(), "Closest onion to UR-1 is [%s] at position (%f, %f, %f)",
                ur1_closest.first.c_str(),
                ur1_closest.second.x(), ur1_closest.second.y(), ur1_closest.second.z());
                RCLCPP_INFO(logger, " ");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
            }



            if (!ur2_bin_reached){
                ur2_goal_reached = false;
            }

            else {
                ur2_closest = findClosestOnionQuick(ur2_current_EE_position, onion_ids, "ur2", ur2_current_state, ur2_joint_model_group);

                ur2_goal_reached = false;
                ur2_onion_target_reached = false;
                ur2_onion_target_approach = false;
                ur2_onion_target_retreat = false;
                ur2_safe_pos_reached = false;
                ur2_inspection_pose_reached = false;
                ur2_inspection_pose_approach = false;
                ur2_inspection_pose_retreat = false;
                ur2_bin_pose_approach = false;
                ur2_bin_reached = false;

                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, " ");
                RCLCPP_INFO(node->get_logger(), "Closest onion to UR-2 is [%s] at position (%f, %f, %f)",
                ur2_closest.first.c_str(),
                ur2_closest.second.x(), ur2_closest.second.y(), ur2_closest.second.z());
                RCLCPP_INFO(logger, " ");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
            }



            if (!ur3_bin_reached){
                ur3_goal_reached = false;
            }

            else {
                ur3_closest = findClosestOnionQuick(ur3_current_EE_position, onion_ids, "ur3", ur3_current_state, ur3_joint_model_group);

                ur3_goal_reached = false;
                ur3_onion_target_reached = false;
                ur3_onion_target_approach = false;
                ur3_onion_target_retreat = false;
                ur3_safe_pos_reached = false;
                ur3_inspection_pose_reached = false;
                ur3_inspection_pose_approach = false;
                ur3_inspection_pose_retreat = false;
                ur3_bin_pose_approach = false;
                ur3_bin_reached = false;

                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, " ");
                RCLCPP_INFO(node->get_logger(), "Closest onion to UR-3 is [%s] at position (%f, %f, %f)",
                ur3_closest.first.c_str(),
                ur3_closest.second.x(), ur3_closest.second.y(), ur3_closest.second.z());
                RCLCPP_INFO(logger, " ");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
            }



            if (!ur4_bin_reached){
                ur4_goal_reached = false;
            }

            else {
                ur4_closest = findClosestOnionQuick(ur4_current_EE_position, onion_ids, "ur4", ur4_current_state, ur4_joint_model_group);

                ur4_goal_reached = false;
                ur4_onion_target_reached = false;
                ur4_onion_target_approach = false;
                ur4_onion_target_retreat = false;
                ur4_safe_pos_reached = false;
                ur4_inspection_pose_reached = false;
                ur4_inspection_pose_approach = false;
                ur4_inspection_pose_retreat = false;
                ur4_bin_pose_approach = false;
                ur4_bin_reached = false;

                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, " ");
                RCLCPP_INFO(node->get_logger(), "Closest onion to UR-4 is [%s] at position (%f, %f, %f)",
                ur4_closest.first.c_str(),
                ur4_closest.second.x(), ur4_closest.second.y(), ur4_closest.second.z());
                RCLCPP_INFO(logger, " ");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
                RCLCPP_INFO(logger, "=========================================================================");
            }


        }




    }



    RCLCPP_INFO(logger, " ");
    RCLCPP_INFO(logger, "|||||||||||||||||||||||||||        OVER          |||||||||||||||||||||||");
    RCLCPP_INFO(logger, " ");


    // Keep the node alive while the motion is being executed
    // rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}