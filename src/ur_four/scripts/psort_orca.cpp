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

struct Agent {
    Eigen::Vector3d position;
    double radius;
    Eigen::Vector3d velocity;
};


double distance(const Eigen::Vector3d v1, const Eigen::Vector3d v2) {
    return (v1 - v2).norm();
}



int angle_match(const std::vector<double> a1, const std::vector<double> a2){
    if (((a1[0] - a2[0]) > 0.1) || ((a1[1] - a2[1]) > 0.1) || ((a1[2] - a2[2]) > 0.1)){
        return 1;
    }

    else {
        return 0;
    }
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



geometry_msgs::msg::Pose set_pose_target(const Eigen::Vector3d& ur_current_EE_position, const Eigen::Vector3d& ur_goal_EE_orientation)
{
    geometry_msgs::msg::Pose ur_msg;

        ur_msg.position.x = ur_current_EE_position(0); 
        ur_msg.position.y = ur_current_EE_position(1);
        ur_msg.position.z = ur_current_EE_position(2); 
        

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



// Returns a vector of topic names starting with "/model"
std::vector<std::string> getModelTopics(const rclcpp::Node::SharedPtr & node)
{
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



// Global dictionary holding onion poses, populated once using getAllOnionPoses.
std::unordered_map<std::string, Eigen::Vector3d> global_onion_pose_map;

// Function to find the closest onion based on current UR position.
std::pair<std::string, Eigen::Vector3d> findClosestOnion(const Eigen::Vector3d &ur_ee_position, std::vector<std::string> &onion_ids, const rclcpp::Node::SharedPtr &node)
{
  double min_distance = std::numeric_limits<double>::max();
  std::string closest_onion_id = "";
  Eigen::Vector3d closest_onion_position(0.0, 0.0, 0.0);

  // Iterate through each onion id.
  for (const auto &onion_id : onion_ids)
  {
    // Form the topic name.
    std::string topic = "/model/" + onion_id + "/odometry";

    // Retrieve the odometry message (with a 1-second timeout).
    auto odom_msg = getOdomMessage(node, topic, 5.0);
    if (!odom_msg)
    {
      RCLCPP_WARN(node->get_logger(), "No odometry message received for onion id: %s", onion_id.c_str());
      continue;
    }

    // Extract the onion position from the odometry message.
    const auto &p = odom_msg->pose.pose.position;
    Eigen::Vector3d onion_position(p.x, p.y, p.z);

    // Compute the Euclidean distance between the UR position and the onion.
    double distance = (ur_ee_position - onion_position).norm();

    // Log the computed distance.
    RCLCPP_INFO(node->get_logger(), "Distance from UR to onion [%s]: %f", onion_id.c_str(), distance);

    // Update if this onion is closer.
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_onion_id = onion_id;
      closest_onion_position = onion_position;
    }
  }

  // If a closest onion was found, remove it from the list.
  if (!closest_onion_id.empty())
  {
    auto it = std::remove(onion_ids.begin(), onion_ids.end(), closest_onion_id);
    onion_ids.erase(it, onion_ids.end());
  }

  return std::make_pair(closest_onion_id, closest_onion_position);
}


std::pair<std::string, Eigen::Vector3d> findClosestOnionQuick(const Eigen::Vector3d &ur_ee_position, std::vector<std::string> &onion_ids)
{
    double min_distance = std::numeric_limits<double>::max();
    std::string closest_onion_id = "";
    Eigen::Vector3d closest_onion_position(0.0, 0.0, 0.0);

    // Iterate over the onion IDs provided.
    for (const auto &onion_id : onion_ids)
    {
        // Check if the onion's pose is available in the global dictionary.
        auto it = global_onion_pose_map.find(onion_id);
        if (it == global_onion_pose_map.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("findClosestOnionQuick"),
                        "Onion ID '%s' not found in global pose map", onion_id.c_str());
            continue;
        }

        const Eigen::Vector3d &onion_position = it->second;
        double distance = (ur_ee_position - onion_position).norm();

        // Update if this onion is closer.
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_onion_id = onion_id;
            closest_onion_position = onion_position;
        }
    }

    // If a closest onion was found, remove its ID from the list.
    if (!closest_onion_id.empty())
    {
        auto it = std::remove(onion_ids.begin(), onion_ids.end(), closest_onion_id);
        onion_ids.erase(it, onion_ids.end());
    }

    return std::make_pair(closest_onion_id, closest_onion_position);
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

  // Execute the command.
  int ret = std::system(command.c_str());
  if(ret != 0){
    std::cerr << "Command failed with return code: " << ret << std::endl;
  }
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



std::vector<Eigen::Vector3d> ORCA(const Agent reference_agent, const std::vector<Agent> other_agents) 
{
    double responsibility = 1.0;
    std::vector<Eigen::Vector3d> relative_velocity;
    std::vector<std::vector<Eigen::Vector3d>> circle;

    for (const auto& agent : other_agents) {
        Eigen::Vector3d relative_position = agent.position - reference_agent.position;
        double combined_radius = agent.radius + reference_agent.radius;

        // Parametric equation of the sphere
        // Note: This part should ideally be implemented with a 3D visualization library like VTK or Open3D
        //       For the sake of simplicity, the sphere visualization is omitted here.

        // Parametric equation of the circle
        Eigen::Vector3d cc = relative_position - relative_position * (std::pow(combined_radius, 2) / relative_position.squaredNorm());
        double c_radius = combined_radius * std::sqrt((relative_position.squaredNorm() - std::pow(combined_radius, 2)) / relative_position.squaredNorm());

        // 2 orthogonal directional unit vectors
        Eigen::Vector3d v1(relative_position(1), -relative_position(0), 0);
        v1.normalize();
        Eigen::Vector3d v2(-relative_position(0) * relative_position(2), -relative_position(1) * relative_position(2), relative_position(0) * relative_position(0) + relative_position(1) * relative_position(1));
        v2.normalize();

        std::vector<Eigen::Vector3d> circle_points;
        
        circle_points.push_back(cc);
        
        for (double t = 0; t <= 2 * M_PI; t += 0.01) {
            Eigen::Vector3d point = cc + c_radius * std::cos(t) * v1 + c_radius * std::sin(t) * v2;
            circle_points.push_back(point);
        }

        circle.push_back(circle_points);

        // Reference agent velocity
        Eigen::Vector3d ref_vel = reference_agent.velocity;

        // Agent velocity
        Eigen::Vector3d agent_vel = agent.velocity;

        // Relative velocity
        Eigen::Vector3d rel_vel = ref_vel - agent_vel;
        relative_velocity.push_back(rel_vel);

        // Collision cone
        // Note: Collision visualization is omitted here.
    }

    std::vector<Eigen::Vector3d> escape_list;

    for (size_t i = 0; i < other_agents.size(); ++i) {
        Eigen::Vector3d center = circle[i][0];  // Center of the circle 
        std::vector<Eigen::Vector3d> periphery(circle[i].begin() + 1, circle[i].end());  // Exclude the center point

        Eigen::Vector3d first_point = periphery[0];  //  First Circle point on the periphery

        bool condition_1 = center.norm() > relative_velocity[i].norm();  // TODO
        bool condition_2 = (relative_velocity[i].dot(center) / (center.norm() * relative_velocity[i].norm())) > (first_point.dot(center) / (center.norm() * first_point.norm()));

        if (condition_2)   //  TODO - Need to add condition_1 
        {
            std::cout << "Reference Agent is colliding with another agent." << std::endl;

            double denominator = first_point.dot(first_point);
            double min_dist = std::numeric_limits<double>::infinity();
            Eigen::Vector3d line;
            int t_num;

            for (size_t j = 0; j < periphery.size(); ++j) {
                double lambda = relative_velocity[i].dot(periphery[j]) / denominator;
                Eigen::Vector3d escape_vector = lambda * periphery[j] - relative_velocity[i];

                double dist = escape_vector.norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    line = periphery[j];
                    t_num = j;
                }
            }

            double lambda = relative_velocity[i].dot(line) / denominator;
            Eigen::Vector3d escape_vector = lambda * line - relative_velocity[i];

            escape_list.push_back(escape_vector*responsibility);

            std::cout << "Tangent Line number: - " << t_num << std::endl;
        }
    }

    std::cout << std::string(100, '-') << std::endl;

    return escape_list;
}



int main(int argc, char * argv[])
{

    const std::string& ur1_ee = "UR1_wrist_3_link";
    const std::string& ur2_ee = "UR2_wrist_3_link";

    const std::string& ur1_planning_group = "UR1_manipulator";
    const std::string& ur2_planning_group = "UR2_manipulator";
    const std::string& ur_combined_planning_group = "UR_manipulator";

    Eigen::Vector3d ur1_current_EE_vel(0, 0, 0);
    Eigen::Vector3d ur2_current_EE_vel(0, 0, 0);

    double epsilon = 0.03;
    double len = 0.05;

    double ur1_speed = 1.0;
    double ur1_pref_speed = 1.0; //0.05
    double ur1_max_speed = 0.2;

    double ur2_speed = 0.0;
    double ur2_pref_speed = 0.05;
    double ur2_max_speed = 0.2;

    double stagnant_count = 0;


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

    ur_move_group_interface.setMaxVelocityScalingFactor(1.0);
    ur_move_group_interface.setMaxAccelerationScalingFactor(1.0);
    ur_move_group_interface.setPlanningTime(20);

    //   ur1_move_group_interface.setPlanningTime(10.0);  // Set a higher planning time, e.g., 10 seconds.
    //   ur1_move_group_interface.setGoalTolerance(0.1);  // Set a reasonable goal tolerance.

    // Start the state monitors
    ur_move_group_interface.startStateMonitor();
    ur1_move_group_interface.startStateMonitor();
    ur2_move_group_interface.startStateMonitor();

    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    const moveit::core::JointModelGroup* ur1_joint_model_group = kinematic_model->getJointModelGroup("UR1_manipulator");
    const moveit::core::JointModelGroup* ur2_joint_model_group = kinematic_model->getJointModelGroup("UR2_manipulator");

    const std::vector<std::string>& ur1_joint_names = ur1_joint_model_group->getVariableNames();
    const std::vector<std::string>& ur2_joint_names = ur2_joint_model_group->getVariableNames();

    std::vector<double> ur1_joint_values;
    std::vector<double> ur2_joint_values;


    // Get Current Robot EE Pose
    moveit::core::RobotStatePtr ur_combined_state = ur_move_group_interface.getCurrentState();
    moveit::core::RobotStatePtr ur1_current_state = ur1_move_group_interface.getCurrentState();
    moveit::core::RobotStatePtr ur2_current_state = ur2_move_group_interface.getCurrentState();

    Eigen::Affine3d ur1_current_EE_state = ur1_current_state->getGlobalLinkTransform(ur1_ee);
    Eigen::Affine3d ur2_current_EE_state = ur2_current_state->getGlobalLinkTransform(ur2_ee);

    Eigen::Vector3d ur1_current_EE_position = ur1_current_EE_state.translation();
    Eigen::Vector3d ur2_current_EE_position = ur2_current_EE_state.translation();

    Eigen::Quaterniond ur1_current_EE_orientation(ur1_current_EE_state.rotation());
    Eigen::Quaterniond ur2_current_EE_orientation(ur2_current_EE_state.rotation());

    // Convert quaternion to Euler angles
    Eigen::Vector3d ur1_euler_angles = ur1_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
    Eigen::Vector3d ur2_euler_angles = ur2_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw

    // Store Euler angles in std::vector for compatibility
    std::vector<double> ur1_current_EE_orientation_euler = {ur1_euler_angles[0], ur1_euler_angles[1], ur1_euler_angles[2]};
    std::vector<double> ur2_current_EE_orientation_euler = {ur2_euler_angles[0], ur2_euler_angles[1], ur2_euler_angles[2]};

    auto ur1_closest = findClosestOnionQuick(ur1_current_EE_position, onion_ids);
    RCLCPP_INFO(node->get_logger(), "Closest onion to UR-1 is [%s] at position (%f, %f, %f)",
                ur1_closest.first.c_str(),
                ur1_closest.second.x(), ur1_closest.second.y(), ur1_closest.second.z());

    auto ur2_closest = findClosestOnionQuick(ur2_current_EE_position, onion_ids);
    RCLCPP_INFO(node->get_logger(), "Closest onion to UR-2 is [%s] at position (%f, %f, %f)",
                ur2_closest.first.c_str(),
                ur2_closest.second.x(), ur2_closest.second.y(), ur2_closest.second.z());



    // Poses in Gazebo World

    std::vector<double> ur1_bin = {0.35, -0.49, 0.96, 3.14, 0.0, -3.14};
    std::vector<double> ur2_bin = {1.16, 0.49, 0.96, 3.14, 0.0, 3.14};

    std::vector<double> ur1_bin_approach = {0.63, -0.35, 1.0, 3.14, 0.0, 3.14};
    std::vector<double> ur2_bin_approach = {0.87, 0.35, 1.0, 3.14, 0.0, 3.14};

    std::vector<double> ur1_inspect_pos = {0.6, 0.28, 1.0, 3.14, 0, 3.14};
    std::vector<double> ur2_inspect_pos = {0.9, 0.28, 1.0, 3.14, 0, 3.14};

    std::vector<double> ur1_inspect_approach = {0.6, 0.28, 1.0, -3.14, -1.3, -3.14};
    std::vector<double> ur2_inspect_approach = {0.9, 0.28, 1.0, -3.14, 1.3, -3.14};

    
    // Define Robot EE Goal Pose

    std::vector<double> ur1_goal_EE_state;
    std::vector<double> ur2_goal_EE_state;

    Eigen::Vector3d ur1_goal_EE_position;
    Eigen::Vector3d ur2_goal_EE_position;

    Eigen::Vector3d ur1_goal_EE_orientation;  // [roll, pitch, yaw]
    Eigen::Vector3d ur2_goal_EE_orientation;  // [roll, pitch, yaw]

    bool ur1_start = true;
    bool ur2_start = true;

    
    // Status Info
    bool ur1_goal_reached = false;
    bool ur2_goal_reached = false;

    bool ur1_onion_target_reached = false;
    bool ur2_onion_target_reached = false;

    bool ur1_onion_target_approach = false;
    bool ur2_onion_target_approach = false;

    bool ur1_onion_target_retreat = false;
    bool ur2_onion_target_retreat = false;

    bool ur1_inspection_pose_reached = false;
    bool ur2_inspection_pose_reached = false;

    bool ur1_inspection_pose_approach = false;
    bool ur2_inspection_pose_approach = false;

    bool ur1_inspection_pose_retreat = false;
    bool ur2_inspection_pose_retreat = false;

    bool ur1_bin_pose_approach = false;
    bool ur2_bin_pose_approach = false;

    bool ur1_bin_reached = false;
    bool ur2_bin_reached = false;


    std::string ur1_target_id;
    std::string ur2_target_id;


    while (!onion_ids.empty()){                   // TODO: Update --> while(ur1_closest is not "" || ur2_closest is not "")

        ur1_goal_reached = false;
        ur2_goal_reached = false;

        //  Continue till the goal of both URs are reached

        while (!ur1_goal_reached || !ur2_goal_reached){

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


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_onion_target_reached = true;
                }
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


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_onion_target_approach = true;

                    // publishToTopic("attach", "ur1", ur1_target_id);
                }
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


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_onion_target_retreat = true;
                }
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


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_inspection_pose_reached = true;
                }
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


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_inspection_pose_approach = true;
                }
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


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_inspection_pose_retreat = true;
                }
            }            

            // UR-1 Bin Approach Position
            else if (!ur1_bin_pose_approach){

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_bin_approach[0];
                ur1_goal_EE_position[1] = ur1_bin_approach[1];
                ur1_goal_EE_position[2] = ur1_bin_approach[2];

                ur1_goal_EE_orientation[0] = 3.14;
                ur1_goal_EE_orientation[1] = 0.0;
                ur1_goal_EE_orientation[2] = 3.14;


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_bin_pose_approach = true;
                }
            }

            // UR-1 Bin Reached
            else if (!ur1_bin_reached){

                ur1_target_id = ur1_closest.first;
                ur1_goal_EE_position[0] = ur1_bin[0];
                ur1_goal_EE_position[1] = ur1_bin[1];
                ur1_goal_EE_position[2] = ur1_bin[2];

                ur1_goal_EE_orientation[0] = 3.14;
                ur1_goal_EE_orientation[1] = 0.0;
                ur1_goal_EE_orientation[2] = 3.14;


                ur1_goal_reached = goal_reached(ur1_current_EE_position, ur1_goal_EE_position, epsilon);
                if (ur1_goal_reached){
                    ur1_bin_reached = true;

                    // publishToTopic("detach", "ur1", ur1_target_id);

                    // ur1_closest = findClosestOnionQuick(ur1_current_EE_position, onion_ids, node);
                }
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


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_onion_target_reached = true;
                }
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


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_onion_target_approach = true;

                    // publishToTopic("attach", "ur2", ur2_target_id);
                }
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


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_onion_target_retreat = true;
                }
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


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_inspection_pose_reached = true;
                }
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


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_inspection_pose_approach = true;
                }
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


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_inspection_pose_retreat = true;
                }
            }            

            // UR-2 Bin Approach Position
            else if (!ur2_bin_pose_approach){

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_bin_approach[0];
                ur2_goal_EE_position[1] = ur2_bin_approach[1];
                ur2_goal_EE_position[2] = ur2_bin_approach[2];

                ur2_goal_EE_orientation[0] = 3.14;
                ur2_goal_EE_orientation[1] = 0.0;
                ur2_goal_EE_orientation[2] = 3.14;


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_bin_pose_approach = true;
                }
            }

            // UR-2 Bin Reached
            else if (!ur2_bin_reached){

                ur2_target_id = ur2_closest.first;
                ur2_goal_EE_position[0] = ur2_bin[0];
                ur2_goal_EE_position[1] = ur2_bin[1];
                ur2_goal_EE_position[2] = ur2_bin[2];

                ur2_goal_EE_orientation[0] = 3.14;
                ur2_goal_EE_orientation[1] = 0.0;
                ur2_goal_EE_orientation[2] = 3.14;


                ur2_goal_reached = goal_reached(ur2_current_EE_position, ur2_goal_EE_position, epsilon);
                if (ur2_goal_reached){
                    ur2_bin_reached = true;

                    // publishToTopic("detach", "ur2", ur2_target_id);

                    ur2_closest = findClosestOnionQuick(ur2_current_EE_position, onion_ids);
                }
            }




            // Plan for those Goal Position using ORCA

            Agent ur1_agent;
            ur1_agent.position = ur1_current_EE_position;
            ur1_agent.radius = 0.085;    // 0.085
            ur1_agent.velocity = ur1_current_EE_vel;

            Agent ur2_agent;
            ur2_agent.position = ur2_current_EE_position;
            ur2_agent.radius = 0.085;    // 0.085
            ur2_agent.velocity = ur2_current_EE_vel;

            std::vector<Eigen::Vector3d> next_ur1_vel;
            next_ur1_vel = ORCA(ur1_agent, {ur2_agent});

            std::vector<Eigen::Vector3d> next_ur2_vel;
            next_ur2_vel = ORCA(ur2_agent, {ur1_agent});



            // ----------------------------------------------------------------------------------------------------------------------------------------- //



            Eigen::Vector3d ur1_target_unit_vec;
            Eigen::Vector3d ur2_target_unit_vec;

            if (((next_ur1_vel.size() == 1) && (ur1_start == false)) && distance(ur1_current_EE_position, ur2_current_EE_position) < 0.3)  // Collision with UR - 2
            {
                RCLCPP_INFO(logger, "UR - 1 Collision Case");

                if ((ur1_current_EE_vel + next_ur1_vel[0]).norm() > ur1_max_speed){
                    ur1_speed = ur1_max_speed;
                }

                else{
                    ur1_speed = (ur1_current_EE_vel + next_ur1_vel[0]).norm();
                }

                RCLCPP_INFO(logger, "UR - 1 Speed :  %f]", ur1_speed);
                
                ur1_target_unit_vec = (ur1_current_EE_vel + next_ur1_vel[0])/(ur1_current_EE_vel + next_ur1_vel[0]).norm();
            } 

            else  // No Collision
            {
                ur1_start = false;
                RCLCPP_INFO(logger, "UR - 1 Non - Collision Case");

                ur1_speed = ur1_pref_speed;
                ur1_target_unit_vec = (ur1_goal_EE_position - ur1_current_EE_position)/distance(ur1_current_EE_position, ur1_goal_EE_position);
            }






            if (((next_ur2_vel.size() == 1) && (ur2_start == false)) && distance(ur1_current_EE_position, ur2_current_EE_position) < 0.3)  // Collision with UR - 1
            {
                RCLCPP_INFO(logger, "UR - 2 Collision Case");

                if ((ur2_current_EE_vel + next_ur2_vel[0]).norm() > ur2_max_speed){
                    ur2_speed = ur2_max_speed;
                }

                else{
                    ur2_speed = (ur2_current_EE_vel + next_ur2_vel[0]).norm();
                }

                RCLCPP_INFO(logger, "UR - 2 Speed :  %f]", ur2_speed);
                
                ur2_target_unit_vec = (ur2_current_EE_vel + next_ur2_vel[0])/(ur2_current_EE_vel + next_ur2_vel[0]).norm();
            } 

            else  // No Collision
            {
                ur2_start = false;
                RCLCPP_INFO(logger, "UR - 2 Non - Collision Case");

                ur2_speed = ur2_pref_speed;
                ur2_target_unit_vec = (ur2_goal_EE_position - ur2_current_EE_position)/distance(ur2_current_EE_position, ur2_goal_EE_position);
            }






            // Set the Pose Target

            geometry_msgs::msg::Pose ur1_target_pose = set_vel_pose_target(ur1_current_EE_position, ur1_target_unit_vec, len, ur1_goal_EE_orientation, ur1_goal_reached);

            geometry_msgs::msg::Pose ur2_target_pose = set_vel_pose_target(ur2_current_EE_position, ur2_target_unit_vec, len, ur2_goal_EE_orientation, ur2_goal_reached);






            // Solve for the IK Solution of the Target Pose

            double timeout = 1.0;
            bool ur1_found_ik = ur_combined_state->setFromIK(ur1_joint_model_group, ur1_target_pose, timeout);
            bool ur2_found_ik = ur_combined_state->setFromIK(ur2_joint_model_group, ur2_target_pose, timeout);


            if (ur1_found_ik && ur2_found_ik)
            {
                ur_combined_state->copyJointGroupPositions(ur1_joint_model_group, ur1_joint_values);
                ur_combined_state->copyJointGroupPositions(ur2_joint_model_group, ur2_joint_values);

                // for (std::size_t i = 0; i < ur1_joint_names.size(); ++i)
                // {
                // RCLCPP_INFO(node->get_logger(), "UR - 1 Joint %s: %f", ur1_joint_names[i].c_str(), ur1_joint_values[i]);
                // RCLCPP_INFO(node->get_logger(), "UR - 2 Joint %s: %f", ur2_joint_names[i].c_str(), ur2_joint_values[i]);
                // }
            }
            else
            {
                if (!ur1_found_ik && !ur2_found_ik){
                    RCLCPP_INFO(node->get_logger(), "Did not find IK solution for UR-1 and UR-2");
                }

                else if (!ur1_found_ik){
                    RCLCPP_INFO(node->get_logger(), "Did not find IK solution for UR-1");
                }

                else {
                    RCLCPP_INFO(node->get_logger(), "Did not find IK solution for UR-2");
                }
                
            }


            ur_move_group_interface.setJointValueTarget(ur1_joint_names, ur1_joint_values);
            ur_move_group_interface.setJointValueTarget(ur2_joint_names, ur2_joint_values);


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

            ur1_current_state = ur1_move_group_interface.getCurrentState();
            ur2_current_state = ur2_move_group_interface.getCurrentState();

            ur1_current_EE_state = ur1_current_state->getGlobalLinkTransform(ur1_ee);
            ur2_current_EE_state = ur2_current_state->getGlobalLinkTransform(ur2_ee);

            Eigen::Vector3d target_position_ur1(ur1_target_pose.position.x, ur1_target_pose.position.y, ur1_target_pose.position.z);

            ur1_current_EE_position = ur1_current_EE_state.translation();
            ur2_current_EE_position = ur2_current_EE_state.translation();

            ur1_current_EE_orientation = Eigen::Quaterniond(ur1_current_EE_state.rotation());
            ur2_current_EE_orientation = Eigen::Quaterniond(ur2_current_EE_state.rotation());


            // Convert quaternion to Euler angles
            ur1_euler_angles = ur1_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
            ur2_euler_angles = ur2_current_EE_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw

            // Store Euler angles in std::vector for compatibility
            ur1_current_EE_orientation_euler = {ur1_euler_angles[0], ur1_euler_angles[1], ur1_euler_angles[2]};
            ur2_current_EE_orientation_euler = {ur2_euler_angles[0], ur2_euler_angles[1], ur2_euler_angles[2]};


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

            RCLCPP_INFO(logger, "UR - 1 New pose position: [x: %f, y: %f, z: %f]", 
            ur1_current_EE_position(0), 
            ur1_current_EE_position(1), 
            ur1_current_EE_position(2));

            RCLCPP_INFO(logger, " ");

            RCLCPP_INFO(logger, "UR - 2 New pose position: [x: %f, y: %f, z: %f]", 
            ur2_current_EE_position(0), 
            ur2_current_EE_position(1), 
            ur2_current_EE_position(2));

            RCLCPP_INFO(logger, " ");

            RCLCPP_INFO(logger, "The Distance between the 2 links is   :   %f", distance(ur1_current_EE_position, ur2_current_EE_position));

            RCLCPP_INFO(logger, " ");

            // RCLCPP_INFO(logger, "%s", std::string(50, '=').c_str());

            ur1_current_EE_vel = ur1_target_unit_vec*ur1_speed;
            ur2_current_EE_vel = ur2_target_unit_vec*ur2_speed;






            // Update the ur1_goal_reached and ur2_goal_reached to avoid exiting the loop
            if (!ur1_bin_reached){
                ur1_goal_reached = false;
            }

            else {
                ur1_closest = findClosestOnionQuick(ur1_current_EE_position, onion_ids);

                ur1_goal_reached = false;

                ur1_onion_target_reached = false;

                ur1_onion_target_approach = false;

                ur1_onion_target_retreat = false;

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
                ur2_closest = findClosestOnionQuick(ur2_current_EE_position, onion_ids);

                ur2_goal_reached = false;

                ur2_onion_target_reached = false;

                ur2_onion_target_approach = false;

                ur2_onion_target_retreat = false;

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


//                                              TODO:=

//  Line 343:  "ur1_current_EE_vel" should be not be the current velocity but the optimal velocity to goal.
//             For that to happen, goal position of all the agents should be introduced to ORCA.

//  Explore TOTG and Multi-Planner

//  Explore IK pose failure
