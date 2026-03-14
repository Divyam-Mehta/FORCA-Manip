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

// Alias for the action type.
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;


Eigen::Vector3d ur1_current_EE_vel(0, 0, 0);
Eigen::Vector3d ur2_current_EE_vel(0, 0, 0);

Eigen::Vector3d ur1_prev_EE_position; 
Eigen::Vector3d ur2_prev_EE_position; 

double ur1_speed, ur2_speed;

double ur1_total_exec_time = 1.0;
double ur2_total_exec_time = 1.0;


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




// std::vector<Eigen::Vector3d> ORCA(const std::shared_ptr<rclcpp::Node> &node,
//                                   const Agent reference_agent,
//                                   const std::vector<Agent> other_agents,
//                                   const Eigen::Vector3d &goal_EE_position,
//                                   const Eigen::Vector3d &current_EE_position,
//                                   double pref_speed,
//                                   const std::string & robot_id,
//                                   double threshold_distance) 
// {
//     // Distance between reference agent and first other agent
//     double agent_distance = (reference_agent.position - other_agents[0].position).norm();

    
    
//     // If distance > threshold, just go toward goal
//     if (agent_distance > threshold_distance)
//     {
//         Eigen::Vector3d goal_velocity = (goal_EE_position - current_EE_position);
//         if (goal_velocity.norm() > 1e-3) {
//             goal_velocity = goal_velocity.normalized() * pref_speed;
//         } else {
//             goal_velocity = Eigen::Vector3d::Zero();
//         }

//         std::cout << std::string(100, '-') << std::endl;
//         std::cout << "Non - Collision Case " << std::endl;
//         std::cout << std::string(100, '-') << std::endl;

//         return {goal_velocity};  // Return as vector of one element
//     }



//     // Else do ORCA computation
//     double responsibility = 0.5;
//     std::vector<Eigen::Vector3d> relative_velocity;
//     std::vector<std::vector<Eigen::Vector3d>> circle;

//     for (const auto& agent : other_agents) {
//         Eigen::Vector3d relative_position = agent.position - reference_agent.position;
//         double combined_radius = agent.radius + reference_agent.radius;

//         // Parametric equation of the sphere

//         // Parametric equation of the circle
//         Eigen::Vector3d cc = relative_position - relative_position * (std::pow(combined_radius, 2) / relative_position.squaredNorm());
//         double c_radius = combined_radius * std::sqrt((relative_position.squaredNorm() - std::pow(combined_radius, 2)) / relative_position.squaredNorm());

//         // 2 orthogonal directional unit vectors
//         Eigen::Vector3d v1(relative_position(1), -relative_position(0), 0);
//         v1.normalize();
//         Eigen::Vector3d v2(-relative_position(0) * relative_position(2), -relative_position(1) * relative_position(2), relative_position(0) * relative_position(0) + relative_position(1) * relative_position(1));
//         v2.normalize();

//         std::vector<Eigen::Vector3d> circle_points;

//         circle_points.push_back(cc);
        
//         for (double t = 0; t <= 2 * M_PI; t += 0.01) {
//             Eigen::Vector3d point = cc + c_radius * std::cos(t) * v1 + c_radius * std::sin(t) * v2;
//             circle_points.push_back(point);
//         }

//         circle.push_back(circle_points);


//         // Relative velocity
//         Eigen::Vector3d rel_vel = reference_agent.velocity - agent.velocity;
//         relative_velocity.push_back(rel_vel);
//     }

//     std::vector<Eigen::Vector3d> escape_list;

//     for (size_t i = 0; i < other_agents.size(); ++i) {
//         Eigen::Vector3d center = circle[i][0];  // Center of the circle 
//         std::vector<Eigen::Vector3d> periphery(circle[i].begin() + 1, circle[i].end());  // Exclude the center point

//         Eigen::Vector3d first_point = periphery[0];  //  First Circle point on the periphery

//         bool collision_condition_1 = center.norm() > relative_velocity[i].norm();  // TODO
//         bool collision_condition_2 = (relative_velocity[i].dot(center) / (center.norm() * relative_velocity[i].norm())) > (first_point.dot(center) / (center.norm() * first_point.norm()));

//         double lhs = relative_velocity[i].dot(center) / (center.norm() * relative_velocity[i].norm());
//         double rhs = first_point.dot(center) / (center.norm() * first_point.norm());

//         // std::cout << robot_id << "   |   LHS = " << lhs << "   RHS = " << rhs << std::endl;
//         RCLCPP_INFO(node->get_logger(), "[%s]    |    Relative Velocity = [%f, %f, %f]    Circle Center = [%f, %f, %f]     First Point = [%f, %f, %f]", robot_id.c_str(), relative_velocity[i].x(), relative_velocity[i].y(), relative_velocity[i].z(), center.x(), center.y(), center.z(), first_point.x(), first_point.y(), first_point.z());
//         RCLCPP_INFO(node->get_logger(), "[%s]    |    LHS = [%f]    RHS = [%f]", robot_id.c_str(), lhs, rhs);

//         if (collision_condition_2)   //  TODO - Need to add condition_1 
//         {
//             Eigen::Vector3d goal_velocity = (goal_EE_position - current_EE_position);
//             if (goal_velocity.norm() > 1e-3) {
//                 goal_velocity = goal_velocity.normalized() * pref_speed;
//             } else {
//                 goal_velocity = Eigen::Vector3d::Zero();
//             }

//             double denominator = first_point.dot(first_point);
//             double min_dist = std::numeric_limits<double>::infinity();
//             Eigen::Vector3d line;
//             int t_num;

//             for (size_t j = 0; j < periphery.size(); ++j) {
//                 double lambda = relative_velocity[i].dot(periphery[j]) / denominator;
//                 Eigen::Vector3d escape_vector = lambda * periphery[j] - relative_velocity[i];
//                 Eigen::Vector3d candidate_velocity = goal_velocity + escape_vector * responsibility;

//                 double dot_result = candidate_velocity.dot(Eigen::Vector3d(1.0, 0.0, 0.0));

//                 // Apply robot-specific directional filtering
//                 if ((robot_id == "ur1" && dot_result > 0.0) || (robot_id == "ur2" && dot_result < 0.0)) {
//                     continue;  // Skip this candidate
//                 }

//                 double dist = escape_vector.norm();
//                 if (dist < min_dist) {
//                     min_dist = dist;
//                     line = periphery[j];
//                     t_num = j;
//                 }
//             }

//             double lambda = relative_velocity[i].dot(line) / denominator;
//             Eigen::Vector3d escape_vector = lambda * line - relative_velocity[i];
//             escape_vector.z() = 0;

//             escape_list.push_back(0.25*goal_velocity + escape_vector*responsibility);

//             // std::cout << "Tangent Line number: - " << t_num << std::endl;
//         }
//     }


//     // Check if escape list is empty
//     if (escape_list.empty()) {
//         Eigen::Vector3d goal_velocity = (goal_EE_position - current_EE_position);
//         if (goal_velocity.norm() > 1e-3) {
//             goal_velocity = goal_velocity.normalized() * pref_speed;
//         } else {
//             goal_velocity = Eigen::Vector3d::Zero();
//         }

//         std::cout << std::string(100, '-') << std::endl;
//         std::cout << "Non - Collision Case (Robots are close to each other)" << std::endl;
//         std::cout << std::string(100, '-') << std::endl;
        
//         return {goal_velocity};
//     }

//     else{

//         std::cout << std::string(100, '-') << std::endl;
//         std::cout << "Collision Case " << std::endl;
//         std::cout << std::string(100, '-') << std::endl;

//         return escape_list;
//     }
// }




std::vector<Eigen::Vector3d> ORCA(const std::shared_ptr<rclcpp::Node> &node,
                                  const Agent reference_agent,
                                  const std::vector<Agent> other_agents,
                                  const Eigen::Vector3d &goal_EE_position,
                                  const Eigen::Vector3d &current_EE_position,
                                  double pref_speed,
                                  const std::string & robot_id,
                                  double threshold_distance,
                                  moveit::planning_interface::MoveGroupInterface &move_group,
                                  const moveit::core::JointModelGroup *joint_model_group,
                                  const Eigen::Vector3d &goal_EE_orientation) 
{
    double agent_distance = (reference_agent.position - other_agents[0].position).norm();

    if (agent_distance > threshold_distance)
    {
        Eigen::Vector3d goal_velocity = (goal_EE_position - current_EE_position);
        if (goal_velocity.norm() > 1e-3) {
            goal_velocity = goal_velocity.normalized() * pref_speed;
        } else {
            goal_velocity = Eigen::Vector3d::Zero();
        }

        std::cout << std::string(100, '-') << std::endl;
        std::cout << "Non - Collision Case " << std::endl;
        std::cout << std::string(100, '-') << std::endl;

        return {goal_velocity};
    }

    double responsibility = 0.5;
    std::vector<Eigen::Vector3d> relative_velocity;
    std::vector<std::vector<Eigen::Vector3d>> circle;

    for (const auto& agent : other_agents) {
        Eigen::Vector3d relative_position = agent.position - reference_agent.position;
        double combined_radius = agent.radius + reference_agent.radius;

        Eigen::Vector3d cc = relative_position - relative_position * (std::pow(combined_radius, 2) / relative_position.squaredNorm());
        double c_radius = combined_radius * std::sqrt((relative_position.squaredNorm() - std::pow(combined_radius, 2)) / relative_position.squaredNorm());

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

        Eigen::Vector3d rel_vel = reference_agent.velocity - agent.velocity;
        relative_velocity.push_back(rel_vel);
    }

    std::vector<Eigen::Vector3d> escape_list;

    for (size_t i = 0; i < other_agents.size(); ++i) {
        Eigen::Vector3d center = circle[i][0];
        std::vector<Eigen::Vector3d> periphery(circle[i].begin() + 1, circle[i].end());
        Eigen::Vector3d first_point = periphery[0];

        bool collision_condition_2 = (relative_velocity[i].dot(center) / (center.norm() * relative_velocity[i].norm())) > (first_point.dot(center) / (center.norm() * first_point.norm()));

        if (collision_condition_2) {
            Eigen::Vector3d goal_velocity = (goal_EE_position - current_EE_position);
            if (goal_velocity.norm() > 1e-3) {
                goal_velocity = goal_velocity.normalized() * pref_speed;
            } else {
                goal_velocity = Eigen::Vector3d::Zero();
            }

            double denominator = first_point.dot(first_point);
            double min_dist = std::numeric_limits<double>::infinity();
            Eigen::Vector3d line;
            int t_num = 0;

            for (size_t j = 0; j < periphery.size(); ++j) {
                double lambda = relative_velocity[i].dot(periphery[j]) / denominator;
                Eigen::Vector3d escape_vector = lambda * periphery[j] - relative_velocity[i];
                Eigen::Vector3d candidate_velocity = goal_velocity + escape_vector * responsibility;

                double dot_result = candidate_velocity.dot(Eigen::Vector3d(1.0, 0.0, 0.0));
                if ((robot_id == "ur1" && dot_result > 0.0) || (robot_id == "ur2" && dot_result < 0.0)) {
                    continue;
                }

                double dist = escape_vector.norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    line = periphery[j];
                    t_num = j;
                }
            }

            // Check neighbors if no valid IK found for best escape
            int max_checks = 10;
            Eigen::Vector3d valid_velocity = Eigen::Vector3d::Zero();
            bool found_valid = false;

            for (int offset = 0; offset <= max_checks; ++offset) {
                for (int direction : {-1, 1}) {
                    // int idx = static_cast<int>(t_num) + direction * offset;
                    // if (idx < 0 || idx >= static_cast<int>(periphery.size())) continue;

                    Eigen::Vector3d line_candidate = periphery[idx];
                    double lambda = relative_velocity[i].dot(line_candidate) / denominator;
                    Eigen::Vector3d escape_vector = lambda * line_candidate - relative_velocity[i];
                    escape_vector.z() = 0;
                    Eigen::Vector3d candidate_velocity = goal_velocity + escape_vector * responsibility;
                    Eigen::Vector3d target_position = current_EE_position + candidate_velocity;

                    geometry_msgs::msg::Pose ik_pose = set_pose_target(target_position, goal_EE_orientation);

                    moveit::core::RobotStatePtr temp_state(new moveit::core::RobotState(move_group.getRobotModel()));
                    temp_state->setToDefaultValues();

                    if (temp_state->setFromIK(joint_model_group, ik_pose, 1.0)) {
                        valid_velocity = candidate_velocity;
                        found_valid = true;
                        break;
                    }
                }
                if (found_valid) break;
            }

            if (found_valid) {
                escape_list.push_back(valid_velocity);
            }
        }
    }

    if (escape_list.empty()) {
        Eigen::Vector3d goal_velocity = (goal_EE_position - current_EE_position);
        if (goal_velocity.norm() > 1e-3) {
            goal_velocity = goal_velocity.normalized() * pref_speed;
        } else {
            goal_velocity = Eigen::Vector3d::Zero();
        }

        std::cout << std::string(100, '-') << std::endl;
        std::cout << "Non - Collision Case (Robots are close)" << std::endl;
        std::cout << std::string(100, '-') << std::endl;

        return {goal_velocity};
    } else {
        std::cout << std::string(100, '-') << std::endl;
        std::cout << "Collision Case" << std::endl;
        std::cout << std::string(100, '-') << std::endl;

        return escape_list;
    }
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

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.a = 0.6;
  marker.color.r = (robot_id == "ur1") ? 1.0 : 0.0;
  marker.color.g = 0.0;
  marker.color.b = (robot_id == "ur2") ? 1.0 : 0.0;

  marker_pub->publish(marker);
}



void publish_velocity_vector_marker(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & marker_pub,
                                    const Eigen::Vector3d & center,
                                    const Eigen::Vector3d & velocity,
                                    const std::string & robot_id,
                                    int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";  // Replace with your planning frame if different
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = robot_id + "_vel_vector";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Calculate start and end point such that center is midpoint
  geometry_msgs::msg::Point start, end;
  Eigen::Vector3d half_vector = velocity * 0.5;
  Eigen::Vector3d start_point = center - half_vector;
  Eigen::Vector3d end_point = center + half_vector;

  start.x = start_point.x(); start.y = start_point.y(); start.z = start_point.z();
  end.x = end_point.x();     end.y = end_point.y();     end.z = end_point.z();

  marker.points.push_back(start);
  marker.points.push_back(end);

  marker.scale.x = 0.02;  // shaft diameter
  marker.scale.y = 0.04;  // head diameter
  marker.scale.z = 0.1;   // head length

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.lifetime = rclcpp::Duration::from_seconds(0.5);  // short lifetime to update dynamically
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
  double max_len = 0.05;
  double alpha = 0.5;
  double robot_speed = 0.2;     // Initialize
  double pref_speed = 0.2; 
  double max_speed = 1.0;
  double stagnant_count = 0;
  
  double goal_velocity_factor = 1.0;
  
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



  while (rclcpp::ok())
  { 

    if(robot_goal_reached)
    {
        break;
    }

    bool onion_target_reached = false;
    
    
    while(!robot_goal_reached)   // One whole Pass of PSort
    {

        // RCLCPP_INFO(node->get_logger(), " ");
        // RCLCPP_INFO(node->get_logger(), "[%s]   CHECKPOINT", robot_id.c_str());
        // RCLCPP_INFO(node->get_logger(), " ");

        // Get Current Robot End-Effector State

        current_state = move_group.getCurrentState();
        current_EE_state = current_state->getGlobalLinkTransform(robot_ee);
        current_EE_position = current_EE_state.translation();

        // publish_marker(marker_pub, current_EE_position, robot_id, 0);

        // Compute X-axis direction (roll axis) from the EE transform
        Eigen::Vector3d roll_axis_direction = current_EE_state.rotation().col(2);  // X-axis of EE frame

        // Shift the marker 0.11 meter along that axis
        Eigen::Vector3d marker_position = current_EE_position + roll_axis_direction.normalized() * 0.0;   // 0.075

        // Call the marker function with the new point
        publish_marker(marker_pub, marker_position, robot_id, 0);



        // --------------------------------  Target Pose  -------------------------------- //

        
        // Set Target Pose 
        if(!onion_target_reached)        
        {
            if(robot_id == "ur1")
            {
                goal_EE_position[0] = 0.73;
                goal_EE_position[1] = 0.15;         // 0.15
                goal_EE_position[2] = 1.0;

                goal_EE_orientation[0] = 3.14;
                goal_EE_orientation[1] = 0.0;  // 1.57
                goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "[%s] Goal Target Approach Case", robot_id.c_str());
                RCLCPP_INFO(node->get_logger(), " ");

                robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
                if (robot_goal_reached)
                {
                    onion_target_reached = true;
                }

                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }



            else
            {
                goal_EE_position[0] = 0.75;
                goal_EE_position[1] = -0.25;        // -0.25
                goal_EE_position[2] = 1.0;

                goal_EE_orientation[0] = 3.14;
                goal_EE_orientation[1] = 0.0;  // -1.57
                goal_EE_orientation[2] = 3.14;

                RCLCPP_INFO(node->get_logger(), " ");
                RCLCPP_INFO(node->get_logger(), "[%s] Goal Target Approach Case", robot_id.c_str());
                RCLCPP_INFO(node->get_logger(), " ");

                robot_goal_reached = goal_reached(current_EE_position, goal_EE_position, epsilon);
                if (robot_goal_reached)
                {
                    onion_target_reached = true;
                }

                len = min(max_len, alpha*distance(current_EE_position, goal_EE_position));
            }
        }

            
        



        // ----------------------------------------------------------------------------------------------------------------------------------------- //



        // Plan for those Goal Position using ORCA

        other_robot_current_state = other_robot_move_group.getCurrentState();
        other_robot_current_EE_state = other_robot_current_state->getGlobalLinkTransform(other_robot_ee);
        other_robot_current_EE_position = other_robot_current_EE_state.translation();

        
        Eigen::Vector3d other_robot_roll_axis_direction = other_robot_current_EE_state.rotation().col(2);
        Eigen::Vector3d other_robot_marker_position = other_robot_current_EE_position + other_robot_roll_axis_direction.normalized() * 0.0;  // 0.075
 
        Eigen::Vector3d next_EE_vel;

        if (robot_id == "ur1") 
        {
            Agent main_agent;
            main_agent.position = marker_position;
            main_agent.radius = 0.1;       // 0.085    
            main_agent.velocity = ur1_current_EE_vel;   

            Agent other_agent;
            other_agent.position = other_robot_marker_position;
            other_agent.radius = 0.1;    
            other_agent.velocity = ur2_current_EE_vel;

            next_EE_vel = ORCA(node, main_agent, {other_agent}, goal_EE_position, current_EE_position, pref_speed, robot_id, 1.0)[0];

            // RCLCPP_INFO(node->get_logger(), "UR - 1  Speed: [%f]     |     UR - 2  Speed: [%f]", ((current_EE_position - ur1_prev_EE_position)/ur1_total_exec_time).norm(), ((other_robot_current_EE_position - ur2_prev_EE_position)/ur2_total_exec_time).norm());

            RCLCPP_INFO(node->get_logger(), "[%s] -> Next Velocity is ([%f], [%f], [%f])", robot_id.c_str(), next_EE_vel.x(), next_EE_vel.y(), next_EE_vel.z());

            ur1_current_EE_vel = next_EE_vel;
            ur1_prev_EE_position = current_EE_position;
            ur1_speed = next_EE_vel.norm();
        } 
        
        else 
        {
            Agent main_agent;
            main_agent.position = marker_position;
            main_agent.radius = 0.1;       // 0.085    
            main_agent.velocity = ur1_current_EE_vel;  

            Agent other_agent;
            other_agent.position = other_robot_marker_position;
            other_agent.radius = 0.1;    
            other_agent.velocity = ur2_current_EE_vel; 

            next_EE_vel = ORCA(node, main_agent, {other_agent}, goal_EE_position, current_EE_position, pref_speed, robot_id, 1.0)[0];

            // RCLCPP_INFO(node->get_logger(), "UR - 1  Speed: [%f]     |     UR - 2  Speed: [%f]", ((other_robot_current_EE_position - ur1_prev_EE_position)/ur1_total_exec_time).norm(), ((other_robot_current_EE_position - ur2_prev_EE_position)/ur2_total_exec_time).norm());

            RCLCPP_INFO(node->get_logger(), "[%s] -> Next Velocity is ([%f], [%f], [%f])", robot_id.c_str(), next_EE_vel.x(), next_EE_vel.y(), next_EE_vel.z());

            ur2_current_EE_vel = next_EE_vel;
            ur2_prev_EE_position = current_EE_position;
            ur2_speed = next_EE_vel.norm();

            current_EE_vel = ur2_current_EE_vel;
        }


        publish_velocity_vector_marker(marker_pub, marker_position, next_EE_vel, robot_id, 1);


        Eigen::Vector3d target_unit_vec;

        target_unit_vec = next_EE_vel/next_EE_vel.norm();

        // target_unit_vec = next_EE_vel;


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
            // EE_speed_control(node, current_state, plan_copy, robot_ee, 0.1);

            trajectory_msgs::msg::JointTrajectory trajectory = plan_copy.trajectory.joint_trajectory;

            // Right after EE_speed_control(...) and before executeTrajectoryAndWait(...)

            if (robot_id == "ur1"){
                ur1_total_exec_time = rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
                RCLCPP_INFO(node->get_logger(), "UR-1 Final Execution Time After Speed Control: %.3f seconds", ur1_total_exec_time);
            }

            else{
                ur2_total_exec_time = rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
                RCLCPP_INFO(node->get_logger(), "UR-2 Final Execution Time After Speed Control: %.3f seconds", ur2_total_exec_time);
            }


            // Execute the trajectory and wait for its completion.
            executeTrajectoryAndWait(node, trajectory, action_name, robot_id);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "[%s] Planning failed!", robot_id.c_str());
            // Optionally, wait a short time before retrying.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }
  }
}






int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create one node to be used by both MoveGroupInterface instances.
  auto node = rclcpp::Node::make_shared("mt_orca_test");
  
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

  std::this_thread::sleep_for(std::chrono::seconds(3));
  
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