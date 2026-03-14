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


struct Agent {
    Eigen::Vector3d position;
    double radius;
    Eigen::Vector3d velocity;
};


double distance(const Eigen::Vector3d v1, const Eigen::Vector3d v2) {
    return (v1 - v2).norm();
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

  Eigen::Vector3d ur1_current_EE_vel(0, 0, 0);
  Eigen::Vector3d ur2_current_EE_vel(0, 0, 0);

  double epsilon = 0.01;
  double len = 0.075;
  double ur1_speed = 0;
  double ur1_pref_speed = 0.05;
  double ur1_max_speed = 0.1;

  bool ur1_start = true;


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
  auto ur2_move_group_interface = MoveGroupInterface(node, ur2_planning_group);

//   ur1_move_group_interface.setPlanningTime(10.0);  // Set a higher planning time, e.g., 10 seconds.
//   ur1_move_group_interface.setGoalTolerance(0.1);  // Set a reasonable goal tolerance.

  // Start the state monitor
  ur1_move_group_interface.startStateMonitor();


  // Get Current Robot EE Pose
  moveit::core::RobotStatePtr ur1_current_state = ur1_move_group_interface.getCurrentState();
  moveit::core::RobotStatePtr ur2_current_state = ur2_move_group_interface.getCurrentState();

  Eigen::Affine3d ur1_current_EE_state = ur1_current_state->getGlobalLinkTransform(ur1_ee);
  Eigen::Affine3d ur2_current_EE_state = ur2_current_state->getGlobalLinkTransform(ur2_ee);

  Eigen::Vector3d ur1_current_EE_position = ur1_current_EE_state.translation();
  Eigen::Vector3d ur2_current_EE_position = ur2_current_EE_state.translation();
  
  //   std::vector<double> current_positions;
  //   moveit::core::RobotStatePtr current_state = ur1_move_group_interface.getCurrentState();
  //   current_state->copyJointGroupPositions("UR1_manipulator", current_positions);

  // Define Robot EE Goal Pose

  Eigen::Vector3d ur1_goal_EE_position(0.73, -0.35, 1.0);   // [x, y, z]             Originally (0.73, -0.35, 1.0)
  Eigen::Vector3d ur1_goal_EE_orientation(0.0, 3.14, 0.0);  // [roll, pitch, yaw]


  while (distance(ur1_current_EE_position, ur1_goal_EE_position) > epsilon) {

    Agent ur1_agent;
    ur1_agent.position = ur1_current_EE_position;
    ur1_agent.radius = 0.09;
    ur1_agent.velocity = ur1_current_EE_vel;

    Agent ur2_agent;
    ur2_agent.position = ur2_current_EE_position;
    ur2_agent.radius = 0.09;
    ur2_agent.velocity = ur2_current_EE_vel;

    std::vector<Eigen::Vector3d> next_ur1_vel;
    next_ur1_vel = ORCA(ur1_agent, {ur2_agent});

    // // Log the values of next_ur1_vel
    // RCLCPP_INFO(logger, "UR - 1 Current pose position: [x: %f, y: %f, z: %f]", 
    //   next_ur1_vel[0](0),
    //   next_ur1_vel[0](1), 
    //   next_ur1_vel[0](2));
    

    Eigen::Vector3d ur1_target_unit_vec;

    if ((next_ur1_vel.size() == 1) && (ur1_start == false))  // Collision with UR - 2
    {
        RCLCPP_INFO(logger, "Collision Case");

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
        RCLCPP_INFO(logger, "Non - Collision Case");

        ur1_speed = ur1_pref_speed;
        ur1_target_unit_vec = (ur1_goal_EE_position - ur1_current_EE_position)/distance(ur1_current_EE_position, ur1_goal_EE_position);
    }

    RCLCPP_INFO(logger, "UR - 1 Current pose position: [x: %f, y: %f, z: %f]", 
      ur1_current_EE_position(0),
      ur1_current_EE_position(1), 
      ur1_current_EE_position(2));
    
    // Set UR-1 target Pose
    auto const ur1_target_pose = [&ur1_current_EE_position, &ur1_target_unit_vec, len, &ur1_goal_EE_orientation]{
    geometry_msgs::msg::Pose ur1_msg;
    ur1_msg.position.x = ur1_current_EE_position(0) + ur1_target_unit_vec(0)*len;
    ur1_msg.position.y = ur1_current_EE_position(1) + ur1_target_unit_vec(1)*len;
    ur1_msg.position.z = ur1_current_EE_position(2) + ur1_target_unit_vec(2)*len;

    // Define roll, pitch, and yaw
    double ur1_roll = ur1_goal_EE_orientation(0); 
    double ur1_pitch = ur1_goal_EE_orientation(1); 
    double ur1_yaw = ur1_goal_EE_orientation(2); 

    // Convert roll, pitch, and yaw to quaternion
    tf2::Quaternion ur1_quaternion;
    ur1_quaternion.setRPY(ur1_roll, ur1_pitch, ur1_yaw);
    ur1_msg.orientation.x = ur1_quaternion.x();
    ur1_msg.orientation.y = ur1_quaternion.y();
    ur1_msg.orientation.z = ur1_quaternion.z();
    ur1_msg.orientation.w = ur1_quaternion.w();

    return ur1_msg;
    }();

    RCLCPP_INFO(logger, "UR - 1 Target pose position: [x: %f, y: %f, z: %f]", 
      ur1_target_pose.position.x, 
      ur1_target_pose.position.y, 
      ur1_target_pose.position.z);

    ur1_move_group_interface.setPoseTarget(ur1_target_pose);
    //  ur1_move_group_interface.setJointValueTarget(target_pose, end_effector_link);

    // Set the joint value target with approximation
    //   std::string end_effector_link = "UR1_wrist_3_link";  

    // Create a plan to that target pose
    auto const [ur1_success, ur1_plan] = [&ur1_move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan ur1_msg;
        auto const ur1_ok = static_cast<bool>(ur1_move_group_interface.plan(ur1_msg));
        return std::make_pair(ur1_ok, ur1_msg);
    }();

    if(ur1_success) {
        auto ur1_plan_copy = ur1_plan; // Copy the plan to avoid const issues
        EE_speed_control(node, ur1_current_state, ur1_plan_copy, ur1_ee, ur1_speed); // Pass the copy of the plan

        // Execute the plan after modifying it
        ur1_move_group_interface.execute(ur1_plan_copy);
        RCLCPP_INFO(logger, "UR - 1 is executing the planned motion.");
    } 
    
    else {
        RCLCPP_ERROR(logger, "UR - 1 Planning failed!");
    }

    ur1_current_state = ur1_move_group_interface.getCurrentState();
    ur2_current_state = ur2_move_group_interface.getCurrentState();

    ur1_current_EE_state = ur1_current_state->getGlobalLinkTransform(ur1_ee);
    ur2_current_EE_state = ur2_current_state->getGlobalLinkTransform(ur2_ee);

    Eigen::Vector3d target_position_ur1(ur1_target_pose.position.x, ur1_target_pose.position.y, ur1_target_pose.position.z);

    ur1_current_EE_position = ur1_current_EE_state.translation();
    ur2_current_EE_position = ur2_current_EE_state.translation();

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
  
  }

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