#!/usr/bin/env python3
"""
mt_orca_test.py

This script implements dual–robot motion planning and execution with dynamic waypoint speed control and ORCA–based collision avoidance.
It is a Python “translation” of mt_orca_test.cpp using MoveIt–py and ROS2 (with reference to waypoint_speed_control.py).
"""

import os
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import tf_transformations
import math
import threading
import time
from ament_index_python import get_package_share_directory
from pathlib import Path

# --- Import MoveIt–py related modules (update these imports according to your MoveIt2 Python API) ---
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit.core.robot_trajectory import RobotTrajectory

# Global robot state variables (shared between threads)
ur1_current_EE_vel = np.zeros(3)
ur2_current_EE_vel = np.zeros(3)
ur1_prev_EE_position = None  # will be initialized once the robot state is read
ur2_prev_EE_position = None
ur1_speed = 0.5  # preferred speed
ur2_speed = 0.5  # preferred speed

# --- Utility Functions ---

def distance(v1: np.ndarray, v2: np.ndarray) -> float:
    return np.linalg.norm(v1 - v2)

def goal_reached(current_state: np.ndarray, goal_state: np.ndarray, tolerance: float) -> bool:
    return distance(current_state, goal_state) < tolerance

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Computes quaternion from roll, pitch, yaw."""
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def quaternion_from_matrix(matrix: np.ndarray) -> list:
    # Assume matrix is 4x4 with rotation in the upper left 3x3.
    quat = tf_transformations.quaternion_from_matrix(matrix)
    return quat.tolist()

def set_pose_target(ee_position: np.ndarray, ee_orientation: np.ndarray) -> Pose:
    """Constructs a Pose from position and Euler angles."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = ee_position.tolist()
    quat = quaternion_from_euler(ee_orientation[0], ee_orientation[1], ee_orientation[2])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    return pose

def set_vel_pose_target(current_EE_position: np.ndarray,
                        target_unit_vec: np.ndarray,
                        length: float,
                        goal_EE_orientation: np.ndarray,
                        goal_reached_flag: bool) -> Pose:
    """Generates a target pose based on current state and unit direction vector."""
    pose = Pose()
    if not goal_reached_flag:
        new_pos = current_EE_position + target_unit_vec * length
    else:
        new_pos = current_EE_position
    pose.position.x, pose.position.y, pose.position.z = new_pos.tolist()
    quat = quaternion_from_euler(goal_EE_orientation[0], goal_EE_orientation[1], goal_EE_orientation[2])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    return pose

# def EE_speed_control(kinematic_state: RobotState, plan: RobotTrajectory, end_effector: str, speed: float, logger) -> RobotTrajectory:
#     """
#     Adjusts the trajectory's waypoint timestamps, velocities, and accelerations so that the end-effector
#     moves at the desired average speed.
    
#     This implementation mimics the C++ version; note that a full forward kinematics
#     computation is not provided (a dummy approximation is used instead).
#     """
#     traj = plan.trajectory  # JointTrajectory message
#     points = traj.points
#     num_points = len(points)
#     joint_names = traj.joint_names

#     # Get current end-effector state (assumes kinematic_state returns a 4x4 numpy array)
#     current_state_mat = kinematic_state.get_global_link_transform(end_effector)
#     current_EE_position = current_state_mat[:3, 3]

#     # Update timestamps for each pair of waypoints using a dummy distance measure.
#     for i in range(num_points - 1):
#         curr_point = points[i]
#         next_point = points[i + 1]
#         # Dummy euclidean distance based on joint space differences (placeholder for FK)
#         diff = np.array(next_point.positions) - np.array(curr_point.positions)
#         euclidean_distance = np.linalg.norm(diff)
#         new_timestamp = curr_point.time_from_start + (euclidean_distance / speed)
#         next_point.time_from_start = new_timestamp
#     # Update velocities and accelerations for each waypoint
#     for i in range(num_points):
#         if i == 0:
#             dt1 = dt2 = points[1].time_from_start - points[0].time_from_start if num_points > 1 else 0.01
#             q1 = np.array(points[1].positions)
#             q2 = np.array(points[0].positions)
#             q3 = q1
#         elif i < num_points - 1:
#             dt1 = points[i].time_from_start - points[i - 1].time_from_start
#             dt2 = points[i + 1].time_from_start - points[i].time_from_start
#             q1 = np.array(points[i - 1].positions)
#             q2 = np.array(points[i].positions)
#             q3 = np.array(points[i + 1].positions)
#         else:
#             dt1 = dt2 = points[i].time_from_start - points[i - 1].time_from_start
#             q1 = np.array(points[i - 1].positions)
#             q2 = np.array(points[i].positions)
#             q3 = q1

#         new_velocities = []
#         new_accelerations = []
#         for j in range(len(joint_names)):
#             if dt1 == 0.0 or dt2 == 0.0:
#                 v1 = v2 = a = 0.0
#             else:
#                 v1 = (q2[j] - q1[j]) / dt1
#                 v2 = (q3[j] - q2[j]) / dt2
#                 a = 2.0 * (v2 - v1) / (dt1 + dt2)
#             new_velocities.append((v1 + v2) / 2.0)
#             new_accelerations.append(a)
#         points[i].velocities = new_velocities
#         points[i].accelerations = new_accelerations

#     logger.info("Trajectory velocity and acceleration adjusted")
#     return plan

def EE_vel(robot_name, kinematic_state, plan_result, speed):

    if robot_name == "ur1":
        ee_link = "UR1_wrist_3_link"
        planner_group = "UR1_manipulator"

    elif robot_name == "ur2":
        ee_link = "UR2_wrist_3_link"
        planner_group = "UR2_manipulator"

    if plan_result:
        logger = get_logger("EE_vel")
        logger.info("Iterating through the waypoints")
        robot_trajectory = plan_result.trajectory

        # Use the get_robot_trajectory_msg method to get the trajectory message
        try:
            joint_trajectory_msg = robot_trajectory.get_robot_trajectory_msg().joint_trajectory
        except AttributeError:
            logger.error("The plan result does not have a valid joint_trajectory")
            return
        
        num_waypoints = len(joint_trajectory_msg.points)
        joint_names = joint_trajectory_msg.joint_names

        # Set the joint positions of the first waypoint
        joint_positions_init = np.array(joint_trajectory_msg.points[0].positions, dtype=np.float64)
        kinematic_state.set_joint_group_positions(planner_group, joint_positions_init)
        current_end_effector_state = kinematic_state.get_global_link_transform(ee_link).matrix()[:3, 3]

        for i in range(num_waypoints - 1):
            curr_waypoint = joint_trajectory_msg.points[i]
            next_waypoint = joint_trajectory_msg.points[i + 1]

            # Set joints for the next waypoint
            joint_positions = np.array(next_waypoint.positions, dtype=np.float64)
            kinematic_state.set_joint_group_positions(planner_group, joint_positions)

            # Do forward kinematics to get Cartesian positions of end effector for the next waypoint
            next_end_effector_state = kinematic_state.get_global_link_transform(ee_link).matrix()[:3, 3]

            # Get Euclidean distance between the two waypoints
            euclidean_distance = np.linalg.norm(np.array(next_end_effector_state) - np.array(current_end_effector_state))

            # Update timestamp
            new_timestamp = curr_waypoint.time_from_start.sec + curr_waypoint.time_from_start.nanosec * 1e-9 + (euclidean_distance / speed)
            old_timestamp = next_waypoint.time_from_start.sec + next_waypoint.time_from_start.nanosec * 1e-9

            # Update next waypoint timestamp if constraints allow
            if new_timestamp > old_timestamp:
                next_waypoint.time_from_start = Duration(seconds=int(new_timestamp), nanoseconds=int((new_timestamp % 1) * 1e9))
            else:
                logger.warn(f"{i} -> {i+1} :    New Timestamp: {new_timestamp}   |    Old Timestamp: {old_timestamp}")

            # Update the current_end_effector_state for the next iteration
            current_end_effector_state = next_end_effector_state

        # Update joint velocities and accelerations
        for i in range(num_waypoints):
            curr_waypoint = joint_trajectory_msg.points[i]

            if i > 0:
                prev_waypoint = joint_trajectory_msg.points[i - 1]
            if i < num_waypoints - 1:
                next_waypoint = joint_trajectory_msg.points[i + 1]

            # Handle timestamp differences
            if i == 0:
                dt1 = dt2 = (next_waypoint.time_from_start.sec + next_waypoint.time_from_start.nanosec * 1e-9) - (curr_waypoint.time_from_start.sec + curr_waypoint.time_from_start.nanosec * 1e-9)
            elif i < num_waypoints - 1:
                dt1 = (curr_waypoint.time_from_start.sec + curr_waypoint.time_from_start.nanosec * 1e-9) - (prev_waypoint.time_from_start.sec + prev_waypoint.time_from_start.nanosec * 1e-9)
                dt2 = (next_waypoint.time_from_start.sec + next_waypoint.time_from_start.nanosec * 1e-9) - (curr_waypoint.time_from_start.sec + curr_waypoint.time_from_start.nanosec * 1e-9)
            else:
                dt1 = dt2 = (curr_waypoint.time_from_start.sec + curr_waypoint.time_from_start.nanosec * 1e-9) - (prev_waypoint.time_from_start.sec + prev_waypoint.time_from_start.nanosec * 1e-9)

            # Iterate over all joints in waypoint
            for j in range(len(joint_names)):
                if i == 0:
                    q1 = next_waypoint.positions[j]
                    q2 = curr_waypoint.positions[j]
                    q3 = q1
                elif i < num_waypoints - 1:
                    q1 = prev_waypoint.positions[j]
                    q2 = curr_waypoint.positions[j]
                    q3 = next_waypoint.positions[j]
                else:
                    q1 = prev_waypoint.positions[j]
                    q2 = curr_waypoint.positions[j]
                    q3 = q1

                if dt1 == 0.0 or dt2 == 0.0:
                    v1 = v2 = a = 0.0
                else:
                    v1 = (q2 - q1) / dt1
                    v2 = (q3 - q2) / dt2
                    a = 2.0 * (v2 - v1) / (dt1 + dt2)

                # Set the velocity and acceleration
                curr_waypoint.velocities[j] = (v1 + v2) / 2.0
                curr_waypoint.accelerations[j] = a

        return plan_result

def ORCA(reference_agent: dict, other_agents: list) -> list:
    """
    Computes the escape velocities for collision avoidance using a simplified ORCA method.
    
    Each agent is defined as a dictionary with keys:
      - "position": numpy.ndarray (3,)
      - "radius": float
      - "velocity": numpy.ndarray (3,)
      
    Returns:
      A list of escape (adjustment) numpy.ndarray vectors.
    """
    responsibility = 0.5
    relative_velocity = []
    circle = []
    # For each other agent compute a collision–avoidance “circle”
    for agent in other_agents:
        rel_pos = agent['position'] - reference_agent['position']
        combined_radius = agent['radius'] + reference_agent['radius']
        norm_sq = np.dot(rel_pos, rel_pos)
        if norm_sq == 0:
            cc = np.zeros(3)
        else:
            cc = rel_pos - rel_pos * (combined_radius**2 / norm_sq)
        if norm_sq == 0 or (norm_sq - combined_radius**2) < 0:
            c_radius = 0.0
        else:
            c_radius = combined_radius * np.sqrt((norm_sq - combined_radius**2) / norm_sq)
        # Two orthogonal vectors (if possible)
        v1 = np.array([rel_pos[1], -rel_pos[0], 0.0])
        if np.linalg.norm(v1) != 0:
            v1 = v1 / np.linalg.norm(v1)
        else:
            v1 = np.zeros(3)
        v2 = np.array([-rel_pos[0]*rel_pos[2],
                       -rel_pos[1]*rel_pos[2],
                       rel_pos[0]**2 + rel_pos[1]**2])
        if np.linalg.norm(v2) != 0:
            v2 = v2 / np.linalg.norm(v2)
        else:
            v2 = np.zeros(3)
        circle_points = [cc]
        t = 0.0
        while t <= 2 * math.pi:
            pt = cc + c_radius * math.cos(t) * v1 + c_radius * math.sin(t) * v2
            circle_points.append(pt)
            t += 0.01
        circle.append(circle_points)
        rel_vel = reference_agent['velocity'] - agent['velocity']
        relative_velocity.append(rel_vel)

    escape_list = []
    for i in range(len(other_agents)):
        center = circle[i][0]
        periphery = circle[i][1:]
        if len(periphery) == 0:
            continue
        first_point = periphery[0]
        # Conditions (the original C++ had a TODO regarding condition_1; here we only consider condition_2)
        if np.linalg.norm(center) == 0 or np.linalg.norm(relative_velocity[i]) == 0:
            cond_ratio = 0
        else:
            cond_ratio = np.dot(relative_velocity[i], center) / (np.linalg.norm(center) * np.linalg.norm(relative_velocity[i]))
        if np.linalg.norm(center) == 0 or np.linalg.norm(first_point) == 0:
            first_ratio = 0
        else:
            first_ratio = np.dot(first_point, center) / (np.linalg.norm(center) * np.linalg.norm(first_point))
        if cond_ratio > first_ratio:
            print("Reference Agent is colliding with another agent.")
            denominator = np.dot(first_point, first_point)
            if denominator == 0:
                denominator = 1.0
            min_dist = float('inf')
            selected_pt = None
            t_num = 0
            for j, pt in enumerate(periphery):
                lam = np.dot(relative_velocity[i], pt) / denominator
                escape_vec = lam * pt - relative_velocity[i]
                dist_val = np.linalg.norm(escape_vec)
                if dist_val < min_dist:
                    min_dist = dist_val
                    selected_pt = pt
                    t_num = j
            lam = np.dot(relative_velocity[i], selected_pt) / denominator
            escape_vec = lam * selected_pt - relative_velocity[i]
            escape_list.append(escape_vec * responsibility)
            print("Tangent Line number: -", t_num)
    print("-" * 100)
    return escape_list

def execute_trajectory_and_wait(executor, plan_result, controller_name: str, robot_id: str, logger):
    # Extract the actual trajectory from the plan_result.
    robot_trajectory = plan_result.trajectory  # Ensure plan_result.trajectory is of type RobotTrajectory.
    logger.info(f"[{robot_id}] Sending plan for execution...")
    result = executor.execute(robot_trajectory, controllers=[controller_name])
    if result:
        logger.info(f"[{robot_id}] Execution succeeded")
    else:
        logger.error(f"[{robot_id}] Execution failed")
    time.sleep(0.1)



def publish_marker(marker_pub, position: np.ndarray, robot_id: str, marker_id: int, node: Node):
    """Publishes an RViz Marker for visualization."""
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = f"{robot_id}_marker"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    if robot_id == "ur1":
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    else:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    marker_pub.publish(marker)

def compute_current_pose(planning_component, link: str) -> PoseStamped:
    # Get the current robot state using get_start_state()
    robot_state = planning_component.get_start_state()  # returns a RobotState
    # Compute the transform for the given link
    transform = robot_state.get_global_link_transform(link)
    
    # Create the PoseStamped message
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "world"
    pose_stamped.pose.position.x = transform[0, 3]
    pose_stamped.pose.position.y = transform[1, 3]
    pose_stamped.pose.position.z = transform[2, 3]
    
    # Convert the 3x3 rotation part of the transform into a quaternion.
    from tf_transformations import quaternion_from_matrix
    quat = quaternion_from_matrix(transform)
    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]
    
    return pose_stamped


# --- Main Robot Loop Function ---

def run_robot(node: Node, planning_component: MoveItPy, other_planning_component: MoveItPy,
              robot_id: str, controller_name: str, robot_ee: str, other_robot_ee: str,
              gz_poses: list, marker_pub):
    """
    Continuous planning/execution loop for one robot.
    
    Uses the provided planning_component to continually fetch the robot's current state,
    compute a new target pose (using ORCA for collision avoidance when needed),
    and plan+execute a new motion segment.
    """
    logger = node.get_logger()

    # Extract Gazebo poses for this robot (for example, the inspect and bin poses)
    inspect_pos = np.array(gz_poses[0])
    inspect_approach = np.array(gz_poses[1])
    bin_approach = np.array(gz_poses[2])
    bin_pos = np.array(gz_poses[3])
    
    # Set initial goal flag
    robot_goal_reached = False
    epsilon = 0.02
    max_len = 0.05
    alpha = 0.5
    pref_speed = 0.5
    max_speed = 1.0
    goal_velocity_factor = 1.0

    # Initialize local copies of (global) velocity and previous position variables
    global ur1_current_EE_vel, ur2_current_EE_vel, ur1_prev_EE_position, ur2_prev_EE_position, ur1_speed, ur2_speed

    # Initially set start state (this call is assumed to update the planning component’s state)
    planning_component.set_start_state_to_current_state()
    other_planning_component.set_start_state_to_current_state()

    # Get current end-effector pose for own robot and for the other robot
    current_pose_stamped = compute_current_pose(planning_component, robot_ee)
    current_EE_position = np.array([current_pose_stamped.pose.position.x,
                                    current_pose_stamped.pose.position.y,
                                    current_pose_stamped.pose.position.z])
    other_pose_stamped = compute_current_pose(other_planning_component, other_robot_ee)
    other_current_EE_position = np.array([other_pose_stamped.pose.position.x,
                                          other_pose_stamped.pose.position.y,
                                          other_pose_stamped.pose.position.z])
    # Initialize previous positions if needed.
    if robot_id == "ur1":
        ur1_prev_EE_position = current_EE_position.copy()
        ur2_prev_EE_position = other_current_EE_position.copy()
    else:
        ur2_prev_EE_position = current_EE_position.copy()
        ur1_prev_EE_position = other_current_EE_position.copy()

    # Define goal poses (different for each robot)
    if robot_id == "ur1":
        goal_EE_position = np.array([0.62, -0.25, 1.0])
        goal_EE_orientation = np.array([3.14, 1.57, 3.14])
    else:
        goal_EE_position = np.array([0.88, 0.15, 1.0])
        goal_EE_orientation = np.array([3.14, -1.57, 3.14])

    rate = 10  # Hz loop rate
    r = node.create_rate(rate)
    
    while rclpy.ok() and not robot_goal_reached:
        # Update current state from planning component
        planning_component.set_start_state_to_current_state()
        current_pose_stamped = compute_current_pose(planning_component, robot_ee)
        current_EE_position = np.array([current_pose_stamped.pose.position.x,
                                        current_pose_stamped.pose.position.y,
                                        current_pose_stamped.pose.position.z])
        # Publish marker shifted along an axis (here we use a fixed direction for simplicity)
        shifted_direction = np.array([0, 0, 1])  # Replace with actual EE axis if available
        marker_position = current_EE_position + shifted_direction * 0.075
        publish_marker(marker_pub, marker_position, robot_id, 0, node)
        
        # Check if we have reached the ultimate goal (to switch from approach to final)
        if goal_reached(current_EE_position, goal_EE_position, epsilon):
            robot_goal_reached = True
            logger.info(f"[{robot_id}] Goal reached.")
            break

        # Compute movement step length
        len_step = min(max_len, alpha * distance(current_EE_position, goal_EE_position))

        # Get other robot state from its planning component
        other_planning_component.set_start_state_to_current_state()
        other_pose_stamped = compute_current_pose(other_planning_component, other_robot_ee)
        other_current_EE_position = np.array([other_pose_stamped.pose.position.x,
                                              other_pose_stamped.pose.position.y,
                                              other_pose_stamped.pose.position.z])
        
        # Update velocities based on difference with previous position (avoid division by zero)
        if robot_id == "ur1":
            ur1_dist = current_EE_position - ur1_prev_EE_position
            ur2_dist = other_current_EE_position - ur2_prev_EE_position
            if np.linalg.norm(ur1_dist) < 0.001:
                current_EE_vel = ur1_dist
            else:
                current_EE_vel = ur1_dist * ur1_speed / np.linalg.norm(ur1_dist)
            if np.linalg.norm(ur2_dist) < 0.001:
                other_current_EE_vel = ur2_dist
            else:
                other_current_EE_vel = ur2_dist * ur2_speed / np.linalg.norm(ur2_dist)
        else:
            ur2_dist = current_EE_position - ur2_prev_EE_position
            ur1_dist = other_current_EE_position - ur1_prev_EE_position
            if np.linalg.norm(ur2_dist) < 0.001:
                current_EE_vel = ur2_dist
            else:
                current_EE_vel = ur2_dist * ur2_speed / np.linalg.norm(ur2_dist)
            if np.linalg.norm(ur1_dist) < 0.001:
                other_current_EE_vel = ur1_dist
            else:
                other_current_EE_vel = ur1_dist * ur1_speed / np.linalg.norm(ur1_dist)

        # Build agent dictionaries for ORCA:
        # For simplicity, we assume the “marker” position (shifted along a fixed axis) as the agent position.
        main_agent = {
            "position": current_EE_position + shifted_direction * 0.075,
            "radius": 0.2,
            "velocity": current_EE_vel
        }
        other_agent = {
            "position": other_current_EE_position + shifted_direction * 0.075,
            "radius": 0.2,
            "velocity": other_current_EE_vel
        }
        # Compute ORCA escape velocity adjustments (this returns a list; if a single adjustment is returned we use it)
        orca_adj = ORCA(main_agent, [other_agent])
        
        # Decide on motion: if collision situation detected (criteria similar to C++ version)
        if (len(orca_adj) == 1 and 
            distance(current_EE_position, other_current_EE_position) < 0.3 and
            (np.linalg.norm(current_EE_vel) > 0.001 or np.linalg.norm(other_current_EE_vel) > 0.001)):
            logger.info(f"[{robot_id}] Collision Case")
            next_speed = np.linalg.norm(goal_velocity_factor*current_EE_vel + orca_adj[0])
            robot_speed = max_speed if next_speed > max_speed else next_speed
            logger.info(f"[{robot_id}] Speed: {robot_speed}")
            target_unit_vec = (goal_velocity_factor*current_EE_vel + orca_adj[0]) / (np.linalg.norm(goal_velocity_factor*current_EE_vel + orca_adj[0]) + 1e-6)
        else:
            logger.info(f"[{robot_id}] Non-Collision Case")
            robot_speed = pref_speed
            target_unit_vec = (goal_EE_position - current_EE_position) / (distance(current_EE_position, goal_EE_position) + 1e-6)
        
        # Update global state variables
        if robot_id == "ur1":
            ur1_current_EE_vel = current_EE_vel
            ur1_prev_EE_position = current_EE_position.copy()
            ur1_speed = robot_speed
        else:
            ur2_current_EE_vel = current_EE_vel
            ur2_prev_EE_position = current_EE_position.copy()
            ur2_speed = robot_speed
        
        # Define target pose using velocity‐based displacement
        target_pose = set_vel_pose_target(current_EE_position, target_unit_vec, len_step, goal_EE_orientation, robot_goal_reached)
        # Convert to PoseStamped for planning; assume "world" frame.
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = "world"
        target_pose_stamped.pose = target_pose

        # Set the goal state for planning using the IK solver
        planning_component.set_goal_state(pose_stamped_msg=target_pose_stamped, pose_link=robot_ee)
        
        # Plan the motion segment
        plan_result = planning_component.plan()
        if plan_result:
            # Adjust the trajectory timing based on speed
            # plan_result = EE_vel(planning_component.get_current_state(), plan_result, robot_ee, 0.5, logger)
            # Execute the trajectory segment
            execute_trajectory_and_wait(moveit_instance, plan_result, controller_name, robot_id, logger)

        else:
            logger.error(f"[{robot_id}] Planning failed. Retrying...")
            time.sleep(0.1)
        r.sleep()

# --- Main Function ---

def main():
    rclpy.init()
    node = Node("dual_ur_async_moveit_control_node")
    logger = node.get_logger()

    # Create MoveIt configurations using MoveItConfigsBuilder (update paths to your packages and files)
    # Here we assume a single robot description that includes both manipulators.
    # Adjust robot_name, package_name, and file paths as needed.
    moveit_configs = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_two_moveit_config")
        # .robot_description(file_path="path/to/your/urdf/substitute.urdf")
        .robot_description_semantic(Path("srdf") / "ur.srdf", {"name": "ur3e"})
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_configs.to_dict()
    moveit_config_dict.update({
        "workspace_bounds": {
            "min_corner": [0.0, -0.85, 0.0],
            "max_corner": [1.50, 0.85, 2.0]
        },
        "use_sim_time": True,
        'qos_overrides./clock.subscription.durability': 'transient_local',
        'qos_overrides./clock.subscription.history': 'keep_last',
        'qos_overrides./clock.subscription.depth': 100,
        'qos_overrides./clock.subscription.reliability': 'reliable'
    })

    # Initialize MoveItPy instance
    global moveit_instance
    moveit_instance = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)
    logger.info("MoveItPy instance created")

    # Get planning components for both robots
    ur1_planning_component = moveit_instance.get_planning_component("UR1_manipulator")
    ur2_planning_component = moveit_instance.get_planning_component("UR2_manipulator")

    # Define Gazebo world poses (as in the original C++ code)
    ur1_bin         = [0.35, -0.49, 1.0, 3.14, 0.0, 3.14]
    ur2_bin         = [1.16,  0.49, 1.0, 3.14, 0.0, 3.14]
    ur1_bin_approach= [0.63, -0.35, 1.0, 3.14, 0.0, 3.14]
    ur2_bin_approach= [0.87,  0.35, 1.0, 3.14, 0.0, 3.14]
    ur1_inspect_pos = [0.6,   0.28, 1.0, 3.14, 0.0, 3.14]
    ur2_inspect_pos = [0.9,   0.28, 1.0, 3.14, 0.0, 3.14]
    ur1_inspect_approach = [0.6, 0.28, 1.0, -3.14, -1.3, -3.14]
    ur2_inspect_approach = [0.9, 0.28, 1.0, -3.14,  1.3, -3.14]
    ur1_gz_poses = [ur1_inspect_pos, ur1_inspect_approach, ur1_bin_approach, ur1_bin]
    ur2_gz_poses = [ur2_inspect_pos, ur2_inspect_approach, ur2_bin_approach, ur2_bin]

    # Create marker publishers for visualization
    ur1_marker_pub = node.create_publisher(Marker, "ur1_marker", 10)
    ur2_marker_pub = node.create_publisher(Marker, "ur2_marker", 10)

    time.sleep(1.0)  # Allow time for publishers to connect

    # Define action controller names (as used by your controllers)
    ur1_controller_name = "joint_trajectory_controller_ur1"
    ur2_controller_name = "joint_trajectory_controller_ur2"

    # Define end-effector link names for each robot
    ur1_ee = "UR1_wrist_3_link"
    ur2_ee = "UR2_wrist_3_link"

    # Start each robot’s planning/execution loop in separate threads.
    thread_ur1 = threading.Thread(target=run_robot,
                                  args=(node, ur1_planning_component, ur2_planning_component,
                                        "ur1", ur1_controller_name, ur1_ee, ur2_ee, ur1_gz_poses, ur1_marker_pub))
    thread_ur2 = threading.Thread(target=run_robot,
                                  args=(node, ur2_planning_component, ur1_planning_component,
                                        "ur2", ur2_controller_name, ur2_ee, ur1_ee, ur2_gz_poses, ur2_marker_pub))
    thread_ur1.start()
    thread_ur2.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Shutting down dual robot controller...")

    thread_ur1.join()
    thread_ur2.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()