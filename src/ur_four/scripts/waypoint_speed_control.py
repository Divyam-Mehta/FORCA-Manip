#!/usr/bin/env python3

import os
import time
import numpy as np
import rclpy
from rclpy.logging import get_logger
from rclpy.duration import Duration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit.core.kinematic_constraints import construct_joint_constraint
from geometry_msgs.msg import PoseStamped
import math
from pathlib import Path

from moveit.core.robot_trajectory import RobotTrajectory

def initialize_rclpy():
    rclpy.init()
    return get_logger("moveit_py.pose_goal")

def configure_moveit():

    ur_two = get_package_share_directory("ur_two")

    # Set workspace bounds (min and max corners of the planning volume)
    workspace_bounds = {
        "workspace_bounds": {
            "min_corner": [0.0, -0.85, 0.0],  # Set appropriate min corner values
            "max_corner": [1.50, 0.85, 2.0]     # Set appropriate max corner values
        }
    }

    moveit_configs = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_two_moveit_config")
        .robot_description(file_path=os.path.join(ur_two, "urdf", "substitute.urdf"))
        .robot_description_semantic(Path("srdf") / "ur.srdf", {"name": "ur3e"})
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_configs.to_dict()

    # Update with workspace bounds
    moveit_config_dict.update(workspace_bounds)

    # Add QoS overrides for the /clock topic
    moveit_config_dict.update({
        'use_sim_time': True,
        'qos_overrides./clock.subscription.durability': 'volatile',  # 'transient_local'
        'qos_overrides./clock.subscription.history': 'keep_last',
        'qos_overrides./clock.subscription.depth': 100,
        'qos_overrides./clock.subscription.reliability': 'reliable',
    })

    return moveit_config_dict

def initialize_moveit(moveit_config_dict):
    ur_move = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)
    logger.info("MoveItPy instance created")
    return ur_move

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def plan(planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
):
    """Helper function to plan a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    return plan_result


def execute(robot, plan_result, controller_name, sleep_time=0.0):
    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[controller_name])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)



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



def main():
    global logger
    logger = initialize_rclpy()

    ur1_ee_link = "UR1_wrist_3_link"
    ur2_ee_link = "UR2_wrist_3_link"

    moveit_config = configure_moveit()
    ur_move = initialize_moveit(moveit_config)
    
    ur1 = ur_move.get_planning_component("UR1_manipulator")
    # ur2 = ur_move.get_planning_component("UR2_manipulator")

    

    ur_model = ur_move.get_robot_model()
    ur_state = RobotState(ur_model)

    # set plan start state to current state
    ur1.set_start_state_to_current_state()

    ur1_roll = 0.0
    ur1_pitch = 1.57
    ur1_yaw = 0.0

    ur1_ort = quaternion_from_euler(ur1_roll, ur1_pitch, ur1_yaw)

    ur1_pose_goal = PoseStamped()
    ur1_pose_goal.header.frame_id = "world"
    ur1_pose_goal.pose.orientation.x = ur1_ort[0]
    ur1_pose_goal.pose.orientation.y = ur1_ort[1]
    ur1_pose_goal.pose.orientation.z = ur1_ort[2]
    ur1_pose_goal.pose.orientation.w = ur1_ort[3]
    ur1_pose_goal.pose.position.x = 0.75
    ur1_pose_goal.pose.position.y = 0.0    # 0.25 & 0.05
    ur1_pose_goal.pose.position.z = 1.0
    ur1.set_goal_state(pose_stamped_msg=ur1_pose_goal, pose_link="UR1_wrist_3_link")

    # plan to goal
    ur1_planned_trajectory = plan(ur1, logger)

    # ur1_fixed_vel_trajectory = EE_vel("ur1", ur_state, ur1_planned_trajectory, 0.0001)

    # execute(ur_move, ur1_fixed_vel_trajectory)

    ur1_controller_name = "joint_trajectory_controller_ur1";

    execute(ur_move, ur1_planned_trajectory, ur1_controller_name)

if __name__ == "__main__":
    main()