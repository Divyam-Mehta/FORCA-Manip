# FORCA-Manip

**FORCA-Manip** is the official code repository accompanying the thesis **“Decentralized Collision Avoidance and Motion Planning for Multi-Robot Manipulator Systems”** by **Divyam Mehta**. This repository presents the implementation of **Fast-ORCA (FORCA)**, a decentralized collision-avoidance and motion-planning framework for multi-robot manipulators performing precision sorting tasks in a shared workspace.

The project is built on **ROS 2**, **MoveIt 2**, and **Gazebo Harmonic**, and focuses on enabling multiple robotic manipulators to operate simultaneously with reduced collision risk while preserving task throughput.

---

## Project Overview

Coordinating multiple robotic manipulators in a shared environment is a challenging problem in robotics. Each robot must achieve its own task objective while simultaneously accounting for neighboring robots, shared obstacles, limited workspace, and execution constraints. As the number of manipulators grows, naïve independent planning often becomes unsafe, while centralized planning becomes increasingly expensive and difficult to scale.

FORCA-Manip addresses this problem by integrating:
- a **Gazebo-based multi-robot simulation environment**
- **MoveIt 2** for motion planning and execution
- a **decentralized ORCA-inspired coordination layer**
- baseline methods for comparison against the proposed FORCA framework

The repository is centered around a **four-robot Universal Robots simulation setup** and is intended for:
- research and benchmarking
- reproducible experimentation
- multi-arm motion planning studies
- decentralized collision avoidance evaluation
- shared-workspace robotic manipulation

---

## Problem Statement

In a multi-robot manipulator system, several robotic arms operate simultaneously in a common workspace. Even if each manipulator has a feasible individual plan, simultaneous execution may still lead to unsafe interactions such as collisions, deadlocks, or inefficient motion.

This creates several major challenges:

1. **Dynamic collision risk**  
   Each manipulator acts both as a task-executing robot and as a moving obstacle for the others.

2. **Scalability limitations of centralized planning**  
   Planning in the combined configuration space of multiple manipulators becomes computationally expensive as robot count increases.

3. **Execution-time coordination**  
   Safe simultaneous execution requires continuous coordination rather than one-time path generation alone.

4. **Realistic deployment constraints**  
   Practical solutions must work with robot kinematics, inverse kinematics, collision checking, controller interfaces, and motion-planning infrastructure that resemble real deployment settings.

The goal of FORCA-Manip is to provide a framework in which **decentralized collision avoidance and manipulation planning can be studied together**, while maintaining motion feasibility, task progress, and compatibility with standard robotics software tools.

---

## Method Summary

The core contribution of this repository is the implementation of **Fast-ORCA (FORCA)** for multi-robot manipulators.

FORCA adapts the ideas of **Optimal Reciprocal Collision Avoidance (ORCA)** from point-agent style collision avoidance into a manipulator setting. Instead of directly applying velocity commands to articulated robots, FORCA translates collision-free velocity reasoning into position-level robot control that remains compatible with inverse kinematics and path planning constraints.

### High-level workflow

1. A multi-robot environment is created in Gazebo Harmonic.
2. Four UR-based manipulators are spawned into a shared workspace together with task-relevant objects.
3. Each robot is assigned goals or task-specific motion objectives.
4. FORCA computes a candidate collision-free end-effector displacement using 3D velocity-obstacle reasoning.
5. The candidate displacement is accepted only if it yields a valid inverse-kinematics solution.
6. A feasible path is then generated using **RRTConnect**.
7. A waypoint speed-control module retimes the motion so that the executed trajectory remains consistent with the FORCA command.
8. The proposed method is compared against baseline strategies such as greedy straight-to-goal and other planning-based approaches.

### What is implemented in this repository

The repository includes:
- a **FORCA / ORCA-inspired multi-robot execution pipeline**
- a four-robot Gazebo simulation setup
- MoveIt 2 configuration for the multi-robot system
- baseline implementations for comparison
- launch utilities for simulation, planning, and experiments

### Main research executable

The core implementation centers around the FORCA-based multi-robot execution logic, including the main research executable:

- `multi_thread_psort_orca`

### Example baseline executables

Depending on your package layout, the repository may also include executables such as:
- `baseline`
- `centralized_baseline`
- `dec_rrt_baseline`

---

## System Architecture

The project is organized around two main ROS 2 packages:

- `ur_four`  
  Contains the multi-robot simulation environment, URDF/Xacro assets, controllers, task objects, Gazebo launches, and research executables.

- `ur_four_moveit_config`  
  Contains MoveIt 2 configuration assets such as planning pipelines, joint limits, kinematics, servo settings, semantic descriptions, and launch files for planning and visualization.

### Conceptual architecture

```text
User / Researcher
        │
        ▼
 Experiment Launch Layer
        │
        ├── Gazebo simulation launch
        ├── MoveIt launch
        └── FORCA / baseline launch
        │
        ▼
 Multi-Robot World (ROS 2 + Gazebo Harmonic)
        │
        ├── 4x UR manipulators
        ├── controllers
        ├── robot_state_publisher
        ├── ros_gz bridge
        ├── static transforms
        └── task objects / sorting environment
        │
        ▼
 Planning + Coordination Layer
        │
        ├── MoveIt 2 planning interface
        ├── inverse kinematics validation
        ├── collision checking
        ├── FORCA decentralized coordination
        └── baseline comparison methods
        │
        ▼
 Execution + Evaluation
        │
        ├── trajectory execution
        ├── RViz visualization
        ├── Gazebo monitoring
        └── qualitative / quantitative analysis
```

---

## Setup Instructions
Prerequisites

Make sure the following are installed in your ROS 2 environment:

ROS 2 - Jazzy

MoveIt 2

Gazebo Harmonic

required UR robot dependencies

any additional dependencies used by `ur_four` and `ur_four_moveit_config`
