# FORCA-Manip

**FORCA-Manip** is a ROS 2 + MoveIt 2 research framework for **decentralized collision avoidance and motion planning in multi-manipulator systems**. The repository is organized around a four-robot Universal Robots simulation setup and is intended for research, benchmarking, and reproducible experimentation in shared-workspace manipulation.

The project combines:
- a **Gazebo-based multi-robot simulation environment**
- **MoveIt 2** for planning and execution
- a **decentralized ORCA-inspired coordination layer**
- baseline methods for comparison against the proposed FORCA pipeline

This repository is designed to present the project in a clean, research-grade format suitable for academic sharing, thesis support, and future publication.

---

## Project Overview

Coordinating multiple robot manipulators in a shared workspace is difficult because each arm must complete its own task while avoiding collisions with neighboring robots, static infrastructure, and moving objects. Traditional centralized planning methods can become computationally expensive as the number of manipulators increases, while fully independent planning often fails in dense shared environments.

FORCA-Manip addresses this by integrating **decentralized collision avoidance** with **manipulation planning** for a four-arm setup. The repository contains:
- a custom ROS 2 package for multi-robot simulation and execution
- a MoveIt configuration package for planning and visualization
- baseline implementations for comparison
- launch utilities for simulation, planning, and experiment workflows

Although the framework can be extended to other industrial manipulation tasks, the current setup is particularly suitable for **multi-arm coordination, sorting, pick-and-place research, and shared-workspace trajectory generation**.

---

## Problem Statement

In a multi-robot manipulator system, several robotic arms operate simultaneously in a confined shared environment. Each arm may have an individual goal, but its motion directly affects the feasible motion of the others. This creates several challenges:

1. **Dynamic collision risk**  
   Each manipulator is both a planner-controlled agent and a moving obstacle for the others.

2. **Scalability limits of centralized planning**  
   Joint planning in a high-dimensional combined configuration space becomes expensive and difficult to scale.

3. **Execution-time coordination**  
   Even when individual plans are feasible, simultaneous execution can create near-collisions, deadlocks, or poor throughput.

4. **Realistic deployment constraints**  
   Practical systems must work with robot models, controllers, simulation middleware, and motion-planning pipelines that resemble real deployment conditions.

The goal of FORCA-Manip is to provide a framework in which **decentralized coordination and manipulation planning can be studied together**, with a focus on reducing conflicts while maintaining task progress and motion feasibility.

---

## Method Summary

The core idea behind FORCA-Manip is to combine **sampling-based or MoveIt-generated manipulator plans** with an **online decentralized coordination layer** inspired by ORCA-style collision avoidance.

### High-level workflow

1. A multi-robot environment is created in Gazebo.
2. Four UR robots are spawned into a shared workspace together with task-relevant assets such as bins, conveyor elements, and cameras.
3. MoveIt 2 provides robot models, planning interfaces, kinematics, and execution infrastructure.
4. Each robot is assigned goals or motion objectives.
5. A decentralized coordination layer adjusts motion behavior to reduce inter-robot conflicts while preserving task execution.
6. The proposed method is compared against baseline strategies.

### What is implemented in this repository

The repository includes:
- a **FORCA / ORCA-inspired multi-robot execution pipeline**
- baseline executables such as:
  - `baseline`
  - `centralized_baseline`
  - `dec_rrt_baseline`
- the main research executable:
  - `multi_thread_psort_orca`
- a four-robot Gazebo setup with URDF/Xacro assets, controllers, and RViz support
- MoveIt 2 configuration for the multi-robot model

### Research intent

This codebase is intended for studying questions such as:
- How well does decentralized coordination scale for multiple arms?
- How does the proposed method compare against centralized or simpler baselines?
- Can simultaneous multi-arm execution remain feasible and safe in dense shared workspaces?
- What trade-offs arise between speed, safety, and coordination smoothness?

---

## System Architecture

The project is built around two main ROS 2 packages:

- `ur_four`  
  Contains the multi-robot simulation environment, URDF/Xacro assets, controllers, task objects, Gazebo launches, and research executables.

- `ur_four_moveit_config`  
  Contains MoveIt 2 configuration assets such as planning pipelines, joint limits, kinematics, servo settings, SRDF, and launch files for visualization and planning.

### Conceptual architecture

```text
User / Researcher
        │
        ▼
 Experiment Launch Layer
        │
        ├── Gazebo simulation launch
        ├── MoveIt launch
        └── Planner / baseline launch
        │
        ▼
 Multi-Robot World (ROS 2 + Gazebo)
        │
        ├── 4x UR robots
        ├── controllers
        ├── robot_state_publisher
        ├── ros_gz_bridge
        ├── static transforms
        └── task objects (bins, conveyor, cameras, etc.)
        │
        ▼
 Planning + Coordination Layer
        │
        ├── MoveIt 2 planning interface
        ├── kinematics + collision checking
        ├── decentralized coordination logic
        └── baseline / comparison methods
        │
        ▼
 Execution + Evaluation
        │
        ├── trajectory execution
        ├── RViz visualization
        ├── Gazebo monitoring
        └── qualitative / quantitative results
