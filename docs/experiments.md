
---

## `docs/experiment.md`

# Experiments

This document summarizes the experimental setup and evaluation protocol used in the thesis **“Decentralized Collision Avoidance and Motion Planning for Multi-Robot Manipulator Systems.”**

## Objective

The experiments evaluate **Fast-ORCA (FORCA)** for decentralized collision avoidance and motion planning in a multi-robot precision-sorting task. The central research question is whether FORCA can preserve safety in a shared workspace while maintaining competitive throughput relative to a non-avoiding baseline. :contentReference[oaicite:12]{index=12} :contentReference[oaicite:13]{index=13}

## Experimental Setup

All experiments were conducted in **Gazebo Harmonic** using a simulated workspace containing **four UR3e robotic manipulators**. The environment includes:

- a central conveyor
- four robot manipulators
- two shared bins
- four RGB cameras, one associated with each manipulator for inspection :contentReference[oaicite:14]{index=14} :contentReference[oaicite:15]{index=15}

Two robots are placed on each side of the conveyor belt, and each pair shares a common bin. The cameras provide visual inspection viewpoints for the onion-sorting task. :contentReference[oaicite:16]{index=16}

## Task Definition

The task is **precision onion sorting**. A centralized allocator assigns onions to robots using a cost metric based on Euclidean proximity and inverse-kinematics feasibility. After assignment, each manipulator executes a pick–inspect–place cycle:

1. pick the onion from the conveyor
2. move to the inspection waypoint under the assigned camera
3. deposit the onion into the shared bin :contentReference[oaicite:17]{index=17}

Task allocation is centralized, but execution and collision avoidance are decentralized. :contentReference[oaicite:18]{index=18}

## Inspection Layouts

Two inspection regimes were tested:

### Far Inspection
Robots travel a greater distance toward the inspection camera before inspection. :contentReference[oaicite:19]{index=19}

### Near Inspection
Inspection is performed from a comparatively shorter distance to the camera. :contentReference[oaicite:20]{index=20}

## Onion Densities

The workspace density was increased progressively to test planner performance under higher congestion.

### Far Inspection densities
- 30 onions
- 35 onions
- 40 onions
- 45 onions
- 50 onions

### Near Inspection densities
- 30 onions
- 40 onions
- 50 onions

Each density was evaluated using **three different random spatial configurations**. :contentReference[oaicite:21]{index=21} :contentReference[oaicite:22]{index=22}

## Methods Compared

### FORCA
The proposed method performs decentralized collision avoidance by converting ORCA-style velocity reasoning into feasible manipulator motion. A candidate displacement is accepted only if it has a valid IK solution, then a path is generated using **RRTConnect** and retimed using a waypoint speed-control module. :contentReference[oaicite:23]{index=23} :contentReference[oaicite:24]{index=24}

### STG Baseline
The baseline is a greedy **Straight-to-Goal (STG)** planner. It drives each manipulator toward its assigned goal through predefined subgoal waypoints without considering neighboring manipulators during planning. This provides a fair baseline because it shares the same overall task structure but does not explicitly resolve inter-robot conflicts. :contentReference[oaicite:25]{index=25} :contentReference[oaicite:26]{index=26}

## Evaluation Metrics

Two primary metrics were used:

### 1. Throughput
Overall throughput measures the number of onions sorted per unit time, reported as the combined sorting rate of all four UR3e manipulators. :contentReference[oaicite:27]{index=27}

### 2. Collision Count
Collision count measures the number of inter-robot contacts during execution and captures the safety of the generated motions. :contentReference[oaicite:28]{index=28}

These two metrics jointly evaluate both **productivity** and **safety**. :contentReference[oaicite:29]{index=29}

## Summary of Results

### Throughput

In the **Far Inspection** setup, FORCA’s throughput is consistently slightly lower than STG, typically by about **2–5%** across onion densities. The reported combined throughput values are:

| Onion Density | FORCA (onions/min) | STG (onions/min) |
|---|---:|---:|
| 30 | 7.584 | 7.748 |
| 35 | 7.132 | 7.452 |
| 40 | 7.152 | 7.360 |
| 45 | 6.564 | 6.900 |
| 50 | 6.348 | 6.588 |

:contentReference[oaicite:30]{index=30}

In the **Near Inspection** setup, both methods achieve higher throughput because the inspection travel distance is shorter. FORCA remains about **3–6%** lower than STG:

| Onion Density | FORCA Mean | FORCA Std Dev | STG Mean | STG Std Dev |
|---|---:|---:|---:|---:|
| 30 | 14.33 | 0.389 | 14.95 | 0.375 |
| 40 | 12.21 | 0.296 | 13.92 | 0.523 |
| 50 | 11.07 | 0.262 | 12.14 | 0.774 |

:contentReference[oaicite:31]{index=31}

### Safety

The key safety result is that **FORCA achieved zero collisions across all tested configurations**, while STG’s collision counts increased with workspace density, reaching as high as **8–10 collisions per run** in the thesis summary. :contentReference[oaicite:32]{index=32}

## Interpretation

The results show a clear trade-off:

- **STG** is slightly faster because it does not perform mutual avoidance.
- **FORCA** is safer because it actively adjusts motion to avoid neighboring manipulators.

The thesis conclusion is that FORCA preserves collision-free execution with only modest throughput loss, making it practical for dense shared-workspace multi-arm systems. :contentReference[oaicite:33]{index=33}

## Reproducing the Experiments

A typical experiment run consists of:

1. launching the Gazebo world
2. launching MoveIt and RViz
3. spawning onions on the conveyor
4. running either FORCA or STG
5. recording throughput and collision behavior

Example commands:

```bash
ros2 launch ur_four ur_gazebo.launch.py
ros2 launch ur_four_moveit_config ur_moveit.launch.py
ros2 launch ur_four onion_spawner.launch.py
ros2 launch ur_four forca.launch.py
```
