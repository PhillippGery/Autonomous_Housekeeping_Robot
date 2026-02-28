# 🤖 Autonomous Housekeeping Robot

A ROS2-based autonomous robot system built on **TurtleBot3** and simulated in **Gazebo**, capable of exploring unknown environments, navigating using A* path planning with RRT* reactive local planning, detecting and avoiding obstacles, and tracking colored objects using computer vision.


---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Tasks](#tasks)
- [Key Algorithms](#key-algorithms)
- [MATLAB PID Optimization](#matlab-pid-optimization)
- [Configuration & Tuning](#configuration--tuning)
- [Dependencies](#dependencies)
- [Installation & Build](#installation--build)
- [Running the Simulation](#running-the-simulation)
- [Project Structure](#project-structure)

---

## Overview

This project implements a fully autonomous housekeeping robot that can:

- **Autonomously explore and map** an unknown environment using frontier-based SLAM exploration
- **Localize itself** on a pre-built map using AMCL (Adaptive Monte Carlo Localization)
- **Navigate** to user-defined goals using a **two-layer planner**: A* global planning + RRT* reactive local planning
- **Detect and avoid** both static and dynamic obstacles in real time, estimating obstacle geometry from LiDAR
- **Track colored balls** (red, green, blue) using computer vision and approach them autonomously
- **Follow walls** in narrow spaces using a PID controller
- **Optimized motion control** parameters via MATLAB multi-start fmincon optimization with noise modeling

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                        ROS2 Node                             │
│                                                              │
│   /map ──────────► SLAM Map Processor                        │
│   /scan ─────────► Obstacle Detector / LiDAR Geometry Est.  │
│   /odom ─────────► TF Transform (map ← odom)                 │
│   /camera ───────► Ball Detector (Task 3)                    │
│   /pose ─────────► AMCL Pose Estimator                       │
│                           │                                  │
│                    State Machine                             │
│        ┌──────────────────┼───────────────────┐             │
│      IDLE          ASTAR_PATH            WALL_FOLLOWING      │
│        │            FOLLOWING                  │             │
│        ▼                 ▼                     ▼             │
│   Frontier        A* Global Path         PID Wall            │
│   Selection       Follower               Follower            │
│                          │                                   │
│              ┌───────────┴────────────┐                      │
│        No obstacle              Obstacle detected            │
│              │                        │                      │
│        Follow A* path       Stop → Align → Estimate size     │
│                             Add to costmap                   │
│                                       │                      │
│                             RRT* Local Planner               │
│                             (bypass segment)                 │
│                                       │                      │
│                             Reconnect to A* path             │
│                                       │                      │
│                           /cmd_vel publisher                  │
└──────────────────────────────────────────────────────────────┘
```

---

## Tasks

### Task 1 — Autonomous Exploration & Mapping (`task1.py`)

The robot explores a completely **unknown environment** autonomously using frontier-based exploration combined with SLAM.

**How it works:**
- Subscribes to the live `/map` topic from a SLAM node
- Identifies **frontier cells** — free cells adjacent to unknown cells — as exploration targets
- Ranks frontiers using a cost function balancing **distance** and **information gain**
- Plans a path to the best frontier using A* on the live map graph
- Inflates only wall obstacles (not unknown cells) for safe clearance without blocking exploration
- Switches to **wall following** mode in narrow/unknown spaces
- Corrects odometry drift using SLAM TF transforms
- Declares exploration complete when less than 1% of known cells remain unexplored and no path is possible

**Key design decisions:**
- Frontiers require at least 5 free neighbors and a minimum distance of 0.6 m to avoid trivial targets
- Skips angle alignment at waypoints (saves time without sacrificing accuracy)
- Wall follower acts as fallback for narrow corridors where A* planning fails

**State Machine:**
```
IDLE → ASTARPATH_FOLLOWING → RETREATING → WALL_FOLLOWING → MAP_EXPLORED
```

---

### Task 2 — Localization & Navigation with Obstacle Avoidance (`task2.py`)

The robot navigates a **known, pre-built map** to user-defined goals while avoiding dynamic obstacles detected at runtime.

**How it works:**
- Loads a pre-built `.yaml` map and inflates obstacles for safe path planning
- Uses **AMCL** for localization on the known map
- Accepts navigation goals from RViz (`/move_base_simple/goal`)
- Plans paths with A* and follows them using a PID path follower
- When an unmapped obstacle is detected:
  1. Stops and retreats (dual safety zones: front + back during retreat)
  2. Aligns to face the obstacle directly
  3. Estimates the obstacle's **diameter** using LiDAR jump detection: `L = 2R · sin(Δθ/2)`
  4. Calculates world-frame obstacle center position
  5. Adds the obstacle to the costmap with correct inflation
  6. Replans around it using A*
  7. Resets costmap after goal reached to avoid accumulating phantom obstacles
- Publishes the inflated costmap to `/custom_costmap` for visualization in RViz

**State Machine:**
```
IDLE → ASTARPATH_FOLLOWING → RETREATING → ALIGNING_TO_OBS → REPLANNING → ASTARPATH_FOLLOWING
```

---

### Task 2 Bonus — RRT* Reactive Local Planner (`task2_bonus.py`)

Extension of Task 2 that introduces a **two-layer planning architecture** combining A* global planning with an **RRT* local planner** (`RRTStarGrid` class) for reactive obstacle avoidance.

**Architecture:**
```
Global Layer:  A* plans full path from start to goal on static map
                          │
                          ▼
Local Layer:   Obstacle detected mid-execution
                          │
               Stop → Estimate geometry → Add to costmap
                          │
               RRT* plans local bypass path segment
               (from current position to a point ahead on the A* path)
                          │
               Execute bypass → Reconnect to A* global path
                          │
               Replan A* from reconnection point to final goal
```

**Why RRT* as local planner:**
- RRT* (Rapidly-exploring Random Tree Star) efficiently finds collision-free paths in continuous space with asymptotic optimality
- Unlike re-running full A* (grid-based, discrete), RRT* explores the local configuration space and finds smooth bypass paths
- The robot autonomously visits randomly selected valid waypoints in sequence, demonstrating fully autonomous patrol-style navigation without any user input

---

### Task 3 — Ball Detection & Navigation (`task3.py`)

The robot navigates the environment and uses its **camera** to detect and approach colored balls (red, green, blue).

**How it works:**
- Uses OpenCV HSV color segmentation to detect colored balls in the camera image
- **Circularity filter** (threshold > 0.75) rejects bricks and non-spherical objects, eliminating false detections
- Adaptive HSV thresholds handle near/large objects that fill the image
- Pipeline: Align → Estimate world position → Plan approach with correct final orientation → Re-estimate at close range → Identify color via mask overlap
- Distance estimated from known physical diameter vs. apparent pixel size
- A PID controller centers the robot on the ball and drives toward it
- Stores world position of detected balls for revisiting; falls back to A* between ball-seeking behaviors
- Estimation accuracy: ≥ 0.01 m

---

## Key Algorithms

### A* Path Planning
Implemented from scratch using a priority queue. Heuristic is Euclidean distance in grid space. Supports 8-directional movement (cardinal + diagonal, diagonal cost = √2). If the start or goal is inside an obstacle, the nearest valid free cell within a configurable radius is used instead.

### RRT* Local Planner (`RRTStarGrid` class)
Rapidly-exploring Random Tree Star operating on the robot's local costmap:
- Samples random nodes in the local planning area around the robot
- Extends the tree toward sampled points, checking for obstacle collisions in the costmap
- Rewires the tree to minimize cumulative path cost (asymptotically optimal)
- Triggered only upon obstacle detection; operates on the segment between current position and the next A* waypoint ahead
- Returns a smooth, collision-free bypass path segment that reconnects to the global A* plan

### Frontier-Based Exploration (Task 1)
Frontiers are free cells adjacent to unknown cells. Ranked by:
```
cost = W_dist × euclidean_distance + W_power / local_area_gain
```
where `local_area_gain` is the fraction of unknown cells in a local window — rewarding frontiers that reveal large unexplored areas.

### Obstacle Geometry Estimation
When an obstacle is detected, the robot aligns to face it and uses LiDAR angular sweep to detect the angular span via jump detection in range values. Obstacle diameter:
```
L = 2R · sin(Δθ / 2)
```
where R is the measured range and Δθ is the detected angular span. World-frame center position is calculated from robot pose and bearing.

### PID Path Follower (Polar Coordinates)
Converts Cartesian error to polar coordinates (ρ, α, β):
- Linear speed: proportional to ρ (distance to lookahead point)
- Angular speed: PID on α (heading error) + β correction term for final orientation alignment

### Dynamic Lookahead
```
lookahead = speed × lookahead_ratio + min_lookahead
```

### Line-of-Sight Shortcutting (Bresenham)
Before selecting the next waypoint, checks if a straight line to the final goal is collision-free using Bresenham's line algorithm. Shortcuts directly to goal if clear, reducing path length and execution time.

### Wall Follower (PID)
Maintains desired distance from the right wall using the 45° angled LiDAR reading as primary input (more stable than 90°). Stops and turns left when an obstacle appears in front.

---

## MATLAB PID Optimization

The PID path follower parameters were optimized offline using **MATLAB's `fmincon`** with a **multi-start strategy** to escape local minima and find globally robust gains.

### Optimization Setup

| Setting | Value |
|---|---|
| Optimizer | `fmincon` (SQP algorithm) |
| Strategy | Multi-start: 20 random initial points |
| Parameters tuned | `k_rho`, `kp_ang`, `ki_ang`, `kd_ang`, `k_beta`, `lookahead_dist` |
| Cost weights | `W_error = 40.0`, `W_time = 0.15` |

### Cost Function
```
J = W_error × MSE(position_error) + W_time × time_penalty
```
Penalizes both tracking error and execution time — creating an explicit speed/precision trade-off.

### Simulation Fidelity
The MATLAB optimizer runs a closed-loop simulation that includes:
- **First-order actuator dynamics**: τ_v = 0.2 s (linear), τ_ω = 0.1 s (angular)
- **Sensor noise**: position σ = 0.01 m, angular σ = 0.02 rad

This ensures parameters are robust to real-world actuator lag and sensor noise, not just ideal conditions.

### MATLAB GUI PID Tuner
A companion interactive GUI (MATLAB App Designer) provides live sliders for each parameter with real-time step response visualization, enabling rapid manual fine-tuning and intuitive understanding of parameter interactions.

### Optimized Parameters

| Parameter | Value | Description |
|---|---|---|
| `k_rho` | 0.8608 | Proportional gain for linear speed |
| `kp_angular` | 2.0747 | Proportional gain for angular PID |
| `ki_angular` | optimized | Integral gain for angular PID |
| `kd_angular` | optimized | Derivative gain for angular PID |
| `k_beta` | optimized | Final orientation correction gain |
| `lookahead_dist` | optimized | Base lookahead distance |

---

## Configuration & Tuning

Key parameters in the task files:

| Parameter | Default | Description |
|---|---|---|
| `speed_max` | 0.31 m/s | Maximum linear speed |
| `rotspeed_max` | 1.9 rad/s | Maximum angular speed |
| `goal_tolerance` | 0.1 m | Distance to consider goal reached |
| `inflation_kernel_size` | 4–10 cells | Obstacle inflation radius |
| `min_frontier_distance` | 0.6 m | Ignore frontiers closer than this |
| `Frontier_W_dist` | 1.0 | Weight for distance in frontier cost |
| `Frontier_W_power` | 3.0 | Weight for information gain in frontier cost |
| `k_rho` | 0.8608 | Proportional gain for linear speed (MATLAB optimized) |
| `kp_angular` | 2.0747 | Proportional gain for angular PID (MATLAB optimized) |
| `min_front_obstacle_distance` | 0.35 m | Trigger obstacle avoidance |
| `retreat_distance` | 0.25 m | How far to reverse when obstacle detected |
| `rrt_star_iterations` | configurable | RRT* planning iterations for local bypass |

---

## Dependencies

### ROS2 Packages
- `rclpy` — ROS2 Python client library
- `nav_msgs` — OccupancyGrid, Path, Odometry
- `geometry_msgs` — PoseStamped, Twist, PoseWithCovarianceStamped
- `sensor_msgs` — LaserScan, Image
- `vision_msgs` — BoundingBox2D
- `tf2_ros` — TF2 transform listener
- `cv_bridge` — ROS ↔ OpenCV image conversion

### Python Libraries
```
numpy
opencv-python (cv2)
Pillow (PIL)
PyYAML
pandas
```

### Simulation
- **Gazebo** — physics simulation
- **TurtleBot3** — robot model and packages
- **SLAM Toolbox** — for Task 1 live mapping

### Optimization (offline)
- **MATLAB** — `fmincon` optimizer, Control System Toolbox
- **MATLAB App Designer** — GUI PID tuner

---

## Installation & Build

```bash
# 1. Clone the repository
git clone https://github.com/PhillippGery/Autonomous_Housekeeping_Robot.git
cd Autonomous_Housekeeping_Robot/src

# 2. Install ROS2 dependencies
rosdep install --from-paths . --ignore-src -r -y

# 3. Build the workspace
colcon build --symlink-install

# 4. Source the workspace
source install/setup.bash
```

---

## Running the Simulation

### Task 1 — Autonomous Exploration

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# Terminal 2: Launch SLAM Toolbox (online async)
ros2 launch turtlebot3_gazebo mapper.launch.py

# Terminal 3: Run the exploration node
ros2 run turtlebot3_gazebo task1
```

### Task 2 — Navigation on Known Map

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# Terminal 2: Launch AMCL localization
ros2 launch turtlebot3_gazebo amcl.launch.py

# Terminal 3: Run the navigation node
ros2 run turtlebot3_gazebo task2

# In RViz: Use "2D Nav Goal" tool to set a navigation target
```

### Task 2 Bonus — RRT* Reactive Patrol

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# Terminal 2: Launch AMCL localization
ros2 launch turtlebot3_gazebo amcl.launch.py

# Terminal 3: Run the RRT* patrol node
ros2 run turtlebot3_gazebo task2_bonus
```

### Task 3 — Ball Tracking

```bash
# Terminal 1: Launch Gazebo with balls spawned
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# Terminal 2: Spawn colored ball objects
ros2 run turtlebot3_gazebo spawn_objects

# Terminal 3: Launch AMCL
ros2 launch turtlebot3_gazebo amcl.launch.py

# Terminal 4: Run the ball tracking node
ros2 run turtlebot3_gazebo task3
```

---

## Project Structure

```
Autonomous_Housekeeping_Robot/
└── src/
    ├── turtlebot3_gazebo/
    │   ├── src/lab4/
    │   │   ├── task1.py             # Autonomous SLAM exploration
    │   │   ├── task2.py             # Navigation + obstacle avoidance (A*)
    │   │   ├── task2_bonus.py       # A* global + RRT* local planner patrol
    │   │   ├── task3.py             # Ball detection & navigation
    │   │   ├── spawn_objects.py     # Spawns balls in Gazebo
    │   │   └── static_obstacles.py  # Spawns static obstacles
    │   ├── src_fcn/
    │   │   ├── AStar.py             # Standalone A* implementation
    │   │   ├── auto_navigator.py    # Shared navigation utilities
    │   │   ├── red_ball_tracker.py  # Standalone ball tracker node
    │   │   └── wall_follower.py     # PID wall following controller
    │   ├── maps/
    │   │   ├── map.yaml             # Pre-built house map
    │   │   └── sync_classroom_map.yaml
    │   ├── launch/
    │   │   ├── turtlebot3_house.launch.py
    │   │   ├── amcl.launch.py
    │   │   ├── mapper.launch.py
    │   │   └── ...
    │   └── params/
    │       ├── amcl.yaml
    │       └── mapper_params_online_async.yaml
    ├── sim_utils/
    │   └── sim_utils/
    │       └── red_ball_control.py  # Ball motion controller for simulation
    └── matlab/
        ├── pid_optimizer.m          # Multi-start fmincon PID optimization
        └── pid_tuner_gui.mlapp      # Interactive MATLAB GUI PID tuner
```

---

## Design Philosophy

Built with a **safety-first, bottom-up** approach: robust low-level motion control was established first, then higher-level behaviors were layered on top. Every state has explicit safety zones and fallback behaviors.

The monolithic file structure (single Python file per task) was a conscious decision for rapid iteration during the course, and is acknowledged as a limitation for production systems. Future improvements would include proper ROS2 node decomposition, improved SLAM parameter tuning, and better utilization of the broader ROS2 ecosystem (nav2, costmap plugins, etc.).

---

## Authors

**Phillipp Gery** — Purdue University, MS Interdisciplinary Engineering (Autonomy & Robotics)  
Fulbright Scholar 