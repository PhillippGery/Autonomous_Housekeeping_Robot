
# Autonomous Housekeeping Robot

A ROS2-based autonomous robot system built on **TurtleBot3** and simulated in **Gazebo**, capable of exploring unknown environments, navigating using A* path planning, detecting and avoiding obstacles, and tracking colored objects using computer vision.

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Tasks](#tasks)
- [Dependencies](#dependencies)
- [Installation & Build](#installation--build)
- [Running the Simulation](#running-the-simulation)
- [Project Structure](#project-structure)
- [Key Algorithms](#key-algorithms)
- [Configuration & Tuning](#configuration--tuning)

---

## Overview

This project implements a fully autonomous housekeeping robot that can:

- **Autonomously explore and map** an unknown environment using frontier-based SLAM exploration
- **Localize itself** on a pre-built map using AMCL (Adaptive Monte Carlo Localization)
- **Navigate** to user-defined goals using A* path planning with obstacle inflation
- **Detect and avoid** both static and dynamic obstacles in real time
- **Track colored balls** (red, green, blue) using computer vision and approach them autonomously
- **Follow walls** in narrow spaces using a PID controller

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     ROS2 Node                           │
│                                                         │
│   /map ──────────► SLAM Map Processor                   │
│   /scan ─────────► Obstacle Detector                    │
│   /odom ─────────► TF Transform (map ← odom)            │
│   /camera ───────► Ball Detector (Task 3)               │
│   /pose ─────────► Robot Pose Estimator                 │
│                           │                             │
│                    State Machine                        │
│             ┌─────────────┼──────────────┐              │
│           IDLE      ASTAR_PATH      WALL_FOLLOWING       │
│             │        FOLLOWING           │              │
│             ▼             ▼              ▼              │
│      Frontier       PID Path        PID Wall            │
│      Selection      Follower        Follower            │
│             └─────────────┼──────────────┘              │
│                           ▼                             │
│                   /cmd_vel publisher                    │
└─────────────────────────────────────────────────────────┘
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
- Inflates obstacles in the costmap for safe clearance
- Switches to **wall following** mode in narrow spaces
- Declares exploration complete when less than 0.06% of known cells are still unknown

**State Machine:**
```
IDLE → ASTARPATH_FOLLOWING → RETREATING → WALL_FOLLOWING → MAP_EXPLORED
```

---

### Task 2 — Localization & Navigation with Obstacle Avoidance (`task2.py`)

The robot navigates a **known, pre-built map** to user-defined goals while avoiding dynamic obstacles.

**How it works:**
- Loads a pre-built `.yaml` map and inflates obstacles for safe path planning
- Uses **AMCL** for localization on the known map
- Accepts navigation goals from RViz (`/move_base_simple/goal`)
- Plans paths with A* and follows them using a PID path follower
- When an unmapped obstacle is detected:
  1. Stops and retreats
  2. Aligns to face the obstacle directly
  3. Estimates the obstacle's **size and world position** from LiDAR data
  4. Adds the obstacle to the costmap and **replans** around it
- Publishes the inflated costmap to `/custom_costmap` for visualization in RViz

**State Machine:**
```
IDLE → ASTARPATH_FOLLOWING → RETREATING → ALIGNING_TO_OBS → REPLANNING → ASTARPATH_FOLLOWING
```

---

### Task 2 Bonus — Random Waypoint Navigation (`task2_bonus.py`)

Extension of Task 2 where the robot autonomously visits randomly selected valid waypoints on the map in sequence, demonstrating fully autonomous patrol-style navigation without any user input.

---

### Task 3 — Ball Detection & Navigation (`task3.py`)

The robot navigates the environment and uses its **camera** to detect and approach colored balls (red, green, blue).

**How it works:**
- Uses OpenCV HSV color segmentation to detect colored balls in the camera image
- Calculates the ball's angular position and estimated distance using its apparent size
- A PID controller centers the robot on the ball and drives toward it
- Stops at a safe target distance once the ball is close enough
- Stores the world position of detected balls for revisiting
- Falls back to A* navigation between ball-seeking behaviors

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
    │   │   ├── task2.py             # Navigation + obstacle avoidance
    │   │   ├── task2_bonus.py       # Random waypoint patrol
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
    └── sim_utils/
        └── sim_utils/
            └── red_ball_control.py  # Ball motion controller for simulation
```

---

## Key Algorithms

### A* Path Planning
Implemented from scratch using a priority queue. The heuristic is Euclidean distance to the goal in grid space. Supports 8-directional movement (cardinal + diagonal, diagonal cost = √2). If the start or goal is inside an obstacle, the nearest valid free cell within a configurable radius is used instead.

### Frontier-Based Exploration (Task 1)
Frontiers are identified by scanning every free cell and checking if any of its 8 neighbors is unknown (-1). Frontiers are ranked by:

```
cost = W_dist × euclidean_distance + W_power / local_area_gain
```

where `local_area_gain` is the fraction of unknown cells in a local window around the frontier — rewarding cells that would reveal large unexplored areas.

### Obstacle Inflation
A rectangular kernel is convolved over the obstacle cells to create a safety buffer. Task 1 inflates only known walls (leaving unknown cells accessible for exploration). Task 2 inflates all non-free cells.

### PID Path Follower (Polar Coordinates)
The path follower converts Cartesian error to polar coordinates (ρ, α, β) and applies:
- Linear speed proportional to ρ (distance to goal)
- Angular speed from a PID controller on α (heading error) + a β correction term for final orientation

### Dynamic Lookahead
The lookahead distance for waypoint selection scales with current speed:
```
lookahead = speed × lookahead_ratio + min_lookahead
```

### Line-of-Sight Shortcutting
Before following the next waypoint in the path, the robot checks if a straight line to the final goal is clear using Bresenham's line algorithm. If clear, it shortcuts directly to the goal.

### Wall Follower (PID)
Maintains a desired distance from the right wall. Uses the 45° angled LiDAR reading as the primary control input (more stable than the 90° side reading). Stops and turns left when an obstacle appears in front.

### Ball Detection (Task 3)
Uses OpenCV HSV color segmentation with tuned masks for red, green, and blue. Contours are filtered by area and circularity. The angular error of the ball's center from the image center drives a PID angular controller. Estimated distance is computed from the ball's known physical diameter and apparent pixel size.

---

## Configuration & Tuning

Key parameters in the task files (adjust as needed):

| Parameter | Default | Description |
|---|---|---|
| `speed_max` | 0.31 m/s | Maximum linear speed |
| `rotspeed_max` | 1.9 rad/s | Maximum angular speed |
| `goal_tolerance` | 0.1 m | Distance to consider goal reached |
| `inflation_kernel_size` | 4–10 cells | Obstacle inflation radius |
| `min_frontier_distance` | 0.6 m | Ignore frontiers closer than this |
| `Frontier_W_dist` | 1.0 | Weight for distance in frontier cost |
| `Frontier_W_power` | 3.0 | Weight for information gain in frontier cost |
| `k_rho` | 0.8608 | Proportional gain for linear speed |
| `kp_angular` | 2.0747 | Proportional gain for angular PID |
| `min_front_obstacle_distance` | 0.35 m | Trigger obstacle avoidance |
| `retreat_distance` | 0.25 m | How far to reverse when obstacle detected |

---

## Authors

Phillipp Gery — Purdue University ECE