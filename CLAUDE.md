# Autonomous Housekeeping Robot — Claude Context

## Project Overview
ROS2 (Humble) simulation project for an autonomous housekeeping robot using TurtleBot3 Waffle in Gazebo. The robot autonomously explores, maps, navigates, and detects colored objects in a house environment.

## Workspace Layout
```
Autonomous_Housekeeping_Robot/          ← ROS2 workspace root
├── CLAUDE.md                           ← this file
└── src/
    ├── turtlebot3_gazebo/              ← main package (ament_cmake)
    │   ├── src/                        ← Python node scripts
    │   ├── launch/                     ← launch files
    │   ├── maps/                       ← pre-built map (map.yaml + map.pgm)
    │   ├── models/                     ← Gazebo model files
    │   ├── worlds/                     ← Gazebo world files
    │   ├── params/                     ← YAML parameter files
    │   ├── rviz/                       ← RViz config files
    │   ├── urdf/                       ← robot URDF
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── sim_utils/                      ← Python utility package (ament_python)
```

## Build & Run
```bash
cd ~/Autonomous_Housekeeping_Robot
colcon build --packages-select turtlebot3_gazebo
source install/setup.bash
```

Always `source install/setup.bash` after every build before launching.

## Node / Script Reference

| File (`src/`) | Class | ROS Node Name | Purpose |
|---|---|---|---|
| `slam_explorer.py` | `SlamExplorer` | `slam_explorer` | SLAM + autonomous frontier exploration |
| `map_navigator.py` | `MapNavigator` | `map_navigator` | AMCL localisation + A* navigation on pre-built map |
| `map_navigator_RRT_star.py` | `MapNavigatorRRTStar` | `map_navigator_rrt_star` | Same as above but uses RRT* planner (bonus variant) |
| `vision_navigator.py` | `VisionNavigator` | `vision_navigator` | A* navigation + OpenCV ball detection (red/green/blue) |
| `dynamic_obstacles.py` | `GazeboModelHandler` | `dynamic_obstacles` | Spawns and moves obstacles/cricket ball in Gazebo |
| `static_obstacles.py` | — | `static_obstacles` | Spawns static obstacle set |
| `spawn_objects.py` | — | `spawn_objects` | Spawns additional objects |

## Launch Files

### Mapping (task 1 equivalent)
```bash
ros2 launch turtlebot3_gazebo mapper.launch.py
# Optional: bonus:=true  → launches bonus Gazebo world instead of house
```
Starts: Gazebo house world + SLAM Toolbox (online async) + `slam_explorer` node.

### Navigation (task 2 / task 3 equivalent)
```bash
# Standard navigation with static obstacles
ros2 launch turtlebot3_gazebo navigator.launch.py static_obstacles:=true

# RRT* planner variant (bonus)
ros2 launch turtlebot3_gazebo navigator.launch.py static_obstacles:=true bonus:=true

# Vision navigator with dynamic obstacles
ros2 launch turtlebot3_gazebo navigator.launch.py spawn_objects:=true
```
Starts: Gazebo + map_server + AMCL + lifecycle_manager + appropriate navigator node.

Launch arguments:
- `static_obstacles` (default: false) — enables static obstacle spawner
- `bonus` (default: false) — switches from `map_navigator` to `map_navigator_RRT_star`
- `spawn_objects` (default: false) — spawns dynamic obstacles + starts `vision_navigator`
- `use_sim_time` (default: true)
- `use_rviz` (default: true)

## Key Technical Details

### Coordinate Frames
- `map` — fixed world frame (from SLAM or pre-built map)
- `odom` — local odometry frame (drifts over time)
- `base_footprint` — robot base frame
- TF2 is used to get the SLAM-corrected pose: `map → odom → base_footprint`

### Path Planning
- A* runs on an inflated OccupancyGrid
- Grid graph built from `map.pgm` (0.05 m/cell resolution)
- Obstacle inflation radius: 0.4 m (map_navigator), 0.4 m (slam_explorer)
- Dynamic lookahead path follower with PID angular control

### Frontier Exploration (slam_explorer)
- Subscribes to `/map` (OccupancyGrid from slam_toolbox)
- Frontier = free cell (value 0) adjacent to unknown cell (value -1)
- Goal selection: `cost = W_dist * distance + W_power / area_gain`
- Falls back to wall-following in narrow spaces

### Dynamic Obstacle Avoidance (map_navigator)
- Detects obstacles from `/scan` (LaserScan)
- Estimates obstacle world position, inflates costmap, replans
- States: `CLEAR → RETREATING → ALIGNING_TO_OBS → REPLANNING`

### Ball Detection (vision_navigator)
- Camera topic: `/camera/image_raw`
- OpenCV HSV masking for red, green, blue balls
- 3D ball position from camera + LIDAR fusion
- Publishes: `/bbox`, `/red_pos`, `/blue_pos`, `/green_pos`

## Map Files
Located at `src/turtlebot3_gazebo/maps/`:
- `map.yaml` — map metadata (resolution 0.05, origin [-5.51, -6.25, 0])
- `map.pgm` — occupancy grid image
- `sync_classroom_map.yaml` / `.pgm` — alternate classroom map

**Important:** Both files use lowercase names. Linux is case-sensitive — `Map.yaml` ≠ `map.yaml`.

## Important ROS Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/map` | OccupancyGrid | slam_toolbox | slam_explorer |
| `/amcl_pose` | PoseWithCovarianceStamped | amcl | map_navigator |
| `/pose` | PoseWithCovarianceStamped | — | slam_explorer |
| `/odom` | Odometry | gazebo | slam_explorer |
| `/scan` | LaserScan | gazebo | all navigators |
| `/cmd_vel` | Twist | navigators | gazebo |
| `/move_base_simple/goal` | PoseStamped | RViz | all navigators |
| `/camera/image_raw` | Image | gazebo | vision_navigator |
| `global_plan` | Path | navigators | RViz |
| `astar_time` | Float32 | navigators | — |

## Naming Conventions
- File names: `snake_case.py`
- Class names: `PascalCase`
- ROS node names: `snake_case` (no `_node` suffix)
- All scripts are installed via `install(PROGRAMS ...)` in CMakeLists.txt — they run directly as executables

## Known Issues / Notes
- `gzserver` logs "Non-unique names detected in link" — this is a pre-existing Gazebo world model issue, not a code bug, can be ignored
- `gzclient` logs "context mismatch in svga_surface_destroy" — VMware/GPU driver issue, harmless
- `map_server.yaml` in `params/` has a hardcoded path to `/final_project_ws/` — this file is NOT used by any launch file (navigator.launch.py inlines the map path directly); ignore it
- `src_fcn/` directory (AStar.py, wall_follower.py, etc.) is legacy code from a prior development phase — it is not wired into the build system and should be ignored

## Package Dependencies (package.xml)
ROS: `gazebo_ros_pkgs`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `rclpy`, `sensor_msgs`, `tf2`, `tf2_ros`, `vision_msgs`, `cv_bridge`
Python: `python3-numpy`, `python3-opencv`, `python3-pil`, `python3-yaml`
