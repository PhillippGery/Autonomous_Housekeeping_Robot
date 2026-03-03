# Extracted from monolithic nodes. No logic changes.

import math
import numpy as np
from math import sqrt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlebot3_gazebo.common.graph_utils import AStar
from turtlebot3_gazebo.common.map_utils import SLAMMapProcessor

# --- Constants ---
DEFAULT_FRONTIER_W_DIST = 1.0    # Cost weight for distance to frontier
DEFAULT_FRONTIER_W_POWER = 3.0   # Cost weight for information power gain
DEFAULT_MIN_FRONTIER_DIST = 0.6  # Minimum useful frontier distance (meters)
DEFAULT_SEARCH_RADIUS_CELLS = 7  # Cells radius for area gain calculation
DEFAULT_MIN_FREE_NEIGHBORS = 5   # Min free cell neighbors for a valid frontier
DEFAULT_INFLATION_KERNEL = 4     # Inflation kernel size for SLAM map
# Path follower gains
K_RHO = 0.8608
KP_ANGULAR = 2.0747
KI_ANGULAR = 0.1692
KD_ANGULAR = -0.02
K_BETA = -0.1
KP_FINAL_YAW = 0.8
MAX_ANGLE_ALPHA_TO_STARTDRIVE = 55.0 / 180.0 * math.pi  # radians
# Speed limits
SPEED_MAX = 0.31
ROTSPEED_MAX = 1.9
GOAL_TOLERANCE = 0.1
MIN_LOOKAHEAD_DIST = 0.2
LOOKAHEAD_RATIO = 0.5
# Obstacle avoidance
MIN_FRONT_OBS_DIST = 0.35
MIN_BACK_OBS_DIST = 0.25
SAFETY_DIST = 0.2
RETREAT_DISTANCE = -0.25
RETREAT_SPEED = -0.15
# Narrow space detection
MIN_SIDE_DISTANCE = 0.7
MIN_TIME_IN_NARROW_SPACE_SEC = 2.0
# Exploration completion threshold
EXPLORATION_COMPLETE_PERCENT = 0.06
# Map significantly grown threshold (cells)
MAP_GROWTH_THRESHOLD = 20
# Timer rate
TIMER_RATE_HZ = 10.0
# PID integral anti-windup clamp
INTEGRAL_CLAMP = 1.0


class WallFollower:
    """PID-based wall-following algorithm, maintaining a fixed distance from the right wall."""

    def __init__(self, logger, desired_distance=0.5, kp=1.0, ki=0.01, kd=0.2,
                 max_angular_speed=1.2, max_linear_speed=0.8):
        """Initialise the WallFollower with PID gains and speed limits."""
        self.logger = logger
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None
        self.obstactle_infront = False
        self.desired_distance = desired_distance
        self.safety_distance = 0.4
        self.safety_Side_distance = 0.4
        self.forward_speed = max_linear_speed
        self.turning_speed = 1.2
        self.max_angular_speed = max_angular_speed

    def compute_velocities(self, scan_msg, current_time):
        """Compute linear and angular speeds from a LaserScan to follow the right wall."""
        if self.last_time is None:
            self.last_time = current_time
            return 0.0, 0.0

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt == 0.0:
            return 0.0, 0.0

        ranges = np.array(scan_msg.ranges)
        ranges[np.isinf(ranges)] = np.nan
        ranges[ranges == 0.0] = np.nan

        front_slice = np.concatenate((ranges[0:20], ranges[345:360]))
        right_slice = ranges[260:295]
        angled_slice = ranges[310:320]

        try:
            front_dist = np.nanmin(front_slice)
            right_dist = np.nanmin(right_slice)
            angled_dist = np.nanmin(angled_slice)
        except ValueError:
            self.logger.warn("laser readings are 'nan'. Skipping loop.", throttle_duration_sec=1)
            return 0.0, 0.0

        if not np.isnan(angled_dist) and right_dist > self.safety_Side_distance:
            control_dist = angled_dist / sqrt(2)
        elif not np.isnan(right_dist) and right_dist < 1.5:
            control_dist = right_dist
        else:
            self.logger.info("No right wall found", throttle_duration_sec=1)
            linear_speed = self.forward_speed * 0.5
            if np.isnan(angled_dist):
                angular_speed = -self.turning_speed * 0.1
            else:
                angular_speed = -self.turning_speed * 0.0
            return linear_speed, angular_speed

        if front_dist < self.safety_distance:
            self.logger.warn("Obstacle in front! Turning left.", throttle_duration_sec=1)
            linear_speed = 0.0
            angular_speed = self.turning_speed
            self.reset_pid()
        else:
            kp_fw = np.clip((front_dist - self.safety_distance) / 1.0, 0.4, 1.0)
            linear_speed = self.forward_speed * kp_fw

            error = self.desired_distance - control_dist

            self.integral += error * dt
            self.integral = np.clip(self.integral, -1.0, 1.0)
            derivative = (error - self.prev_error) / dt

            pid_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            angular_speed = np.clip(pid_output, -self.max_angular_speed, self.max_angular_speed)

            self.prev_error = error

        return linear_speed, angular_speed

    def reset_pid(self):
        """Reset PID state when switching away from wall follower."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None


class FrontierExplorer(Node):
    """Autonomously explores and maps the environment using frontier-based exploration."""

    def __init__(self):
        """Initialise FrontierExplorer: SLAM map processor, TF2, subscriptions, publishers."""
        super().__init__('frontier_explorer')

        self.declare_parameter('frontier_w_dist', DEFAULT_FRONTIER_W_DIST)
        self.declare_parameter('frontier_w_power', DEFAULT_FRONTIER_W_POWER)
        self.declare_parameter('min_frontier_distance', DEFAULT_MIN_FRONTIER_DIST)
        self.declare_parameter('search_radius_cells', DEFAULT_SEARCH_RADIUS_CELLS)
        self.declare_parameter('min_free_neighbors', DEFAULT_MIN_FREE_NEIGHBORS)
        self.declare_parameter('inflation_kernel_size', DEFAULT_INFLATION_KERNEL)

        self.Frontier_W_dist = self.get_parameter('frontier_w_dist').get_parameter_value().double_value
        self.Frontier_W_power = self.get_parameter('frontier_w_power').get_parameter_value().double_value
        self.min_frontier_distance = self.get_parameter('min_frontier_distance').get_parameter_value().double_value
        self.search_radius_cells = self.get_parameter('search_radius_cells').get_parameter_value().integer_value
        self.min_free_neighbors_for_frontier = self.get_parameter('min_free_neighbors').get_parameter_value().integer_value
        self.inflation_kernel_size = self.get_parameter('inflation_kernel_size').get_parameter_value().integer_value

        # State
        self.path = Path()
        self.goal_pose = None
        self.ttbot_pose = None
        self.start_time = 0.0
        self.state = 'IDLE'
        self.obstacle_state = 'CLEAR'
        self.retreat_distance = RETREAT_DISTANCE
        self.current_retreat_distance = 0.0
        self.obstacle_behind = False
        self.safety_dist_critical = False
        self.map_initialized = False
        self.raw_map_data_array = None
        self.rejected_goals_grid = []
        self.Frotier_Counter = 0
        self.Contrains_Relaxed = False
        self.FinalGoal_Origin_Set = False
        self.last_known_cells_count = 0
        self.FLg_NO_Angle_Alignment = True
        self.In_narrow_space = False
        self.enter_time = None
        self.scan_msg = None
        self.frontier_points = []
        self.exploration_start_time = 0.0
        self.last_commanded_speed = 0.0
        self.shortcut_active = False
        self.current_goal = None
        self.current_path_idx = 0
        self.integral_error_angular = 0.0
        self.previous_error_angular = 0.0

        # PID and speed parameters
        self.k_rho = K_RHO
        self.kp_angular = KP_ANGULAR
        self.ki_angular = KI_ANGULAR
        self.kd_angular = KD_ANGULAR
        self.k_beta = K_BETA
        self.kp_final_yaw = KP_FINAL_YAW
        self.max_angle_alpha_to_startdrive = MAX_ANGLE_ALPHA_TO_STARTDRIVE
        self.speed_max = SPEED_MAX
        self.rotspeed_max = ROTSPEED_MAX
        self.goal_tolerance = GOAL_TOLERANCE
        self.min_lookahead_dist = MIN_LOOKAHEAD_DIST
        self.lookahead_ratio = LOOKAHEAD_RATIO
        self.use_dynamic_lookahead = True
        self.use_line_of_sight_check = True
        self.max_dist_alternate_Ponit = 1.5
        self.yaw_tolerance = 0.1
        self.min_front_obstacle_distance = MIN_FRONT_OBS_DIST
        self.min_back_obstacle_distance = MIN_BACK_OBS_DIST
        self.safety_distance = SAFETY_DIST
        self.retreat_speed = RETREAT_SPEED
        self.min_side_distance = MIN_SIDE_DISTANCE
        self.min_time_in_narrow_space = MIN_TIME_IN_NARROW_SPACE_SEC

        self.inflation_kernel = self.rect_kernel(self.inflation_kernel_size, 1)
        self.map_processor = SLAMMapProcessor()
        self.get_logger().info("SLAM processor initialized. Waiting for /map data...")

        self.wall_follower = WallFollower(
            self.get_logger(),
            desired_distance=0.5,
            kp=1.2,
            ki=0.0,
            kd=0.4,
            max_angular_speed=1.3,
            max_linear_speed=0.2
        )

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_frame = 'map'
        self.odom_frame = 'odom'

        # Subscriptions
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self._goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self._ttbot_pose_cbk, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cbk, 10)
        self.create_subscription(LaserScan, '/scan', self._check_for_obstacles, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cbk, 1)
        self.create_subscription(Bool, '/goal_reached', self._goal_reached_cbk, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time', 10)

        self.rate = TIMER_RATE_HZ
        self.timer = self.create_timer(1.0 / self.rate, self.run_loop)

    def _goal_reached_cbk(self, data):
        """Trigger next frontier selection when the current goal is reached."""
        if data.data:
            self.get_logger().info("Goal reached signal received. Selecting next frontier.")
            self._select_and_set_goal(self.frontier_points)

    def _goal_pose_cbk(self, data):
        """Receive goal, plan path, and publish it."""
        if not self.map_initialized:
            self.get_logger().warn("Map not yet initialized by SLAM. Cannot plan path.")
            return

        self.goal_pose = data

        if self.ttbot_pose is None:
            self.get_logger().warn("Cannot plan path, robot pose is not yet available.")
            return

        self.path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)

        if self.goal_pose:
            start_world = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
            end_grid = self._world_to_grid(start_world)
            grid_name = f"{end_grid[0]},{end_grid[1]}"
            self.rejected_goals_grid.append(grid_name)
            self.get_logger().info(f"Rejected goals: {self.rejected_goals_grid}")

        if self.path.poses:
            self.path_pub.publish(self.path)
            self.current_path_idx = 0
            self.shortcut_active = False
            self.state = 'ASTARPATH_FOLLOWING'
            self.obstacle_state = 'CLEAR'
            self.integral_error_angular = 0.0
            self.previous_error_angular = 0.0
        else:
            self.get_logger().warn("A* failed to find a path to the goal.")
            self.move_ttbot(0.0, 0.0)

    def _ttbot_pose_cbk(self, data):
        """Update internal robot pose from /pose topic."""
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose = data.pose.pose
        self.ttbot_pose = pose_stamped

    def _odom_cbk(self, data: Odometry):
        """Receive odometry and transform to the /map frame via TF2."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                data.child_frame_id,
                rclpy.time.Time()
            )
        except Exception:
            return

        corrected_pose = PoseStamped()
        corrected_pose.header.frame_id = self.map_frame
        corrected_pose.pose.position.x = t.transform.translation.x
        corrected_pose.pose.position.y = t.transform.translation.y
        corrected_pose.pose.orientation = t.transform.rotation

        self.ttbot_pose = corrected_pose

    def _map_cbk(self, data):
        """Process live SLAM OccupancyGrid: inflate walls, rebuild A* graph, find frontiers."""
        if self.map_processor is None:
            self.exploration_start_time = self.get_clock().now().nanoseconds * 1e-9
            return

        self.map_processor.map.resolution = data.info.resolution
        self.map_processor.map.origin[0] = data.info.origin.position.x
        self.map_processor.map.origin[1] = data.info.origin.position.y
        self.map_processor.map.height = data.info.height
        self.map_processor.map.width = data.info.width

        H = data.info.height
        W = data.info.width

        map_2d = np.array(data.data, dtype=np.int8).reshape(H, W)

        self.raw_map_data_array = np.flipud(map_2d)

        wall_array_raw = np.zeros_like(map_2d, dtype=int)
        wall_array_raw[map_2d == 100] = 1
        current_wall_array = np.flipud(wall_array_raw)

        current_costmap = np.zeros_like(map_2d, dtype=int)
        current_costmap[(map_2d == 100) | (map_2d == -1)] = 1
        current_costmap = np.flipud(current_costmap)

        self.map_processor.inf_map_img_array = np.copy(current_costmap)

        inflation_kernel_matrix = self.rect_kernel(self.inflation_kernel_size * 2 + 1, 1)

        obstacle_indices = np.where(current_wall_array == 1)
        for i, j in zip(*obstacle_indices):
            self.map_processor._inflate_obstacle(
                inflation_kernel_matrix,
                self.map_processor.inf_map_img_array,
                i, j,
                absolute=True
            )

        self.map_processor.inf_map_img_array[self.map_processor.inf_map_img_array > 0] = 1

        self.map_processor.get_graph_from_map()
        self.frontier_points = self._find_frontiers()

        current_known_cells = np.sum(
            (self.raw_map_data_array == 0) | (self.raw_map_data_array == 100)
        )
        if current_known_cells > self.last_known_cells_count + MAP_GROWTH_THRESHOLD:
            if len(self.rejected_goals_grid) > 0:
                self.get_logger().info(
                    f"Map grown significantly ({current_known_cells} vs "
                    f"{self.last_known_cells_count}). Resetting {len(self.rejected_goals_grid)} rejected goals."
                )
                self.rejected_goals_grid = []
        self.last_known_cells_count = current_known_cells

        if not self.map_initialized:
            self.get_logger().info(f"Initial SLAM map received (H:{H}, W:{W}). A* graph built.")
            self.map_initialized = True

    def run_loop(self):
        """Main control loop at TIMER_RATE_HZ: manage exploration, path following, wall following."""
        self.get_logger().info(f"Current State: {self.state}", throttle_duration_sec=4.0)

        if self.state == 'MAP_EXPLORED':
            self.get_logger().info("Map fully explored. Stopping robot.", throttle_duration_sec=5.0)
            self.move_ttbot(0.0, 0.0)

        elif self.goal_pose is None or self.state == 'IDLE':
            self.get_logger().info(
                "Current mission complete or no goal. Initiating Frontier Search.",
                throttle_duration_sec=5.0
            )
            if self.map_initialized:
                self._select_and_set_goal(self.frontier_points)
            return

        elif self.obstacle_state == 'RETREATING':
            dt = 1.0 / self.rate
            distance_moved = abs(self.retreat_speed * dt)

            if self.current_retreat_distance < abs(self.retreat_distance):
                self.move_ttbot(self.retreat_speed, 0.0)
                self.current_retreat_distance += distance_moved
                if self.obstacle_behind:
                    self.obstacle_state = 'CLEAR'
                    self.state = 'IDLE'
                    self.get_logger().warn("Obstacle still behind during retreat", throttle_duration_sec=1.0)
                return
            else:
                self.move_ttbot(0.0, 0.0)
                self.get_logger().info("Retreat complete. Transitioning to REPLANNING.")
                self.obstacle_state = 'CLEAR'
                self.state = 'IDLE'
                self.FinalGoal_Origin_Set = False
                return

        elif self.state == 'ASTARPATH_FOLLOWING':
            final_goal_pose = self.path.poses[-1]

            self.path_optimization(final_goal_pose)

            speed, heading = self.path_follower(self.ttbot_pose, self.current_goal)
            self.move_ttbot(speed, heading)
            self.last_commanded_speed = speed

            self.check_goal_alignment(final_goal_pose)

        elif self.state == 'WALL_FOLLOWING':
            current_time = self.get_clock().now()
            linear_speed, angular_speed = self.wall_follower.compute_velocities(self.scan_msg, current_time)
            self.move_ttbot(float(linear_speed), float(angular_speed))

        else:
            self.get_logger().warn(f"Unknown state: {self.state}. Stopping robot for safety.")
            self.move_ttbot(0.0, 0.0)

    def a_star_path_planner(self, start_pose, end_pose):
        """Plan a path from start_pose to end_pose using A* on the current SLAM map."""
        path = Path()

        if not self.map_initialized:
            self.get_logger().error("Map not available for planning.")
            return Path()

        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        start_world = (start_pose.pose.position.x, start_pose.pose.position.y)
        end_world = (end_pose.pose.position.x, end_pose.pose.position.y)

        start_grid = self._world_to_grid(start_world)
        end_grid = self._world_to_grid(end_world)

        start_name = f"{start_grid[0]},{start_grid[1]}"
        end_name = f"{end_grid[0]},{end_grid[1]}"

        is_start_valid = start_name in self.map_processor.map_graph.g
        is_end_valid = end_name in self.map_processor.map_graph.g

        if not is_start_valid:
            self.get_logger().warn(f"start pose {start_name} is NOT valid. taking closest point.")
            min_dist_sq = float('inf')
            max_dist_sq = (self.max_dist_alternate_Ponit / self.map_processor.map.resolution) ** 2
            closest_node_name = None
            original_strt_y, original_strt_x = start_grid

            for valid_node_name in self.map_processor.map_graph.g:
                node_y, node_x = map(int, valid_node_name.split(','))
                dist_sq = (node_x - original_strt_x) ** 2 + (node_y - original_strt_y) ** 2
                if dist_sq < min_dist_sq and dist_sq <= max_dist_sq:
                    min_dist_sq = dist_sq
                    closest_node_name = valid_node_name

            if closest_node_name:
                self.get_logger().info(f"New start set to the closest valid point: {closest_node_name}")
                start_name = closest_node_name
                start_grid = tuple(map(int, closest_node_name.split(',')))
            else:
                self.get_logger().error("Could not find any valid nodes in the map. Planning failed.")
                return Path()

        if not is_end_valid:
            self.get_logger().warn(f"Goal pose {end_name} is NOT valid. taking closest point.")
            min_dist_sq = float('inf')
            max_dist_sq = (self.max_dist_alternate_Ponit / self.map_processor.map.resolution) ** 2
            closest_node_name = None
            original_end_y, original_end_x = end_grid

            for valid_node_name in self.map_processor.map_graph.g:
                node_y, node_x = map(int, valid_node_name.split(','))
                dist_sq = (node_x - original_end_x) ** 2 + (node_y - original_end_y) ** 2
                if dist_sq < min_dist_sq and dist_sq <= max_dist_sq:
                    min_dist_sq = dist_sq
                    closest_node_name = valid_node_name

            if closest_node_name:
                self.get_logger().info(f"New goal set to the closest valid point: {closest_node_name}")
                end_name = closest_node_name
                end_grid = tuple(map(int, closest_node_name.split(',')))
            else:
                self.get_logger().error("Could not find any valid nodes in the map. Planning failed.")
                return Path()

        start_node = self.map_processor.map_graph.g[start_name]
        end_node = self.map_processor.map_graph.g[end_name]

        astar_solver = AStar(self.map_processor.map_graph)

        for name in astar_solver.h.keys():
            node_grid = tuple(map(int, name.split(',')))
            astar_solver.h[name] = math.sqrt(
                (end_grid[0] - node_grid[0]) ** 2 + (end_grid[1] - node_grid[1]) ** 2
            )

        path_names, path_dist = astar_solver.solve(start_node, end_node)

        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        if path_names:
            self.get_logger().info(f"A* found a path of length {len(path_names)}")
            self.state = 'ASTARPATH_FOLLOWING'
            for name in path_names:
                grid_coords = tuple(map(int, name.split(',')))
                world_coords = self._grid_to_world(grid_coords)

                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = world_coords[0]
                pose.pose.position.y = world_coords[1]
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        else:
            self.get_logger().warn("A* failed to find a path.")
            self.move_ttbot(0.0, 0.0)

        astar_time = Float32()
        astar_time.data = float(self.get_clock().now().nanoseconds * 1e-9 - self.start_time)
        self.calc_time_pub.publish(astar_time)

        return path

    def get_path_idx(self, path, vehicle_pose):
        """Return the dynamic-lookahead index in path from the robot's current position."""
        min_dist = float('inf')
        closest_idx = 0
        for i, pose in enumerate(path.poses):
            dx = pose.pose.position.x - vehicle_pose.pose.position.x
            dy = pose.pose.position.y - vehicle_pose.pose.position.y
            dist = math.sqrt(dx ** 2 + dy ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        if self.use_dynamic_lookahead:
            lookahead_dist = self.last_commanded_speed * self.lookahead_ratio + self.min_lookahead_dist
        else:
            lookahead_dist = self.min_lookahead_dist

        for i in range(closest_idx, len(path.poses)):
            dx = path.poses[i].pose.position.x - vehicle_pose.pose.position.x
            dy = path.poses[i].pose.position.y - vehicle_pose.pose.position.y
            dist = math.sqrt(dx ** 2 + dy ** 2)
            if dist > lookahead_dist:
                return i

        return len(path.poses) - 1

    def _is_path_clear(self, start_grid, end_grid):
        """Check straight-line clearance between two grid points using Bresenham's algorithm."""
        x0, y0 = start_grid[1], start_grid[0]
        x1, y1 = end_grid[1], end_grid[0]

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        map_array = self.map_processor.inf_map_img_array
        h, w = map_array.shape

        while True:
            if not (0 <= y0 < h and 0 <= x0 < w) or map_array[y0, x0] == 1:
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return True

    def path_follower(self, vehicle_pose, current_goal_pose):
        """PID path follower in polar coordinates; returns (speed, heading)."""
        dt = 1.0 / self.rate
        self.integral_error_angular = 0.0
        self.previous_error_angular = 0.0

        robot_x = vehicle_pose.pose.position.x
        robot_y = vehicle_pose.pose.position.y
        quat = vehicle_pose.pose.orientation

        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        robot_theta = math.atan2(siny_cosp, cosy_cosp)

        goal_x = current_goal_pose.pose.position.x
        goal_y = current_goal_pose.pose.position.y

        delta_x = goal_x - robot_x
        delta_y = goal_y - robot_y

        rho = math.sqrt(delta_x ** 2 + delta_y ** 2)
        alpha = -robot_theta + math.atan2(delta_y, delta_x)

        if alpha > math.pi:
            alpha -= 2 * math.pi
        elif alpha < -math.pi:
            alpha += 2 * math.pi

        beta = -robot_theta - alpha

        p_term = self.kp_angular * alpha

        self.integral_error_angular += alpha * dt
        self.integral_error_angular = np.clip(self.integral_error_angular, -INTEGRAL_CLAMP, INTEGRAL_CLAMP)
        i_term = self.ki_angular * self.integral_error_angular

        derivative_error = (alpha - self.previous_error_angular) / dt
        d_term = self.kd_angular * derivative_error

        self.previous_error_angular = alpha

        if abs(alpha) > self.max_angle_alpha_to_startdrive:
            speed = 0.0
        else:
            speed = min(self.k_rho * rho, self.speed_max)

        heading = p_term + i_term + d_term + self.k_beta * beta
        heading = np.clip(heading, -self.rotspeed_max, self.rotspeed_max)

        return speed, heading

    def move_ttbot(self, speed, heading):
        """Publish a Twist command on cmd_vel."""
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def check_goal_alignment(self, final_goal_pose):
        """Check if robot reached final goal position+orientation; transition to IDLE if so."""
        if self.goal_pose is None:
            return

        dx = final_goal_pose.pose.position.x - self.ttbot_pose.pose.position.x
        dy = final_goal_pose.pose.position.y - self.ttbot_pose.pose.position.y
        dist_to_final_goal = math.sqrt(dx ** 2 + dy ** 2)

        if dist_to_final_goal < self.goal_tolerance:
            goal_q = self.goal_pose.pose.orientation
            goal_siny_cosp = 2 * (goal_q.w * goal_q.z + goal_q.x * goal_q.y)
            goal_cosy_cosp = 1 - 2 * (goal_q.y * goal_q.y + goal_q.z * goal_q.z)
            goal_yaw = math.atan2(goal_siny_cosp, goal_cosy_cosp)

            robot_q = self.ttbot_pose.pose.orientation
            robot_siny_cosp = 2 * (robot_q.w * robot_q.z + robot_q.x * robot_q.y)
            robot_cosy_cosp = 1 - 2 * (robot_q.y * robot_q.y + robot_q.z * robot_q.z)
            robot_yaw = math.atan2(robot_siny_cosp, robot_cosy_cosp)

            yaw_error = goal_yaw - robot_yaw

            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            if abs(yaw_error) < self.yaw_tolerance or self.FLg_NO_Angle_Alignment:
                self.get_logger().info("Goal reached and aligned to Goal Pose!")
                self.move_ttbot(0.0, 0.0)
                self.path = Path()
                self.goal_pose = None
                self.state = 'IDLE'
                self.last_commanded_speed = 0.0
                self.integral_error_angular = 0.0
                self.previous_error_angular = 0.0
                self.Frotier_Counter += 1
                self.get_logger().info(f"Frontier Goals Reached So Far: {self.Frotier_Counter}")
                return
            else:
                speed = 0.0
                heading = self.kp_final_yaw * yaw_error
                heading = np.clip(heading, -self.rotspeed_max, self.rotspeed_max)
                self.move_ttbot(speed, heading)
                return

    def path_optimization(self, final_goal_pose):
        """Use line-of-sight shortcut if clear; otherwise select lookahead waypoint."""
        self.current_goal = None

        if self.use_line_of_sight_check and not self.shortcut_active:
            start_grid = self._world_to_grid(
                (self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y)
            )
            end_grid = self._world_to_grid(
                (final_goal_pose.pose.position.x, final_goal_pose.pose.position.y)
            )
            if self._is_path_clear(start_grid, end_grid):
                self.current_goal = final_goal_pose
                self.shortcut_active = True
                self.get_logger().info("Path is clear. Taking a shortcut to the final goal.")

        if self.shortcut_active:
            self.current_goal = final_goal_pose
        else:
            idx = self.get_path_idx(self.path, self.ttbot_pose)
            self.current_goal = self.path.poses[idx]
        return self

    def _world_to_grid(self, world_coords):
        """Convert world coordinates to grid indices using current SLAM map metadata."""
        origin_x = self.map_processor.map.origin[0]
        origin_y = self.map_processor.map.origin[1]
        resolution = self.map_processor.map.resolution
        map_height_pixels = self.map_processor.map.height

        relative_x = world_coords[0] - origin_x
        relative_y = world_coords[1] - origin_y

        grid_x = int(relative_x / resolution)
        grid_y = map_height_pixels - 1 - int(relative_y / resolution)

        return (grid_y, grid_x)

    def _grid_to_world(self, grid_coords):
        """Convert grid indices to world coordinates using current SLAM map metadata."""
        origin_x = self.map_processor.map.origin[0]
        origin_y = self.map_processor.map.origin[1]
        resolution = self.map_processor.map.resolution
        map_height_pixels = self.map_processor.map.height

        grid_y, grid_x = grid_coords

        unflipped_grid_y = map_height_pixels - 1 - grid_y

        world_x = (grid_x + 0.5) * resolution + origin_x
        world_y = (unflipped_grid_y + 0.5) * resolution + origin_y

        return (world_x, world_y)

    def rect_kernel(self, size, value):
        """Return a rectangular kernel of ones of the given size."""
        return np.ones(shape=(size, size))

    def _check_for_obstacles(self, scan_msg):
        """Process LIDAR scan: detect obstacles, narrow spaces, and transition states."""
        if self.ttbot_pose is None or self.goal_pose is None:
            return

        self.scan_msg = scan_msg

        ranges = np.array(scan_msg.ranges)
        ranges[np.isinf(ranges)] = np.nan
        ranges[ranges == 0.0] = np.nan
        front_slice = np.concatenate((ranges[0:28], ranges[332:360]))
        right_slice = ranges[75:105]
        left_slice = ranges[255:285]
        back_slice = ranges[160:220]
        safety_slice = np.concatenate((ranges[0:90], ranges[270:360]))

        try:
            front_dist = np.nanmin(front_slice)
            right_dist = np.nanmin(right_slice)
            left_dist = np.nanmin(left_slice)
            back_dist = np.nanmin(back_slice)
            safety_dist = np.nanmin(safety_slice)
        except ValueError:
            self.get_logger().warn("laser readings are 'nan'. Skipping loop.", throttle_duration_sec=1)
            return

        narrow_space_detected = (right_dist < self.min_side_distance) and \
                                (left_dist < self.min_side_distance)

        if narrow_space_detected:
            if not self.In_narrow_space:
                self.In_narrow_space = True
                self.enter_time = self.get_clock().now()

            if self.state != 'WALL_FOLLOWING' and self.enter_time is not None:
                time_in_narrow_space = (self.get_clock().now() - self.enter_time).nanoseconds * 1e-9

                if time_in_narrow_space >= self.min_time_in_narrow_space:
                    is_area_unknown = self._check_area_ahead_unknown(
                        distance_m=1.0, width_m=0.5, required_percentage=40.0,
                        wall_threshold_percentage=5.0
                    )
                    if is_area_unknown:
                        self.get_logger().warn("Narrow space confirmed. Switching to WALL_FOLLOWING mode.")
                        self.state = 'WALL_FOLLOWING'
                        self.enter_time = None
        else:
            if self.In_narrow_space:
                self.In_narrow_space = False
                self.enter_time = None

            if self.state == 'WALL_FOLLOWING':
                self.get_logger().info("Exiting WALL_FOLLOWING mode. Resuming primary mission.")
                self.state = 'IDLE'

        obstacle_close = front_dist < self.min_front_obstacle_distance
        self.obstacle_behind = back_dist < self.min_back_obstacle_distance
        self.safety_dist_critical = safety_dist < self.safety_distance

        if obstacle_close and self.obstacle_state == 'CLEAR':
            self.state = 'OBSTACLE_AVOIDANCE'
            self.move_ttbot(0.0, 0.0)
            self.get_logger().warn(f"Obstacle detected at {front_dist:.2f} m! Initiating avoidance maneuver.")
            self.obstacle_state = 'RETREATING'
            self.current_retreat_distance = 0.0
        elif self.safety_dist_critical and self.obstacle_state == 'CLEAR':
            self.state = 'OBSTACLE_AVOIDANCE'
            self.move_ttbot(0.0, 0.0)
            self.get_logger().warn(
                f"Critical obstacle at {safety_dist:.2f} m in safety zone! Initiating avoidance."
            )
            self.obstacle_state = 'RETREATING'
            self.current_retreat_distance = 0.0

    def _find_frontiers(self):
        """Identify all frontier cells: free cells adjacent to unknown cells."""
        if self.raw_map_data_array is None:
            return []

        raw_data = self.raw_map_data_array
        H, W = raw_data.shape
        frontiers = []

        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        MIN_FREE_NEIGHBORS = self.min_free_neighbors_for_frontier

        for i in range(1, H - 1):
            for j in range(1, W - 1):
                if raw_data[i, j] == 0:
                    unknown_neighbor_count = 0
                    free_neighbor_count = 0
                    is_frontier = False

                    for di, dj in neighbors:
                        if 0 <= i + di < H and 0 <= j + dj < W:
                            neighbor_value = raw_data[i + di, j + dj]
                            if neighbor_value == -1:
                                is_frontier = True
                                unknown_neighbor_count += 1
                            elif neighbor_value == 0:
                                free_neighbor_count += 1

                    if is_frontier and free_neighbor_count >= MIN_FREE_NEIGHBORS:
                        frontiers.append((i, j, unknown_neighbor_count))

        return frontiers

    def _select_and_set_goal(self, frontier_points):
        """Select the best frontier goal based on distance and information gain heuristic."""
        if frontier_points and (self.ttbot_pose is not None):
            robot_x = self.ttbot_pose.pose.position.x
            robot_y = self.ttbot_pose.pose.position.y
            best_goal_pose = None
            min_cost = float('inf')

            for i, j, count in frontier_points:
                grid_name = f"{i},{j}"

                if grid_name in self.rejected_goals_grid:
                    continue

                world_coords = self._grid_to_world((i, j))
                goal_x, goal_y = world_coords[0], world_coords[1]

                distance = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)

                if distance < self.min_frontier_distance:
                    self.rejected_goals_grid.append(grid_name)
                    continue

                area_gain_cells = self._calculate_local_area_gain(i, j)
                if area_gain_cells == 0:
                    information_gain_term = float('inf')
                else:
                    information_gain_term = self.Frontier_W_power / area_gain_cells

                total_cost = (self.Frontier_W_dist * distance) + information_gain_term

                if total_cost < min_cost:
                    min_cost = total_cost

                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.pose.position.x = goal_x
                    goal_pose.pose.position.y = goal_y
                    goal_pose.pose.orientation.w = 1.0
                    best_goal_pose = goal_pose

            if best_goal_pose:
                self.get_logger().info(f"Selected new frontier goal. Cost: {min_cost:.2f}")
                self._goal_pose_cbk(best_goal_pose)
                self.FinalGoal_Origin_Set = False
                return

        frontier_points_count = len(frontier_points)

        known_cells = np.sum(
            (self.raw_map_data_array == 0) | (self.raw_map_data_array == 100)
        )
        unknown_percentage_vs_known = (frontier_points_count / known_cells) * 100.0

        self.get_logger().info(
            f"Unknown Map Percentage (vs Known): {unknown_percentage_vs_known:.2f}%",
            throttle_duration_sec=1.0
        )

        if unknown_percentage_vs_known < EXPLORATION_COMPLETE_PERCENT:
            self.get_logger().warn("Exploration Complete: Less than 0.06% of the map is unknown.")
            time_now = self.get_clock().now().nanoseconds * 1e-9
            time = (time_now - self.exploration_start_time - 18.0 * 60.0) / 60.0
            self.get_logger().info(f"Total Exploration Time: {time:.2f} minutes")
            self.get_logger().info(f"Total Frontier Goals Reached: {self.Frotier_Counter}")
            self.state = 'MAP_EXPLORED'

        elif self.Contrains_Relaxed and not self.FinalGoal_Origin_Set:
            self.get_logger().info(
                "All constraints relaxed and still no goal found. Setting origin as goal.",
                throttle_duration_sec=5.0
            )
            origin_pose = PoseStamped()
            origin_pose.header.frame_id = 'map'
            origin_pose.pose.position.x = 0.0
            origin_pose.pose.position.y = 0.0
            origin_pose.pose.orientation.w = 1.0
            self.rejected_goals_grid = []
            self._goal_pose_cbk(origin_pose)
            self.FinalGoal_Origin_Set = True

        else:
            self.get_logger().info(
                "No valid frontier goal found. All candidates rejected or unreachable.",
                throttle_duration_sec=5.0
            )
            self.get_logger().info(
                f"Frontier Points Available: {frontier_points_count}",
                throttle_duration_sec=5.0
            )
            self.get_logger().info(
                "Relaxing frontier selection criteria for next iteration.",
                throttle_duration_sec=5.0
            )
            self.min_frontier_distance = 0.0
            self.min_free_neighbors_for_frontier = 0
            self.search_radius_cells = 1
            self.frontier_points = self._find_frontiers()
            self.rejected_goals_grid = []
            self.state = 'IDLE'
            self.Contrains_Relaxed = True

    def _calculate_local_area_gain(self, i, j):
        """Count unknown cells within a square window around (i, j) to estimate information gain."""
        if self.raw_map_data_array is None:
            return 0

        search_radius_cells = self.search_radius_cells

        raw_data = self.raw_map_data_array
        H, W = raw_data.shape

        min_i = max(0, i - search_radius_cells)
        max_i = min(H, i + search_radius_cells + 1)
        min_j = max(0, j - search_radius_cells)
        max_j = min(W, j + search_radius_cells + 1)

        window = raw_data[min_i:max_i, min_j:max_j]

        num_cells = window.size

        unknown_count = np.sum(window == -1) / num_cells

        return unknown_count

    def _check_area_ahead_unknown(self, distance_m, width_m, required_percentage, wall_threshold_percentage):
        """Check fraction of unknown cells in a rectangular window ahead of the robot."""
        if self.ttbot_pose is None or self.raw_map_data_array is None:
            return False

        raw_data = self.raw_map_data_array
        H, W = raw_data.shape
        resolution = self.map_processor.map.resolution

        dist_cells = int(distance_m / resolution)
        width_cells = int(width_m / resolution)

        robot_i, robot_j = self._world_to_grid(
            (self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y)
        )

        quat = self.ttbot_pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        robot_theta = math.atan2(siny_cosp, cosy_cosp)

        target_x = self.ttbot_pose.pose.position.x + dist_cells * resolution * math.cos(robot_theta)
        target_y = self.ttbot_pose.pose.position.y + dist_cells * resolution * math.sin(robot_theta)

        target_i, target_j = self._world_to_grid((target_x, target_y))

        min_i = min(robot_i, target_i) - width_cells
        max_i = max(robot_i, target_i) + width_cells
        min_j = min(robot_j, target_j) - width_cells
        max_j = max(robot_j, target_j) + width_cells

        min_i = max(0, min_i)
        max_i = min(H, max_i)
        min_j = max(0, min_j)
        max_j = min(W, max_j)

        if max_i <= min_i or max_j <= min_j:
            return False

        window = raw_data[min_i:max_i, min_j:max_j]

        total_window_cells = window.size

        if total_window_cells == 0:
            return False

        occupied_cells = np.sum(window == 100)
        occupied_percentage = (occupied_cells / total_window_cells) * 100.0

        if occupied_percentage >= wall_threshold_percentage:
            self.get_logger().info(
                f"Ahead Occupied: {occupied_percentage:.1f}%. Detected wall, skipping unknown check.",
                throttle_duration_sec=1.0
            )
            return False

        unknown_cells = np.sum(window == -1)
        unknown_percentage = (unknown_cells / total_window_cells) * 100.0

        self.get_logger().info(
            f"Ahead Unknown: {unknown_percentage:.1f}% ({unknown_cells} cells)",
            throttle_duration_sec=1.0
        )

        return unknown_percentage >= required_percentage

    def destroy_node(self):
        """Cancel timers and destroy the node."""
        if hasattr(self, 'timer'):
            self.timer.cancel()
        super().destroy_node()


def main(args=None):
    """Entry point: spin FrontierExplorer until interrupted."""
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
