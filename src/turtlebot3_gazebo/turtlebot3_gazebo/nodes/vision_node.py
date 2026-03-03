# Extracted from monolithic nodes. No logic changes.

import math
import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from turtlebot3_gazebo.common.graph_utils import AStar
from turtlebot3_gazebo.common.map_utils import MapProcessor

# --- Constants ---
# PID gains for ball-following angular controller
BF_PID_P_ANGULAR = 0.4
BF_PID_I_ANGULAR = 0.1
BF_PID_D_ANGULAR = 0.4
BF_P_LINEAR = 0.3

# Ball detection thresholds
BF_CIRCULARITY_THRESHOLD = 0.8    # minimum circularity ratio to accept contour as ball
MIN_BALL_AREA_PX = 50 ** 2        # pixels² — smallest area that counts as a ball
BALL_AREA_TH_PX = 300 ** 2        # pixels² — area at which low-circularity balls still count
BALL_DIAMETER_M = 0.3             # physical ball diameter (metres)

# Approach distances (metres)
TARGET_DISTANCE_BALL_M = 1.5      # desired stop distance from ball
MAX_DISTANCE_BALL_M = 4.0         # beyond this range the ball is ignored

# Camera
CAMERA_HFOV_DEGREES = 60.0        # horizontal field of view of the RGB camera
BF_MAX_LINEAR_SPEED = 0.2         # m/s — cap for ball-approach linear speed
BF_MAX_ANGULAR_SPEED = 1.0        # rad/s — cap for ball-approach angular speed

# Navigation controller gains
K_RHO = 0.8608
KP_ANGULAR = 2.0747
KI_ANGULAR = 0.1692
KD_ANGULAR = -0.02
K_BETA = -0.1
KP_FINAL_YAW = 0.8
MAX_ANGLE_ALPHA = 1.0             # radians — threshold above which linear speed is zeroed

# Navigation speed / tolerance
SPEED_MAX = 0.31                  # m/s
ROTSPEED_MAX = 1.9                # rad/s
GOAL_TOLERANCE_M = 0.2            # metres
YAW_TOLERANCE_RAD = 0.1           # radians

# Dynamic lookahead
MIN_LOOKAHEAD_DIST_M = 0.2
LOOKAHEAD_RATIO = 0.5

# Inflation
DEFAULT_INFLATION_KERNEL = 10
MAX_DIST_ALTERNATE_POINT_M = 1.0  # metres — fallback search radius for invalid poses

# Obstacle avoidance
MIN_FRONT_OBS_DIST_M = 0.35
MIN_BACK_OBS_DIST_M = 0.25
SAFETY_DIST_M = 0.2
RETREAT_DISTANCE_M = -0.20        # negative = backward
RETREAT_SPEED = -0.15             # m/s
MAX_OBS_DIAMETER_M = 0.4

# Timer
TIMER_RATE_HZ = 10.0


class VisionNavigator(Node):
    """Navigation node with OpenCV-based colored ball detection and A* path planning."""

    def __init__(self):
        """Initialise VisionNavigator: load map, set up camera/lidar, configure mission."""
        super().__init__('vision_navigator')

        pkg_share_path = get_package_share_directory('turtlebot3_gazebo')
        default_map_path = os.path.join(pkg_share_path, 'maps', 'map.yaml')
        self.declare_parameter('map_yaml_path', default_map_path)
        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value

        # Navigation state
        self.path = Path()
        self.goal_pose = None
        self.ttbot_pose = None
        self.start_time = 0.0
        self.state = 'IDLE'
        self.current_path_idx = 0
        self.shortcut_active = False
        self.last_commanded_speed = 0.0
        self.integral_error_angular = 0.0
        self.previous_error_angular = 0.0

        # Controller gains (from constants)
        self.k_rho = K_RHO
        self.kp_angular = KP_ANGULAR
        self.ki_angular = KI_ANGULAR
        self.kd_angular = KD_ANGULAR
        self.k_beta = K_BETA
        self.kp_final_yaw = KP_FINAL_YAW
        self.max_angle_alpha_to_startdrive = MAX_ANGLE_ALPHA
        self.speed_max = SPEED_MAX
        self.rotspeed_max = ROTSPEED_MAX
        self.goal_tolerance = GOAL_TOLERANCE_M
        self.yaw_tolerance = YAW_TOLERANCE_RAD
        self.min_lookahead_dist = MIN_LOOKAHEAD_DIST_M
        self.lookahead_ratio = LOOKAHEAD_RATIO
        self.use_dynamic_lookahead = True
        self.use_line_of_sight_check = True
        self.inflation_kernel_size = DEFAULT_INFLATION_KERNEL
        self.max_dist_alternate_point = MAX_DIST_ALTERNATE_POINT_M

        # Obstacle avoidance
        self.obstacle_state = 'CLEAR'
        self.min_front_obstacle_distance = MIN_FRONT_OBS_DIST_M
        self.min_back_obstacle_distance = MIN_BACK_OBS_DIST_M
        self.safety_distance = SAFETY_DIST_M
        self.retreat_distance = RETREAT_DISTANCE_M
        self.current_retreat_distance = 0.0
        self.retreat_speed = RETREAT_SPEED
        self.max_obstacle_diameter = MAX_OBS_DIAMETER_M
        self.obstacle_behind = False
        self.safety_dist_critical = False
        self.front_dist = float('inf')
        self.front_slice = np.array([])
        self.scan_msg = None
        self.ranges = np.array([])
        self.obstacle_pos_world = (0.0, 0.0)
        self.estimated_diameter = 0.0

        # Ball-following controller
        self.BF_pid_p_angular = BF_PID_P_ANGULAR
        self.BF_pid_i_angular = BF_PID_I_ANGULAR
        self.BF_pid_d_angular = BF_PID_D_ANGULAR
        self.BF_p_linear = BF_P_LINEAR
        self.BF_integral_angular = 0.0
        self.BF_previous_error_angular = 0.0
        self.BF_circularity_threshold = BF_CIRCULARITY_THRESHOLD
        self.min_Ball_area = MIN_BALL_AREA_PX
        self.Ball_area_Th = BALL_AREA_TH_PX
        self.Ball_diameter = BALL_DIAMETER_M
        self.Flg_apprach_Ball = False
        self.Target_Distance_Ball = TARGET_DISTANCE_BALL_M
        self.Max_Distance_Ball = MAX_DISTANCE_BALL_M
        self.camera_hfov_degrees = CAMERA_HFOV_DEGREES
        self.BF_max_linear_speed = BF_MAX_LINEAR_SPEED
        self.BF_max_angular_speed = BF_MAX_ANGULAR_SPEED

        # Ball detection state
        self.RED_BALL_Pos = None
        self.GREEN_BALL_Pos = None
        self.BLUE_BALL_Pos = None
        self.detected_color = 'UNKNOWN'
        self.Ball_heading_error_degrees = 0.0
        self.Camera_distance_to_object = 0.0
        self.Ball_pos_world = (0.0, 0.0)
        self.delay_timer = None
        self.Reset_Flg_apprach_Ball_timer = None
        self.last_time = None
        self.last_detection_time = self.get_clock().now()

        # HSV color thresholds
        self.lower_red_1 = np.array([0, 190, 70])
        self.upper_red_1 = np.array([10, 255, 255])
        self.lower_red_2 = np.array([170, 190, 70])
        self.upper_red_2 = np.array([180, 255, 255])
        self.lower_blue = np.array([100, 130, 30])
        self.upper_blue = np.array([135, 255, 255])
        self.lower_green = np.array([35, 130, 55])
        self.upper_green = np.array([80, 255, 255])

        # Mission waypoints — duplicate 'Room_5' keys intentionally preserved
        self.mission_waypoints = {
            'Room_1':         (8.0,  -3.0,  -math.pi / 2),
            'Room_2_UpTable': (8.5,   1.5,   math.pi),
            'Room_2_LowTable':(6.0,   3.0,   np.radians(90)),
            'Room_3':         (3.8,   3.7,   np.radians(240)),
            'Room_4':         (0.7,   2.5,   np.radians(90)),
            'Room_5':         (-3.3,  2.6,   -math.pi / 2),
            'Room_6':         (-4.6, -3.1,   np.radians(0)),
            'WP4_End':        (0.0,   0.0,   0.0),
        }
        self.mission_keys = list(self.mission_waypoints.keys())
        self.current_waypoint_index = 0

        # Map and planner
        self.get_logger().info(f"Loading map from '{self.map_yaml_path}' and building graph...")
        self.map_processor = MapProcessor(self.map_yaml_path)
        inflation_kernel = self.map_processor.rect_kernel(self.inflation_kernel_size, 1)
        self.map_processor.inflate_map(inflation_kernel)
        self.map_processor.get_graph_from_map()
        self.get_logger().info("Graph built successfully.")

        # Subscriptions
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self._goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._ttbot_pose_cbk, 10)
        self.create_subscription(LaserScan, '/scan', self._check_for_obstacles, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time', 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/custom_costmap', 1)
        self.bbox_publisher = self.create_publisher(BoundingBox2D, '/bbox', 10)
        self.red_pos_pub = self.create_publisher(Pose, '/red_pos', 10)
        self.blue_pos_pub = self.create_publisher(Pose, '/blue_pos', 10)
        self.green_pos_pub = self.create_publisher(Pose, '/green_pos', 10)

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.initial_pose_timer = self.create_timer(2.0, self.publish_initial_pose)

        self.bridge = CvBridge()
        self.rate = TIMER_RATE_HZ
        self.timer = self.create_timer(1.0 / self.rate, self.run_loop)

        self._publish_inflated_map()

    # ── Camera ──────────────────────────────────────────────────────────────

    def camera_callback(self, msg):
        """Process an incoming camera frame: detect balls and run visual servo controller."""
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt == 0:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        empty_mask = np.zeros_like(hsv_image[:, :, 0], dtype=np.uint8)

        if self.RED_BALL_Pos is not None:
            red_mask = empty_mask
        else:
            red_mask1 = cv2.inRange(hsv_image, self.lower_red_1, self.upper_red_1)
            red_mask2 = cv2.inRange(hsv_image, self.lower_red_2, self.upper_red_2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        blue_mask = empty_mask if self.BLUE_BALL_Pos is not None else \
            cv2.inRange(hsv_image, self.lower_blue, self.upper_blue)

        green_mask = empty_mask if self.GREEN_BALL_Pos is not None else \
            cv2.inRange(hsv_image, self.lower_green, self.upper_green)

        color_masks = {"RED": red_mask, "BLUE": blue_mask, "GREEN": green_mask}
        mask = red_mask | blue_mask | green_mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        potential_balls = []
        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                x, y, w, h = cv2.boundingRect(contour)
                if perimeter == 0 or h == 0:
                    continue
                circularity = (4 * np.pi * area) / (perimeter * perimeter)
                if circularity > self.BF_circularity_threshold and area > self.min_Ball_area:
                    potential_balls.append(contour)
                elif circularity > self.BF_circularity_threshold * 0.8 and area > self.Ball_area_Th:
                    potential_balls.append(contour)

        if potential_balls and not self.Flg_apprach_Ball:
            self.state = "BALL_TRACKING"
            self.last_detection_time = self.get_clock().now()
            largest_contour = max(potential_balls, key=cv2.contourArea)
            self.detected_color = self._get_contour_color(largest_contour, color_masks)
            self.get_logger().info(f"BALL Color: {self.detected_color}", throttle_duration_sec=5.0)

            x, y, w, h = cv2.boundingRect(largest_contour)
            centroid_x = x + w // 2
            image_width = cv_image.shape[1]
            image_center_x = image_width // 2

            bbox_msg = BoundingBox2D()
            bbox_msg.center.position.x = float(centroid_x)
            bbox_msg.center.position.y = float(y + h // 2)
            bbox_msg.center.theta = 0.0
            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)
            self.bbox_publisher.publish(bbox_msg)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            pixel_ratio = w / image_width
            alpha_radians = pixel_ratio * np.radians(self.camera_hfov_degrees)
            self.Camera_distance_to_object = self.Ball_diameter / (2.0 * math.tan(alpha_radians / 2.0))

            error_x_unnorm = centroid_x - image_center_x
            error_x = error_x_unnorm / image_center_x
            self.Ball_heading_error_degrees = error_x_unnorm * (self.camera_hfov_degrees / image_width)
            self.get_logger().info(
                f"Heading Error: {self.Ball_heading_error_degrees:.2f} degrees",
                throttle_duration_sec=5)

            self.BF_integral_angular += error_x
            derivative_angular = error_x - self.BF_previous_error_angular
            p_term = self.BF_pid_p_angular * error_x
            i_term = np.clip(self.BF_pid_i_angular * self.BF_integral_angular * dt, -0.5, 0.5)
            d_term = (self.BF_pid_d_angular * derivative_angular) / dt

            if abs(self.Ball_heading_error_degrees) < 0.8:
                angular_velocity = 0.0
                if self.delay_timer is None:
                    self.delay_timer = self.get_clock().now().nanoseconds * 1e-9
                if (self.get_clock().now().nanoseconds * 1e-9 - self.delay_timer) > 0.5:
                    self.get_logger().info("Object centered.", throttle_duration_sec=5)
                    self.BF_previous_error_angular = 0.0
                    self.BF_integral_angular = 0.0
                    self.update_ball_estimate()
                    self.delay_timer = None
                else:
                    self.get_logger().info("Centering... please wait.", throttle_duration_sec=5)
            else:
                angular_velocity = p_term + i_term + d_term
                self.BF_previous_error_angular = error_x
                self.delay_timer = None

            self.move_ttbot(0.0, -angular_velocity)
        else:
            self.get_logger().info("No object detected.", throttle_duration_sec=5)
            self.BF_previous_error_angular = 0.0
            self.BF_integral_angular = 0.0
            if self.state == 'BALL_TRACKING':
                self.state = 'IDLE'

        scale = 0.4
        width = int(cv_image.shape[1] * scale)
        height = int(cv_image.shape[0] * scale)
        resized_image = cv2.resize(cv_image, (width, height), interpolation=cv2.INTER_AREA)
        cv2.imshow("Object Detector", resized_image)
        cv2.waitKey(1)

    # ── Goal / pose callbacks ────────────────────────────────────────────────

    def _goal_pose_cbk(self, data):
        """Receive a new goal pose, plan A* path, and publish it."""
        self.goal_pose = data
        self.get_logger().info(
            'New goal received: {:.4f}, {:.4f}'.format(
                self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))
        if self.ttbot_pose is None:
            self.get_logger().warn("Cannot plan path, robot pose not yet available.")
            return
        self.path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
        if self.path.poses:
            self.path_pub.publish(self.path)
            self.current_path_idx = 0
            self.shortcut_active = False
        else:
            self.get_logger().warn("A* failed to find a path to the goal.")
            self.move_ttbot(0.0, 0.0)

    def _ttbot_pose_cbk(self, data):
        """Update the robot pose from AMCL."""
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose = data.pose.pose
        self.ttbot_pose = pose_stamped

    # ── A* planner ──────────────────────────────────────────────────────────

    def a_star_path_planner(self, start_pose, end_pose):
        """Plan an A* path from start_pose to end_pose; return a nav_msgs/Path."""
        path = Path()
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        start_world = (start_pose.pose.position.x, start_pose.pose.position.y)
        end_world = (end_pose.pose.position.x, end_pose.pose.position.y)

        start_grid = self._world_to_grid(start_world)
        end_grid = self._world_to_grid(end_world)
        start_name = f"{start_grid[0]},{start_grid[1]}"
        end_name = f"{end_grid[0]},{end_grid[1]}"

        if start_name not in self.map_processor.map_graph.g:
            self.get_logger().warn(f"Start pose {start_name} not valid. Finding closest point.")
            max_dist_sq = (self.max_dist_alternate_point / self.map_processor.map.resolution) ** 2
            min_dist_sq = float('inf')
            closest = None
            orig_y, orig_x = start_grid
            for vn in self.map_processor.map_graph.g:
                ny, nx = map(int, vn.split(','))
                d = (nx - orig_x) ** 2 + (ny - orig_y) ** 2
                if d < min_dist_sq and d <= max_dist_sq:
                    min_dist_sq = d
                    closest = vn
            if closest:
                start_name = closest
                start_grid = tuple(map(int, closest.split(',')))
                self.get_logger().info(f"New start: {start_name}")
            else:
                self.get_logger().error("No valid start node found. Planning failed.")
                return Path()

        if end_name not in self.map_processor.map_graph.g:
            self.get_logger().warn(f"Goal pose {end_name} not valid. Finding closest point.")
            max_dist_sq = (self.max_dist_alternate_point / self.map_processor.map.resolution) ** 2
            min_dist_sq = float('inf')
            closest = None
            orig_y, orig_x = end_grid
            for vn in self.map_processor.map_graph.g:
                ny, nx = map(int, vn.split(','))
                d = (nx - orig_x) ** 2 + (ny - orig_y) ** 2
                if d < min_dist_sq and d <= max_dist_sq:
                    min_dist_sq = d
                    closest = vn
            if closest:
                end_name = closest
                end_grid = tuple(map(int, closest.split(',')))
                self.get_logger().info(f"New goal: {end_name}")
            else:
                self.get_logger().error("No valid goal node found. Planning failed.")
                return Path()

        start_node = self.map_processor.map_graph.g[start_name]
        end_node = self.map_processor.map_graph.g[end_name]
        astar_solver = AStar(self.map_processor.map_graph)
        for name in astar_solver.h.keys():
            ng = tuple(map(int, name.split(',')))
            astar_solver.h[name] = math.sqrt(
                (end_grid[0] - ng[0]) ** 2 + (end_grid[1] - ng[1]) ** 2)

        path_names, path_dist = astar_solver.solve(start_node, end_node)
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        if path_names:
            self.get_logger().info(f"A* path length: {len(path_names)}")
            self.state = 'ASTARPATH_FOLLOWING'
            for name in path_names:
                gc = tuple(map(int, name.split(',')))
                wc = self._grid_to_world(gc)
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = wc[0]
                pose.pose.position.y = wc[1]
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        else:
            self.get_logger().warn("A* failed to find a path.")
            self.move_ttbot(0.0, 0.0)
            self.map_processor = MapProcessor(self.map_yaml_path)
            kernel = self.map_processor.rect_kernel(self.inflation_kernel_size, 1)
            self.map_processor.inflate_map(kernel)
            self.map_processor.get_graph_from_map()
            self._publish_inflated_map()
            self.get_logger().info("Map reset.")

        astar_time = Float32()
        astar_time.data = float(self.get_clock().now().nanoseconds * 1e-9 - self.start_time)
        self.calc_time_pub.publish(astar_time)
        self.get_logger().info(f"A* planning time: {astar_time.data:.4f} s")
        return path

    # ── Path following ───────────────────────────────────────────────────────

    def get_path_idx(self, path, vehicle_pose):
        """Return the lookahead index in path nearest the vehicle with dynamic lookahead."""
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
        """Return True if a Bresenham line between start_grid and end_grid is obstacle-free."""
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
        """Compute (speed, heading) using polar-coordinate PID controller (rho, alpha, beta)."""
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
        self.integral_error_angular = np.clip(self.integral_error_angular, -1.0, 1.0)
        i_term = self.ki_angular * self.integral_error_angular
        derivative_error = (alpha - self.previous_error_angular) / dt
        d_term = self.kd_angular * derivative_error
        self.previous_error_angular = alpha
        speed = 0.0 if abs(alpha) > self.max_angle_alpha_to_startdrive else \
            min(self.k_rho * rho, self.speed_max)
        heading = np.clip(
            p_term + i_term + d_term + self.k_beta * beta,
            -self.rotspeed_max, self.rotspeed_max)
        return speed, heading

    def move_ttbot(self, speed, heading):
        """Publish a Twist command to /cmd_vel."""
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    # ── Main control loop ────────────────────────────────────────────────────

    def run_loop(self):
        """10 Hz control loop: mission management, path following, obstacle avoidance."""
        self.get_logger().info(f"State: {self.state}", throttle_duration_sec=4.0)

        if self.ttbot_pose is None:
            self.publish_initial_pose()
            self.get_logger().warn("Waiting for robot pose...", throttle_duration_sec=5.0)
            return

        if (self.RED_BALL_Pos is not None and self.GREEN_BALL_Pos is not None
                and self.BLUE_BALL_Pos is not None):
            self.get_logger().warn("MISSION COMPLETE: All balls found.", throttle_duration_sec=5.0)
            self.get_logger().info(f"RED: {self.RED_BALL_Pos}", throttle_duration_sec=5.0)
            self.get_logger().info(f"BLUE: {self.BLUE_BALL_Pos}", throttle_duration_sec=5.0)
            self.get_logger().info(f"GREEN: {self.GREEN_BALL_Pos}", throttle_duration_sec=5.0)
            self.move_ttbot(0.0, 0.0)
            return

        if self.state == 'IDLE':
            if self.current_waypoint_index >= len(self.mission_keys):
                self.get_logger().warn("MISSION COMPLETE: All waypoints reached.")
                self.move_ttbot(0.0, 0.0)
                if (self.RED_BALL_Pos is None or self.GREEN_BALL_Pos is None
                        or self.BLUE_BALL_Pos is None):
                    self.get_logger().warn(
                        "Not all balls found yet, restarting waypoints...", throttle_duration_sec=5.0)
                    self.current_waypoint_index = 0
                return
            current_key = self.mission_keys[self.current_waypoint_index]
            x, y, yaw = self.mission_waypoints[current_key]
            self.goal_pose = PoseStamped()
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.pose.position.x = x
            self.goal_pose.pose.position.y = y
            self.goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            self.goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.get_logger().info(f"Moving to waypoint: {current_key}")
            self._goal_pose_cbk(self.goal_pose)
            return

        elif self.state == 'BALL_TRACKING':
            return

        elif self.obstacle_state == 'RETREATING':
            dt = 1.0 / self.rate
            distance_moved = abs(self.retreat_speed * dt)
            if self.current_retreat_distance < abs(self.retreat_distance):
                self.move_ttbot(self.retreat_speed, 0.0)
                self.current_retreat_distance += distance_moved
                if self.obstacle_behind:
                    self.obstacle_state = 'CLEAR'
                    self.state = 'ASTARPATH_FOLLOWING'
                    self.get_logger().warn("Obstacle behind during retreat.", throttle_duration_sec=1.0)
                return
            else:
                self.move_ttbot(0.0, 0.0)
                self.get_logger().info("Retreat complete. Transitioning to ALIGNING_TO_OBS.")
                self.obstacle_state = 'ALIGNING_TO_OBS'
                return

        elif self.obstacle_state == 'ALIGNING_TO_OBS':
            self.update_obstacle_estimate()

        elif self.state == 'ASTARPATH_FOLLOWING' and self.obstacle_state == 'CLEAR':
            if not self.path.poses:
                return
            final_goal_pose = self.path.poses[-1]
            self.path_optimization(final_goal_pose)
            speed, heading = self.path_follower(self.ttbot_pose, self.current_goal)
            self.move_ttbot(speed, heading)
            self.last_commanded_speed = speed
            self.check_goal_alignment(final_goal_pose)

        elif self.obstacle_state == 'REPLANNING':
            self.replan_with_obstacle()
            self.move_ttbot(0.0, 0.0)

    def check_goal_alignment(self, final_goal_pose):
        """Stop and align to the final goal orientation once position is within tolerance."""
        dx = final_goal_pose.pose.position.x - self.ttbot_pose.pose.position.x
        dy = final_goal_pose.pose.position.y - self.ttbot_pose.pose.position.y
        dist_to_final_goal = math.sqrt(dx ** 2 + dy ** 2)
        if dist_to_final_goal < self.goal_tolerance:
            goal_q = self.goal_pose.pose.orientation
            goal_yaw = math.atan2(
                2 * (goal_q.w * goal_q.z + goal_q.x * goal_q.y),
                1 - 2 * (goal_q.y * goal_q.y + goal_q.z * goal_q.z))
            robot_q = self.ttbot_pose.pose.orientation
            robot_yaw = math.atan2(
                2 * (robot_q.w * robot_q.z + robot_q.x * robot_q.y),
                1 - 2 * (robot_q.y * robot_q.y + robot_q.z * robot_q.z))
            yaw_error = goal_yaw - robot_yaw
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            if abs(yaw_error) < self.yaw_tolerance:
                self.get_logger().info("Goal reached and aligned!")
                self.move_ttbot(0.0, 0.0)
                self.path = Path()
                self.goal_pose = None
                self.last_commanded_speed = 0.0
                self.state = 'IDLE'
                if not self.Flg_apprach_Ball:
                    self.current_waypoint_index += 1
                else:
                    self.Flg_apprach_Ball = False
            else:
                heading = np.clip(
                    self.kp_final_yaw * yaw_error,
                    -self.rotspeed_max, self.rotspeed_max)
                self.move_ttbot(0.0, heading)
                self.Flg_apprach_Ball = False

    def path_optimization(self, final_goal_pose):
        """Use line-of-sight shortcutting to advance toward the final goal directly."""
        self.current_goal = None
        if self.use_line_of_sight_check and not self.shortcut_active:
            sg = self._world_to_grid(
                (self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))
            eg = self._world_to_grid(
                (final_goal_pose.pose.position.x, final_goal_pose.pose.position.y))
            if self._is_path_clear(sg, eg):
                self.current_goal = final_goal_pose
                self.shortcut_active = True
                self.get_logger().info("Path clear. Taking shortcut to final goal.")
        if self.shortcut_active:
            self.current_goal = final_goal_pose
        else:
            idx = self.get_path_idx(self.path, self.ttbot_pose)
            self.current_goal = self.path.poses[idx]

    # ── Obstacle avoidance ───────────────────────────────────────────────────

    def _check_for_obstacles(self, scan_msg):
        """LaserScan callback: detect close obstacles and update obstacle_state."""
        self.scan_msg = scan_msg
        ranges = np.array(scan_msg.ranges)
        ranges[np.isinf(ranges)] = np.nan
        ranges[ranges == 0.0] = np.nan
        self.ranges = ranges
        if self.ttbot_pose is None or self.goal_pose is None:
            return
        self.front_slice = np.concatenate((ranges[0:28], ranges[332:360]))
        back_slice = ranges[160:220]
        safety_slice = np.concatenate((ranges[0:90], ranges[270:360]))
        try:
            self.front_dist = np.nanmin(self.front_slice)
            back_dist = np.nanmin(back_slice)
            safety_dist = np.nanmin(safety_slice)
        except ValueError:
            self.get_logger().warn("All laser readings nan. Skipping.", throttle_duration_sec=1)
            return
        obstacle_close = self.front_dist < self.min_front_obstacle_distance
        self.obstacle_behind = back_dist < self.min_back_obstacle_distance
        self.safety_dist_critical = safety_dist < self.safety_distance
        if obstacle_close and self.obstacle_state == 'CLEAR':
            self.get_logger().warn(f"Obstacle at {self.front_dist:.2f} m. Retreating.")
            self.state = 'OBSTACLE_AVOIDANCE'
            self.obstacle_state = 'RETREATING'
            self.move_ttbot(0.0, 0.0)
            self.current_retreat_distance = 0.0
        elif self.safety_dist_critical and self.obstacle_state == 'CLEAR':
            self.get_logger().warn(f"Critical obstacle at {safety_dist:.2f} m.")
            self.state = 'OBSTACLE_AVOIDANCE'
            self.move_ttbot(0.0, 0.0)
            self.obstacle_state = 'RETREATING'
            self.current_retreat_distance = 0.0
        if self.obstacle_state == 'DETECTED':
            self.current_retreat_distance = 0.0
            self.obstacle_state = 'RETREATING'

    def update_obstacle_estimate(self):
        """Estimate obstacle world position and diameter; transition to REPLANNING when aligned."""
        obstacle_slice_index = np.nanargmin(self.front_slice)
        full_scan_index = obstacle_slice_index if obstacle_slice_index < 28 else \
            332 + (obstacle_slice_index - 28)
        ranges = np.array(self.scan_msg.ranges)
        idx_start, idx_end = self._find_obstacle_cluster_bounds(ranges, full_scan_index, 0.3)
        angular_width_rad = (idx_end - idx_start + 1) * self.scan_msg.angle_increment
        center_index = int((idx_start + idx_end) / 2)
        raw_angle = center_index * self.scan_msg.angle_increment + self.scan_msg.angle_min
        angle_from_robot_centerline = raw_angle
        if angle_from_robot_centerline > math.pi:
            angle_from_robot_centerline -= 2 * math.pi
        elif angle_from_robot_centerline < -math.pi:
            angle_from_robot_centerline += 2 * math.pi
        obs_out_of_angle_bounds = abs(angle_from_robot_centerline) > math.radians(45)
        if obs_out_of_angle_bounds and self.obstacle_state == 'ALIGNING_TO_OBS':
            self.move_ttbot(0.0, 0.0)
            if self.goal_pose is not None:
                self._goal_pose_cbk(self.goal_pose)
                if self.path.poses:
                    self.get_logger().info("Replanning successful.")
                    self.path_pub.publish(self.path)
                    self.current_path_idx = 0
                    self.obstacle_state = 'CLEAR'
        elif abs(angle_from_robot_centerline) > math.radians(18) and \
                self.obstacle_state == 'ALIGNING_TO_OBS':
            angular_speed = np.clip(
                self.kp_final_yaw * angle_from_robot_centerline,
                -self.rotspeed_max, self.rotspeed_max)
            self.move_ttbot(0.0, angular_speed)
            self.get_logger().info(f"Angle to obstacle: {math.degrees(angle_from_robot_centerline):.2f} deg")
            return
        else:
            self.move_ttbot(0.0, 0.0)
            self.get_logger().info("Aligned with obstacle. Calculating position.")
            self.obstacle_state = 'REPLANNING'
        cluster_ranges = ranges[idx_start:idx_end + 1]
        obs_dist = np.nanmean(cluster_ranges)
        if obs_dist >= 1.0:
            self.get_logger().warn(f"Obstacle at {obs_dist:.2f} m too far for estimation.")
            self.move_ttbot(0.0, 0.0)
            if self.goal_pose is not None:
                self._goal_pose_cbk(self.goal_pose)
                if self.path.poses:
                    self.get_logger().info("Replanning successful.")
                    self.path_pub.publish(self.path)
                    self.current_path_idx = 0
                    self.obstacle_state = 'CLEAR'
        self.estimated_diameter = min(
            self._estimate_obstacle_diameter(angular_width_rad, obs_dist),
            self.max_obstacle_diameter)
        quat = self.ttbot_pose.pose.orientation
        robot_theta = math.atan2(
            2 * (quat.w * quat.z + quat.x * quat.y),
            1 - 2 * (quat.y * quat.y + quat.z * quat.z))
        R_obs = self.estimated_diameter / 2.0
        distance_to_center = obs_dist + R_obs
        world_angle = robot_theta + angle_from_robot_centerline
        obs_world_x = self.ttbot_pose.pose.position.x + distance_to_center * math.cos(world_angle)
        obs_world_y = self.ttbot_pose.pose.position.y + distance_to_center * math.sin(world_angle)
        self.obstacle_pos_world = (obs_world_x, obs_world_y)
        self.get_logger().warn(f"Obstacle at {self.front_dist:.2f} m. Triggering replan.")
        self.get_logger().info(f"Obstacle world pos: x={obs_world_x:.2f}, y={obs_world_y:.2f}")

    def update_ball_estimate(self):
        """Estimate ball world position from LIDAR and camera; navigate to approach point."""
        if self.ttbot_pose is None:
            self.get_logger().warn("Robot pose not available. Cannot calculate ball position.",
                                   throttle_duration_sec=1)
            return
        num_rays = len(self.ranges)
        if num_rays == 0:
            self.get_logger().warn("Lidar data empty.", throttle_duration_sec=1)
            return
        target_index = int(round(self.Ball_heading_error_degrees))
        slice_half_width = 2
        indices_to_check = [(target_index + i) % num_rays
                            for i in range(-slice_half_width, slice_half_width + 1)]
        ball_distance_slice = self.ranges[indices_to_check]
        try:
            ball_distance = np.nanmean(ball_distance_slice)
            if np.isnan(ball_distance):
                return
            self.get_logger().info(f"Ball distance: {ball_distance} m", throttle_duration_sec=3)
        except ValueError:
            self.get_logger().warn("Ball lidar nan.", throttle_duration_sec=3)
            return
        if np.isnan(ball_distance):
            return
        distance_not_valid = (self.Camera_distance_to_object / ball_distance) - 1.0 > 0.3
        if distance_not_valid:
            self.get_logger().warn(
                f"Distance discrepancy: Camera={self.Camera_distance_to_object:.2f}, "
                f"Lidar={ball_distance:.2f}", throttle_duration_sec=3)
            self.Flg_apprach_Ball = True
            if self.Reset_Flg_apprach_Ball_timer is not None and \
                    self.Reset_Flg_apprach_Ball_timer.is_valid():
                self.Reset_Flg_apprach_Ball_timer.cancel()
            self.Reset_Flg_apprach_Ball_timer = self.create_timer(3.0, self._reset_approach_flag)
            return
        quat = self.ttbot_pose.pose.orientation
        robot_theta = math.atan2(
            2 * (quat.w * quat.z + quat.x * quat.y),
            1 - 2 * (quat.y * quat.y + quat.z * quat.z))
        robot_x = self.ttbot_pose.pose.position.x
        robot_y = self.ttbot_pose.pose.position.y
        R_obs = self.Ball_diameter / 2.0
        distance_to_center = ball_distance + R_obs
        world_angle = robot_theta - math.radians(self.Ball_heading_error_degrees)
        Ball_world_x = robot_x + distance_to_center * math.cos(world_angle)
        Ball_world_y = robot_y + distance_to_center * math.sin(world_angle)
        self.Ball_pos_world = (Ball_world_x, Ball_world_y)
        self.get_logger().info(
            f"Ball world pos: x={Ball_world_x:.2f}, y={Ball_world_y:.2f}", throttle_duration_sec=3)
        if ball_distance <= self.Target_Distance_Ball + 0.2:
            self.get_logger().info("Measurement pose reached. Publishing ball position.",
                                   throttle_duration_sec=3)
            obs_grid_coords = self._world_to_grid(self.Ball_pos_world)
            obs_y, obs_x = obs_grid_coords
            static_safety_buffer_m = self.inflation_kernel_size * self.map_processor.map.resolution
            total_radius_m = (self.Ball_diameter / 2.0) + static_safety_buffer_m
            pixel_radius = int(total_radius_m / self.map_processor.map.resolution)
            H, W = self.map_processor.inf_map_img_array.shape
            Y, X = np.ogrid[-obs_y:H - obs_y, -obs_x:W - obs_x]
            mask = X ** 2 + Y ** 2 <= pixel_radius ** 2
            self.map_processor.inf_map_img_array[mask] = 1
            self._publish_inflated_map()
            self.map_processor.get_graph_from_map()
            self.get_logger().info("Graph updated with ball.")
            ball_pose_msg = Pose()
            ball_pose_msg.position.x = Ball_world_x
            ball_pose_msg.position.y = Ball_world_y
            ball_pose_msg.position.z = 0.0
            ball_pose_msg.orientation.w = 1.0
            if self.detected_color == 'RED':
                self.get_logger().info("RED BALL FOUND", throttle_duration_sec=5)
                self.RED_BALL_Pos = self.Ball_pos_world
                self.red_pos_pub.publish(ball_pose_msg)
            elif self.detected_color == 'GREEN':
                self.get_logger().info("GREEN BALL FOUND", throttle_duration_sec=5)
                self.GREEN_BALL_Pos = self.Ball_pos_world
                self.green_pos_pub.publish(ball_pose_msg)
            elif self.detected_color == 'BLUE':
                self.get_logger().info("BLUE BALL FOUND", throttle_duration_sec=5)
                self.BLUE_BALL_Pos = self.Ball_pos_world
                self.blue_pos_pub.publish(ball_pose_msg)
            else:
                self.get_logger().info("Color detection error.", throttle_duration_sec=5)
            self.state = 'IDLE'
            return
        new_dist_from_robot = distance_to_center - self.Target_Distance_Ball
        Goal_world_x = robot_x + new_dist_from_robot * math.cos(world_angle)
        Goal_world_y = robot_y + new_dist_from_robot * math.sin(world_angle)
        Goal_pose = PoseStamped()
        Goal_pose.header.frame_id = 'map'
        Goal_pose.pose.position.x = Goal_world_x
        Goal_pose.pose.position.y = Goal_world_y
        Goal_pose.pose.orientation.x = 0.0
        Goal_pose.pose.orientation.y = 0.0
        Goal_pose.pose.orientation.z = math.sin(world_angle / 2.0)
        Goal_pose.pose.orientation.w = math.cos(world_angle / 2.0)
        self._goal_pose_cbk(Goal_pose)
        self.Flg_apprach_Ball = True

    def _reset_approach_flag(self):
        """Timer callback to clear the ball-approach flag after a delay."""
        self.Flg_apprach_Ball = False
        if self.Reset_Flg_apprach_Ball_timer is not None:
            self.destroy_timer(self.Reset_Flg_apprach_Ball_timer)
            self.Reset_Flg_apprach_Ball_timer = None

    def replan_with_obstacle(self):
        """Inject a detected obstacle into the costmap and replan the path."""
        obs_grid_coords = self._world_to_grid(self.obstacle_pos_world)
        obs_y, obs_x = obs_grid_coords
        static_safety_buffer_m = self.inflation_kernel_size * self.map_processor.map.resolution
        total_radius_m = (self.estimated_diameter / 2.0) + static_safety_buffer_m
        self.get_logger().info(f"Inflating obstacle, total radius: {total_radius_m:.2f} m")
        resolution = self.map_processor.map.resolution
        pixel_radius = int(total_radius_m / resolution)
        self.get_logger().info(
            f"Obstacle at grid ({obs_y}, {obs_x}) radius {pixel_radius} cells.")
        H, W = self.map_processor.inf_map_img_array.shape
        Y, X = np.ogrid[-obs_y:H - obs_y, -obs_x:W - obs_x]
        mask = X ** 2 + Y ** 2 <= pixel_radius ** 2
        self.map_processor.inf_map_img_array[mask] = 1
        self._publish_inflated_map()
        self.map_processor.get_graph_from_map()
        self.get_logger().info("Graph updated with obstacle.")
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        self.path_pub.publish(empty_path)
        self.path = Path()
        self.current_path_idx = 0
        self.shortcut_active = False
        if self.goal_pose is not None:
            self._goal_pose_cbk(self.goal_pose)
            if self.path.poses:
                self.get_logger().info("Replanning successful.")
                self.path_pub.publish(self.path)
                self.current_path_idx = 0
                self.obstacle_state = 'CLEAR'
            else:
                self.get_logger().error("Replanning failed.")
                self.move_ttbot(0.0, 0.0)
        else:
            self.get_logger().warn("Goal cleared during replanning.")
            self.move_ttbot(0.0, 0.0)
            self.obstacle_state = 'CLEAR'

    # ── Map utilities ────────────────────────────────────────────────────────

    def _world_to_grid(self, world_coords):
        """Convert world (x, y) to grid (row, col) using current map metadata."""
        origin_x = self.map_processor.map.origin[0]
        origin_y = self.map_processor.map.origin[1]
        resolution = self.map_processor.map.resolution
        map_height = self.map_processor.map.height
        grid_x = int((world_coords[0] - origin_x) / resolution)
        grid_y = map_height - 1 - int((world_coords[1] - origin_y) / resolution)
        return (grid_y, grid_x)

    def _grid_to_world(self, grid_coords):
        """Convert grid (row, col) to world (x, y) using current map metadata."""
        origin_x = self.map_processor.map.origin[0]
        origin_y = self.map_processor.map.origin[1]
        resolution = self.map_processor.map.resolution
        map_height = self.map_processor.map.height
        grid_y, grid_x = grid_coords
        unflipped_grid_y = map_height - 1 - grid_y
        world_x = (grid_x + 0.5) * resolution + origin_x
        world_y = (unflipped_grid_y + 0.5) * resolution + origin_y
        return (world_x, world_y)

    def _publish_inflated_map(self):
        """Publish inf_map_img_array as a nav_msgs/OccupancyGrid on /custom_costmap."""
        if self.map_processor.inf_map_img_array.size == 0 or self.map_processor.map.height == 0:
            return
        map_msg = OccupancyGrid()
        map_info = self.map_processor.map
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = map_info.resolution
        map_msg.info.width = self.map_processor.inf_map_img_array.shape[1]
        map_msg.info.height = self.map_processor.inf_map_img_array.shape[0]
        map_msg.info.origin.position.x = map_info.origin[0]
        map_msg.info.origin.position.y = map_info.origin[1]
        map_msg.info.origin.orientation.w = 1.0
        ros_data = self.map_processor.inf_map_img_array * 100
        ros_array_flipped = np.flipud(ros_data)
        map_msg.data = ros_array_flipped.flatten().astype(np.int8).tolist()
        self.inflated_map_pub.publish(map_msg)

    # ── Helper methods ───────────────────────────────────────────────────────

    def _estimate_obstacle_diameter(self, delta_angle_rad, distance_m):
        """Estimate obstacle chord diameter: L = 2R * sin(Δθ/2)."""
        if distance_m <= 0 or delta_angle_rad <= 0:
            return 0.0
        return 2 * distance_m * math.sin(delta_angle_rad / 2.0)

    def _find_obstacle_cluster_bounds(self, ranges, min_index, jump_threshold):
        """Return (start_idx, end_idx) of the obstacle cluster centred on min_index."""
        num_readings = len(ranges)
        index_start = min_index
        for i in range(min_index, 0, -1):
            if abs(ranges[i] - ranges[i - 1]) > jump_threshold:
                index_start = i
                break
        index_end = min_index
        for i in range(min_index, num_readings - 1):
            if abs(ranges[i + 1] - ranges[i]) > jump_threshold:
                index_end = i
                break
        return index_start, index_end

    def _get_contour_color(self, contour, color_masks, min_overlap_ratio=0.8):
        """Identify which color mask has the highest overlap with the given contour."""
        best_color = "UNKNOWN"
        max_overlap = 0.0
        contour_mask = np.zeros_like(color_masks["RED"])
        cv2.drawContours(contour_mask, [contour], -1, 255, cv2.FILLED)
        contour_area_pixels = np.sum(contour_mask == 255)
        if contour_area_pixels == 0:
            return "UNKNOWN"
        for color, mask in color_masks.items():
            overlap_mask = cv2.bitwise_and(contour_mask, mask)
            overlap_pixels = np.sum(overlap_mask == 255)
            overlap_ratio = overlap_pixels / contour_area_pixels
            if overlap_ratio > max_overlap and overlap_ratio >= min_overlap_ratio:
                max_overlap = overlap_ratio
                best_color = color
        return best_color

    def publish_initial_pose(self):
        """Publish the robot's initial pose to AMCL once, then cancel the timer."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        self.get_logger().info("Publishing initial pose to AMCL.")
        self.initial_pose_pub.publish(pose_msg)
        self.initial_pose_timer.cancel()

    def destroy_node(self):
        """Cancel all timers and destroy the node cleanly."""
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if hasattr(self, 'initial_pose_timer'):
            self.initial_pose_timer.cancel()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Entry point: spin VisionNavigator until interrupted."""
    rclpy.init(args=args)
    node = VisionNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
