# Extracted from monolithic nodes. No logic changes.

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import os

from turtlebot3_gazebo.common.map_utils import MapProcessor

# --- Constants ---
DEFAULT_K_RHO = 0.8608
DEFAULT_KP_ANGULAR = 2.0747
DEFAULT_KI_ANGULAR = 0.1692
DEFAULT_KD_ANGULAR = -0.02
DEFAULT_K_BETA = -0.1
DEFAULT_SPEED_MAX = 0.31
DEFAULT_ROTSPEED_MAX = 1.9
DEFAULT_GOAL_TOLERANCE = 0.1
DEFAULT_YAW_TOLERANCE = 0.1
DEFAULT_MIN_LOOKAHEAD_DIST = 0.2
DEFAULT_LOOKAHEAD_RATIO = 0.5
DEFAULT_INFLATION_KERNEL = 10
# Angle beyond which robot stops translating and only rotates (radians)
MAX_ANGLE_ALPHA_TO_STARTDRIVE = 1.0
# Final yaw correction proportional gain
KP_FINAL_YAW = 0.8
# PID integral anti-windup clamp
INTEGRAL_CLAMP = 1.0
# Timer rate (Hz)
TIMER_RATE_HZ = 10.0


class PathFollower(Node):
    """Follows a global path published on /global_plan using a PID angular controller."""

    def __init__(self):
        """Initialise PathFollower: declare parameters, load map, create subs/pubs/timer."""
        super().__init__('path_follower')

        self.declare_parameter('k_rho', DEFAULT_K_RHO)
        self.declare_parameter('kp_angular', DEFAULT_KP_ANGULAR)
        self.declare_parameter('ki_angular', DEFAULT_KI_ANGULAR)
        self.declare_parameter('kd_angular', DEFAULT_KD_ANGULAR)
        self.declare_parameter('k_beta', DEFAULT_K_BETA)
        self.declare_parameter('speed_max', DEFAULT_SPEED_MAX)
        self.declare_parameter('rotspeed_max', DEFAULT_ROTSPEED_MAX)
        self.declare_parameter('goal_tolerance', DEFAULT_GOAL_TOLERANCE)
        self.declare_parameter('yaw_tolerance', DEFAULT_YAW_TOLERANCE)
        self.declare_parameter('min_lookahead_dist', DEFAULT_MIN_LOOKAHEAD_DIST)
        self.declare_parameter('lookahead_ratio', DEFAULT_LOOKAHEAD_RATIO)
        self.declare_parameter('use_dynamic_lookahead', True)
        self.declare_parameter('use_line_of_sight_check', True)
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('inflation_kernel_size', DEFAULT_INFLATION_KERNEL)

        self.k_rho = self.get_parameter('k_rho').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.ki_angular = self.get_parameter('ki_angular').get_parameter_value().double_value
        self.kd_angular = self.get_parameter('kd_angular').get_parameter_value().double_value
        self.k_beta = self.get_parameter('k_beta').get_parameter_value().double_value
        self.speed_max = self.get_parameter('speed_max').get_parameter_value().double_value
        self.rotspeed_max = self.get_parameter('rotspeed_max').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').get_parameter_value().double_value
        self.min_lookahead_dist = self.get_parameter('min_lookahead_dist').get_parameter_value().double_value
        self.lookahead_ratio = self.get_parameter('lookahead_ratio').get_parameter_value().double_value
        self.use_dynamic_lookahead = self.get_parameter('use_dynamic_lookahead').get_parameter_value().bool_value
        self.use_line_of_sight_check = self.get_parameter('use_line_of_sight_check').get_parameter_value().bool_value
        self.inflation_kernel_size = self.get_parameter('inflation_kernel_size').get_parameter_value().integer_value

        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        if not self.map_yaml_path:
            pkg_share_path = get_package_share_directory('turtlebot3_gazebo')
            self.map_yaml_path = os.path.join(pkg_share_path, 'maps', 'map.yaml')

        self.get_logger().info(f"Loading map from '{self.map_yaml_path}'...")
        self.map_processor = MapProcessor(self.map_yaml_path)
        inflation_kernel = self.map_processor.rect_kernel(self.inflation_kernel_size, 1)
        self.map_processor.inflate_map(inflation_kernel)
        self.map_processor.get_graph_from_map()
        self.get_logger().info("Map loaded for line-of-sight checks.")

        # State
        self.path = Path()
        self.ttbot_pose = None
        self.goal_pose = None
        self.current_goal = None
        self.stopped = False
        self.shortcut_active = False
        self.last_commanded_speed = 0.0
        self.integral_error_angular = 0.0
        self.previous_error_angular = 0.0

        # Subscriptions
        self.create_subscription(Path, '/global_plan', self._path_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._ttbot_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self._ttbot_pose_cbk, 10)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self._goal_pose_cbk, 10)
        self.create_subscription(Bool, '/stop_robot', self._stop_cbk, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.rate = TIMER_RATE_HZ
        self.timer = self.create_timer(1.0 / self.rate, self.run_loop)

    def _path_cbk(self, data):
        """Update the stored path and reset follower state on new path receipt."""
        self.path = data
        self.shortcut_active = False
        if data.poses:
            self.goal_pose = data.poses[-1]

    def _ttbot_pose_cbk(self, data):
        """Update internal robot pose from amcl_pose or pose topic."""
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose = data.pose.pose
        self.ttbot_pose = pose_stamped

    def _goal_pose_cbk(self, data):
        """Store the current navigation goal for orientation alignment check."""
        self.goal_pose = data

    def _stop_cbk(self, data):
        """Update stopped flag; when True the robot is commanded to hold position."""
        self.stopped = data.data

    def run_loop(self):
        """Main 10 Hz loop: run path optimization, path follower, and goal alignment."""
        if self.stopped:
            self.move_ttbot(0.0, 0.0)
            return

        if self.ttbot_pose is None or not self.path.poses:
            return

        final_goal_pose = self.path.poses[-1]

        self.path_optimization(final_goal_pose)

        if self.current_goal is None:
            return

        speed, heading = self.path_follower(self.ttbot_pose, self.current_goal)
        self.move_ttbot(speed, heading)
        self.last_commanded_speed = speed

        self.check_goal_alignment(final_goal_pose)

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

    def get_path_idx(self, path, vehicle_pose):
        """Return the lookahead index in path from the robot's current position."""
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

        if abs(alpha) > MAX_ANGLE_ALPHA_TO_STARTDRIVE:
            speed = 0.0
        else:
            speed = min(self.k_rho * rho, self.speed_max)

        heading = p_term + i_term + d_term + self.k_beta * beta
        heading = np.clip(heading, -self.rotspeed_max, self.rotspeed_max)

        return speed, heading

    def check_goal_alignment(self, final_goal_pose):
        """Check if robot reached final goal position+orientation; publish goal_reached."""
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

            if abs(yaw_error) < self.yaw_tolerance:
                self.get_logger().info("Goal reached and aligned to Goal Pose!")
                self.move_ttbot(0.0, 0.0)
                self.path = Path()
                self.goal_pose = None
                self.last_commanded_speed = 0.0
                self.shortcut_active = False

                reached_msg = Bool()
                reached_msg.data = True
                self.goal_reached_pub.publish(reached_msg)
                return
            else:
                speed = 0.0
                heading = KP_FINAL_YAW * yaw_error
                heading = np.clip(heading, -self.rotspeed_max, self.rotspeed_max)
                self.move_ttbot(speed, heading)
                return

    def move_ttbot(self, speed, heading):
        """Publish a Twist command on cmd_vel."""
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def _world_to_grid(self, world_coords):
        """Convert world coordinates to grid indices using current map metadata."""
        origin_x = self.map_processor.map.origin[0]
        origin_y = self.map_processor.map.origin[1]
        resolution = self.map_processor.map.resolution
        map_height_pixels = self.map_processor.map.height

        relative_x = world_coords[0] - origin_x
        relative_y = world_coords[1] - origin_y

        grid_x = int(relative_x / resolution)
        grid_y = map_height_pixels - 1 - int(relative_y / resolution)

        return (grid_y, grid_x)

    def destroy_node(self):
        """Cancel timers and destroy the node."""
        if hasattr(self, 'timer'):
            self.timer.cancel()
        super().destroy_node()


def main(args=None):
    """Entry point: spin PathFollower until interrupted."""
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
