# Extracted from monolithic nodes. No logic changes.

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, PointStamped
from std_msgs.msg import Bool

from turtlebot3_gazebo.common.lidar_utils import (
    process_scan, get_front_slice, get_back_slice, get_safety_slice,
    find_obstacle_cluster_bounds, estimate_obstacle_diameter
)

# --- Constants ---
# Default obstacle detection thresholds (meters)
DEFAULT_MIN_FRONT_DIST = 0.35
DEFAULT_MIN_BACK_DIST = 0.25
DEFAULT_SAFETY_DIST = 0.2
DEFAULT_RETREAT_DIST = 0.20   # positive; negated internally
DEFAULT_RETREAT_SPEED = -0.15
DEFAULT_MAX_OBS_DIAMETER = 0.4
# Alignment angle thresholds (degrees)
ALIGN_ANGLE_FAR_DEG = 45.0
ALIGN_ANGLE_NEAR_DEG = 18.0
# Distance beyond which obstacle estimation is unreliable (meters)
OBS_FAR_THRESHOLD_M = 1.0
# Timer rate (Hz)
TIMER_RATE_HZ = 10.0


class ObstacleDetector(Node):
    """Detects dynamic obstacles from LIDAR and manages retreat-and-replan state machine."""

    def __init__(self):
        """Initialise ObstacleDetector: declare parameters, subscribe to sensors, create publishers."""
        super().__init__('obstacle_detector')

        self.declare_parameter('min_front_obstacle_distance', DEFAULT_MIN_FRONT_DIST)
        self.declare_parameter('min_back_obstacle_distance', DEFAULT_MIN_BACK_DIST)
        self.declare_parameter('safety_distance', DEFAULT_SAFETY_DIST)
        self.declare_parameter('retreat_distance', DEFAULT_RETREAT_DIST)
        self.declare_parameter('retreat_speed', DEFAULT_RETREAT_SPEED)
        self.declare_parameter('max_obstacle_diameter', DEFAULT_MAX_OBS_DIAMETER)

        self.min_front_obstacle_distance = self.get_parameter('min_front_obstacle_distance').get_parameter_value().double_value
        self.min_back_obstacle_distance = self.get_parameter('min_back_obstacle_distance').get_parameter_value().double_value
        self.safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value
        self.retreat_distance = -abs(self.get_parameter('retreat_distance').get_parameter_value().double_value)
        self.retreat_speed = self.get_parameter('retreat_speed').get_parameter_value().double_value
        self.max_obstacle_diameter = self.get_parameter('max_obstacle_diameter').get_parameter_value().double_value

        self.kp_final_yaw = 0.8
        self.rotspeed_max = 1.9

        # State
        self.obstacle_state = 'CLEAR'
        self.current_retreat_distance = 0.0
        self.ttbot_pose = None
        self.scan_msg = None
        self.front_slice = None
        self.front_dist = float('inf')
        self.obstacle_behind = False
        self.safety_dist_critical = False
        self.obstacle_pos_world = (0.0, 0.0)
        self.estimated_diameter = 0.0

        self.rate = TIMER_RATE_HZ

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self._check_for_obstacles, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._ttbot_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self._ttbot_pose_cbk, 10)

        # Publishers
        self.detected_obstacle_pub = self.create_publisher(PointStamped, '/detected_obstacle', 10)
        self.stop_robot_pub = self.create_publisher(Bool, '/stop_robot', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(1.0 / self.rate, self.run_loop)

    def _ttbot_pose_cbk(self, data):
        """Update internal robot pose from amcl_pose or pose topic."""
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose = data.pose.pose
        self.ttbot_pose = pose_stamped

    def _check_for_obstacles(self, scan_msg):
        """Process incoming LaserScan; transition to RETREATING if obstacle detected."""
        if self.ttbot_pose is None:
            return

        self.scan_msg = scan_msg

        ranges = process_scan(scan_msg.ranges)
        self.front_slice = get_front_slice(ranges)
        back_slice = get_back_slice(ranges)
        safety_slice = get_safety_slice(ranges)

        try:
            self.front_dist = np.nanmin(self.front_slice)
            back_dist = np.nanmin(back_slice)
            safety_dist = np.nanmin(safety_slice)
        except ValueError:
            self.get_logger().warn("laser readings are 'nan'. Skipping loop.", throttle_duration_sec=1)
            return

        obstacle_close = self.front_dist < self.min_front_obstacle_distance
        self.obstacle_behind = back_dist < self.min_back_obstacle_distance
        self.safety_dist_critical = safety_dist < self.safety_distance

        if obstacle_close and self.obstacle_state == 'CLEAR':
            self.get_logger().warn(
                f"Obstacle detected at {self.front_dist:.2f} m! Initiating avoidance maneuvers."
            )
            self.obstacle_state = 'RETREATING'
            self._publish_stop(True)
            self._move_ttbot(0.0, 0.0)
            self.current_retreat_distance = 0.0

        elif self.safety_dist_critical and self.obstacle_state == 'CLEAR':
            self.get_logger().warn(
                f"Critical obstacle at {safety_dist:.2f} m in safety zone! Initiating avoidance."
            )
            self._publish_stop(True)
            self._move_ttbot(0.0, 0.0)
            self.obstacle_state = 'RETREATING'
            self.current_retreat_distance = 0.0

        if self.obstacle_state == 'DETECTED':
            self.current_retreat_distance = 0.0
            self.obstacle_state = 'RETREATING'

    def run_loop(self):
        """Main control loop: manage retreat and alignment state transitions."""
        if self.obstacle_state == 'RETREATING':
            dt = 1.0 / self.rate
            distance_moved = abs(self.retreat_speed * dt)

            if self.current_retreat_distance < abs(self.retreat_distance):
                self._move_ttbot(self.retreat_speed, 0.0)
                self.current_retreat_distance += distance_moved
                if self.obstacle_behind:
                    self.obstacle_state = 'CLEAR'
                    self._publish_stop(False)
                    self.get_logger().warn("Obstacle still behind during retreat", throttle_duration_sec=1.0)
                return
            else:
                self._move_ttbot(0.0, 0.0)
                self.get_logger().info("Retreat complete. Transitioning to ALIGNING_TO_OBS.")
                self.obstacle_state = 'ALIGNING_TO_OBS'
                return

        elif self.obstacle_state == 'ALIGNING_TO_OBS':
            self.update_obstacle_estimate()

        elif self.obstacle_state == 'REPLANNING':
            self._publish_detected_obstacle()
            self._publish_stop(False)
            self.obstacle_state = 'CLEAR'

    def update_obstacle_estimate(self):
        """Estimate obstacle world position from LIDAR data; transition states accordingly."""
        if self.scan_msg is None or self.front_slice is None:
            return

        obstacle_slice_index = np.nanargmin(self.front_slice)

        if obstacle_slice_index < 28:
            full_scan_index = obstacle_slice_index
        else:
            full_scan_index = 332 + (obstacle_slice_index - 28)

        ranges = process_scan(self.scan_msg.ranges)
        idx_start, idx_end = find_obstacle_cluster_bounds(ranges, full_scan_index, jump_threshold=0.3)

        angular_width_rad = (idx_end - idx_start + 1) * self.scan_msg.angle_increment

        center_index = int((idx_start + idx_end) / 2)

        raw_angle = center_index * self.scan_msg.angle_increment + self.scan_msg.angle_min
        angle_from_robot_centerline = raw_angle

        if angle_from_robot_centerline > math.pi:
            angle_from_robot_centerline -= 2 * math.pi
        elif angle_from_robot_centerline < -math.pi:
            angle_from_robot_centerline += 2 * math.pi

        if abs(angle_from_robot_centerline) > math.radians(ALIGN_ANGLE_FAR_DEG) and self.obstacle_state == 'ALIGNING_TO_OBS':
            self._move_ttbot(0.0, 0.0)
            self.obstacle_state = 'REPLANNING'

        elif abs(angle_from_robot_centerline) > math.radians(ALIGN_ANGLE_NEAR_DEG) and self.obstacle_state == 'ALIGNING_TO_OBS':
            angular_speed = self.kp_final_yaw * angle_from_robot_centerline
            angular_speed = np.clip(angular_speed, -self.rotspeed_max, self.rotspeed_max)
            self._move_ttbot(0.0, angular_speed)
            self.get_logger().info(f"Angle to obstacle: {math.degrees(angle_from_robot_centerline):.2f} degrees")
            return

        else:
            self._move_ttbot(0.0, 0.0)
            self.get_logger().info("Aligned with obstacle. Calculating position.")
            self.obstacle_state = 'REPLANNING'

        cluster_ranges = ranges[idx_start:idx_end + 1]
        obs_dist = np.nanmean(cluster_ranges)

        if obs_dist >= OBS_FAR_THRESHOLD_M:
            self.get_logger().warn(f"Obstacle at {obs_dist:.2f} m is too far for reliable estimation")
            self.obstacle_state = 'REPLANNING'
            return

        self.estimated_diameter = estimate_obstacle_diameter(angular_width_rad, obs_dist)
        self.estimated_diameter = min(self.estimated_diameter, self.max_obstacle_diameter)
        self.get_logger().info(f"Angle to obstacle center: {math.degrees(angle_from_robot_centerline):.2f} degrees")
        self.get_logger().info(f"Estimated obstacle diameter: {self.estimated_diameter:.2f} m")

        quat = self.ttbot_pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        robot_theta = math.atan2(siny_cosp, cosy_cosp)

        R_obs = self.estimated_diameter / 2.0
        distance_to_center = obs_dist + R_obs

        world_angle_to_center = robot_theta + angle_from_robot_centerline

        obs_world_x = self.ttbot_pose.pose.position.x + distance_to_center * math.cos(world_angle_to_center)
        obs_world_y = self.ttbot_pose.pose.position.y + distance_to_center * math.sin(world_angle_to_center)

        self.obstacle_pos_world = (obs_world_x, obs_world_y)

        self.get_logger().warn(f"Unmapped obstacle at {self.front_dist:.2f} m. Triggering replanning.")
        self.get_logger().info(f"Estimated obstacle position (world): x={obs_world_x:.2f}, y={obs_world_y:.2f}")

    def _publish_detected_obstacle(self):
        """Publish obstacle world position and diameter on /detected_obstacle."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = self.obstacle_pos_world[0]
        msg.point.y = self.obstacle_pos_world[1]
        msg.point.z = self.estimated_diameter
        self.detected_obstacle_pub.publish(msg)

    def _publish_stop(self, stop):
        """Publish a stop command on /stop_robot."""
        msg = Bool()
        msg.data = stop
        self.stop_robot_pub.publish(msg)

    def _move_ttbot(self, speed, heading):
        """Publish a Twist command on cmd_vel."""
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def destroy_node(self):
        """Cancel timers and destroy the node."""
        if hasattr(self, 'timer'):
            self.timer.cancel()
        super().destroy_node()


def main(args=None):
    """Entry point: spin ObstacleDetector until interrupted."""
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
