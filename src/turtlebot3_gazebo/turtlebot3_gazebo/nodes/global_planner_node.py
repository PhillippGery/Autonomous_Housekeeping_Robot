# Extracted from monolithic nodes. No logic changes.

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Point
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
import os

from turtlebot3_gazebo.common.graph_utils import AStar
from turtlebot3_gazebo.common.map_utils import MapProcessor, RRTStarGrid

# --- Constants ---
DEFAULT_PLANNER_TYPE = 'astar'         # 'astar' or 'rrtstar'
DEFAULT_MAX_DIST_ALTERNATE = 1.0       # meters: fallback search radius for invalid start/end
DEFAULT_INFLATION_KERNEL = 10          # cells
# Timer rate (Hz)
TIMER_RATE_HZ = 10.0


class GlobalPlanner(Node):
    """Plans global paths using A* or RRT* and publishes them on /global_plan."""

    def __init__(self):
        """Initialise GlobalPlanner: load map, build graph, create subs/pubs."""
        super().__init__('global_planner')

        self.declare_parameter('planner_type', DEFAULT_PLANNER_TYPE)
        self.declare_parameter('max_dist_alternate_point', DEFAULT_MAX_DIST_ALTERNATE)
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('inflation_kernel_size', DEFAULT_INFLATION_KERNEL)

        self.planner_type = self.get_parameter('planner_type').get_parameter_value().string_value
        self.max_dist_alternate_point = self.get_parameter('max_dist_alternate_point').get_parameter_value().double_value
        self.inflation_kernel_size = self.get_parameter('inflation_kernel_size').get_parameter_value().integer_value

        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        if not self.map_yaml_path:
            pkg_share_path = get_package_share_directory('turtlebot3_gazebo')
            self.map_yaml_path = os.path.join(pkg_share_path, 'maps', 'map.yaml')

        self.get_logger().info(f"Loading map from '{self.map_yaml_path}' and building graph...")
        self.map_processor = MapProcessor(self.map_yaml_path)
        inflation_kernel = self.map_processor.rect_kernel(self.inflation_kernel_size, 1)
        self.map_processor.inflate_map(inflation_kernel)
        self.map_processor.get_graph_from_map()
        self.get_logger().info("Graph built successfully.")

        if self.planner_type == 'rrtstar':
            self.rrt_planner = RRTStarGrid(self.map_processor, node_instance=self)
            self.get_logger().info("RRT* planner initialised.")

        # State
        self.ttbot_pose = None
        self.goal_pose = None
        self.path = Path()
        self.start_time = 0.0

        # Subscriptions
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self._goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._ttbot_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self._ttbot_pose_cbk, 10)
        self.create_subscription(OccupancyGrid, '/custom_costmap', self._costmap_cbk, 1)
        self.create_subscription(Point, '/new_obstacle', self._new_obstacle_cbk, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initial pose publisher (for AMCL bootstrap)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.initial_pose_timer = self.create_timer(2.0, self.publish_initial_pose)

    def _goal_pose_cbk(self, data):
        """Receive a new goal pose, plan a path, and publish it."""
        self.goal_pose = data
        self.get_logger().info(
            'New goal received: {:.4f}, {:.4f}'.format(
                self.goal_pose.pose.position.x, self.goal_pose.pose.position.y
            )
        )

        if self.ttbot_pose is None:
            self.get_logger().warn("Cannot plan path, robot pose is not yet available.")
            return

        self.path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)

        if self.path.poses:
            self.path_pub.publish(self.path)
        else:
            self.get_logger().warn("Planner failed to find a path to the goal.")
            self.move_ttbot(0.0, 0.0)

    def _ttbot_pose_cbk(self, data):
        """Update internal robot pose from amcl_pose or pose topic."""
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose = data.pose.pose
        self.ttbot_pose = pose_stamped

    def _costmap_cbk(self, data):
        """Update local inflated map from /custom_costmap and rebuild the graph."""
        H = data.info.height
        W = data.info.width

        if H == 0 or W == 0:
            return

        ros_array = np.array(data.data, dtype=np.int8).reshape(H, W)
        local_array = np.where(ros_array >= 50, 1, 0).astype(int)
        self.map_processor.inf_map_img_array = np.flipud(local_array)
        self.map_processor.map.resolution = data.info.resolution
        self.map_processor.map.origin[0] = data.info.origin.position.x
        self.map_processor.map.origin[1] = data.info.origin.position.y
        self.map_processor.map.height = H
        self.map_processor.get_graph_from_map()

    def _new_obstacle_cbk(self, data):
        """Trigger replanning when a new obstacle is reported by the costmap server."""
        self.get_logger().info(
            f"New obstacle at ({data.x:.2f}, {data.y:.2f}). Replanning..."
        )
        if self.goal_pose is not None and self.ttbot_pose is not None:
            self._goal_pose_cbk(self.goal_pose)

    def a_star_path_planner(self, start_pose, end_pose):
        """Plan a path from start_pose to end_pose using A* (or RRT* if configured)."""
        path = Path()
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        start_world = (start_pose.pose.position.x, start_pose.pose.position.y)
        end_world = (end_pose.pose.position.x, end_pose.pose.position.y)

        start_grid = self._world_to_grid(start_world)
        end_grid = self._world_to_grid(end_world)

        start_name = f"{start_grid[0]},{start_grid[1]}"
        end_name = f"{end_grid[0]},{end_grid[1]}"

        is_start_valid = start_name in self.map_processor.map_graph.g
        is_end_valid = end_name in self.map_processor.map_graph.g

        # 1. If start is not valid, find closest valid node
        if not is_start_valid:
            self.get_logger().warn(f"start pose {start_name} is NOT valid. taking closest point.")
            min_dist_sq = float('inf')
            max_dist_sq = (self.max_dist_alternate_point / self.map_processor.map.resolution) ** 2
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

        # 2. If end is not valid, find closest valid node
        if not is_end_valid:
            self.get_logger().warn(f"Goal pose {end_name} is NOT valid. taking closest point.")
            min_dist_sq = float('inf')
            max_dist_sq = (self.max_dist_alternate_point / self.map_processor.map.resolution) ** 2
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

        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        if self.planner_type == 'rrtstar':
            path_names, path_dist = self.rrt_planner.solve(start_grid, end_grid)
            if path_names:
                self.get_logger().info(f"RRT* found a path of length {len(path_names)}")
                path = self.rrt_planner.path_to_pose_stamped(path_names, path_dist)
            else:
                self.get_logger().warn("RRT* failed to find a path.")
                self.move_ttbot(0.0, 0.0)
                self._reset_map()
        else:
            astar_solver = AStar(self.map_processor.map_graph)
            for name in astar_solver.h.keys():
                node_grid = tuple(map(int, name.split(',')))
                astar_solver.h[name] = math.sqrt(
                    (end_grid[0] - node_grid[0]) ** 2 + (end_grid[1] - node_grid[1]) ** 2
                )

            path_names, path_dist = astar_solver.solve(start_node, end_node)

            if path_names:
                self.get_logger().info(f"A* found a path of length {len(path_names)}")
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
                self._reset_map()

        astar_time = Float32()
        astar_time.data = float(self.get_clock().now().nanoseconds * 1e-9 - self.start_time)
        self.calc_time_pub.publish(astar_time)
        self.get_logger().info(f"Planning time: {astar_time.data:.4f} seconds")

        return path

    def _reset_map(self):
        """Reload and reinflate the map after a planning failure."""
        self.map_processor = MapProcessor(self.map_yaml_path)
        inflation_kernel = self.map_processor.rect_kernel(self.inflation_kernel_size, 1)
        self.map_processor.inflate_map(inflation_kernel)
        self.map_processor.get_graph_from_map()
        self.get_logger().info("Reset the map.")

    def move_ttbot(self, speed, heading):
        """Publish a Twist command on cmd_vel."""
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def publish_initial_pose(self):
        """Publish the initial pose to AMCL once and cancel the timer."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Publishing initial pose to AMCL-topic")
        self.initial_pose_pub.publish(pose_msg)
        self.initial_pose_timer.cancel()

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

    def _grid_to_world(self, grid_coords):
        """Convert grid indices to world coordinates using current map metadata."""
        origin_x = self.map_processor.map.origin[0]
        origin_y = self.map_processor.map.origin[1]
        resolution = self.map_processor.map.resolution
        map_height_pixels = self.map_processor.map.height

        grid_y, grid_x = grid_coords

        unflipped_grid_y = map_height_pixels - 1 - grid_y

        world_x = (grid_x + 0.5) * resolution + origin_x
        world_y = (unflipped_grid_y + 0.5) * resolution + origin_y

        return (world_x, world_y)

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

    def destroy_node(self):
        """Cancel timers and destroy the node."""
        if hasattr(self, 'initial_pose_timer'):
            self.initial_pose_timer.cancel()
        super().destroy_node()


def main(args=None):
    """Entry point: spin GlobalPlanner until interrupted."""
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
