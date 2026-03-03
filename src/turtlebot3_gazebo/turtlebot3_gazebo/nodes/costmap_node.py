# Extracted from monolithic nodes. No logic changes.

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point
from ament_index_python.packages import get_package_share_directory
import os

from turtlebot3_gazebo.common.map_utils import MapProcessor, SLAMMapProcessor

# --- Constants ---
# Default inflation kernel size (cells)
DEFAULT_INFLATION_KERNEL = 10
# Obstacle radius scale: inflation_kernel_size * resolution
# (used in replan_with_obstacle; kept here for reference)
OBSTACLE_CIRCLE_SCALE = 1


class CostmapServer(Node):
    """Builds and publishes an inflated occupancy costmap from a pre-built map or live SLAM data."""

    def __init__(self):
        """Initialise CostmapServer: load map, inflate, build graph, publish costmap."""
        super().__init__('costmap_server')

        self.declare_parameter('inflation_kernel_size', DEFAULT_INFLATION_KERNEL)
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('use_slam_map', False)

        self.inflation_kernel_size = self.get_parameter('inflation_kernel_size').get_parameter_value().integer_value
        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.use_slam_map = self.get_parameter('use_slam_map').get_parameter_value().bool_value

        if not self.map_yaml_path:
            pkg_share_path = get_package_share_directory('turtlebot3_gazebo')
            self.map_yaml_path = os.path.join(pkg_share_path, 'maps', 'map.yaml')

        # Publishers
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/custom_costmap', 1)
        self.new_obstacle_pub = self.create_publisher(Point, '/new_obstacle', 10)

        # Subscribers
        if self.use_slam_map:
            self.map_processor = SLAMMapProcessor()
            self.get_logger().info("SLAM mode: waiting for /map data...")
            self.create_subscription(OccupancyGrid, '/map', self._map_cbk, 1)
        else:
            self.get_logger().info(f"Loading pre-built map from '{self.map_yaml_path}'...")
            self.map_processor = MapProcessor(self.map_yaml_path)
            inflation_kernel = self.map_processor.rect_kernel(self.inflation_kernel_size, 1)
            self.map_processor.inflate_map(inflation_kernel)
            self.map_processor.get_graph_from_map()
            self.get_logger().info("Graph built successfully.")
            self._publish_inflated_map()

        self.create_subscription(PointStamped, '/detected_obstacle', self._detected_obstacle_cbk, 10)

    def _map_cbk(self, data):
        """Process live SLAM OccupancyGrid: inflate walls, rebuild graph, publish costmap."""
        self.map_processor.map.resolution = data.info.resolution
        self.map_processor.map.origin[0] = data.info.origin.position.x
        self.map_processor.map.origin[1] = data.info.origin.position.y
        self.map_processor.map.height = data.info.height
        self.map_processor.map.width = data.info.width

        H = data.info.height
        W = data.info.width

        map_2d = np.array(data.data, dtype=np.int8).reshape(H, W)

        wall_array_raw = np.zeros_like(map_2d, dtype=int)
        wall_array_raw[map_2d == 100] = 1
        current_wall_array = np.flipud(wall_array_raw)

        current_costmap = np.zeros_like(map_2d, dtype=int)
        current_costmap[(map_2d == 100) | (map_2d == -1)] = 1
        current_costmap = np.flipud(current_costmap)

        self.map_processor.inf_map_img_array = np.copy(current_costmap)

        inflation_kernel_matrix = self._rect_kernel(self.inflation_kernel_size * 2 + 1, 1)

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
        self._publish_inflated_map()

    def _rect_kernel(self, size, value):
        """Return a rectangular kernel of ones of the given size."""
        return np.ones(shape=(size, size))

    def _detected_obstacle_cbk(self, msg):
        """Inject detected obstacle into the inflated map, rebuild graph, publish updates."""
        obs_world_x = msg.point.x
        obs_world_y = msg.point.y
        estimated_diameter = msg.point.z

        obs_world = (obs_world_x, obs_world_y)
        obs_grid = self._world_to_grid(obs_world)
        obs_y, obs_x = obs_grid

        static_safety_buffer_m = self.inflation_kernel_size * self.map_processor.map.resolution
        total_radius_m = (estimated_diameter / 2.0) + static_safety_buffer_m
        resolution = self.map_processor.map.resolution
        pixel_radius = int(total_radius_m / resolution)

        self.get_logger().info(
            f"Injecting obstacle at grid ({obs_y}, {obs_x}) radius {pixel_radius} cells."
        )

        H, W = self.map_processor.inf_map_img_array.shape
        Y, X = np.ogrid[-obs_y:H - obs_y, -obs_x:W - obs_x]
        mask = X ** 2 + Y ** 2 <= pixel_radius ** 2
        self.map_processor.inf_map_img_array[mask] = 1

        self.map_processor.get_graph_from_map()
        self._publish_inflated_map()

        new_obs_msg = Point()
        new_obs_msg.x = obs_world_x
        new_obs_msg.y = obs_world_y
        new_obs_msg.z = 0.0
        self.new_obstacle_pub.publish(new_obs_msg)
        self.get_logger().info(
            f"Published /new_obstacle at x={obs_world_x:.2f}, y={obs_world_y:.2f}"
        )

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

    def _publish_inflated_map(self):
        """Convert inf_map_img_array to OccupancyGrid and publish to /custom_costmap."""
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

    def destroy_node(self):
        """Cancel timers and destroy the node."""
        super().destroy_node()


def main(args=None):
    """Entry point: spin CostmapServer until interrupted."""
    rclpy.init(args=args)
    node = CostmapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
