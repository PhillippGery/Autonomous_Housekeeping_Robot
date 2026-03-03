# Extracted from monolithic nodes. No logic changes.

import os
import math
import random
import numpy as np
from copy import copy
from PIL import Image
import yaml

from turtlebot3_gazebo.common.graph_utils import Node, Tree, Queue, AStar


class Map():
    """Parses a ROS map YAML file and loads the corresponding occupancy grid image."""

    def __init__(self, name):
        """Load the YAML metadata and derive a binary obstacle grid from the PGM image."""
        with open(name, 'r') as f:
            self.map_yaml = yaml.safe_load(f)

        map_directory = os.path.dirname(name)
        image_filename = self.map_yaml['image']
        self.image_file_name = os.path.join(map_directory, image_filename)

        self.resolution = self.map_yaml['resolution']
        self.origin = self.map_yaml['origin']
        self.negate = self.map_yaml['negate']
        self.occupied_thresh = self.map_yaml['occupied_thresh']
        self.free_thresh = self.map_yaml['free_thresh']

        map_image = Image.open(self.image_file_name)
        raw_image_array = np.array(map_image)

        self.height = raw_image_array.shape[0]

        free_pixel_threshold = int(255 * (1 - self.free_thresh))

        grid = np.ones_like(raw_image_array, dtype=int)
        grid[raw_image_array > free_pixel_threshold] = 0

        self.image_array = grid


class MapProcessor():
    """Inflates obstacles in a pre-built map and builds an 8-connected A* graph."""

    def __init__(self, name):
        """Load the map from the YAML path and initialise the inflated array and graph."""
        self.map = Map(name)
        self.inf_map_img_array = np.copy(self.map.image_array)
        self.map_graph = Tree(name)

    def __modify_map_pixel(self, map_array, i, j, value, absolute):
        """Set or increment a single pixel in map_array if coordinates are in bounds."""
        if (i >= 0) and (i < map_array.shape[0]) and (j >= 0) and (j < map_array.shape[1]):
            if absolute:
                map_array[i][j] = value
            else:
                map_array[i][j] += value

    def __inflate_obstacle(self, kernel, map_array, i, j, absolute):
        """Apply the inflation kernel centred on pixel (i, j) in map_array."""
        dx = int(kernel.shape[0] // 2)
        dy = int(kernel.shape[1] // 2)
        if (dx == 0) and (dy == 0):
            self.__modify_map_pixel(map_array, i, j, kernel[0][0], absolute)
        else:
            for k in range(i - dx, i + dx):
                for l in range(j - dy, j + dy):
                    self.__modify_map_pixel(map_array, k, l, kernel[k - i + dx][l - j + dy], absolute)

    def inflate_map(self, kernel, absolute=True):
        """Dilate every obstacle pixel by the kernel to produce inf_map_img_array."""
        self.inf_map_img_array = np.copy(self.map.image_array)
        obstacle_indices = np.where(self.map.image_array == 1)
        for i, j in zip(*obstacle_indices):
            self.__inflate_obstacle(kernel, self.inf_map_img_array, i, j, absolute)
        self.inf_map_img_array[self.inf_map_img_array > 0] = 1

    def get_graph_from_map(self):
        """Build an 8-connected graph of free pixels from inf_map_img_array."""
        self.map_graph.g = {}
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    node = Node('%d,%d' % (i, j))
                    self.map_graph.add_node(node)

        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    if i > 0:
                        if self.inf_map_img_array[i - 1][j] == 0:
                            child_up = self.map_graph.g['%d,%d' % (i - 1, j)]
                            self.map_graph.g['%d,%d' % (i, j)].add_children([child_up], [1])
                    if i < (self.map.image_array.shape[0] - 1):
                        if self.inf_map_img_array[i + 1][j] == 0:
                            child_dw = self.map_graph.g['%d,%d' % (i + 1, j)]
                            self.map_graph.g['%d,%d' % (i, j)].add_children([child_dw], [1])
                    if j > 0:
                        if self.inf_map_img_array[i][j - 1] == 0:
                            child_lf = self.map_graph.g['%d,%d' % (i, j - 1)]
                            self.map_graph.g['%d,%d' % (i, j)].add_children([child_lf], [1])
                    if j < (self.map.image_array.shape[1] - 1):
                        if self.inf_map_img_array[i][j + 1] == 0:
                            child_rg = self.map_graph.g['%d,%d' % (i, j + 1)]
                            self.map_graph.g['%d,%d' % (i, j)].add_children([child_rg], [1])
                    if (i > 0) and (j > 0):
                        if self.inf_map_img_array[i - 1][j - 1] == 0:
                            child_up_lf = self.map_graph.g['%d,%d' % (i - 1, j - 1)]
                            self.map_graph.g['%d,%d' % (i, j)].add_children([child_up_lf], [np.sqrt(2)])
                    if (i > 0) and (j < (self.map.image_array.shape[1] - 1)):
                        if self.inf_map_img_array[i - 1][j + 1] == 0:
                            child_up_rg = self.map_graph.g['%d,%d' % (i - 1, j + 1)]
                            self.map_graph.g['%d,%d' % (i, j)].add_children([child_up_rg], [np.sqrt(2)])
                    if (i < (self.map.image_array.shape[0] - 1)) and (j > 0):
                        if self.inf_map_img_array[i + 1][j - 1] == 0:
                            child_dw_lf = self.map_graph.g['%d,%d' % (i + 1, j - 1)]
                            self.map_graph.g['%d,%d' % (i, j)].add_children([child_dw_lf], [np.sqrt(2)])
                    if (i < (self.map.image_array.shape[0] - 1)) and (j < (self.map.image_array.shape[1] - 1)):
                        if self.inf_map_img_array[i + 1][j + 1] == 0:
                            child_dw_rg = self.map_graph.g['%d,%d' % (i + 1, j + 1)]
                            self.map_graph.g['%d,%d' % (i, j)].add_children([child_dw_rg], [np.sqrt(2)])

    def gaussian_kernel(self, size, sigma=1):
        """Return a normalised Gaussian kernel of the given size and sigma."""
        size = int(size) // 2
        x, y = np.mgrid[-size:size + 1, -size:size + 1]
        normal = 1 / (2.0 * np.pi * sigma ** 2)
        g = np.exp(-((x ** 2 + y ** 2) / (2.0 * sigma ** 2))) * normal
        r = np.max(g) - np.min(g)
        sm = (g - np.min(g)) * 1 / r
        return sm

    def rect_kernel(self, size, value):
        """Return a rectangular kernel of ones with the given size."""
        m = np.ones(shape=(size, size))
        return m

    def draw_path(self, path):
        """Return a copy of inf_map_img_array with path cells set to 0.5."""
        path_tuple_list = []
        path_array = copy(self.inf_map_img_array)
        for idx in path:
            tup = tuple(map(int, idx.split(',')))
            path_tuple_list.append(tup)
            path_array[tup] = 0.5
        return path_array


class MapData:
    """Plain-data container for SLAM map metadata."""

    def __init__(self):
        """Initialise default metadata values."""
        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]
        self.height = 0
        self.width = 0


class SLAMMapProcessor:
    """Adapts live SLAM OccupancyGrid data into the same graph interface as MapProcessor."""

    def __init__(self):
        """Initialise the SLAM map processor with empty metadata and graph."""
        self.map = MapData()
        self.map_graph = Tree("SLAM_Graph")
        self.inf_map_img_array = np.array([[]], dtype=int)

    def get_graph_from_map(self):
        """Build an 8-connected graph from the current inf_map_img_array."""
        self.map_graph.g = {}
        H, W = self.inf_map_img_array.shape
        for i in range(H):
            for j in range(W):
                if self.inf_map_img_array[i][j] == 0:
                    node = Node('%d,%d' % (i, j))
                    self.map_graph.add_node(node)

        for i in range(H):
            for j in range(W):
                if self.inf_map_img_array[i][j] == 0:
                    if (i > 0) and self.inf_map_img_array[i - 1][j] == 0:
                        child_up = self.map_graph.g['%d,%d' % (i - 1, j)]
                        self.map_graph.g['%d,%d' % (i, j)].add_children([child_up], [1])
                    if (i < (H - 1)) and self.inf_map_img_array[i + 1][j] == 0:
                        child_dw = self.map_graph.g['%d,%d' % (i + 1, j)]
                        self.map_graph.g['%d,%d' % (i, j)].add_children([child_dw], [1])
                    if (j > 0) and self.inf_map_img_array[i][j - 1] == 0:
                        child_lf = self.map_graph.g['%d,%d' % (i, j - 1)]
                        self.map_graph.g['%d,%d' % (i, j)].add_children([child_lf], [1])
                    if (j < (W - 1)) and self.inf_map_img_array[i][j + 1] == 0:
                        child_rg = self.map_graph.g['%d,%d' % (i, j + 1)]
                        self.map_graph.g['%d,%d' % (i, j)].add_children([child_rg], [1])
                    if ((i > 0) and (j > 0)) and self.inf_map_img_array[i - 1][j - 1] == 0:
                        child_up_lf = self.map_graph.g['%d,%d' % (i - 1, j - 1)]
                        self.map_graph.g['%d,%d' % (i, j)].add_children([child_up_lf], [np.sqrt(2)])
                    if ((i > 0) and (j < (W - 1))) and self.inf_map_img_array[i - 1][j + 1] == 0:
                        child_up_rg = self.map_graph.g['%d,%d' % (i - 1, j + 1)]
                        self.map_graph.g['%d,%d' % (i, j)].add_children([child_up_rg], [np.sqrt(2)])
                    if ((i < (H - 1)) and (j > 0)) and self.inf_map_img_array[i + 1][j - 1] == 0:
                        child_dw_lf = self.map_graph.g['%d,%d' % (i + 1, j - 1)]
                        self.map_graph.g['%d,%d' % (i, j)].add_children([child_dw_lf], [np.sqrt(2)])
                    if ((i < (H - 1)) and (j < (W - 1))) and self.inf_map_img_array[i + 1][j + 1] == 0:
                        child_dw_rg = self.map_graph.g['%d,%d' % (i + 1, j + 1)]
                        self.map_graph.g['%d,%d' % (i, j)].add_children([child_dw_rg], [np.sqrt(2)])

    def _modify_map_pixel(self, map_array, i, j, value, absolute):
        """Set or threshold a single pixel in map_array if coordinates are in bounds."""
        H, W = map_array.shape
        if (i >= 0) and (i < H) and (j >= 0) and (j < W):
            if absolute:
                map_array[i][j] = value
            else:
                if map_array[i][j] == 0 or value == 1:
                    map_array[i][j] = 1

    def _inflate_obstacle(self, kernel, map_array, i, j, absolute):
        """Apply the inflation kernel centred on pixel (i, j) in map_array."""
        dx = int(kernel.shape[0] // 2)
        dy = int(kernel.shape[1] // 2)
        if (dx == 0) and (dy == 0):
            self._modify_map_pixel(map_array, i, j, kernel[0][0], absolute)
        else:
            for k in range(i - dx, i + dx + 1):
                for l in range(j - dy, j + dy + 1):
                    self._modify_map_pixel(map_array, k, l, kernel[k - i + dx][l - j + dy], absolute)


class RRTStarGrid:
    """RRT* path planner adapted for a discrete occupancy grid environment."""

    def __init__(self, map_processor, step_size=5, search_radius=10, max_iterations=5000, node_instance=None):
        """Initialise RRT* with the given MapProcessor and planning parameters."""
        self.map_processor = map_processor
        self.map_array = map_processor.inf_map_img_array
        self.height, self.width = self.map_array.shape
        self.step_size = step_size
        self.search_radius = search_radius
        self.max_iterations = max_iterations
        self.node_instance = node_instance
        self.nodes = {}

    def _get_node_name(self, y, x):
        """Return the canonical string name for grid coordinates (y, x)."""
        return f"{y},{x}"

    def _sample_free(self):
        """Randomly sample a free (value 0) pixel from the grid."""
        while True:
            y = random.randint(0, self.height - 1)
            x = random.randint(0, self.width - 1)
            if self.map_array[y, x] == 0:
                return y, x

    def _find_nearest_node(self, y_rand, x_rand):
        """Return the name of the tree node nearest to (y_rand, x_rand)."""
        min_dist_sq = float('inf')
        nearest_node_name = None
        for name in self.nodes.keys():
            y_node, x_node = map(int, name.split(','))
            dist_sq = (x_rand - x_node) ** 2 + (y_rand - y_node) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                nearest_node_name = name
        return nearest_node_name

    def _steer(self, name_nearest, y_rand, x_rand):
        """Steer step_size cells from name_nearest toward (y_rand, x_rand); return new node or None."""
        y_near, x_near = map(int, name_nearest.split(','))
        angle = math.atan2(y_rand - y_near, x_rand - x_near)
        y_new = y_near + int(self.step_size * math.sin(angle))
        x_new = x_near + int(self.step_size * math.cos(angle))
        y_new = np.clip(y_new, 0, self.height - 1)
        x_new = np.clip(x_new, 0, self.width - 1)
        name_new = self._get_node_name(y_new, x_new)
        if self.map_array[y_new, x_new] == 0 and self.node_instance._is_path_clear((y_near, x_near), (y_new, x_new)):
            return name_new, y_new, x_new
        return None, None, None

    def _find_near_nodes(self, y_new, x_new):
        """Return all tree nodes within search_radius of (y_new, x_new) with distances."""
        near_nodes = []
        for name in self.nodes.keys():
            y_node, x_node = map(int, name.split(','))
            dist_sq = (x_new - x_node) ** 2 + (y_new - y_node) ** 2
            if dist_sq <= self.search_radius ** 2:
                distance = math.sqrt(dist_sq)
                near_nodes.append((name, distance))
        return near_nodes

    def _rewire(self, name_new, near_nodes):
        """Rewire nearby nodes through name_new if a cheaper collision-free path exists."""
        cost_new = self.nodes[name_new]['cost']
        for name_near, dist_to_near in near_nodes:
            if name_near == name_new:
                continue
            cost_through_new = cost_new + dist_to_near
            if cost_through_new < self.nodes[name_near]['cost']:
                y_new, x_new = map(int, name_new.split(','))
                y_near, x_near = map(int, name_near.split(','))
                if self.node_instance._is_path_clear((y_new, x_new), (y_near, x_near)):
                    self.nodes[name_near]['parent'] = name_new
                    self.nodes[name_near]['cost'] = cost_through_new

    def _reconstruct_rrt_path(self, start_name, end_name):
        """Walk parent pointers from end_name back to start_name to build the path."""
        path_names = []
        current = end_name
        total_cost = self.nodes[end_name]['cost']
        while current is not None:
            path_names.append(current)
            if current == start_name:
                break
            current = self.nodes[current]['parent']
        return path_names[::-1], total_cost

    def solve(self, start_grid, end_grid):
        """Run the RRT* main loop from start_grid to end_grid; return (path_names, cost)."""
        start_name = self._get_node_name(*start_grid)
        end_name = self._get_node_name(*end_grid)

        self.nodes = {
            start_name: {'parent': None, 'cost': 0.0}
        }

        final_path = None
        min_cost = float('inf')

        for i in range(self.max_iterations):
            y_rand, x_rand = self._sample_free()
            name_nearest = self._find_nearest_node(y_rand, x_rand)
            if name_nearest is None:
                continue
            name_new, y_new, x_new = self._steer(name_nearest, y_rand, x_rand)
            if name_new is None or name_new in self.nodes:
                continue
            near_nodes = self._find_near_nodes(y_new, x_new)
            cost_min = self.nodes[name_nearest]['cost'] + self.step_size
            parent_name = name_nearest
            for name_near, dist_to_near in near_nodes:
                cost_through_near = self.nodes[name_near]['cost'] + dist_to_near
                y_near, x_near = map(int, name_near.split(','))
                if (cost_through_near < cost_min and
                        self.node_instance._is_path_clear((y_near, x_near), (y_new, x_new))):
                    cost_min = cost_through_near
                    parent_name = name_near
            self.nodes[name_new] = {'parent': parent_name, 'cost': cost_min}
            self.rewire(name_new, near_nodes)
            dist_to_goal = math.sqrt((x_new - end_grid[1]) ** 2 + (y_new - end_grid[0]) ** 2)
            if dist_to_goal < self.step_size and self.node_instance._is_path_clear((y_new, x_new), end_grid):
                cost_to_goal = cost_min + dist_to_goal
                if cost_to_goal < min_cost:
                    self.nodes[end_name] = {'parent': name_new, 'cost': cost_to_goal}
                    final_path, final_cost = self._reconstruct_rrt_path(start_name, end_name)
                    min_cost = final_cost

        if final_path:
            return final_path, min_cost

        return [], np.inf

    def rewire(self, name_new, near_nodes):
        """Public wrapper that delegates to _rewire."""
        self._rewire(name_new, near_nodes)

    def path_to_pose_stamped(self, path_names, path_dist):
        """Convert grid-name path list to a nav_msgs/Path via the node instance."""
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        path = Path()
        path.header.stamp = self.node_instance.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for name in path_names:
            grid_coords = tuple(map(int, name.split(',')))
            world_coords = self.node_instance._grid_to_world(grid_coords)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = world_coords[0]
            pose.pose.position.y = world_coords[1]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path
