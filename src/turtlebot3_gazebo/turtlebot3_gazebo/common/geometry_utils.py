# Extracted from monolithic nodes. No logic changes.

import math
from geometry_msgs.msg import PoseStamped, Quaternion


def world_to_grid(world_coords, origin_x, origin_y, resolution, map_height):
    """Convert world coordinates to grid cell indices."""
    relative_x = world_coords[0] - origin_x
    relative_y = world_coords[1] - origin_y
    grid_x = int(relative_x / resolution)
    grid_y = map_height - 1 - int(relative_y / resolution)
    return (grid_y, grid_x)


def grid_to_world(grid_coords, origin_x, origin_y, resolution, map_height):
    """Convert grid cell indices to world coordinates."""
    grid_y, grid_x = grid_coords
    unflipped_grid_y = map_height - 1 - grid_y
    world_x = (grid_x + 0.5) * resolution + origin_x
    world_y = (unflipped_grid_y + 0.5) * resolution + origin_y
    return (world_x, world_y)


def yaw_from_quaternion(quat):
    """Extract yaw angle from a quaternion."""
    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi] range."""
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle


def pose_stamped_from_world(x, y, yaw, frame_id, stamp):
    """Build a PoseStamped message from world x, y, yaw."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose
