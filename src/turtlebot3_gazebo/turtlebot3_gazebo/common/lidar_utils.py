# Extracted from monolithic nodes. No logic changes.

import math
import numpy as np


def process_scan(ranges):
    """Clean LaserScan ranges: replace inf and 0 with nan. Returns numpy array."""
    ranges = np.array(ranges)
    ranges[np.isinf(ranges)] = np.nan
    ranges[ranges == 0.0] = np.nan
    return ranges


def get_front_slice(ranges):
    """Return indices [0:28] + [332:360] of the ranges array (front-facing)."""
    return np.concatenate((ranges[0:28], ranges[332:360]))


def get_back_slice(ranges):
    """Return indices [160:220] of the ranges array (rear-facing)."""
    return ranges[160:220]


def get_safety_slice(ranges):
    """Return indices [0:90] + [270:360] (wide front safety zone)."""
    return np.concatenate((ranges[0:90], ranges[270:360]))


def find_obstacle_cluster_bounds(ranges, min_index, jump_threshold):
    """Find start and end index of an obstacle cluster from the given scan index."""
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

    if index_start == min_index and index_end == min_index:
        pass

    return index_start, index_end


def estimate_obstacle_diameter(delta_angle_rad, distance_m):
    """Estimate obstacle diameter: L = 2 * distance_m * sin(delta_angle_rad / 2)."""
    if distance_m <= 0 or delta_angle_rad <= 0:
        return 0.0
    estimated_diameter = 2 * distance_m * math.sin(delta_angle_rad / 2.0)
    return estimated_diameter
