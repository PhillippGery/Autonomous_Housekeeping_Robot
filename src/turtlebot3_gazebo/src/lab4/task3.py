#!/usr/bin/env python3

import math
import sys
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

from PIL import Image as PILImage
import yaml
import pandas as pd
from copy import copy
import time
from queue import PriorityQueue
# Import other python packages that you think necessary


class Task3(Node):
    """
    Environment localization and navigation task.
    You can also inherit from Task 2 node if most of the code is duplicated
    """
    def __init__(self):
        super().__init__('task3_node')
        self.timer = self.create_timer(0.1, self.timer_cb)
        # Fill in the initialization member variables that you need

        self.path = Path()
        self.goal_pose = None
        self.ttbot_pose = None
        self.start_time = 0.0
        self.state = 'IDLE'  # Possible states: IDLE, ASTARPATH_FOLLOWING, OBSTACLE_AVOIDANCE

        # Map variables
        pkg_share_path = get_package_share_directory('turtlebot3_gazebo')
        default_map_path = os.path.join(pkg_share_path, 'maps', 'Map.yaml')
        self.declare_parameter('map_yaml_path', default_map_path)
        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value

        #Obstacle avoidance variables
        self.obstacle_state = 'CLEAR'
        self.min_front_obstacle_distance = 0.35  # Meters
        self.min_back_obstacle_distance = 0.25   # Meters
        self.safety_distance = 0.2  
        self.obstacle_inflation_radius_m = 0.4
        self.retreat_distance = -0.20      # Distance to retreat in meters 
        self.current_retreat_distance = 0.0 # Tracker for retreat distance 
        self.retreat_speed = -0.15          # Reverse linear speed (m/s)
        self.max_obstacle_diameter = 0.4    # Maximum diameter of obstacles to consider (meters)
      
        self.inflation_kernel_size = 10
        self.max_dist_alternate_Ponit = 1.0  # if start or stop pose is not valid, search for alternate point within this distance (meters)
        self.max_angle_alpha_to_startdrive = 1.0  # radian (57 degrees)
        
        self.k_rho = 0.8608         # Proportional gain for linear speed
        self.kp_angular = 2.0747    # Proportional gain for angular velocity
        self.ki_angular = 0.1692    # Integral gain for angular velocity
        self.kd_angular = -0.02   # Derivative gain for angular velocity
        self.k_beta = -0.1        # Gain for final orientation correction

        self.yaw_tolerance = 0.1  # Radians (~5.7 degrees)
        self.kp_final_yaw = 0.8   # Proportional gain for final yaw correction --> faster

        self.min_lookahead_dist = 0.2  # The smallest the lookahead distance can be
        self.lookahead_ratio = 0.5     # How much the lookahead increases with speed

        #Controller Ball follower Init prms
        self.BF_pid_p_angular = 0.4
        self.BF_pid_i_angular = 0.1
        self.BF_pid_d_angular = 0.4
        self.BF_p_linear = 0.3
        self.BF_integral_angular = 0.0
        self.BF_previous_error_angular = 0.0       
        self.BF_des_ball_size = 150 # desired ball size in pixels

        # Camera parameters
        self.camera_hfov_degrees = 60.0
        self.BF_max_linear_speed = 0.2
        self.BF_max_angular_speed = 1.0
        self.last_time = None
        self.last_detection_time = self.get_clock().now()

        # Color parameters
        self.lower_red_1 = np.array([0, 130, 100])
        self.upper_red_1 = np.array([10, 255, 255])
        self.lower_red_2 = np.array([170, 130, 100])
        self.upper_red_2 = np.array([180, 255, 255])

        # Speed and tolerance settings
        self.speed_max = 0.31
        self.rotspeed_max = 1.9
        self.goal_tolerance = 0.1
        self.align_threshold = 0.4 #
        self.last_commanded_speed = 0.0
        self.use_dynamic_lookahead = True # Enable dynamic lookahead based on speed
        self.use_line_of_sight_check = True # Enable line-of-sight shortcut checking
        self.shortcut_active = False #if a shortcut is being taken true

        self.get_logger().info(f"Loading map from '{self.map_yaml_path}' and building graph...")
        self.map_processor = MapProcessor(self.map_yaml_path)
        inflation_kernel = self.map_processor.rect_kernel(self.inflation_kernel_size, 1)
        self.map_processor.inflate_map(inflation_kernel)
        
        self.map_processor.get_graph_from_map()
        self.get_logger().info("Graph built successfully.")

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.create_subscription(LaserScan, '/scan', self._check_for_obstacles, 10)
     

        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time',10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/custom_costmap', 1)
        self.bbox_publisher = self.create_publisher(BoundingBox2D, '/bbox', 10)

        #set inatl pose automaticly 
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.initial_pose_timer = self.create_timer(2.0, self.publish_initial_pose)

        self.bridge = CvBridge()
        self.ranges = []

        self.rate = 10.0
        self.timer = self.create_timer(1.0 / self.rate, self.run_loop)

        self._publish_inflated_map()

    def timer_cb(self):
        self.get_logger().info('Task3 node is alive.', throttle_duration_sec=1)
        # Feel free to delete this line, and write your algorithm in this callback function

    def camera_callback(self, msg):


        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return  
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt == 0:
            return

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        

        red_mask1 = cv2.inRange(hsv_image, self.lower_red_1, self.upper_red_1)
        red_mask2 = cv2.inRange(hsv_image, self.lower_red_2, self.upper_red_2)

        # Combine the two masks
        mask = cv2.bitwise_or(red_mask1, red_mask2)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        potential_balls = []

        # if contours:
        #     for contour in contours:
        #         area = cv2.contourArea(contour)
        #         perimeter = cv2.arcLength(contour, True)
                
        #         # Avoid division by zero
        #         if perimeter == 0:
        #             continue
                    
        #         # check if circle bacause detacting red briks ....
        #         circularity = (4 * np.pi * area) / (perimeter * perimeter)
                
        #         circularity_threshold = 0.8
        #         min_area = 400 # pixels
                
        #         if circularity > circularity_threshold and area > min_area:
        #             potential_balls.append(contour)
            
        # if potential_balls:

        #     self.state = "TRACKING"

        #     self.last_detection_time = self.get_clock().now()
        #     # Find the largest contour *among the circular ones*
        #     largest_contour = max(potential_balls, key=cv2.contourArea)
            

        #     #  box coordinates
        #     x, y, w, h = cv2.boundingRect(largest_contour)

        #     # center
        #     centroid_x = x + w // 2
        #     centroid_y = y + h // 2

        #     image_width = cv_image.shape[1]
        #     image_center_x = image_width // 2

        #     #create bounding box as done in previous task_5
        #     self.get_logger().info(f"Object Centroid: (x={centroid_x}, y={centroid_y}), Size: (w={w}, h={h})")
        #     bbox_msg = BoundingBox2D()                
        #     bbox_msg.center.position.x = float(centroid_x)
        #     bbox_msg.center.position.y = float(centroid_y)
        #     bbox_msg.center.theta = 0.0
        #     bbox_msg.size_x = float(w)
        #     bbox_msg.size_y = float(h)                
        #     self.bbox_publisher.publish(bbox_msg)                
        #     cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        #     # error detection 
        #     #distance_to_object = depth_image[int(y), int(x)]
        #     #self.get_logger().info(f"Distance to Object: {distance_to_object} meters")

        #     distance_to_object = (self.BF_des_ball_size)/w
        #     error_distance = distance_to_object - 1.0  # desired distance is 1.0 meter
        #     self.get_logger().info(f"Distance: {distance_to_object}")

        #     error_x_unnorm = centroid_x - image_center_x
        #     error_x = error_x_unnorm / image_center_x  # Normalize error
        #     #self.get_logger().info(f"Error X: {error_x} pixels")
        #     #Heading error in Pixels
        #     heading_error_degrees = error_x_unnorm * (self.camera_hfov_degrees / image_width)
        #     self.get_logger().info(f"Heading Error: {heading_error_degrees:.2f} degrees")


        #     num_rays = len(self.ranges)

        #     target_index = int(round(heading_error_degrees))              
        #     # 3. Define the slice width (you asked for +- 5)
        #     slice_half_width = 5 
        #     # 4. Build a list of indices, handling the wrap-around
        #     indices_to_check = []
        #     for i in range(-slice_half_width, slice_half_width + 1):
        #         # Use modulo (%) to wrap indices (e.g., -2 % 360 = 358)
        #         idx = (target_index + i) % num_rays
        #         indices_to_check.append(idx)
                
        #     # 5. Get the slice from the ranges array using the index list
        #     ball_distance_slice = self.ranges[indices_to_check]
            
        #     try:
        #         ball_distance = np.nanmin(ball_distance_slice)
        #         self.get_logger().info(f"Ball Distance from scan: {ball_distance} meters")
        #     except ValueError:
        #         self.get_logger().warn("laser readings for ball distance are 'nan'. Skipping loop.", throttle_duration_sec=1)
                

        #     #Controller
        #     #error_x = heading_error_degrees
        #     self.BF_integral_angular += error_x
        #     derivative_angular = error_x - self.BF_previous_error_angular

        #     p_term = self.BF_pid_p_angular * error_x
        #     i_term = (self.BF_pid_i_angular * self.BF_integral_angular)*dt
        #     d_term = (self.BF_pid_d_angular * derivative_angular)/dt

        #     #windup for integral term
        #     i_term = np.clip(i_term, -0.5, 0.5)

        #     if abs(error_distance) > 0.1:
        #         linear_velocity = self.BF_p_linear * error_distance
        #     else:
        #         linear_velocity = 0.0
            
        #     if abs(error_x) < 0.05:
        #         angular_velocity = 0.0
        #         self.BF_previous_error_angular = 0.0
        #         self.BF_integral_angular = 0.0
        #         self.get_logger().info("Object centered.")
        #     else:
        #         angular_velocity = p_term + i_term + d_term
        #         self.BF_previous_error_angular = error_x


            

        #     #twist_msg.linear.x = np.clip(linear_velocity, -self.BF_max_linear_speed, self.BF_max_linear_speed)
        #     #twist_msg.angular.z = np.clip(-angular_velocity, -self.BF_max_angular_speed, self.BF_max_angular_speed)
        #     #self.publisher_.publish(twist_msg)

        # else: 

        #     # When no object wait 3 sec then turn to search
        #     # when object in room and not covered by obstacle robot wil find it          
        #     time_since_last_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        #     self.get_logger().info(f"Time since last detection: {time_since_last_detection} seconds")   
        #     if time_since_last_detection > 3.0 and time_since_last_detection <= 13.0:
        #         self.get_logger().info("Turning while Searching for object...")
        #         twist_msg.linear.x = 0.0
        #         twist_msg.angular.z = 0.3
        #         self.publisher_.publish(twist_msg)
        #         self.BF_previous_error_angular = 0.0
        #         self.BF_integral_angular = 0.0
        #         self.wall_follower.reset_pid()
        #         self.state = "TRACKING"

        #     elif time_since_last_detection > 13.0:
        #         self.get_logger().info("Exploring with Wallfolower..." )
        #         self.BF_previous_error_angular = 0.0
        #         self.BF_integral_angular = 0.0
        #         # call Wallfollower
        #         self.state = "SEARCHING"
                
        #     else:
        #         self.get_logger().info("No object detected.")
        #         twist_msg.linear.x = 0.0
        #         twist_msg.angular.z = 0.0
        #         self.publisher_.publish(twist_msg)
        #         self.BF_previous_error_angular = 0.0
        #         self.BF_integral_angular = 0.0
        #         self.wall_follower.reset_pid()
        #         self.state = "DEFAULT"

                
        
        # Display vieo
        scale = 0.7

        # Calculate the new dimensions
        width = int(cv_image.shape[1] * scale)
        height = int(cv_image.shape[0] * scale)
        dim = (width, height)

        # Resize the image
        resized_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

        # Display the RESIZED image
        cv2.imshow("Object Detector", resized_image)
        cv2.waitKey(1)

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
                  
        self.goal_pose = data
        self.get_logger().info(
            'New goal received: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))
        
        if self.ttbot_pose is None:
            self.get_logger().warn("Cannot plan path, robot pose is not yet available.")
            return
            
        # Call the path planner to generate a new path
        self.path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
        
        # If a valid path was found, publish it and reset the path follower index and shortcut flag
        if self.path.poses:
            self.path_pub.publish(self.path)
            self.current_path_idx = 0
            self.shortcut_active = False
        else:
            self.get_logger().warn("A* failed to find a path to the goal.")
            self.move_ttbot(0.0, 0.0)

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose = data.pose.pose
        self.ttbot_pose = pose_stamped

    def a_star_path_planner(self, start_pose, end_pose):
        """! A Start path planner. """
        path = Path()
        #self.get_logger().info('A* planner.\n> start: {},\n> end: {}'.format(start_pose.pose.position, end_pose.pose.position))
        self.start_time = self.get_clock().now().nanoseconds*1e-9 #Do not edit this line (required for autograder)

        start_world = (start_pose.pose.position.x, start_pose.pose.position.y)
        end_world = (end_pose.pose.position.x, end_pose.pose.position.y)
        
        start_grid = self._world_to_grid(start_world)
        #self.get_logger().info(f"Start grid: {start_grid}")
        end_grid = self._world_to_grid(end_world)
        #self.get_logger().info(f"End grid: {end_grid}")

        start_name = f"{start_grid[0]},{start_grid[1]}"
        #self.get_logger().info(f"start_name: {start_name}")
        end_name = f"{end_grid[0]},{end_grid[1]}"
        
        is_start_valid = start_name in self.map_processor.map_graph.g
        #self.get_logger().info(f"is_start_valid: {is_start_valid}")
        is_end_valid = end_name in self.map_processor.map_graph.g
        #self.get_logger().info(f"is_end_valid: {is_end_valid}")
        
        # 1. If the start pose is invalid, find the closest valid one.
        if not is_start_valid:
            self.get_logger().warn(f"start pose {start_name} is NOT valid. taking closest point.")
            
            min_dist_sq = float('inf')
            max_dist_sq = (self.max_dist_alternate_Ponit  / self.map_processor.map.resolution)**2
            closest_node_name = None
            original_strt_y, original_strt_x = start_grid

            # Iterate through all known valid nodes to find the nearest one
            for valid_node_name in self.map_processor.map_graph.g:
                node_y, node_x = map(int, valid_node_name.split(','))
                
      
                dist_sq = (node_x - original_strt_x)**2 + (node_y - original_strt_y)**2
                
                if dist_sq < min_dist_sq and dist_sq <= max_dist_sq:
                    min_dist_sq = dist_sq
                    closest_node_name = valid_node_name

            if closest_node_name:
                self.get_logger().info(f"New start set to the closest valid point: {closest_node_name}")
                # Update the goal variables to the new, valid location
                start_name = closest_node_name
                start_grid = tuple(map(int, closest_node_name.split(',')))
            else:
                self.get_logger().error("Could not find any valid nodes in the map. Planning failed.")
                return Path()
            
        # 2. If the end pose is invalid, find the closest valid one.
        if not is_end_valid:
            self.get_logger().warn(f"Goal pose {end_name} is NOT valid. taking closest point.")
            
            min_dist_sq = float('inf')
            max_dist_sq = (self.max_dist_alternate_Ponit  / self.map_processor.map.resolution)**2
            closest_node_name = None
            original_end_y, original_end_x = end_grid

            # Iterate through all known valid nodes to find the nearest one
            for valid_node_name in self.map_processor.map_graph.g:
                node_y, node_x = map(int, valid_node_name.split(','))
                
                dist_sq = (node_x - original_end_x)**2 + (node_y - original_end_y)**2
                
                if dist_sq < min_dist_sq and dist_sq <= max_dist_sq:
                    min_dist_sq = dist_sq
                    closest_node_name = valid_node_name

            if closest_node_name:
                self.get_logger().info(f"New goal set to the closest valid point: {closest_node_name}")
                # Update the goal variables to the new, valid location
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
            astar_solver.h[name] = math.sqrt((end_grid[0] - node_grid[0])**2 + (end_grid[1] - node_grid[1])**2)
        
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

        self.astarTime = Float32()
        self.astarTime.data = float(self.get_clock().now().nanoseconds*1e-9-self.start_time)
        self.calc_time_pub.publish(self.astarTime)
        self.get_logger().info(f"A* planning time: {self.astarTime.data:.4f} seconds")
        
        return path

    def get_path_idx(self, path, vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  current_goal_pose     PoseStamped object containing the current vehicle position.
        @return idx                   Position in the path pointing to the next goal pose to follow.
        """
        min_dist = float('inf')
        closest_idx = 0
        for i, pose in enumerate(path.poses):
            dx = pose.pose.position.x - vehicle_pose.pose.position.x
            dy = pose.pose.position.y - vehicle_pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        if self.use_dynamic_lookahead:
            lookahead_dist = self.last_commanded_speed * self.lookahead_ratio + self.min_lookahead_dist
        else:
            lookahead_dist = self.min_lookahead_dist # Fallback to a minimum value if disabled

        lookahead_idx = closest_idx
        for i in range(closest_idx, len(path.poses)):
            dx = path.poses[i].pose.position.x - vehicle_pose.pose.position.x
            dy = path.poses[i].pose.position.y - vehicle_pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist > lookahead_dist:
                lookahead_idx = i
                return lookahead_idx
        
        return len(path.poses) - 1

    def _is_path_clear(self, start_grid, end_grid):
        """!
        Checks if a straight line path between two grid points is clear of obstacles.
        Uses Bresenham's line algorithm.
        """
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
            # Check if current point is out of bounds or an obstacle
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
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        # Calculate distance and angle to the target waypoint
        
        dt = 1.0 / self.rate
        self.integral_error_angular = 0.0
        self.previous_error_angular = 0.0

        # 1. Get robot's current state (x_R, y_R, theta_R)
        robot_x = vehicle_pose.pose.position.x
        robot_y = vehicle_pose.pose.position.y
        quat = vehicle_pose.pose.orientation
        
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        robot_theta = math.atan2(siny_cosp, cosy_cosp)

        # 2. Get goal position (x_G, y_G)
        goal_x = current_goal_pose.pose.position.x
        goal_y = current_goal_pose.pose.position.y

        # 3. Calculate Cartesian error
        delta_x = goal_x - robot_x
        delta_y = goal_y - robot_y

        # 4. Transform error to Polar Coordinates (rho, alpha, beta)
        rho = math.sqrt(delta_x**2 + delta_y**2)
        alpha = -robot_theta + math.atan2(delta_y, delta_x)
        
        # Normalize alpha to be between -pi and pi
        if alpha > math.pi:
            alpha -= 2 * math.pi
        elif alpha < -math.pi:
            alpha += 2 * math.pi
        
        beta = -robot_theta - alpha
        
        p_term = self.kp_angular * alpha
        
        
        self.integral_error_angular += alpha * dt
        
        self.integral_error_angular = np.clip(self.integral_error_angular, -1.0, 1.0)
        i_term = self.ki_angular * self.integral_error_angular

        # Derivative term
        derivative_error = (alpha - self.previous_error_angular) / dt
        d_term = self.kd_angular * derivative_error

        # Update the previous error for the next iteration
        self.previous_error_angular = alpha

        # 5. Calculate final speed and heading
        if  abs(alpha) > self.max_angle_alpha_to_startdrive:  # If the angle to the goal is greater than 90 degrees, do not move forward
            speed = 0.0
        else:
            speed = min(self.k_rho * rho, self.speed_max)

        
        # Combine PID terms with the final orientation correction (beta)
        heading = p_term + i_term + d_term + self.k_beta * beta        
        heading = np.clip(heading, -self.rotspeed_max, self.rotspeed_max)

        #self.get_logger().info(f"Controller output -> Speed: {speed:.3f} m/s, Heading: {heading:.3f} rad/s")
            
        return speed, heading

    def move_ttbot(self, speed, heading):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired speed.
        @param  heading   Desired yaw angle.
        @return path      object containing the sequence of waypoints of the created path.
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def run_loop(self):
        """! Main loop of the node, called by a timer. """


        self.get_logger().info(f"Current State: {self.state}", throttle_duration_sec=4.0)

        if self.ttbot_pose is None or self.goal_pose is None or not self.path.poses:
            return       
        

        if self.state == 'IDLE':
            self.move_ttbot(0.0, 0.0)
            return
            
        
        elif self.obstacle_state == 'RETREATING':
            # Calculate distance moved in this cycle (dt)
            dt = 1.0 / self.rate
            distance_moved = abs(self.retreat_speed * dt)
            
            # Check if we have met the retreat distance goal
            if self.current_retreat_distance < abs(self.retreat_distance):
                # Haven't moved far enough back yet.
                self.move_ttbot(self.retreat_speed, 0.0) # Move backward (speed is negative)
                self.current_retreat_distance += distance_moved
                if self.obstacle_behind:
                    self.obstacle_state = 'CLEAR'
                    self.state = 'ASTARPATH_FOLLOWING'
                    self.get_logger().warn("Obstacle still behind during retreat", throttle_duration_sec=1.0)
                return # Skip path following
            else:
                # Retreat distance met. Stop and transition to REPLANNING.
                self.move_ttbot(0.0, 0.0) # Ensure robot is stopped
                self.get_logger().info("Retreat complete. Transitioning to REPLANNING.")                
                self.obstacle_state = 'ALIGNING_TO_OBS'            
                # After setting the state, the next cycle of _check_for_obstacles will call replan_with_obstacle.
                return
        elif self.obstacle_state == 'ALIGNING_TO_OBS':
            self.recalculate_obstacle_position()

        elif self.state == 'ASTARPATH_FOLLOWING' and self.obstacle_state == 'CLEAR':
            final_goal_pose = self.path.poses[-1]                           
            # Get the current goal from the path, possibly using line-of-sight optimization
            self.Path_optimizaton( final_goal_pose)
        
            # Calculate and publish robot commands
            speed, heading = self.path_follower(self.ttbot_pose, self.current_goal)
            self.move_ttbot(speed, heading)
            self.last_commanded_speed = speed


            # Check if we have reached the final goal position and orientation
            self.Check_alingment_Goal( final_goal_pose) 
        

        elif self.obstacle_state == 'REPLANNING':
            
            self.replan_with_obstacle()
            self.move_ttbot(0.0, 0.0)
            return
        
        
        

        

    def Check_alingment_Goal(self, final_goal_pose):
        """! Check if the robot has reached the final goal position and orientation.
        """

        dx = final_goal_pose.pose.position.x - self.ttbot_pose.pose.position.x
        dy = final_goal_pose.pose.position.y - self.ttbot_pose.pose.position.y
        dist_to_final_goal = math.sqrt(dx**2 + dy**2)
        
        # Check if we have reached the final goal position
        if dist_to_final_goal < self.goal_tolerance:
            # Position is correct. Now check for final orientation.
            
            # Get the desired goal orientation (yaw)
            goal_q = self.goal_pose.pose.orientation
            goal_siny_cosp = 2 * (goal_q.w * goal_q.z + goal_q.x * goal_q.y)
            goal_cosy_cosp = 1 - 2 * (goal_q.y * goal_q.y + goal_q.z * goal_q.z)
            goal_yaw = math.atan2(goal_siny_cosp, goal_cosy_cosp)

            # Get the robot's current orientation (yaw)
            robot_q = self.ttbot_pose.pose.orientation
            robot_siny_cosp = 2 * (robot_q.w * robot_q.z + robot_q.x * robot_q.y)
            robot_cosy_cosp = 1 - 2 * (robot_q.y * robot_q.y + robot_q.z * robot_q.z)
            robot_yaw = math.atan2(robot_siny_cosp, robot_cosy_cosp)
            
            # Calculate the orientation error
            yaw_error = goal_yaw - robot_yaw
            
            # Normalize the error to be between -pi and pi
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            # Check if the orientation is also within tolerance
            if abs(yaw_error) < self.yaw_tolerance:
                # GOAL REACHED: Position and orientation are correct. Stop everything.
                self.get_logger().info("Goal reached and Alinged to Goal Pose!")
                self.move_ttbot(0.0, 0.0)
                self.path = Path()
                self.goal_pose = None
                self.last_commanded_speed = 0.0
                self.state = 'IDLE'
                self.map_processor = MapProcessor(self.map_yaml_path)
                inflation_kernel = self.map_processor.rect_kernel(self.inflation_kernel_size, 1)
                self.map_processor.inflate_map(inflation_kernel)                
                self.map_processor.get_graph_from_map()
                self._publish_inflated_map()
                return
            else:
                # ALIGNING: Position is correct, so stop moving and only rotate.
                speed = 0.0
                heading = self.kp_final_yaw * yaw_error
                # Clamp rotation speed to max value
                heading = np.clip(heading, -self.rotspeed_max, self.rotspeed_max)
                self.move_ttbot(speed, heading)
                # Skip the rest of the loop while we are aligning
                return

    def Path_optimizaton(self, final_goal_pose):
        """! Path optimization using line-of-sight checking.
        and shortcutting to the final goal if possible.
        """

        self.current_goal = None

        if self.use_line_of_sight_check and not self.shortcut_active:
            start_grid = self._world_to_grid((self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))
            end_grid = self._world_to_grid((final_goal_pose.pose.position.x, final_goal_pose.pose.position.y))
            if self._is_path_clear(start_grid, end_grid):
                self.current_goal = final_goal_pose
                self.shortcut_active = True
                self.get_logger().info("Path is clear. Taking a shortcut to the final goal.")

        # If no clear path, use the standard logic
        if self.shortcut_active:
            self.current_goal = final_goal_pose
        else:
            idx = self.get_path_idx(self.path, self.ttbot_pose)
            self.current_goal = self.path.poses[idx]
        return self

    def _world_to_grid(self, world_coords):
        """!
        Converts continuous world coordinates (meters) to discrete grid coordinates (pixels).
        """
        origin_x = self.map_processor.map.origin[0]
        origin_y = self.map_processor.map.origin[1]
        resolution = self.map_processor.map.resolution
        map_height_pixels = self.map_processor.map.height

        # Translate world coordinates relative to the map's origin
        relative_x = world_coords[0] - origin_x
        relative_y = world_coords[1] - origin_y

        # Convert from meters to pixels
        grid_x = int(relative_x / resolution)
        grid_y = map_height_pixels - 1 - int(relative_y / resolution)

        # Return as (row, col) which matches NumPy array indexing
        return (grid_y, grid_x)

    def _grid_to_world(self, grid_coords):
        """!
        Converts discrete grid coordinates (pixels) back to continuous world coordinates (meters).
        """
        origin_x = self.map_processor.map.origin[0]
        origin_y = self.map_processor.map.origin[1]
        resolution = self.map_processor.map.resolution
        map_height_pixels = self.map_processor.map.height

        grid_y, grid_x = grid_coords # Expecting (row, col)

        unflipped_grid_y = map_height_pixels - 1 - grid_y

        world_x = (grid_x + 0.5) * resolution + origin_x
        world_y = (unflipped_grid_y + 0.5) * resolution + origin_y
        
        return (world_x, world_y)
    
    def _check_for_obstacles(self, scan_msg):
        """!
        Checks for obstacles while following the path.
        called from scan subscription
        indipendent from run_loop and path following
        """
        if self.ttbot_pose is None or self.goal_pose is None:
            return
        
        self.scan_msg = scan_msg
        
        # Process laser scan data
        ranges = np.array(scan_msg.ranges)
        ranges[np.isinf(ranges)] = np.nan
        ranges[ranges == 0.0] = np.nan
        self.front_slice = np.concatenate((ranges[0:28], ranges[332:360]))

        back_slice = ranges[160:220]
        safety_slice = np.concatenate((ranges[0:90], ranges[270:360]))
        
        try:
            self.front_dist = np.nanmin(self.front_slice)
            back_dist = np.nanmin(back_slice)
            safety_dist = np.nanmin(safety_slice)
        except ValueError:
            # happens if all readings in a slice are 'nan'
            self.get_logger().warn("laser readings are 'nan'. Skipping loop.", throttle_duration_sec=1)
            return
        
        obstacle_close = self.front_dist < self.min_front_obstacle_distance
        self.obstacle_behind = back_dist < self.min_back_obstacle_distance
        self.safety_dist_critical = safety_dist < self.safety_distance

        if obstacle_close and self.obstacle_state == 'CLEAR':
            # 1. New unmapped obstacle detected!
            self.get_logger().warn(f"Obstacle detected at {self.front_dist:.2f} m! Initiating avoidance maneuvers.")
            self.state = 'OBSTACLE_AVOIDANCE'
            self.obstacle_state = 'RETREATING'
            self.move_ttbot(0.0, 0.0) # Stop immediately
            self.current_retreat_distance = 0.0

        elif self.safety_dist_critical and self.obstacle_state == 'CLEAR':
            # Emergency stop if something is too close in safety zones
            self.state = 'OBSTACLE_AVOIDANCE'
            self.move_ttbot(0.0, 0.0) # Stop immediately
            self.get_logger().warn(f"Critical obstacle detected at {safety_dist:.2f} m in safety zone! Initiating avoidance maneuver.")
            self.obstacle_state = 'RETREATING'
            self.current_retreat_distance = 0.0

             
        if self.obstacle_state == 'DETECTED':
            self.current_retreat_distance = 0.0
            self.obstacle_state = 'RETREATING'

    def replan_with_obstacle(self):
        """!
        Adds the detected obstacle to the map costmap and runs A* planning again.
        """

        obs_grid_coords = self._world_to_grid(self.obstacle_pos_world)
        obs_y, obs_x = obs_grid_coords

        static_safety_buffer_m = self.inflation_kernel_size * self.map_processor.map.resolution

        total_radius_m = (self.estimated_diameter / 2.0) + static_safety_buffer_m
        self.get_logger().info(f"Inflating obstacle with total radius: {total_radius_m:.2f} m")
        

        resolution = self.map_processor.map.resolution
        pixel_radius = int(total_radius_m / resolution)
        
        self.get_logger().info(f"Adding obstacle to map at grid pos ({obs_y}, {obs_x}) with radius {pixel_radius} cells.")
        
        H, W = self.map_processor.inf_map_img_array.shape
        Y, X = np.ogrid[-obs_y:H-obs_y, -obs_x:W-obs_x]

        mask = X**2 + Y**2 <= pixel_radius**2
        

        self.map_processor.inf_map_img_array[mask] = 1 
        

        self._publish_inflated_map()
        self.map_processor.get_graph_from_map()
        
        self.get_logger().info("Graph updated with new obstacle.")

        
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        self.path_pub.publish(empty_path)

        # 2. Reset Internal Path Data
        self.path = Path()  # Clears the path object used for following
        self.current_path_idx = 0 
        self.shortcut_active = False
        


        if self.goal_pose is not None:
            #self.path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
            self.__goal_pose_cbk(self.goal_pose)
            if self.path.poses:
                self.get_logger().info("Replanning successful. Resuming path following.")
                self.path_pub.publish(self.path)
                self.current_path_idx = 0
                self.obstacle_state = 'CLEAR'
                
            else:
                self.get_logger().error("Replanning failed. No valid path to goal.")
                self.move_ttbot(0.0, 0.0)
        else:
            self.get_logger().warn("Goal was cleared during replanning. Stopping.")
            self.move_ttbot(0.0, 0.0)
            self.obstacle_state = 'CLEAR' # Clear status if no goal is set

    def publish_initial_pose(self):
        """
        Publishes the initial pose to AMCL to set the robot's starting position
        on the map and then cancels the timer.
        """
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set the position to the map's origin
        pose_msg.pose.pose.position.x = 0.0#-5.4
        pose_msg.pose.pose.position.y = 0.0#-6.18
        pose_msg.pose.pose.position.z = 0.0

        # Set the orientation (0 degrees yaw)
        pose_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Publishing initial pose to AMCL-topic")
        self.initial_pose_pub.publish(pose_msg)

        self.initial_pose_timer.cancel()

    def recalculate_obstacle_position(self):

        obstacle_slice_index = np.nanargmin(self.front_slice)

        if obstacle_slice_index < 28:
            full_scan_index = obstacle_slice_index
        else:
            full_scan_index = 332 + (obstacle_slice_index - 28)

        ranges = np.array(self.scan_msg.ranges) # Use raw ranges array
        idx_start, idx_end = self._find_obstacle_cluster_bounds(
            ranges, 
            full_scan_index, jump_threshold=0.3)
        
        angular_width_rad = (idx_end - idx_start + 1) * self.scan_msg.angle_increment


        center_index = int((idx_start + idx_end) / 2)
        

        raw_angle = center_index * self.scan_msg.angle_increment + self.scan_msg.angle_min

        angle_from_robot_centerline = raw_angle
        
        # Normalize to [-pi, pi] to ensure shortest signed angle
        if angle_from_robot_centerline > math.pi:
            angle_from_robot_centerline -= 2 * math.pi
        elif angle_from_robot_centerline < -math.pi:
            angle_from_robot_centerline += 2 * math.pi

        
        if abs(angle_from_robot_centerline) > math.radians(45)  and self.obstacle_state == 'ALIGNING_TO_OBS':
            self.move_ttbot(0.0, 0.0)
            if self.goal_pose is not None:
            #self.path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
                self.__goal_pose_cbk(self.goal_pose)
                if self.path.poses:
                    self.get_logger().info("Replanning successful. Resuming path following.")
                    self.path_pub.publish(self.path)
                    self.current_path_idx = 0
                    self.obstacle_state = 'CLEAR'

        elif abs(angle_from_robot_centerline) > math.radians(18) and self.obstacle_state == 'ALIGNING_TO_OBS':
            angular_speed = self.kp_final_yaw * angle_from_robot_centerline
            angular_speed = np.clip(angular_speed, -self.rotspeed_max, self.rotspeed_max)
            self.move_ttbot(0.0, angular_speed)
            #self.get_logger().info(f"Rotating to align with obstacle")
            self.get_logger().info(f"Angle to obstacle: {math.degrees(angle_from_robot_centerline):.2f} degrees")
            return
            
        else:
            self.move_ttbot(0.0, 0.0)
            self.get_logger().info(f"Aligned with obstacle. Calculating position.")
            self.obstacle_state = 'REPLANNING' 

        cluster_ranges = ranges[idx_start:idx_end + 1]
        obs_dist = np.nanmean(cluster_ranges)

        self.estimated_diameter = self.estimate_obstacle_diameter(angular_width_rad, obs_dist)
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
        
        # calculate obstacle position in world frame
        obs_world_x = self.ttbot_pose.pose.position.x + distance_to_center * math.cos(world_angle_to_center)
        obs_world_y = self.ttbot_pose.pose.position.y + distance_to_center * math.sin(world_angle_to_center)
        
        self.obstacle_pos_world = (obs_world_x, obs_world_y)
        
        self.get_logger().warn(f"Unmapped obstacle detected at {self.front_dist:.2f}m. Triggering replanning.")
        self.get_logger().info(f"Estimated obstacle position (world): x={obs_world_x:.2f}, y={obs_world_y:.2f}")
        
    def estimate_obstacle_diameter(self, delta_angle_rad, distance_m):
        """
        Estimates the diameter (chord length) of an object using its angular width and distance.
        """
        if distance_m <= 0 or delta_angle_rad <= 0:
            return 0.0

        # Formula: L = 2 * R * sin(Delta_theta / 2)
        estimated_diameter = 2 * distance_m * math.sin(delta_angle_rad / 2.0)
        
        return estimated_diameter

    def _find_obstacle_cluster_bounds(self, ranges, min_index, jump_threshold):
        """
        Dynamically finds the start and end indices of the obstacle cluster 
        centered around the minimum distance reading.
        
        @param ranges: The full 360-degree Lidar ranges array.
        @param min_index: The index of the minimum distance reading (front_dist).
        @param jump_threshold: The meter difference to signal an object edge.
        @return: (index_start, index_end) in the ranges array.
        """
        num_readings = len(ranges)
        
        # --- 1. Find Start Index (Scan backward from min_index) ---
        index_start = min_index
        for i in range(min_index, 0, -1):
            # Stop if distance jumps significantly
            if abs(ranges[i] - ranges[i - 1]) > jump_threshold:
                index_start = i
                break
        
        # --- 2. Find End Index (Scan forward from min_index) ---
        index_end = min_index
        for i in range(min_index, num_readings - 1):
            # Stop if distance jumps significantly
            if abs(ranges[i + 1] - ranges[i]) > jump_threshold:
                index_end = i
                break
        
        # Basic wrap-around check for simplicity (assumes obstacle is in the front slice)
        if index_start == min_index and index_end == min_index:
            # If no jump found, object might be large, use a default range (e.g., 20 indices wide)
            pass 

        return index_start, index_end

    def _publish_inflated_map(self):
        """
        Converts the local self.map_processor.inf_map_img_array (0 or 1) 
        into a nav_msgs/OccupancyGrid and publishes it to /custom_costmap.
        """
        if self.map_processor.inf_map_img_array.size == 0 or self.map_processor.map.height == 0:
            return

        map_msg = OccupancyGrid()
        map_info = self.map_processor.map

        # 1. Set Header (Essential for RViz visualization)
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map' 

        # 2. Set Info (Metadata)
        map_msg.info.resolution = map_info.resolution
        map_msg.info.width = self.map_processor.inf_map_img_array.shape[1]
        map_msg.info.height = self.map_processor.inf_map_img_array.shape[0]
        map_msg.info.origin.position.x = map_info.origin[0]
        map_msg.info.origin.position.y = map_info.origin[1]
        map_msg.info.origin.orientation.w = 1.0 # Assuming no initial map rotation

        # 3. Process Array Data
        # ROS OccupancyGrid expects 100=Occupied, 0=Free, -1=Unknown.
        # Your inf_map_img_array uses 1=Occupied, 0=Free.
        
        # Convert local values (0 or 1) to ROS values (0 or 100)
        ros_data = self.map_processor.inf_map_img_array * 100 
        
        # ROS message expects rows to be stored from bottom-to-top. 
        # Since your NumPy array is flipped (row 0 is top row), flip it back 
        # before flattening and converting to a list.
        ros_array_flipped = np.flipud(ros_data)
        
        # Flatten the array and convert it to a standard list of integers
        map_msg.data = ros_array_flipped.flatten().astype(np.int8).tolist()
        
        # 4. Publish
        self.inflated_map_pub.publish(map_msg)

class Queue():
    def __init__(self, init_queue = []):
        self.queue = copy(init_queue)
        self.start = 0
        self.end = len(self.queue)-1
    
    def __len__(self):
        numel = len(self.queue)
        return numel
    
    def __repr__(self):
        q = self.queue
        tmpstr = ""
        for i in range(len(self.queue)):
            flag = False
            if(i == self.start):
                tmpstr += "<"
                flag = True
            if(i == self.end):
                tmpstr += ">"
                flag = True
            
            if(flag):
                tmpstr += '| ' + str(q[i]) + '|\n'
            else:
                tmpstr += ' | ' + str(q[i]) + '|\n'
            
        return tmpstr
    
    def __call__(self):
        return self.queue
    
    def initialize_queue(self,init_queue = []):
        self.queue = copy(init_queue)
    
    def sort(self,key=str.lower):
        self.queue = sorted(self.queue,key=key)
        
    def push(self,data):
        self.queue.append(data)
        self.end += 1
    
    def pop(self):
        p = self.queue.pop(self.start)
        self.end = len(self.queue)-1
        return p
    
class Node():
    def __init__(self,name):
        self.name = name
        self.children = []
        self.weight = []
        
    def __repr__(self):
        return self.name
        
    def add_children(self,node,w=None):
        if w == None:
            w = [1]*len(node)
        self.children.extend(node)
        self.weight.extend(w)
    
class Tree():
    def __init__(self,name):
        self.name = name
        self.root = 0
        self.end = 0
        self.g = {}
    #     self.g_visual = Graph('G')
    
    # def __call__(self):
    #     for name,node in self.g.items():
    #         if(self.root == name):
    #             self.g_visual.node(name,name,color='red')
    #         elif(self.end == name):
    #             self.g_visual.node(name,name,color='blue')
    #         else:
    #             self.g_visual.node(name,name)
    #         for i in range(len(node.children)):
    #             c = node.children[i]
    #             w = node.weight[i]
    #             #print('%s -> %s'%(name,c.name))
    #             if w == 0:
    #                 self.g_visual.edge(name,c.name)
    #             else:
    #                 self.g_visual.edge(name,c.name,label=str(w))
        return #self.g_visual
    
    def add_node(self, node, start = False, end = False):
        self.g[node.name] = node
        if(start):
            self.root = node.name
        elif(end):
            self.end = node.name
            
    def set_as_root(self,node):
        # These are exclusive conditions
        self.root = True
        self.end = False
    
    def set_as_end(self,node):
        # These are exclusive conditions
        self.root = False
        self.end = True   
        
class AStar():
    def __init__(self,in_tree):
        self.in_tree = in_tree
        self.q = Queue()
        self.dist = {name:np.inf for name,node in in_tree.g.items()}
        self.h = {name:0 for name,node in in_tree.g.items()}
        
        self.via = {name: None for name in in_tree.g}
        for __,node in in_tree.g.items():
            self.q.push(node)
     
    def __get_f_score(self,node):
        return self.dist[node] + self.h[node]
    
    def solve(self, sn, en):

        open_set = PriorityQueue()

        entry_count = 0
        self.dist[sn.name] = 0
        start_f_score = self.__get_f_score(sn.name)
        
        open_set.put((start_f_score, entry_count, sn))

        entry_count += 1

        while not open_set.empty():
            #  3rd position (index 2) !!!
            current_node = open_set.get()[2]

            #check if note i Goal
            if current_node.name == en.name:
                return self.reconstruct_path(sn.name, en.name)

            # Explore childs 
            for i, child_node in enumerate(current_node.children):
                weight = current_node.weight[i]
                tentative_g_score = self.dist[current_node.name] + weight

                if tentative_g_score < self.dist[child_node.name]:
                    self.via[child_node.name] = current_node.name
                    self.dist[child_node.name] = tentative_g_score
                    #steps + disstance
                    #f_score = tentative_g_score + self.h[child_node.name]
                    f_score = self.__get_f_score(child_node.name) 
                    
                    #add to set
                    open_set.put((f_score, entry_count, child_node))
                    entry_count += 1

        return [], np.inf
    
    def reconstruct_path(self,sn,en):
        end_name = en.name if hasattr(en, 'name') else en
        #go back on path in dir start to fin the path
        path = []
        dist = self.dist[end_name]

        current = end_name
        while current is not None:
            path.append(current)
            current = self.via[current]
        
        # path is reverse, so flip it 
        return path[::-1], dist
    
class Map():

    def __init__(self, name):
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

        # Start with a grid where everything is an obstacle (value = 1).
        grid = np.ones_like(raw_image_array, dtype=int)

        grid[raw_image_array > free_pixel_threshold] = 0
        
        self.image_array = grid

class MapProcessor():
    def __init__(self,name):
        self.map = Map(name)
        self.inf_map_img_array = np.copy(self.map.image_array) 
        self.map_graph = Tree(name)

    # Add this method inside your MapProcessor class
    def visualize_map_array(self, map_array, title="Map"):
        """
        Displays the given map array using matplotlib.
        Obstacles (1) will be black, Free space (0) will be white.
        """
        try:
            import matplotlib.pyplot as plt
            plt.imshow(map_array, cmap='gray_r') # gray_r makes 0=white, 1=black
            plt.title(title)
            plt.show()
        except ImportError:
            print("Matplotlib is not installed. Please run 'pip install matplotlib' to visualize the map.")
    
    def __modify_map_pixel(self,map_array,i,j,value,absolute):
        if( (i >= 0) and 
            (i < map_array.shape[0]) and 
            (j >= 0) and
            (j < map_array.shape[1]) ):
            if absolute:
                map_array[i][j] = value
            else:
                map_array[i][j] += value 
    
    def __inflate_obstacle(self,kernel,map_array,i,j,absolute):
        dx = int(kernel.shape[0]//2)
        dy = int(kernel.shape[1]//2)
        if (dx == 0) and (dy == 0):
            self.__modify_map_pixel(map_array,i,j,kernel[0][0],absolute)
        else:
            for k in range(i-dx,i+dx):
                for l in range(j-dy,j+dy):
                    self.__modify_map_pixel(map_array,k,l,kernel[k-i+dx][l-j+dy],absolute)
        
    def inflate_map(self,kernel,absolute=True):
        # Perform an operation like dilation, such that the small wall found during the mapping process
        # are increased in size, thus forcing a safer path.
        self.inf_map_img_array = np.copy(self.map.image_array)
        
        # We need to find the locations of the original obstacles (value=1)
        obstacle_indices = np.where(self.map.image_array == 1)
        
        # Now, for each original obstacle, inflate it onto our new map
        for i, j in zip(*obstacle_indices):
            self.__inflate_obstacle(kernel, self.inf_map_img_array, i, j, absolute)
            
        self.inf_map_img_array[self.inf_map_img_array > 0] = 1
                
    def get_graph_from_map(self):

        self.map_graph.g = {}
        # Create the nodes that will be part of the graph, considering only valid nodes or the free space
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    node = Node('%d,%d'%(i,j))
                    self.map_graph.add_node(node)
        # Connect the nodes through edges
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:                    
                    if (i > 0):
                        if self.inf_map_img_array[i-1][j] == 0:
                            # add an edge up
                            child_up = self.map_graph.g['%d,%d'%(i-1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up],[1])
                    if (i < (self.map.image_array.shape[0] - 1)):
                        if self.inf_map_img_array[i+1][j] == 0:
                            # add an edge down
                            child_dw = self.map_graph.g['%d,%d'%(i+1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw],[1])
                    if (j > 0):
                        if self.inf_map_img_array[i][j-1] == 0:
                            # add an edge to the left
                            child_lf = self.map_graph.g['%d,%d'%(i,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_lf],[1])
                    if (j < (self.map.image_array.shape[1] - 1)):
                        if self.inf_map_img_array[i][j+1] == 0:
                            # add an edge to the right
                            child_rg = self.map_graph.g['%d,%d'%(i,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_rg],[1])
                    if ((i > 0) and (j > 0)):
                        if self.inf_map_img_array[i-1][j-1] == 0:
                            # add an edge up-left 
                            child_up_lf = self.map_graph.g['%d,%d'%(i-1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_lf],[np.sqrt(2)])
                    if ((i > 0) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i-1][j+1] == 0:
                            # add an edge up-right
                            child_up_rg = self.map_graph.g['%d,%d'%(i-1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_rg],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j > 0)):
                        if self.inf_map_img_array[i+1][j-1] == 0:
                            # add an edge down-left 
                            child_dw_lf = self.map_graph.g['%d,%d'%(i+1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_lf],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i+1][j+1] == 0:
                            # add an edge down-right
                            child_dw_rg = self.map_graph.g['%d,%d'%(i+1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_rg],[np.sqrt(2)])                    
        
    def gaussian_kernel(self, size, sigma=1):
        size = int(size) // 2
        x, y = np.mgrid[-size:size+1, -size:size+1]
        normal = 1 / (2.0 * np.pi * sigma**2)
        g =  np.exp(-((x**2 + y**2) / (2.0*sigma**2))) * normal
        r = np.max(g)-np.min(g)
        sm = (g - np.min(g))*1/r
        return sm
    
    def rect_kernel(self, size, value):
        m = np.ones(shape=(size,size))
        return m
    
    def draw_path(self,path):
        path_tuple_list = []
        path_array = copy(self.inf_map_img_array)
        for idx in path:
            tup = tuple(map(int, idx.split(',')))
            path_tuple_list.append(tup)
            path_array[tup] = 0.5
        return path_array

def main(args=None):
    rclpy.init(args=args)

    task3 = Task3()

    try:
        rclpy.spin(task3)
    except KeyboardInterrupt:
        pass
    finally:
        task3.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


