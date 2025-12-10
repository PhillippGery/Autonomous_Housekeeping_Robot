#!/usr/bin/env python3
 
import math
import sys
import os
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from sensor_msgs.msg import Image, LaserScan

from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from PIL import Image
import yaml
import pandas as pd
from math import sqrt

from copy import copy
import time
from queue import PriorityQueue

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Task1(Node):
    """
    Autonmous mapping and navigation class for Turtlebot3 using A* path planning
    """
    def __init__(self):
        super().__init__('task1_node')

        self.path = Path()
        self.goal_pose = None
        self.ttbot_pose = None
        self.start_time = 0.0


        self.state = 'IDLE'  # Possible states: IDLE, ASTARPATH_FOLLOWING


        pkg_share_path = get_package_share_directory('turtlebot3_gazebo')
        #default_map_path = os.path.join(pkg_share_path, 'maps', 'sync_classroom_map.yaml')

        #Obstacle avoidance variables
        self.obstacle_state = 'CLEAR'
        self.min_front_obstacle_distance = 0.35  # Meters
        self.min_back_obstacle_distance = 0.25   # Meters
        self.safety_distance = 0.2              # Meters
        self.obstacle_inflation_radius_m = 0.4
        self.retreat_distance = -0.25      # Distance to retreat in meters 
        self.current_retreat_distance = 0.0 # Tracker for retreat distance 
        self.retreat_speed = -0.15          # Reverse linear speed (m/s)

        self.map_initialized = False # Flag to track if the map has been initialized


        # Frontier exploration variables
        self.map_processor = SLAMMapProcessor() # Initialize with the new SLAM map placeholder
        self.get_logger().info("SLAM processor initialized. Waiting for /map data...")
        self.FLg_NO_Angle_Alignment = True  # If true, the robot will not align to the goal orientation upon arrival
        self.raw_map_data_array = None
        self.rejected_goals_grid = [] # List to store rejected goal grid coordinates
        self.Frotier_Counter = 0
        self.Frontier_W_dist = 1.0
        self.Frontier_W_power = 3.0 
        self.min_frontier_distance = 0.6  # meters
        self.search_radius_cells = 7  # cells
        self.min_free_neighbors_for_frontier = 5  # Minimum free neighbors required for a frontier cell
        self.Contrains_Relaxed = False
        self.FinalGoal_Origin_Set = False
        self.last_known_cells_count = 0


        # Wall following variables
        self.min_side_distance = 0.7  # Meters to trigger wall following
        self.min_time_in_narrow_space = 2.0  # Seconds to trigger wall following
        self.In_narrow_space = False
        self.wall_follower = WallFollower(
            self.get_logger(),  
            desired_distance=0.5, 
            kp=1.2, 
            ki=0.0, 
            kd=0.4,
            max_angular_speed=1.3,
            max_linear_speed=0.2
        )

    
        self.inflation_kernel_size = 4  # Size of the inflation kernel (should be odd)
        self.max_dist_alternate_Ponit = 1.5  # if start or stop pose is not valid, search for alternate point within this distance (meters)
        self.max_angle_alpha_to_startdrive = 55/180 * math.pi 
        
        self.k_rho = 0.8608         # Proportional gain for linear speed
        self.kp_angular = 2.0747    # Proportional gain for angular velocity
        self.ki_angular = 0.1692    # Integral gain for angular velocity
        self.kd_angular = -0.02   # Derivative gain for angular velocity
        self.k_beta = -0.1        # Gain for final orientation correction

        self.yaw_tolerance = 0.1  # Radians (~5.7 degrees)
        self.kp_final_yaw = 0.8   # Proportional gain for final yaw correction --> faster

        self.min_lookahead_dist = 0.2  # The smallest the lookahead distance can be
        self.lookahead_ratio = 0.5     # How much the lookahead increases with speed

        # Speed and tolerance settings
        self.speed_max = 0.46
        self.rotspeed_max = 1.9
        self.goal_tolerance = 0.1
        self.align_threshold = 0.5

        self.last_commanded_speed = 0.0
        self.use_dynamic_lookahead = True # Enable dynamic lookahead based on speed
        self.use_line_of_sight_check = True # Enable line-of-sight shortcut checking
        self.shortcut_active = False #if a shortcut is being taken true

        # wait for map to be ready 
        self.inflation_kernel = self.rect_kernel(self.inflation_kernel_size, 1) # Define kernel once
        self.map_processor = SLAMMapProcessor() # Initialize with the new SLAM map placeholder
        self.get_logger().info("SLAM processor initialized. Waiting for /map data...")

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        #self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.__ttbot_pose_cbk, 10)
        #self.create_subscription( Image, '/camera/image_raw', self.listener_callback, 10)
        self.create_subscription(Odometry, '/odom', self.__odom_cbk, 10)


        self.create_subscription(LaserScan, '/scan', self._check_for_obstacles, 10)
        self.create_subscription(OccupancyGrid, '/map', self.__map_cbk, 1) # QoS set to 1 for reliable map updates

        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time',10)
        #self.bbox_publisher = self.create_publisher(BoundingBox2D, '/bbox', 10)

        #set inatl pose automaticly 
        #self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        #self.initial_pose_timer = self.create_timer(2.0, self.publish_initial_pose)

        #self.bridge = CvBridge()
        self.ranges = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Define the two frames for the transform: Map (stable) and Odom (local/drifting)
        self.map_frame = 'map'
        self.odom_frame = 'odom'

        self.rate = 10.0
        self.timer = self.create_timer(1.0 / self.rate, self.run_loop)

        # start Time for MEasuring time of exporing map
        self.exploration_start_time = 0.0


    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        if not self.map_initialized:
            self.get_logger().warn("Map not yet initialized by SLAM. Cannot plan path.")
            return
                  
        self.goal_pose = data
        #self.get_logger().info('New goal received: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))
        
        if self.ttbot_pose is None:
            self.get_logger().warn("Cannot plan path, robot pose is not yet available.")
            return
            
        # Call the path planner to generate a new path
        self.path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)

        if self.goal_pose:
                start_world = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
                end_grid = self._world_to_grid(start_world)
                grid_name = f"{end_grid[0]},{end_grid[1]}"
                self.rejected_goals_grid.append(grid_name)
                self.get_logger().info(f"Rejected goals: {self.rejected_goals_grid}")
        
        # If a valid path was found, publish it and reset the path follower index and shortcut flag
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

        if not self.map_initialized:
            self.get_logger().error("Map not available for planning.")
            return Path()

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
        #self.get_logger().info(f"A* planning time: {self.astarTime.data:.4f} seconds")
        
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
        # alpha = robot_theta - math.atan2(delta_y, delta_x)
        
        # Normalize alpha to be between -pi and pi
        if alpha > math.pi:
            alpha -= 2 * math.pi
        elif alpha < -math.pi:
            alpha += 2 * math.pi

        #self.get_logger().info(f"Path Follower -> rho: {rho:.3f}, alpha: {alpha:.3f}")

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
               
        
        if self.state == 'MAP_EXPLORED':
            self.get_logger().info("Map fully explored. Stopping robot.", throttle_duration_sec=5.0)
            self.move_ttbot(0.0, 0.0)
        
        elif self.goal_pose is None or self.state == 'IDLE':
            
            self.get_logger().info("Current mission complete or no goal. Initiating Frontier Search.", throttle_duration_sec=5.0)

            if self.map_initialized: # Make sure map is ready
                self._select_and_set_goal(self.frontier_points)        
            return # Skip rest of loop if no goal

        # Obstacle avoidance state machine
        # Retreating and plan new path with new Frontier goal
        elif self.obstacle_state == 'RETREATING':
            # Calculate distance moved in this cycle (dt)
            dt = 1.0 / self.rate
            distance_moved = abs(self.retreat_speed * dt)
            
            # Check if we have met the retreat distance goal
            if self.current_retreat_distance < abs(self.retreat_distance):
                # Haven't moved far enough back yet.
                self.move_ttbot(self.retreat_speed, 0.0) # Move backward (speed is negative)
                self.current_retreat_distance += distance_moved
                #self.get_logger().warn(f"Retreating: {self.current_retreat_distance:.3f} / {abs(self.retreat_distance):.3f} m", throttle_duration_sec=0.1)
                # if obstacle in frońt and behind, return to IDLE
                if self.obstacle_behind:
                    self.obstacle_state = 'CLEAR'
                    self.state = 'IDLE'
                    self.get_logger().warn("Obstacle still behind during retreat", throttle_duration_sec=1.0)

                return # Skip path following
            
            else:
                # Retreat distance met. Stop and transition to REPLANNING.
                self.move_ttbot(0.0, 0.0) # Ensure robot is stopped
                self.get_logger().info("Retreat complete. Transitioning to REPLANNING.")
                self.obstacle_state = 'CLEAR'
                self.state = 'IDLE'
                self.FinalGoal_Origin_Set = False
                return

        elif self.state == 'ASTARPATH_FOLLOWING':
            final_goal_pose = self.path.poses[-1]

            # Get the current goal from the path, possibly using line-of-sight optimization
            self.Path_optimizaton(final_goal_pose)
        
            # Calculate and publish robot commands
            speed, heading = self.path_follower(self.ttbot_pose, self.current_goal)
            self.move_ttbot(speed, heading)
            self.last_commanded_speed = speed

            # Check if we have reached the final goal position and orientation
            self.Check_alingment_Goal( final_goal_pose)

        elif self.state == 'WALL_FOLLOWING':
            # Wallfollower in narrow spaces
            current_time = self.get_clock().now()
            linear_speed, angular_speed = self.wall_follower.compute_velocities(self.scan_msg, current_time)
            self.move_ttbot(float(linear_speed), float(angular_speed))
        
        else:
            self.get_logger().warn(f"Unknown state: {self.state}. Stopping robot for safety.")
            #self.state = 'IDLE'
            self.move_ttbot(0.0, 0.0)

        

    def Check_alingment_Goal(self, final_goal_pose):
        """! Check if the robot has reached the final goal position and orientation.
        """

        if self.goal_pose is None:
        # If the goal was just cleared (mission finished), exit gracefully.
            return

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
            if abs(yaw_error) < self.yaw_tolerance or self.FLg_NO_Angle_Alignment:
                # GOAL REACHED: Position and orientation are correct. Stop everything.
                self.get_logger().info("Goal reached and Alinged to Goal Pose!")
                self.move_ttbot(0.0, 0.0)
                self.path = Path()
                self.goal_pose = None
                self.state = 'IDLE'
                self.last_commanded_speed = 0.0
                self.integral_error_angular = 0.0
                self.previous_error_angular = 0.0
                self.Frotier_Counter += 1
                #self.rejected_goals_grid = []
                self.get_logger().info(f"Frontier Goals Reached So Far: {self.Frotier_Counter}")
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
   
    def rect_kernel(self, size, value):
        return np.ones(shape=(size,size))
    
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
            #self.get_logger().info(f"Minimum front distance: {front_dist:.2f} m")
        except ValueError:
            # happens if all readings in a slice are 'nan'
            self.get_logger().warn("laser readings are 'nan'. Skipping loop.", throttle_duration_sec=1)
            return
        
        # Check for narrow spaces
        narrow_space_detected = (right_dist < self.min_side_distance) and \
                        (left_dist < self.min_side_distance)
        
        if narrow_space_detected:
    
            # 1. Start the timer if we just entered the narrow space
            if not self.In_narrow_space:
                self.In_narrow_space = True
                self.enter_time = self.get_clock().now()
                
            if self.state != 'WALL_FOLLOWING' and self.enter_time is not None:
                time_in_narrow_space = (self.get_clock().now() - self.enter_time).nanoseconds * 1e-9
                
                
                if time_in_narrow_space >= self.min_time_in_narrow_space:
                    is_area_unknown = self._check_area_ahead_unknown(distance_m=1.0, width_m=0.5, required_percentage=40.0, wall_threshold_percentage=5.0)
                    if is_area_unknown: 
                        self.get_logger().warn("Narrow space confirmed. Switching to WALL_FOLLOWING mode.")
                        self.state = 'WALL_FOLLOWING'
                        self.enter_time = None # Reset timer after transition
                    
                   
        else:

            # If robot was in narrow space, reset the flags/timer
            if self.In_narrow_space:
                #self.get_logger().info("Exiting narrow space detection zone.")
                self.In_narrow_space = False
                self.enter_time = None
                
            if self.state == 'WALL_FOLLOWING':
                self.get_logger().info("Exiting WALL_FOLLOWING mode. Resuming primary mission.")
                self.state = 'IDLE' # Transition back to IDLE to trigger new frontier selection/path planning
                
        obstacle_close = front_dist < self.min_front_obstacle_distance
        self.obstacle_behind = back_dist < self.min_back_obstacle_distance
        self.safety_dist_critical = safety_dist < self.safety_distance

        if obstacle_close and self.obstacle_state == 'CLEAR':
            # 1. New unmapped obstacle detected!
            self.state = 'OBSTACLE_AVOIDANCE'
            self.move_ttbot(0.0, 0.0) # Stop immediately
            self.get_logger().warn(f"Obstacle detected at {front_dist:.2f} m! Initiating avoidance maneuver.")
            self.obstacle_state = 'RETREATING'
            self.current_retreat_distance = 0.0
        elif self.safety_dist_critical and self.obstacle_state == 'CLEAR':
            # Emergency stop if something is too close in safety zones
            self.state = 'OBSTACLE_AVOIDANCE'
            self.move_ttbot(0.0, 0.0) # Stop immediately
            self.get_logger().warn(f"Critical obstacle detected at {safety_dist:.2f} m in safety zone! Initiating avoidance maneuver.")
            self.obstacle_state = 'RETREATING'
            self.current_retreat_distance = 0.0

        



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

    def __odom_cbk(self, data: Odometry):
        """! Callback to receive high-frequency odometry and transform it to the stable /map frame. """
        

        try:
            # Look up transform from base_footprint (child) to map (target)
            t = self.tf_buffer.lookup_transform(
                self.map_frame, 
                data.child_frame_id, # 'base_footprint'
                rclpy.time.Time() # Get the latest available transform
            )
        except Exception as e:
            return


        
        # Create a PoseStamped from the Odometry message
        odom_pose_stamped = PoseStamped()
        odom_pose_stamped.header = data.header # Header is usually odom frame
        odom_pose_stamped.pose = data.pose.pose 
        
        corrected_pose = PoseStamped()
        corrected_pose.header.frame_id = self.map_frame
        corrected_pose.pose.position.x = t.transform.translation.x
        corrected_pose.pose.position.y = t.transform.translation.y
        corrected_pose.pose.orientation = t.transform.rotation # SLAM provides the corrected orientation
        
        self.ttbot_pose = corrected_pose

    def __map_cbk(self, data):
        """! Callback to catch the live map data from the SLAM node.
        
        This function processes the raw OccupancyGrid, inflates ONLY the walls,
        and updates the A* graph costmap (where Unknown cells are treated as blocked 
        but not inflated).
        """
        if self.map_processor is None:
            self.exploration_start_time = self.get_clock().now().nanoseconds*1e-9
            return

        # 1. Update Map Metadata
        self.map_processor.map.resolution = data.info.resolution
        self.map_processor.map.origin[0] = data.info.origin.position.x
        self.map_processor.map.origin[1] = data.info.origin.position.y
        self.map_processor.map.height = data.info.height
        self.map_processor.map.width = data.info.width
        
        H = data.info.height
        W = data.info.width


        map_2d = np.array(data.data, dtype=np.int8).reshape(H, W)
        
        # Store Flipped Raw Map for Frontier Detection (FBS logic relies on -1)
        self.raw_map_data_array = np.flipud(map_2d)

        
        # Only Occupied cells (100) are walls
        wall_array_raw = np.zeros_like(map_2d, dtype=int)
        wall_array_raw[map_2d == 100] = 1
        current_wall_array = np.flipud(wall_array_raw) # Flipped for 2D indexing


        current_costmap = np.zeros_like(map_2d, dtype=int)
        current_costmap[(map_2d == 100) | (map_2d == -1)] = 1
        current_costmap = np.flipud(current_costmap)
        
        # Initialize the map_processor's array to the raw A* costmap
        self.map_processor.inf_map_img_array = np.copy(current_costmap)

        # Define the kernel matrix
        inflation_kernel_matrix = self.rect_kernel(self.inflation_kernel_size * 2 + 1, 1)
        
        # Find indices of occupied walls (1)
        obstacle_indices = np.where(current_wall_array == 1)

        for i, j in zip(*obstacle_indices):
            # Call the external helper method from SLAMMapProcessor
            self.map_processor._inflate_obstacle(
                inflation_kernel_matrix, 
                self.map_processor.inf_map_img_array, 
                i, j, 
                absolute=True
            )
            

        self.map_processor.inf_map_img_array[self.map_processor.inf_map_img_array > 0] = 1

        # if map is actually growing with update reset the rejected goals
        


        # 4. Finalize Graph and State
        # Regenerate Graph
        self.map_processor.get_graph_from_map()
        self.frontier_points = self._find_frontiers()

        current_known_cells = np.sum((self.raw_map_data_array == 0) | (self.raw_map_data_array == 100))
        # Check if the map has significantly grown since the last update
        if current_known_cells > self.last_known_cells_count + 20: 
            if len(self.rejected_goals_grid) > 0:
                self.get_logger().info(f"Map grown significantly ({current_known_cells} vs {self.last_known_cells_count}). Resetting {len(self.rejected_goals_grid)} rejected goals.")
                self.rejected_goals_grid = [] # Clear rejected goals
        self.last_known_cells_count = current_known_cells



        if not self.map_initialized:
            self.get_logger().info(f"Initial SLAM map received (H:{H}, W:{W}). A* graph built.")
            self.map_initialized = True

    def _find_frontiers(self):
        """
        Identifies all frontier cells (Free [0] adjacent to Unknown [-1]) 
        by iterating over the entire map array (O(N^2)).
        
        Returns a list of tuples: [(row, col, unknown_neighbor_count), ...].
        """
        if self.raw_map_data_array is None:
            return []

        raw_data = self.raw_map_data_array
        H, W = raw_data.shape
        # Frontiers now store (row, col, count)
        frontiers = [] 

        # Define neighbor offsets (8 directions)
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        MIN_FREE_NEIGHBORS = self.min_free_neighbors_for_frontier  

        for i in range(1, H - 1):
            for j in range(1, W - 1):
                
                if raw_data[i, j] == 0: # Check 1: Is the current cell FREE (0)?
                    
                    unknown_neighbor_count = 0
                    free_neighbor_count = 0 # NEW: Counter for known/free neighbors
                    is_frontier = False
                    
                    # Check 2: Count UNKNOWN (-1) and FREE (0) neighbors
                    for di, dj in neighbors:
                        if 0 <= i + di < H and 0 <= j + dj < W:
                            neighbor_value = raw_data[i + di, j + dj]
                            
                            if neighbor_value == -1:
                                is_frontier = True
                                unknown_neighbor_count += 1
                            elif neighbor_value == 0:
                                # Count adjacent known/free cells
                                free_neighbor_count += 1 
                                
                    if is_frontier and free_neighbor_count >= MIN_FREE_NEIGHBORS: 
                        # Append the coordinates and the count of unknown neighbors
                        frontiers.append((i, j, unknown_neighbor_count))
                        
        return frontiers

    def _select_and_set_goal(self, frontier_points):
        """
        Selects the best goal from the frontier points list based on a combined
        heuristic (distance + information gain).
        """
        if not frontier_points or self.ttbot_pose is None:
            return

        robot_x = self.ttbot_pose.pose.position.x
        robot_y = self.ttbot_pose.pose.position.y
        best_goal_pose = None
        min_cost = float('inf')
        
        for i, j, count in frontier_points:

            grid_name = f"{i},{j}"

            # Skip previously rejected goals
            if grid_name in self.rejected_goals_grid:
                continue
            
            # Check Reachability (A* Graph Node Exists)
            
            if grid_name not in self.map_processor.map_graph.g:
                continue

            # Calculate Costs
            world_coords = self._grid_to_world((i, j))
            goal_x, goal_y = world_coords[0], world_coords[1]
            
            distance = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
            #distance = self._get_astar_distance(self.ttbot_pose, grid_name)
            #self.get_logger().info(f"Frontier at ({i},{j}) - A* Distance: {distance:.2f} m, Info Gain Count: {count}")

            if distance < self.min_frontier_distance:
                # Too close to current position, skip
                self.rejected_goals_grid.append(grid_name)
                continue

            area_gain_cells = self._calculate_local_area_gain(i, j)
            if area_gain_cells == 0:
                 information_gain_term = float('inf') # Treat as highest cost if zero gain
            else:
                 # Information Gain Term (Total Cost is minimized)
                 information_gain_term = self.Frontier_W_power / area_gain_cells

            total_cost = (self.Frontier_W_dist * distance) + information_gain_term

            # Select Best Goal
            if total_cost < min_cost:
                min_cost = total_cost
                
                # Create PoseStamped object for the chosen goal
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.pose.position.x = goal_x
                goal_pose.pose.position.y = goal_y
                goal_pose.pose.orientation.w = 1.0 # Default orientation
                best_goal_pose = goal_pose

        if best_goal_pose:
            self.get_logger().info(f"Selected new frontier goal. Cost: {min_cost:.2f}")
            self.__goal_pose_cbk(best_goal_pose)
            self.FinalGoal_Origin_Set = False
            
            
        else:

            #if less then 1% of the map is unknown consider exploration complete
            # Calculate frontier Points in corelation to known map area
            frontier_points_count = len(frontier_points)
            
            known_cells = np.sum((self.raw_map_data_array == 0) | (self.raw_map_data_array == 100))
            unknown_percentage_vs_known = (frontier_points_count / known_cells) * 100.0        
            
            self.get_logger().info(f"Unknown Map Percentage (vs Known): {unknown_percentage_vs_known:.2f}%", throttle_duration_sec=1.0)
        
            if unknown_percentage_vs_known < 0.06:
                self.get_logger().warn("Exploration Complete: Less than 0.06% of the map is unknown.")
                # print exproation time
                time_now = self.get_clock().now().nanoseconds*1e-9
                time = (time_now - self.exploration_start_time - 18.0*60.0)/60.0    
                self.get_logger().info(f"Total Exploration Time: {time:.2f} minutes")
                self.get_logger().info(f"Total Frontier Goals Reached: {self.Frotier_Counter}")
                self.state = 'MAP_EXPLORED'

            elif self.Contrains_Relaxed == True and not self.FinalGoal_Origin_Set:
                self.get_logger().info("All constraints relaxed and still no goal found. Setting origin as goal.", throttle_duration_sec=5.0)
                origin_pose = PoseStamped()
                origin_pose.header.frame_id = 'map'
                origin_pose.pose.position.x = 0.0
                origin_pose.pose.position.y = 0.0
                origin_pose.pose.orientation.w = 1.0 # Default orientation
                self.rejected_goals_grid = []
                self.__goal_pose_cbk(origin_pose)
                self.FinalGoal_Origin_Set = True
                
            else:
                self.get_logger().info("No valid frontier goal found. All candidates rejected or unreachable.", throttle_duration_sec=5.0)
                self.get_logger().info(f"Frontier Points Available: {frontier_points_count}", throttle_duration_sec=5.0)
                self.get_logger().info("Relaxing frontier selection criteria for next iteration.", throttle_duration_sec=5.0)
                self.min_frontier_distance = 0.0  # meters
                self.min_free_neighbors_for_frontier = 0
                self.search_radius_cells = 1
                self.frontier_points = self._find_frontiers()
                self.rejected_goals_grid = [] # Clear rejected goals to allow re-evaluation
                self.state = 'IDLE'  # Retry goal selection in the next loop
                self.Contrains_Relaxed = True
                            
    def _calculate_local_area_gain(self, i, j):
        """
        Calculates Information Gain by counting unknown (-1) cells within a 
        square window centered at grid point (i, j).
        """
        if self.raw_map_data_array is None:
            return 0
        
        search_radius_cells = self.search_radius_cells
        
        raw_data = self.raw_map_data_array
        H, W = raw_data.shape
        
        # Define the bounding box for the search area
        min_i = max(0, i - search_radius_cells)
        max_i = min(H, i + search_radius_cells + 1)
        min_j = max(0, j - search_radius_cells)
        max_j = min(W, j + search_radius_cells + 1)
        
        # Extract the slice of the map
        window = raw_data[min_i:max_i, min_j:max_j]

        num_cells = window.size
        
        # Count the number of unknown cells (-1) within that window
        unknown_count = np.sum(window == -1)/ num_cells
        
        return unknown_count
    
    def _get_astar_distance(self, start_pose, frontier_grid_name):
        """
        Runs a quick A* search from the robot's position to a specific frontier point
        and returns the actual travel distance (g(n)).
        """
        if not self.map_initialized:
            return float('inf')

        # 1. Setup Start Node
        start_world = (start_pose.pose.position.x, start_pose.pose.position.y)
        start_grid = self._world_to_grid(start_world)
        start_name = f"{start_grid[0]},{start_grid[1]}"
        
        end_name = frontier_grid_name
        
        if start_name not in self.map_processor.map_graph.g or \
           end_name not in self.map_processor.map_graph.g:
            # If start or end node is blocked (e.g., by fresh inflation), return infinity cost
            return float('inf')

        start_node = self.map_processor.map_graph.g[start_name]
        end_node = self.map_processor.map_graph.g[end_name]
        
        astar_solver = AStar(self.map_processor.map_graph)
        
        # Calculate heuristic h(n) based on Euclidean distance (required by A*)
        end_grid = tuple(map(int, end_name.split(',')))
        for name in astar_solver.h.keys():
            node_grid = tuple(map(int, name.split(',')))
            astar_solver.h[name] = math.sqrt((end_grid[0] - node_grid[0])**2 + (end_grid[1] - node_grid[1])**2)
        
        # Run A* and extract the distance
        pathnames, path_dist = astar_solver.solve(start_node, end_node)
        
        return len(pathnames) # Returns float('inf') if no path is found

    def _check_area_ahead_unknown(self, distance_m, width_m, required_percentage, wall_threshold_percentage):
        """
        Checks the percentage of unknown cells (-1) within a rectangular window 
        in front of the robot. Returns False if the area is mostly a known wall.
        """
        if self.ttbot_pose is None or self.raw_map_data_array is None:
            return False # Cannot determine unknown status

        raw_data = self.raw_map_data_array
        H, W = raw_data.shape
        resolution = self.map_processor.map.resolution
        
        # Convert meter dimensions to cell dimensions
        dist_cells = int(distance_m / resolution)
        width_cells = int(width_m / resolution)
        
        # Get the robot's current grid coordinates (i, j)
        robot_i, robot_j = self._world_to_grid(
            (self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y)
        )
        
        # Calculate the robot's current yaw (theta)
        quat = self.ttbot_pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        robot_theta = math.atan2(siny_cosp, cosy_cosp)
        
        # --- 2. Calculate Window Corners (Approximation for performance) ---
        
        # We define a target point ahead of the robot
        target_x = self.ttbot_pose.pose.position.x + dist_cells * resolution * math.cos(robot_theta)
        target_y = self.ttbot_pose.pose.position.y + dist_cells * resolution * math.sin(robot_theta)
        
        # Convert target back to grid
        target_i, target_j = self._world_to_grid((target_x, target_y))
        
        # Define a simplified search window based on the bounding box between robot and target
        min_i = min(robot_i, target_i) - width_cells
        max_i = max(robot_i, target_i) + width_cells
        min_j = min(robot_j, target_j) - width_cells
        max_j = max(robot_j, target_j) + width_cells
        
        # Clamp window to map bounds
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
        
        # 1. Count Occupied cells (value 100)
        occupied_cells = np.sum(window == 100)
        occupied_percentage = (occupied_cells / total_window_cells) * 100.0
        
        if occupied_percentage >= wall_threshold_percentage:
            self.get_logger().info(f"Ahead Occupied: {occupied_percentage:.1f}%. Detected wall, skipping unknown check.", throttle_duration_sec=1.0)
            return False # Area is a known wall, no need for exploration
                   
        # 2. Count Unknown cells (value -1)
        unknown_cells = np.sum(window == -1)
        unknown_percentage = (unknown_cells / total_window_cells) * 100.0
        
        self.get_logger().info(f"Ahead Unknown: {unknown_percentage:.1f}% ({unknown_cells} cells)", throttle_duration_sec=1.0)
        
        return unknown_percentage >= required_percentage

class MapData:
    def __init__(self):
        # Metadata about the map
        self.resolution = 0.05  
        self.origin = [0.0, 0.0, 0.0] # [x, y, yaw]
        self.height = 0
        self.width = 0

class SLAMMapProcessor:
    def __init__(self):
        self.map = MapData() # Holds metadata
        self.map_graph = Tree("SLAM_Graph")
        self.inf_map_img_array = np.array([[]], dtype=int) # Holds map data

    def get_graph_from_map(self):
        # Implementation for graph generation (copied from your old MapProcessor)
        self.map_graph.g = {} # Clear old graph nodes

        H, W = self.inf_map_img_array.shape
        # Create the nodes
        for i in range(H):
            for j in range(W):
                if self.inf_map_img_array[i][j] == 0:
                    node = Node('%d,%d'%(i,j))
                    self.map_graph.add_node(node)
        
        # Create the edges
        for i in range(H):
            for j in range(W):
                if self.inf_map_img_array[i][j] == 0:
                    
                    # UP
                    if (i > 0) and self.inf_map_img_array[i-1][j] == 0: 
                        child_up = self.map_graph.g['%d,%d'%(i-1,j)]
                        self.map_graph.g['%d,%d'%(i,j)].add_children([child_up],[1])
                        
                    # DOWN
                    if (i < (H - 1)) and self.inf_map_img_array[i+1][j] == 0: 
                        child_dw = self.map_graph.g['%d,%d'%(i+1,j)]
                        self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw],[1])
                        
                    # LEFT
                    if (j > 0) and self.inf_map_img_array[i][j-1] == 0: 
                        child_lf = self.map_graph.g['%d,%d'%(i,j-1)]
                        self.map_graph.g['%d,%d'%(i,j)].add_children([child_lf],[1])
                        
                    # RIGHT
                    if (j < (W - 1)) and self.inf_map_img_array[i][j+1] == 0: 
                        child_rg = self.map_graph.g['%d,%d'%(i,j+1)]
                        self.map_graph.g['%d,%d'%(i,j)].add_children([child_rg],[1])
                        
                    # UP-LEFT
                    if ((i > 0) and (j > 0)) and self.inf_map_img_array[i-1][j-1] == 0:
                        child_up_lf = self.map_graph.g['%d,%d'%(i-1,j-1)]
                        self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_lf],[np.sqrt(2)])
                        
                    # UP-RIGHT
                    if ((i > 0) and (j < (W - 1))) and self.inf_map_img_array[i-1][j+1] == 0: 
                        child_up_rg = self.map_graph.g['%d,%d'%(i-1,j+1)]
                        self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_rg],[np.sqrt(2)])
                        
                    # DOWN-LEFT
                    if ((i < (H - 1)) and (j > 0)) and self.inf_map_img_array[i+1][j-1] == 0: 
                        child_dw_lf = self.map_graph.g['%d,%d'%(i+1,j-1)]
                        self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_lf],[np.sqrt(2)])
                        
                    # DOWN-RIGHT
                    if ((i < (H - 1)) and (j < (W - 1))) and self.inf_map_img_array[i+1][j+1] == 0: 
                        child_dw_rg = self.map_graph.g['%d,%d'%(i+1,j+1)]
                        self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_rg],[np.sqrt(2)])


    def _modify_map_pixel(self, map_array, i, j, value, absolute):
        H, W = map_array.shape
        if (i >= 0) and (i < H) and (j >= 0) and (j < W):
            if absolute:
                map_array[i][j] = value
            else:

                if map_array[i][j] == 0 or value == 1:
                    map_array[i][j] = 1

    def _inflate_obstacle(self, kernel, map_array, i, j, absolute):
        dx = int(kernel.shape[0] // 2)
        dy = int(kernel.shape[1] // 2)
        
        if (dx == 0) and (dy == 0):
            self._modify_map_pixel(map_array, i, j, kernel[0][0], absolute)
        else:
            for k in range(i - dx, i + dx + 1): # Include the end point
                for l in range(j - dy, j + dy + 1): # Include the end point
                    # Only apply if the target pixel is valid within the map bounds
                    self._modify_map_pixel(map_array, k, l, kernel[k - i + dx][l - j + dy], absolute)

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
    
class WallFollower:
    """
    PID-based wall-following algorithm.
    
    maintaining  a fixed distance 
    from a wall on the robot's right side. 
    turn left if an obstacle is detected directly in front.

    idee from:
    https://andrewyong7338.medium.com/maze-escape-with-wall-following-algorithm-170c35b88e00
    """
    
    def __init__(self, logger, desired_distance=0.5, kp=1.0, ki=0.01, kd=0.2 , max_angular_speed=1.2, max_linear_speed=0.8):
        
        self.logger = logger #allow logger in non ros pyton file

        #PID Gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        #PID State
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None
        self.obstactle_infront = False
        
        #Controller Setpoints
        self.desired_distance = desired_distance  # Target distance from wall (meters)
        self.safety_distance = 0.4                # Stop and turn if obstacle is this close (meters)
        self.safety_Side_distance = 0.4           # Min distance to wall on the side (meters)
        self.forward_speed = max_linear_speed     # Constant speed to move forward (m/s)
        self.turning_speed = 1.2                  # Speed to turn at when avoiding obstacle (rad/s)
        self.max_angular_speed = max_angular_speed              # Max angular speed for PID (rad/s)

    def compute_velocities(self, scan_msg, current_time):
        """
        LaserScan msg to  get dis to walls and maintain the dis
        Twist message to follow the wall on the right.
        """
        
        
        #  dt
        if self.last_time is None:
            self.last_time = current_time
            return 0.0, 0.0 #for 0 = 0
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt == 0.0:
            return  0.0, 0.0 # Avoid division by zero

        # Scan Sensor ranges --> valid processing
        ranges = np.array(scan_msg.ranges)
        ranges[np.isinf(ranges)] = np.nan
        ranges[ranges == 0.0] = np.nan
        
        #Take slices because of noice readings --> more Robust
        # Front
        front_slice = np.concatenate((ranges[0:20], ranges[345:360]))
        
        # Right
        right_slice = ranges[260:295]
        
        # Angled (45 deg right)
        angled_slice = ranges[310:320]


        try:
            front_dist = np.nanmin(front_slice)
            right_dist = np.nanmin(right_slice)
            angled_dist = np.nanmin(angled_slice)


        except ValueError:
            # This happens if all readings in a slice are 'nan'
            self.logger.warn("laser readings are 'nan'. Skipping loop.", throttle_duration_sec=1)
            return 0.0, 0.0 

        # angled sensor -->primary distance if Not valid --> side sensor
        # if Side distance to small --> no use angled sensor
        if not np.isnan(angled_dist) and right_dist > self.safety_Side_distance:
            # if 45-degree distance invalid --> 90-degree distance
            control_dist = angled_dist / sqrt(2)
        elif not np.isnan(right_dist) and right_dist < 1.5:
            control_dist = right_dist
        else:
            # NO right-side sensors  drive straight
            self.logger.info("No rrigth wall found", throttle_duration_sec=1)
            linear_speed = self.forward_speed * 0.5
            if np.isnan(angled_dist):
                angular_speed = -self.turning_speed * 0.1 # Turn right
            else:
                angular_speed = -self.turning_speed * 0.0 # Turn right
            return linear_speed, angular_speed
            

        # STATE 1: Obstacle in Front 
        # If the front is blocked, stop moving forward and turn left.
        if front_dist < self.safety_distance: # or collAvoid_dist < self.safety_Side_distance:
            self.logger.warn("Obstacle in front! Turning left.", throttle_duration_sec=1)
  
            linear_speed = 0.0
            angular_speed = self.turning_speed
            self.reset_pid() # res PID --> wind up       
            
        # STATE 2: Follow the Wall (PID Control)
        # If the front clear, move forward and use PID to adjust angle.
        else:
            kp_fw = np.clip((front_dist - self.safety_distance) / 1.0, 0.4, 1.0)
            
            linear_speed = self.forward_speed * kp_fw
            
            # error
            error = self.desired_distance - control_dist
            
            # Update PID terms
            self.integral += error * dt
            # Anti-windup
            self.integral = np.clip(self.integral, -1.0, 1.0)
            derivative = (error - self.prev_error) / dt
            
            # PID Output
            pid_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            
            # Set the angular velocity, clamping it to the maximum
            angular_speed = np.clip(pid_output, -self.max_angular_speed, self.max_angular_speed)
            
            self.prev_error = error

        return linear_speed, angular_speed

    def reset_pid(self):
        """Call this when switching away from the wall follower to reset PID state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

def main(args=None):
    rclpy.init(args=args)

    task1 = Task1()

    try:
        rclpy.spin(task1)
    except KeyboardInterrupt:
        pass
    finally:
        task1.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()