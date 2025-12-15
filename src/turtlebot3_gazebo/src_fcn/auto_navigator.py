#!/usr/bin/env python3

import math
import sys
import os 
import numpy as np

import rclpy
from rclpy.node import Node as RosNode
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from ..src.lab4.AStar import Queue, Node, Tree, AStar, MapProcessor, Map


class Navigation(RosNode):
    """! Navigation node class.
    This class should serve as a template to implement the path planning and
    path follower components to move the turtlebot from position A to B.
    """

    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        super().__init__(node_name)

        self.path = Path()
        self.goal_pose = None
        self.ttbot_pose = None
        self.start_time = 0.0
        self.get_logger().info("Graph built successfully.")


        pkg_share_path = get_package_share_directory('task_4')
        default_map_path = os.path.join(pkg_share_path, 'maps', 'sync_classroom_map.yaml')

        # 2. Declare the 'map_yaml_path' parameter with the default value
        self.declare_parameter('map_yaml_path', default_map_path)

        # 3. Get the value of the parameter from the launch file (or use the default)
        map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value

        
        inflation_kernel_size = 8
        self.max_dist_alternate_Ponit = 1.0  # if start or stop pose is not valid, search for alternate point within this distance (meters)
        self.max_angle_alpha_to_startdrive = 1.0  # radian (57 degrees)
        
        self.k_rho = 0.8608         # Proportional gain for linear speed
        self.kp_angular = 2.0747    # Proportional gain for angular velocity
        self.ki_angular = 0.1692    # Integral gain for angular velocity
        self.kd_angular = -0.02   # Derivative gain for angular velocity
        self.k_beta = -0.1        # Gain for final orientation correction

        self.yaw_tolerance = 0.1  # Radians (~5.7 degrees)
        self.kp_final_yaw = 0.8   # Proportional gain for final yaw correction

        self.min_lookahead_dist = 0.2  # The smallest the lookahead distance can be
        self.lookahead_ratio = 0.5     # How much the lookahead increases with speed

        # Speed and tolerance settings
        self.speed_max = 0.31
        self.rotspeed_max = 1.9
        self.goal_tolerance = 0.2
        self.align_threshold = 0.4

        self.last_commanded_speed = 0.0
        self.use_dynamic_lookahead = True # Enable dynamic lookahead based on speed
        self.use_line_of_sight_check = True # Enable line-of-sight shortcut checking
        self.shortcut_active = False #if a shortcut is being taken true

        self.get_logger().info(f"Loading map from '{map_yaml_path}' and building graph...")
        self.map_processor = MapProcessor(map_yaml_path)
        inflation_kernel = self.map_processor.rect_kernel(inflation_kernel_size, 1)
        self.map_processor.inflate_map(inflation_kernel)
        self.map_processor.get_graph_from_map()
        self.get_logger().info("Graph built successfully.")

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time',10)

        self.rate = 10.0
        self.timer = self.create_timer(1.0 / self.rate, self.run_loop)

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

        # If we are very close to the goal, stop the robot to prevent overshoot.
        if rho < 0.05: # 5 cm tolerance
            return 0.0, 0.0

        # 5. Apply the Control Law 
        # v = k_rho * rho
        # omega = k_alpha * alpha + k_beta * beta
        
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
        if self.ttbot_pose is None or self.goal_pose is None or not self.path.poses:
            return
        final_goal_pose = self.path.poses[-1]

        # Check if we have reached the final goal position and orientation
        self.Check_alingment_Goal(self, final_goal_pose)
                       
        # Get the current goal from the path, possibly using line-of-sight optimization
        self.Path_optimizaton(self, final_goal_pose)
        
        # Calculate and publish robot commands
        speed, heading = self.path_follower(self.ttbot_pose, self.current_goal)
        self.move_ttbot(speed, heading)

        self.last_commanded_speed = speed

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
                self.get_logger().info("Goal reached!")
                self.move_ttbot(0.0, 0.0)
                self.path = Path()
                self.goal_pose = None
                self.last_commanded_speed = 0.0
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


def main(args=None):
    rclpy.init(args=args)
    nav = Navigation(node_name='Navigation')        
    self.timer = self.create_timer(0.1, self.timer_cb)
        # Fill in the initialization member variables that you need

    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        super().__init__(node_name)

    try:
        rclpy.spin(nav) 
    except KeyboardInterrupt:
        nav.get_logger().info('Keyboard interrupt received. Stopping the robot...')
        nav.move_ttbot(0.0, 0.0)
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
