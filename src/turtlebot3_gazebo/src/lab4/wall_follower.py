import numpy as np
import rclpy
from math import sqrt
from geometry_msgs.msg import Twist

import numpy as np
from geometry_msgs.msg import Twist

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
        twist_msg = Twist()
        
        #  dt
        if self.last_time is None:
            self.last_time = current_time
            return twist_msg #for 0 = 0
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt == 0.0:
            return twist_msg # Avoid division by zero

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
            return twist_msg

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
            twist_msg.linear.x = self.forward_speed * 0.5
            if np.isnan(angled_dist):
                twist_msg.angular.z = -self.turning_speed * 0.1 # Turn right
            else:
                twist_msg.angular.z = -self.turning_speed * 0.0 # Turn right
            return twist_msg
            
        


        # #obstacle detected previously now turn 90°
        # if self.obstactle_infront:
        #     if self.obstacle_turn_end_time > current_time.nanoseconds:
                
        #         twist_msg.linear.x = 0.0
        #         twist_msg.angular.z = self.turning_speed
        #         return twist_msg
        #     else:
        #         self.obstactle_infront = False
        #         twist_msg.linear.x = 0.0
        #         twist_msg.angular.z = 0.0

        #### Wall Following Logic

        # STATE 1: Obstacle in Front 
        # If the front is blocked, stop moving forward and turn left.
        if front_dist < self.safety_distance: # or collAvoid_dist < self.safety_Side_distance:
            self.logger.warn("Obstacle in front! Turning left.", throttle_duration_sec=1)
            # angle_to_turn = np.pi / 2  # 90 degrees
            # turn_duration = angle_to_turn / self.turning_speed
            # self.obstactle_infront = True
            #self.obstacle_turn_end_time = current_time.nanoseconds + int(turn_duration * 1e9)
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.turning_speed
            self.reset_pid() # res PID --> wind up       
            
        # STATE 2: Follow the Wall (PID Control)
        # If the front clear, move forward and use PID to adjust angle.
        else:
            kp_fw = np.clip((front_dist - self.safety_distance) / 1.0, 0.4, 1.0)
            
            twist_msg.linear.x = self.forward_speed * kp_fw
            
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
            twist_msg.angular.z = np.clip(pid_output, -self.max_angular_speed, self.max_angular_speed)
            
            self.prev_error = error

        return twist_msg

    def reset_pid(self):
        """Call this when switching away from the wall follower to reset PID state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None