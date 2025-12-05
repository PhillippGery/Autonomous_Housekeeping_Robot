import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped
from .wall_follower import WallFollower
from gazebo_msgs.msg import ContactsState


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('red_ball_tracker')
        
        # raw image topic
        self.subscription = self.create_subscription( Image, '/camera/image_raw', self.listener_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        #self.bumper_subscription = self.create_subscription(ContactsState, '/bumper_collisions', self.bumper_callback,10)

        # Does not work because depth image is not avalivle in lab config
        # self.subscription = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        # self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_rect_raw')
        # self.ts = message_filters.ApproximateTimeSynchronizer([self.subscription], 10, 0.1)      
        # # synchronized callback with two msges
        # self.ts.registerCallback(self.listener_callback)
        
        # Publish bounding box data
        self.bbox_publisher = self.create_publisher(BoundingBox2D, '/bbox', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
            
        self.bridge = CvBridge()
        self.ranges = []

        # Wall Follower Controller Init
        self.wall_follower = WallFollower(
            self.get_logger(),  
            desired_distance=0.5, 
            kp=1.2, 
            ki=0.0, 
            kd=0.4,
            max_angular_speed=1.3,
            max_linear_speed=0.2
        )
        self.state = "DEFAULT"  # default state
        self.bumper_hit = False

        #Controller Ball follower Init prms
        self.pid_p_angular = 0.4
        self.pid_i_angular = 0.1
        self.pid_d_angular = 0.4
        self.p_linear = 0.3

        # self.p_linear = 0.8608         # Proportional gain for linear speed
        # self.pid_p_angular = 2.0747    # Proportional gain for angular velocity
        # self.pid_i_angular = 0.1692    # Integral gain for angular velocity
        # self.pid_d_angular = -0.02   # Derivative gain for angular velocity

        self.integral_angular = 0.0
        self.previous_error_angular = 0.0
        
        # desired ball size in pixels
        self.des_ball_size = 150

        # Camera parameters
        self.camera_hfov_degrees = 60.0

        self.max_linear_speed = 0.2
        self.max_angular_speed = 1.0

        self.last_time = None
        self.last_detection_time = self.get_clock().now()

        # define initial pose in origion
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.initial_pose_timer = self.create_timer(2.0, self.publish_initial_pose)


        

        self.get_logger().info("Red Ball follower  started.")

    def listener_callback(self, msg):

        if self.bumper_hit:
            return

        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return
        

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Avoid division by zero if dt is too small
        if dt == 0:
            return

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_red = np.array([0, 130, 100])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)

        lower_red = np.array([170, 130, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)

        # Combine the two masks
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        

        twist_msg = Twist()
        potential_balls = []

        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                
                # Avoid division by zero
                if perimeter == 0:
                    continue
                    
                # check if circle bacause detacting red briks ....
                circularity = (4 * np.pi * area) / (perimeter * perimeter)
                
                circularity_threshold = 0.8
                min_area = 400 # pixels
                
                if circularity > circularity_threshold and area > min_area:
                    potential_balls.append(contour)
            
        if potential_balls:

            self.state = "TRACKING"

            self.last_detection_time = self.get_clock().now()
            # Find the largest contour *among the circular ones*
            largest_contour = max(potential_balls, key=cv2.contourArea)
            

            #  box coordinates
            x, y, w, h = cv2.boundingRect(largest_contour)

            # center
            centroid_x = x + w // 2
            centroid_y = y + h // 2

            image_width = cv_image.shape[1]
            image_center_x = image_width // 2

            #create bounding box as done in previous task_5
            self.get_logger().info(f"Object Centroid: (x={centroid_x}, y={centroid_y}), Size: (w={w}, h={h})")
            bbox_msg = BoundingBox2D()                
            bbox_msg.center.position.x = float(centroid_x)
            bbox_msg.center.position.y = float(centroid_y)
            bbox_msg.center.theta = 0.0
            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)                
            self.bbox_publisher.publish(bbox_msg)                
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # error detection 
            #distance_to_object = depth_image[int(y), int(x)]
            #self.get_logger().info(f"Distance to Object: {distance_to_object} meters")

            distance_to_object = (self.des_ball_size)/w
            error_distance = distance_to_object - 1.0  # desired distance is 1.0 meter
            self.get_logger().info(f"Distance: {distance_to_object}")

            error_x_unnorm = centroid_x - image_center_x
            error_x = error_x_unnorm / image_center_x  # Normalize error
            #self.get_logger().info(f"Error X: {error_x} pixels")
            #Heading error in Pixels
            heading_error_degrees = error_x_unnorm * (self.camera_hfov_degrees / image_width)
            self.get_logger().info(f"Heading Error: {heading_error_degrees:.2f} degrees")


            num_rays = len(self.ranges)

            target_index = int(round(heading_error_degrees))              
            # 3. Define the slice width (you asked for +- 5)
            slice_half_width = 5 
            # 4. Build a list of indices, handling the wrap-around
            indices_to_check = []
            for i in range(-slice_half_width, slice_half_width + 1):
                # Use modulo (%) to wrap indices (e.g., -2 % 360 = 358)
                idx = (target_index + i) % num_rays
                indices_to_check.append(idx)
                
            # 5. Get the slice from the ranges array using the index list
            ball_distance_slice = self.ranges[indices_to_check]
            
            try:
                ball_distance = np.nanmin(ball_distance_slice)
                self.get_logger().info(f"Ball Distance from scan: {ball_distance} meters")
            except ValueError:
                self.get_logger().warn("laser readings for ball distance are 'nan'. Skipping loop.", throttle_duration_sec=1)
                

            #Controller
            #error_x = heading_error_degrees
            self.integral_angular += error_x
            derivative_angular = error_x - self.previous_error_angular

            p_term = self.pid_p_angular * error_x
            i_term = (self.pid_i_angular * self.integral_angular)*dt
            d_term = (self.pid_d_angular * derivative_angular)/dt

            #windup for integral term
            i_term = np.clip(i_term, -0.5, 0.5)

            if abs(error_distance) > 0.1:
                linear_velocity = self.p_linear * error_distance
            else:
                linear_velocity = 0.0
            
            if abs(error_x) < 0.05:
                angular_velocity = 0.0
                self.previous_error_angular = 0.0
                self.integral_angular = 0.0
                self.get_logger().info("Object centered.")
            else:
                angular_velocity = p_term + i_term + d_term
                self.previous_error_angular = error_x

            twist_msg.linear.x = np.clip(linear_velocity, -self.max_linear_speed, self.max_linear_speed)
            twist_msg.angular.z = np.clip(-angular_velocity, -self.max_angular_speed, self.max_angular_speed)
            
            self.publisher_.publish(twist_msg)

        else: 

            # When no object wait 3 sec then turn to search
            # when object in room and not covered by obstacle robot wil find it          
            time_since_last_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            self.get_logger().info(f"Time since last detection: {time_since_last_detection} seconds")   
            if time_since_last_detection > 3.0 and time_since_last_detection <= 13.0:
                self.get_logger().info("Turning while Searching for object...")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.3
                self.publisher_.publish(twist_msg)
                self.previous_error_angular = 0.0
                self.integral_angular = 0.0
                self.wall_follower.reset_pid()
                self.state = "TRACKING"

            elif time_since_last_detection > 13.0:
                self.get_logger().info("Exploring with Wallfolower..." )
                self.previous_error_angular = 0.0
                self.integral_angular = 0.0
                # call Wallfollower
                self.state = "SEARCHING"
                
            else:
                self.get_logger().info("No object detected.")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.publisher_.publish(twist_msg)
                self.previous_error_angular = 0.0
                self.integral_angular = 0.0
                self.wall_follower.reset_pid()
                self.state = "DEFAULT"

                
        
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


    #callback for scan topic 

    def scan_callback(self, scan_msg):

        #Wall follower when searching
        """
        This callback runs the wall follower if the state is "SEARCHING".
        Because with turing no ball was found
        """
        ranges = np.array(scan_msg.ranges)
        ranges[np.isinf(ranges)] = np.nan
        ranges[ranges == 0.0] = np.nan
        self.ranges = ranges

        if self.bumper_hit:
            return
        
        if self.state == "SEARCHING":
            current_time = self.get_clock().now()
            twist_msg = self.wall_follower.compute_velocities(scan_msg, current_time)
            self.publisher_.publish(twist_msg)


    def publish_initial_pose(self):
        """
        Publishes the initial pose to AMCL to set the robot's starting position
        on the map and then cancels the timer.
        """
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set the position to the map's origin
        pose_msg.pose.pose.position.x = -5.4
        pose_msg.pose.pose.position.y = -6.18
        pose_msg.pose.pose.position.z = 0.0

        # Set the orientation (0 degrees yaw)
        pose_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Publishing initial pose to AMCL-topic")
        self.initial_pose_pub.publish(pose_msg)

        self.initial_pose_timer.cancel()

    def stop_ttbot(self):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired speed.
        @param  heading   Desired yaw angle.
        @return path      object containing the sequence of waypoints of the created path.
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().warning("Robot Stopped.")

    def bumper_callback(self, msg):
            """
            High-priority bumper collisions.
            Overrides all other logic if a collision is detected.
            """
            # The msg.states list will be NON-EMPTY if there is a collision
            if msg.states:

                if not self.bumper_hit:
                    self.get_logger().warn("BUMPER HIT! Overriding all logic and reversing.")
                    self.hit_time = self.get_clock().now()
                
                self.bumper_hit = True
                
                # --- SAFETY OVERRIDE: Publish reverse command directly ---
                twist_msg = Twist()
                twist_msg.linear.x = -0.15  # Set a constant reverse speed
                twist_msg.angular.z = 0.3
                self.publisher_.publish(twist_msg)
                
                # Reset the PID controllers so they don't wind up
                self.reset_all_controllers()
                
            else:
                # --- NO BUMPER IS PRESSED ---
                if self.bumper_hit:
                    # This block runs once when the bumper is released

                    # Stop the robot (so it doesn't lurch) before resuming logic
                    twist_msg = Twist()
                    
                    #wait a moment to drive after bumper released
                    if self.get_clock().now().nanoseconds - self.hit_time.nanoseconds > 2e9:
                        #Debug all vel 0
                        twist_msg.linear.x = 0.0
                        twist_msg.angular.z = 0.0
                        self.bumper_hit = False
                        self.get_logger().info("Bumper released.")
                    self.publisher_.publish(twist_msg)


    def reset_all_controllers(self):
        # Reset ball PID
        self.integral_angular = 0.0
        self.previous_error_angular = 0.0
        self.last_time_ball = None
        self.wall_follower.reset_pid()



def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()

    try:
        rclpy.spin(object_detector) 
    except KeyboardInterrupt:
        object_detector.get_logger().warning('Keyboard interrupt received. Stopping the robot...')
        object_detector.stop_ttbot()
    finally:
        object_detector.destroy_node()
        rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()