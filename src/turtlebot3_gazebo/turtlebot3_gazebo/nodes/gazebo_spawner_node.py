# Extracted from monolithic nodes. No logic changes.

import os
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from geometry_msgs.msg import Pose

# --- Constants ---
# Timer period for position updates (seconds)
TIMER_PERIOD_SEC = 0.1
# Period counter reset value
ELAPSED_RESET = 201
# Dynamic obstacles spawn positions
DYN_OBS1_X = 6.0
DYN_OBS1_Y = -0.45
DYN_BALL_X = -4.95
DYN_BALL_Y = -2.0
# Static obstacles spawn positions
STAT_OBS0_X = -3.75
STAT_OBS0_Y = 0.5
STAT_OBS1_X = 6.0
STAT_OBS1_Y = -0.45
# Colored balls spawn positions
BALL_BASE_X = 0.5
BALL_BASE_Y = 0.5
BALL_BASE_Z = 0.1
RED_BALL_OFFSET = 0.3
BLUE_BALL_OFFSET = 0.7
GREEN_BALL_OFFSET = 1.5
RED_BALL_POS_X = -5.0
RED_BALL_POS_Y = -3.0
BLUE_BALL_POS_X = 7.0
BLUE_BALL_POS_Y = -0.7
GREEN_BALL_POS_X = 8.0
GREEN_BALL_POS_Y = -5.0
# Default spawn pose altitude (m) for obstacle models
SPAWN_ALT_OBSTACLE = 1.0


class GazeboSpawner(Node):
    """Spawns and positions Gazebo models for dynamic obstacles, static obstacles, or colored balls."""

    def __init__(self):
        """Initialise GazeboSpawner: read spawn_mode param, wait for services, spawn models."""
        super().__init__('gazebo_spawner')

        self.declare_parameter('spawn_mode', 'dynamic_obstacles')
        self.spawn_mode = self.get_parameter('spawn_mode').get_parameter_value().string_value

        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.set_state_client = self.create_client(SetEntityState, 'set_entity_state')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')

        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set state service not available, waiting...')

        self.elapsed = 0

        if self.spawn_mode == 'dynamic_obstacles':
            self._spawn_dynamic_obstacles()
        elif self.spawn_mode == 'static_obstacles':
            self._spawn_static_obstacles()
        elif self.spawn_mode == 'colored_balls':
            self._spawn_colored_balls()
        else:
            self.get_logger().error(f"Unknown spawn_mode: '{self.spawn_mode}'. No models spawned.")

        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.timer_callback)

    def _spawn_dynamic_obstacles(self):
        """Spawn dynamic obstacle set: trash_can + cricket_ball."""
        self.spawn_model('obstacle1', 'trash_can')
        self.spawn_model('cricket_ball', 'cricket_ball')

    def _spawn_static_obstacles(self):
        """Spawn static obstacle set: two trash_can instances."""
        self.spawn_model('obstacle0', 'trash_can')
        self.spawn_model('obstacle1', 'trash_can')

    def _spawn_colored_balls(self):
        """Spawn three colored ball objects at offset positions."""
        self.spawn_model('red_object', 'red_object', offset=RED_BALL_OFFSET)
        self.spawn_model('blue_object', 'blue_object', offset=BLUE_BALL_OFFSET)
        self.spawn_model('green_object', 'green_object', offset=GREEN_BALL_OFFSET)

    def spawn_model(self, model_name, model, offset=None):
        """Call SpawnEntity service to spawn the named model in the world."""
        initial_pose = Pose()
        if offset is not None:
            # Colored ball spawn uses offset-based positioning
            initial_pose.position.x = BALL_BASE_X
            initial_pose.position.y = BALL_BASE_Y + offset
            initial_pose.position.z = BALL_BASE_Z
        else:
            initial_pose.position.x = 0.0
            initial_pose.position.y = 0.0
            initial_pose.position.z = SPAWN_ALT_OBSTACLE

        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = self.load_model_xml_from_sdf(model)
        request.reference_frame = 'world'
        request.initial_pose = initial_pose

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Model {model_name} spawned successfully')
        else:
            self.get_logger().error(f'Failed to spawn model {model_name}')

    def load_model_xml_from_sdf(self, model_name):
        """Load and return SDF XML string for the named model from the package share directory."""
        xml_model = ""
        try:
            package_path = get_package_share_directory('turtlebot3_gazebo')
            model_path = os.path.join(package_path, 'models', model_name, 'model.sdf')
            with open(model_path, 'r') as xml_file:
                xml_model = xml_file.read().replace('\n', '')
        except Exception as e:
            self.get_logger().error(str(e))
        return xml_model

    def set_model_position(self, x, y, model_name):
        """Call SetEntityState service to move model_name to (x, y) world coordinates."""
        state = EntityState()
        state.name = model_name
        state.pose.position.x = x
        state.pose.position.y = y

        request = SetEntityState.Request()
        request.state = state
        self.set_state_client.call_async(request)

    def timer_callback(self):
        """Update model positions according to the active spawn_mode."""
        if self.spawn_mode == 'dynamic_obstacles':
            self.set_model_position(DYN_OBS1_X, DYN_OBS1_Y, 'obstacle1')
            self.set_model_position(DYN_BALL_X, DYN_BALL_Y, 'cricket_ball')
        elif self.spawn_mode == 'static_obstacles':
            self.set_model_position(STAT_OBS0_X, STAT_OBS0_Y, 'obstacle0')
            self.set_model_position(STAT_OBS1_X, STAT_OBS1_Y, 'obstacle1')
        elif self.spawn_mode == 'colored_balls':
            self.set_model_position(RED_BALL_POS_X, RED_BALL_POS_Y, 'red_object')
            self.set_model_position(BLUE_BALL_POS_X, BLUE_BALL_POS_Y, 'blue_object')
            self.set_model_position(GREEN_BALL_POS_X, GREEN_BALL_POS_Y, 'green_object')

        self.elapsed = (self.elapsed + 1) % ELAPSED_RESET

    def destroy_node(self):
        """Cancel timers and destroy the node."""
        if hasattr(self, 'timer'):
            self.timer.cancel()
        super().destroy_node()


def main(args=None):
    """Entry point: spin GazeboSpawner until interrupted."""
    rclpy.init(args=args)
    node = GazeboSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
