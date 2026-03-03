from setuptools import setup

package_name = 'turtlebot3_gazebo'

setup(
    name=package_name,
    version='2.2.6',
    packages=[package_name, package_name + '.common', package_name + '.nodes'],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'map_navigator = turtlebot3_gazebo.nodes.global_planner_node:main',
            'map_navigator_rrt_star = turtlebot3_gazebo.nodes.global_planner_node:main',
            'slam_explorer = turtlebot3_gazebo.nodes.frontier_explorer_node:main',
            'vision_navigator = turtlebot3_gazebo.nodes.vision_node:main',
            'costmap_server = turtlebot3_gazebo.nodes.costmap_node:main',
            'path_follower = turtlebot3_gazebo.nodes.path_follower_node:main',
            'obstacle_detector = turtlebot3_gazebo.nodes.obstacle_detector_node:main',
            'dynamic_obstacles = turtlebot3_gazebo.nodes.gazebo_spawner_node:main',
            'static_obstacles = turtlebot3_gazebo.nodes.gazebo_spawner_node:main',
            'spawn_objects = turtlebot3_gazebo.nodes.gazebo_spawner_node:main',
            'gazebo_spawner = turtlebot3_gazebo.nodes.gazebo_spawner_node:main',
        ],
    },
)
