from http.server import executable
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('f1tenth_gym_ros'), 'launch'),
         '/gym_bridge_launch.py'])
      )
    obs_detect_config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'obs_detect.yaml'
    )

    # Pure pursuit
    pure_pursuit_node = Node(
        package = 'pure_pursuit_pkg',
        executable = 'pure_pursuit',
        name = 'pure_pursuit_node'
    )

    # Obstacle detection
    obs_detect_node = Node(
        package = 'obs_detect_pkg',
        executable = 'obs_detect_node',
        name = 'obs_detect_node',
        parameters = [obs_detect_config]
    )

    # Obstacle avoidance
    gap_follow_node = Node(
        package = 'gap_follow',
        executable = 'reactive_node',
        name = 'obstacle_avoidance_node'
    )

    ld.add_action(bringup)
    ld.add_action(pure_pursuit_node)
    ld.add_action(obs_detect_node)
    ld.add_action(gap_follow_node)



    return ld