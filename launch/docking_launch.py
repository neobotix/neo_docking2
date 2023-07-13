# Neobotix GmbH

import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_namespace = launch.substitutions.LaunchConfiguration('namespace', default="")
    config = os.path.join(get_package_share_directory('neo_docking2'),'launch','dock_pose.yaml')
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='neo_docking2',
            executable='neo_docking2',
            namespace=robot_namespace,
            output='screen',
            parameters = [config])
    ])
