from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    base_controller_node = Node(
        package     = 'diff_chassis_driver',
        executable  = 'diff_base_controller_node',
        name        = 'diff_base_controller_node',
        output      = 'screen',
        emulate_tty = True,
        parameters  = [{'wheel_radius': 0.173}, {'wheel_spacing': 0.37}]
    )

    base_driver_node = Node(
        package     = 'diff_chassis_driver',
        executable  = 'simulation_diff_base_driver_node',
        name        = 'simulation_diff_base_driver_node',
        output      = 'screen',
        emulate_tty = True,
    )
    
    return LaunchDescription([
        base_controller_node,
        base_driver_node,
    ])
