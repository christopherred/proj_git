from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    mpc_controller_node = Node(
        package     = 'mpc_controller',
        executable  = 'mpc_controller_node',
        name        = 'mpc_controller_node',
        output      = 'screen',
        emulate_tty = True,
    )

    traj_publisher_node = Node(
        package     = 'mpc_controller',
        executable  = 'traj_publisher_node',
        name        = 'traj_publisher_node',
        output      = 'screen',
        emulate_tty = True,
    )
    
    return LaunchDescription([
        mpc_controller_node,
        traj_publisher_node,
    ])
