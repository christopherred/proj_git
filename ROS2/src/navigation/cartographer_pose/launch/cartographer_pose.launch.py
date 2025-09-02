from launch import LaunchDescription
from launch_ros.actions import Node
import os
import pathlib
import yaml


def generate_launch_description():


    # TF2POSE
    tf_to_pose_node = Node(
        package     = 'cartographer_pose',
        executable  = 'tf_to_pose_node',
        name        = 'tf_to_pose_node',
        output      = 'screen'
    )

    # EKF
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'ekf_localization.yaml'
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    ekf_localization_node = Node(
        package     = 'robot_localization',
        executable  = 'ekf_node',
        name        = 'ekf_localization_node',
        parameters  = [
                parameters_file_path,
                str(parameters_file_path)
            ],
        output      = 'screen',
        remappings  = [
            ('/odometry/filtered',   '/cartographer/filtered_odom'),
            ('/cmd_vel',             '/chassis/command/cmd_vel'),
        ],
    )


    return LaunchDescription([
        tf_to_pose_node,
        ekf_localization_node
    ])



