from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    command_switcher_node = Node(
        package     = 'joy_control',
        executable  = 'command_switcher_node',
        name        = 'command_switcher_node',
        output      = 'screen',
        emulate_tty = True,
        # remappings  = [
        #     ('/chassis/command/cmd_vel', '/omni_command/cmd_vel'),  # 重映射命令速度话题
        # ]
    )

    joy_node = Node(
        package     = 'joy',
        executable  = 'joy_node',
        name        = 'joy_node',
        output      ='screen',
        remappings  = [
            ('/joy',              '/controller/joystick/joy'),
            ('/joy/set_feedback', '/controller/joystick/set_feedback')
        ],
        parameters  = [
            {'deadzone': 0.01},
        ]
    )


    joy_controller_node = Node(
        package     = 'joy_control',
        executable  = 'joy_controller_node',
        name        = 'joy_controller_node',
        output      = 'screen',
        parameters  = [
            {'auto_button':     0},
            {'joystick_button': 3},
            {'keyboard_button': 4},
            {'stop_button':     1},
            {'vx_axis':         1},
            {'vy_axis':         0},
            {'vtheta_axis':     3},
            {'vx_scale':        0.3},
            {'vy_scale':        0.3},
            {'vtheta_scale':    1.0},
        ],
    )




    return LaunchDescription([
        command_switcher_node,
        joy_node,
        joy_controller_node,
    ])
