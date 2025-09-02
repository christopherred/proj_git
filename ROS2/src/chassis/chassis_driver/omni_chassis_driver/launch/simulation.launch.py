from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 定义参数，用于切换控制器
    use_global_controller = LaunchConfiguration('use_global_controller')
    use_simulation = LaunchConfiguration('use_simulation')

    # 声明参数
    declare_use_global_controller = DeclareLaunchArgument(
        'use_global_controller',  # 参数名称
        default_value='true',     # 默认值，可以设置为 'false'
        description='Use global base controller if true, otherwise use local base controller'
    )

    declare_use_simulation = DeclareLaunchArgument(
        'use_simulation',  # 参数名称
        default_value='true',  # 默认值，可以设置为 'true'
        description='Use simulation mode if true, otherwise use real hardware'
    )

    # 全局控制器节点
    base_controller_global_node = Node(
        package='omni_chassis_driver',
        executable='omni_base_controller_global_node',
        name='omni_base_controller_global_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'wheel_radius': 0.0375}, {'wheel_spacing': 0.175}],
        condition=IfCondition(use_global_controller),
    )

    # # 局部控制器节点
    # base_controller_local_node = Node(
    #     package='omni_chassis_driver',
    #     executable='omni_base_controller_local_node',
    #     name='omni_base_controller_local_node',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[{'wheel_radius': 0.0375}, {'wheel_spacing': 0.175}],
    #     condition=UnlessCondition(use_global_controller),
    # )

    # # 实物驱动器节点
    # base_driver_node = Node(
    #     package='omni_chassis_driver',
    #     executable='omni_base_driver_node',
    #     name='omni_base_driver_node',
    #     output='screen',
    #     condition=UnlessCondition(use_simulation),
    # )

    # 仿真驱动器节点
    simulation_base_driver_node = Node(
        package='omni_chassis_driver',
        executable='omni_simulation_base_driver_node',
        name='omni_simulation_base_driver_node',
        output='screen',
        condition=IfCondition(use_simulation),
    )

    return LaunchDescription([
        # 声明参数
        declare_use_global_controller,
        declare_use_simulation,

        # 加载全局控制器节点
        base_controller_global_node,


        # 加载仿真驱动器节点
        simulation_base_driver_node,
    ])