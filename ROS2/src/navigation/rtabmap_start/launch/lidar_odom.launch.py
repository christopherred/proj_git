from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 基础参数
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('scan_topic', default_value='/sensors/ladar/scan'),
        
        
        # ICP 里程计节点
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='laser_odom',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'Reg/Strategy': '1',
                'Icp/PointToPlane': 'true',
                'scan_voxel_size': 0.02,  # 覆盖默认0.05，提高精度
                'scan_normal_k': 20,      # 覆盖默认5，增强法线计算
                'Icp/MaxCorrespondenceDistance': '0.5',
                'Icp/Iterations': '30',
                'OdomF2M/MaxSize': '1000',
                'queue_size': '20',
                'scan_cloud_max_points': 5000,  # 限制最大点数
            }],
            remappings=[
                ('scan', LaunchConfiguration('scan_topic')),
                ('odom', '/rtabmap/odom')
            ]
        )
    ])