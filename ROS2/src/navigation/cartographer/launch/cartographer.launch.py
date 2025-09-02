import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定位功能包
    pkg_share = FindPackageShare(package='cartographer').find('cartographer')
    
    #配置节点启动信息
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.01')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer_2d.lua')

    #声明节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        remappings  = [
            ('/scan',          '/sensors/ladar/scan'),
            ('/imu',           '/sensors/imu/imu'),
            ('/constraint_list',       '/cartographer/constraint_list'),
            ('/landmark_poses_list',   '/cartographer/landmark_poses_list'),
            ('/scan_matched_points2',  '/cartographer/scan_matched_points2'),
            # ('/submap_list',           '/cartographer/submap_list'),
            ('/trajectory_node_list',  '/cartographer/trajectory_node_list'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        remappings  = [
            ('/scan',          '/sensors/ladar/scan'),
            ('/imu',           '/sensors/imu/imu'),
            ('/constraint_list',       '/cartographer/constraint_list'),
            ('/landmark_poses_list',   '/cartographer/landmark_poses_list'),
            # ('/map',                   '/cartographer/map'),
            ('/scan_matched_points2',  '/cartographer/scan_matched_points2'),
            # ('/submap_list',           '/cartographer/submap_list'),
            ('/trajectory_node_list',  '/cartographer/trajectory_node_list'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    #启动文件
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld