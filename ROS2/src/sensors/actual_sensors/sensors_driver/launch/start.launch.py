from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import TimerAction


def generate_launch_description():

    # 激光雷达节点
    # radar_driver_node = Node(
    #     package     = 'sllidar_ros2',
    #     executable  = 'sllidar_node',
    #     name        = 'sllidar_node',
    #     parameters  = [
    #         {'channel_type' :    'serial',
    #          'serial_port' :     '/dev/omni_ladar',
    #          'serial_baudrate':  1000000,
    #          'frame_id':         'radar',
    #          'inverted':         False,
    #          'angle_compensate': False,
    #          'scan_mode':        'Standard',
    #          'operation_timeout': 10000
    #     }],
    #     output      = 'screen',
    #     remappings  = [
    #         ('/scan',          '/sensors/ladar/scan'),
    #     ]
    # )
    radar_driver_node = Node(
        package     = 'sllidar_ros2',
        executable  = 'sllidar_node',
        name        = 'sllidar_node',
        parameters  = [
            {'channel_type' :    'tcp',
             'tcp_ip' :          '192.168.0.7',
             'tcp_port':         20108,
             'frame_id':         'radar',
             'inverted':         False,
             'angle_compensate': False,
             'scan_mode':        'Standard',
             'operation_timeout': 10000
        }],
        output      = 'screen',
        remappings  = [
            ('/scan',          '/sensors/ladar/scan'),
        ]
    )


    #imu节点
    imu_driver_node=Node(
        package     ="fdilink_ahrs",
        executable  ="ahrs_driver_node",
        name        ='imu_driver_node',
        parameters  = [
           {'if_debug_':         False,
            'serial_port_':      '/dev/omni_imu',
            'serial_baud_':      230400,
            'imu_topic':         '/sensors/imu/imu',
            'imu_frame_id_':     'imu',
            'mag_pose_2d_topic': '/sensors/imu/mag_pose_2d',
            'Magnetic_topic':    '/sensors/imu/magnetic',
            'Euler_angles_topic':'/sensors/imu/euler_angles',
            'gps_topic':         '/sensors/imu/gps_fix',
            'twist_topic':       '/sensors/imu/system_speed',
            'NED_odom_topic':    '/sensors/imu/NED_odometry',
            'device_type_':1
        }],
        output      = "screen",

    )

    nmea_driver_node = Node(
        package     = 'nmea_navsat_driver',
        executable  = 'nmea_serial_driver',
        name        = 'nmea_driver_node',
        parameters  = [
            {'port':             '/dev/omni_rtk',
             'baud':             921600,
             'frame_id':         'base_link',
             'time_ref_source':  'gps',
             'useRMC':           False,
             'log_level':        40
            }],
        output      = "screen",
        remappings  = [
            ('/fix',                    '/sensors/gps/fix'),
            ('/heading',                '/sensors/gps/heading'),
            ('/vel',                    '/sensors/gps/vel'),
            ('/status',                 '/sensors/gps/status'),
            ('/satellites',             '/sensors/gps/satellites'),
            ('/nmea_sentence',          '/sensors/gps/nmea_sentence'),
            ('/nmea_sentence_ignored',  '/sensors/gps/nmea_sentence_ignored'),
        ],
    )

    #gps节点
    um982_driver_node = Node(
        package     = 'um982_driver',
        executable  = 'um982_driver_node',
        name        = 'um982_driver_node',
        output      = 'screen',
        emulate_tty = True,
        parameters  = [
            {'port': '/dev/omni_rtk'},
            {'baud': 921600}
        ],
        remappings  = [
            ('/gps/odom',   '/sensors/gps/odom'),
            ('/gps/fix'   , '/sensors/gps/fix')
        ],
    )

    #icp节点
    icp_odometry_node = Node(
        package     = 'rtabmap_odom',
        executable  = 'icp_odometry',
        name        = 'icp_odometry_node',
        condition=IfCondition('true'),
        output      = 'screen',
        parameters  = [
            {'subscribe_scan':          'true',
             'subscribe_scan_cloud':    'false',
             #'ground_truth_frame_id':   'map',
             'odom_frame_id':           'start_point',
             'frame_id':                'base_link',
             'publish_tf':              False,

        }],
        remappings  = [
            ('/scan',       '/sensors/ladar/scan'),
            ('/odom',       '/sensors/icp/odom'),
            ('/imu',        '/sensors/imu/imu'),
            ('/gps/fix',    '/sensors/gps/fix'),
        ],
    )

    # 陀螺仪
    gyro_driver_node=Node(
        package     = "hwt101_yaw_ros",
        executable  = "hwt101_yaw_ros_node",
        name        = 'hwt101_yaw_ros_node',
        output      = "screen",

    )
    # delayed_radar_node = TimerAction(
    #     period=.0,
    #     actions=[radar_driver_node]
    # )

    return LaunchDescription([
        # um982_driver_node,
        imu_driver_node,
        radar_driver_node,
        # nmea_driver_node,
        # delayed_radar_node,
        # gyro_driver_node,
        # icp_odometry_node,
    ])


