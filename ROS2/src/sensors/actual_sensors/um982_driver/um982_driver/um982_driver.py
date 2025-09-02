import sys
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion


from spatialmath import SE3
# import os, sys
# current_dir = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(current_dir)
from um982 import UM982Serial


def average_angles(angles):
    # 将所有角度转换为二维向量
    x = np.cos(angles)
    y = np.sin(angles)
    # 计算这些向量的平均值
    mean_x = np.mean(x)
    mean_y = np.mean(y)
    # 计算平均向量的角度
    average_angle = np.arctan2(mean_y, mean_x)
    return average_angle


class UM982DriverROS2(Node):

    def _ros_log_debug(self, log_data):
        self.get_logger().debug(str(log_data))

    def _ros_log_info(self, log_data):
        self.get_logger().info(str(log_data))

    def _ros_log_warn(self, log_data):
        self.get_logger().warn(str(log_data))

    def _ros_log_error(self, log_data):
        self.get_logger().error(str(log_data))


    def __init__(self) -> None:
        super().__init__('um982_serial_driver')
        # Step1：从参数服务器获取port和baud
        self.declare_parameter('port', "/dev/omni_rtk")
        self.declare_parameter('baud', 921600)
        self.declare_parameter('map_x', 440652.0965481297-1.711115977501031)
        self.declare_parameter('map_y', 4423949.870673272+0.4883272570371628)
        # self.declare_parameter('map_theta', math.pi + 0.18 + 0.02)
        port           = self.get_parameter('port').get_parameter_value().string_value
        baud           = self.get_parameter('baud').get_parameter_value().integer_value
        self.map_x     = self.get_parameter('map_x').get_parameter_value().double_value
        self.map_y     = self.get_parameter('map_y').get_parameter_value().double_value
        # self.map_theta = self.get_parameter('map_theta').get_parameter_value().double_value
        self.map_theta = 0
        # Step2：打开串口
        try:
            self.um982serial = UM982Serial(port, baud)
            self._ros_log_info(f'serial {port} open successfully!')
        except:
            self._ros_log_error(f'serial {port} do not open!')
            sys.exit(0)
        # Step3：新建一个线程用于处理串口数据
        self.um982serial.start()
        # Step4：ROS相关
        self.fix_pub        = self.create_publisher(NavSatFix, '/gps/fix',     10)
        self.odom_pub       = self.create_publisher(Odometry,  '/gps/odom',  10)
        self.pub_timer      = self.create_timer(1/20, self.pub_task)
        self.tf_broadcast   = tf2_ros.StaticTransformBroadcaster(self)
        # Step5：校准
        time.sleep(5)
        self._ros_log_info('开始校准')
        robot_xs   = []
        robot_ys   = []
        robot_yaws = []
        for i in range(5):
            utm_x, utm_y = self.um982serial.utmpos
            vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = self.um982serial.vel
            heading, pitch, roll = self.um982serial.orientation
            utm_heading = - heading + 90
            # 坐标变换
            T_earth_gps = SE3(utm_x, utm_y, 0)           * SE3.Rz(utm_heading,    unit='deg')
            T_earth_map = SE3(self.map_x, self.map_y, 0) * SE3.Rz(self.map_theta, unit='rad')
            T_gps_robot = SE3.Rz(math.pi)
            T_map_robot = T_earth_map.inv() * T_earth_gps * T_gps_robot
            robot_x, robot_y, _ = T_map_robot.t.tolist()
            _, _, robot_yaw     = T_map_robot.rpy()
            robot_xs.append(robot_x)
            robot_ys.append(robot_y)
            robot_yaws.append(robot_yaw)
            time.sleep(0.2)
        robot_x_mean = np.mean(robot_xs)
        robot_y_mean = np.mean(robot_ys)
        robot_yaw_mean = average_angles(robot_yaws)
        self.map_theta = robot_yaw_mean

        # t = TransformStamped()
        # # 填写时间戳和帧 ID
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = "map"
        # t.child_frame_id = "start_point"
        # # 填写位置
        # t.transform.translation.x = robot_x_mean
        # t.transform.translation.y = robot_y_mean
        # t.transform.translation.z = 0.0
        # quaternion = quaternion_from_euler(0, 0, robot_yaw_mean)
        # t.transform.rotation.x    = quaternion[0]
        # t.transform.rotation.y    = quaternion[1]
        # t.transform.rotation.z    = quaternion[2]
        # t.transform.rotation.w    = quaternion[3]
        # self.tf_broadcast.sendTransform(t)
        self._ros_log_info(f'校准结束，起始位置\t x: {robot_x_mean} \t y: {robot_y_mean} \t yaw: {math.degrees(robot_yaw_mean)}' )

    def pub_task(self):
        bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.um982serial.fix
        utm_x, utm_y = self.um982serial.utmpos
        vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = self.um982serial.vel
        heading, pitch, roll = self.um982serial.orientation
        utm_heading = - heading + 90
        this_time = self.get_clock().now().to_msg()

        # Publish GPS Fix Data
        fix_msg = NavSatFix()
        fix_msg.header.stamp = this_time
        fix_msg.header.frame_id = 'gps'
        fix_msg.latitude = bestpos_lat
        fix_msg.longitude = bestpos_lon
        fix_msg.altitude = bestpos_hgt
        fix_msg.position_covariance[0] = float(bestpos_latstd)**2
        fix_msg.position_covariance[4] = float(bestpos_lonstd)**2
        fix_msg.position_covariance[8] = float(bestpos_hgtstd)**2
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix_msg)

        # 坐标变换
        T_earth_gps = SE3(utm_x, utm_y, 0)           * SE3.Rz(utm_heading,    unit='deg')
        T_earth_map = SE3(self.map_x, self.map_y, 0) * SE3.Rz(self.map_theta, unit='rad')
        T_gps_robot = SE3.Rz(math.pi)
        T_map_robot = T_earth_map.inv() * T_earth_gps * T_gps_robot
        robot_x, robot_y, _ = T_map_robot.t.tolist()
        _, _, robot_yaw     = T_map_robot.rpy()
        # print("Map坐标系Robot坐标：", robot_x, robot_y, math.degrees(robot_yaw))

        # 速度变换
        T_vearth_gps = SE3(vel_east, vel_north, 0)  * SE3.Rz(utm_heading,    unit='deg')
        T_vearth_map = SE3.Rz(self.map_theta, unit='rad')
        T_vmap_robot = T_vearth_map.inv() * T_vearth_gps * T_gps_robot
        robot_vx, robot_vy, _ = T_vmap_robot.t.tolist()
        # print("Map坐标系Robot速度：", robot_vx, robot_vy)

        # 发布odom
        odom_msg = Odometry()
        odom_msg.header.stamp = this_time
        odom_msg.header.frame_id = 'map'#
        odom_msg.child_frame_id  = 'base_link'
        odom_msg.pose.pose.position.x = robot_x
        odom_msg.pose.pose.position.y = robot_y
        odom_msg.pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0, 0, 0)
        # quaternion = quaternion_from_euler(0, 0, 0)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]
        odom_msg.pose.covariance         = [0.001] * 36
        odom_msg.pose.covariance[0]      = 10*float(bestpos_latstd)
        odom_msg.pose.covariance[7]      = 10*float(bestpos_lonstd)
        odom_msg.pose.covariance[14]     = 10*float(bestpos_hgtstd)
        odom_msg.pose.covariance[21]     = 0.5
        odom_msg.pose.covariance[28]     = 0.5
        odom_msg.pose.covariance[35]     = 0.5
        odom_msg.twist.twist.linear.x    = robot_vx
        odom_msg.twist.twist.linear.y    = robot_vy
        odom_msg.twist.twist.linear.z    = 0.0001
        odom_msg.twist.covariance        = [0.001] * 36
        odom_msg.twist.covariance[0]     = 10*float(vel_east_std)
        odom_msg.twist.covariance[7]     = 10*float(vel_north_std)
        odom_msg.twist.covariance[14]    = 0.0001
        self.odom_pub.publish(odom_msg)

        # t = TransformStamped()
        # # 填写时间戳和帧 ID
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = "map"
        # t.child_frame_id = "base_link"
        # # 填写位置
        # t.transform.translation.x = robot_x
        # t.transform.translation.y = robot_y
        # t.transform.translation.z = 0.0
        # quaternion = quaternion_from_euler(0, 0, 0)
        # t.transform.rotation.x    = quaternion[0]
        # t.transform.rotation.y    = quaternion[1]
        # t.transform.rotation.z    = quaternion[2]
        # t.transform.rotation.w    = quaternion[3]
        # self.tf_broadcast.sendTransform(t)

    def run(self):
        if rclpy.ok():
            rclpy.spin(self)

    def stop(self):
        self.um982serial.stop()
        self.pub_timer.cancel()



import time
import signal




rclpy.init()
um982_driver = UM982DriverROS2()


def signal_handler(sig, frame):
    um982_driver.stop()
    time.sleep(0.1)
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


def main():
    um982_driver.run()

if __name__ == "__main__":
    main()
