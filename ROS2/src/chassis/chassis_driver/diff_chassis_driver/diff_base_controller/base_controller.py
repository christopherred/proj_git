
import math
import numpy as np



class DiffWhellRobotKinematics(object):
    def __init__(self, wheel_diameter, wheel_spacing) -> None:
        """初始化运动学

        Args:
            wheel_diameter (float): 轮子的直径
            wheel_spacing (float): 两个驱动轮之间的距离
        """
        self.wheel_diameter = wheel_diameter
        self.wheel_spacing  = wheel_spacing

    def forward(self, rpm_left, rpm_right):
        """正运动学

        Args:
            left_speed (float): 左轮转速（RPM, 转每分）
            right_speed (float): 右轮速度（RPM, 转每分）

        Returns:
            ( linear_x, angular_z ): 机器人的线速度 (m/s) 和 机器人的角速度 (rad/s)
        """

        # 计算左右轮的线速度 (m/s)
        v_left  = rpm_left * math.pi * self.wheel_diameter / 60.0
        v_right = rpm_right * math.pi * self.wheel_diameter / 60.0

        # 计算机器人的线速度 (m/s)
        linear_x = 0.5 * (v_left + v_right)

        # 计算机器人的角速度 (rad/s)
        angular_z = (v_right - v_left) / self.wheel_spacing

        return (linear_x, angular_z)

    def inverse(self, linear_x, angular_z):
        """逆运动学

        Args:
            linear_x (float): 机器人的线速度 (m/s)
            angular_z (float): 机器人的角速度 (rad/s)

        Returns:
            ( left_speed, right_speed ): 左轮转速（RPM, 转每分） 和 右轮速度（RPM, 转每分）
        """

        # 根据期望的线速度和角速度计算每个轮子的速度
        v_left = linear_x - angular_z * self.wheel_spacing / 2
        v_right = linear_x + angular_z * self.wheel_spacing / 2

        # 将线速度转换为转速 (RPM)
        rpm_left = (v_left * 60) / (math.pi * self.wheel_diameter)
        rpm_right = (v_right * 60) / (math.pi * self.wheel_diameter)

        return (rpm_left, rpm_right)


import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from std_msgs.msg import Float32MultiArray

class BaseControllerROS2(Node):
    def __init__(self):
        super().__init__('base_controller')
        self.get_logger().info("start successfully")

        self.declare_parameter('wheel_diameter', 0.173)  # 轮径为0.37米
        self.declare_parameter('wheel_spacing', 0.37)    # 轮距为0.5米
        wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        wheel_spacing = self.get_parameter('wheel_spacing').get_parameter_value().double_value

        self.kinematics = DiffWhellRobotKinematics(wheel_diameter, wheel_spacing)  # 轮径为0.37米，轮距为0.5米

        self.wheel_speed_msg = Float32MultiArray()
        self.wheel_speed_msg.data = [0.0, 0.0]
        self.pub_timer = self.create_timer(0.05, self.pub_task)

        # 订阅命令速度
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/chassis/command/cmd_vel',
            self.listener_command_cmd_vel,
            10)

        # 发布轮速指令
        self.publisher_wheel_speed = self.create_publisher(
            Float32MultiArray,
            '/chassis/command/wheelspeed',
            10)

        # 订阅实际轮速
        self.subscription_actual_wheel_speed = self.create_subscription(
            Float32MultiArray,
            '/chassis/actual/wheelspeed',
            self.listener_actual_wheel_speed,
            10)

        # 发布实际速度命令
        self.publisher_actual_cmd_vel = self.create_publisher(
            TwistWithCovarianceStamped,
            '/chassis/actual/cmd_vel',
            10)


    def pub_task(self):
        self.publisher_wheel_speed.publish(self.wheel_speed_msg)

    def listener_command_cmd_vel(self, msg):
        # 使用逆运动学转换线速度和角速度为轮速 (RPM)
        rpm_left, rpm_right = self.kinematics.inverse(msg.linear.x, msg.angular.z)
        wheel_speed_msg = Float32MultiArray()
        wheel_speed_msg.data = [rpm_left, rpm_right]
        self.wheel_speed_msg = wheel_speed_msg
        # self.publisher_wheel_speed.publish(wheel_speed_msg)

    def listener_actual_wheel_speed(self, msg):
        # 使用正运动学转换实际轮速为线速度和角速度
        linear_x, angular_z = self.kinematics.forward(msg.data[0], msg.data[1])
        # 创建 TwistWithCovarianceStamped 消息
        cmd_vel_msg = TwistWithCovarianceStamped()
        # 填充 header 信息，设置时间戳和帧ID
        cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_msg.header.frame_id = "base_link"
        # 设置线速度和角速度
        cmd_vel_msg.twist.twist.linear.x = linear_x
        cmd_vel_msg.twist.twist.linear.y = 0.0
        cmd_vel_msg.twist.twist.linear.z = 0.0
        cmd_vel_msg.twist.twist.angular.x = 0.0
        cmd_vel_msg.twist.twist.angular.y = 0.0
        cmd_vel_msg.twist.twist.angular.z = angular_z
        # 设置协方差矩阵
        covariance_matrix = [0.0] * 36  # 初始化为零矩阵
        # 设置 linear.x 的方差 (covariance[0] 对应 linear.x)
        covariance_matrix[0] = 0.0001 if linear_x == 0 else 0.15
        # 设置 linear.y 的方差 (covariance[7] 对应 linear.y)
        covariance_matrix[7] = 0.0001
        # 设置 angular.z 的方差 (covariance[35] 对应 angular.z)
        covariance_matrix[35] = 0.0001 if angular_z == 0 else 0.15
        # 将协方差矩阵赋值给消息
        cmd_vel_msg.twist.covariance = covariance_matrix
        # 发布消息
        self.publisher_actual_cmd_vel.publish(cmd_vel_msg)

    def run(self):
        if rclpy.ok():
            rclpy.spin(self)


import sys
import signal

def signal_handler(sig, frame):
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
rclpy.init()
base_controller = BaseControllerROS2()


def main():
    base_controller.run()


if __name__ == "__main__":
    main()
