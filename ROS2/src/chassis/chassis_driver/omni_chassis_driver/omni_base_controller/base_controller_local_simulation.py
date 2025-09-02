import math
import numpy as np
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
import time
import rclpy
from rclpy.node import Node
import signal
import sys
from rclpy.qos import qos_profile_sensor_data

    # def run_model(self, vx, vy, theta):
    #     # _theta = -np.pi/3 - self.yaw
    #     _theta = -np.pi/3
    #     vx_new = np.cos(_theta)*vx - np.sin(_theta)*vy
    #     vy_new = np.sin(_theta)*vx + np.cos(_theta)*vy
    #     convert_matrix = np.array([
    #         [0,             1,     self.r],
    #         [-np.sqrt(3)/2, -0.5,  self.r],
    #         [+np.sqrt(3)/2, -0.5,  self.r]
    #     ])
    #     dphi = (1/self.R)*convert_matrix@np.array([[vx_new],[vy_new],[theta]])
    #     return dphi.reshape(-1).tolist()

class Omni3WheelRobotKinematics(object):
    def __init__(self, wheel_radius, wheel_spacing):
        """初始化运动学

        Args:
            wheel_radius (float): 轮子的半径
            wheel_spacing (float): 每个轮子到机器中心的距离
        """
        self.wheel_radius = wheel_radius
        self.wheel_spacing = wheel_spacing

    def forward(self, dpm1, dpm2, dpm3):
        # """正运动学

        # Args:
        #     dpm1 (float): 轮子 1 的转速 (DPM，度每秒)
        #     dpm2 (float): 轮子 2 的转速 (DPM，度每秒)
        #     dpm3 (float): 轮子 3 的转速 (DPM，度每秒)

        # Returns:
        #     (linear_x, linear_y, angular_z): 机器人在 X 方向的线速度、Y 方向的线速度，以及绕 Z 轴的角速度
        # """
        #_theta = -np.pi/3
        _theta = 0
        # 构建轮子速度的向量
        dphi = np.array([[dpm1], [dpm2], [dpm3]])

        # 计算转换矩阵的逆
        # convert_matrix_inverse = np.linalg.inv(np.array([
        #     [0,             1,     self.wheel_radius],
        #     [-np.sqrt(3)/2, -0.5,  self.wheel_radius],
        #     [+np.sqrt(3)/2, -0.5,  self.wheel_radius]
        # ]) * (1/self.wheel_spacing))
        convert_matrix_inverse = np.linalg.inv(np.array([
            [-np.sqrt(3)/2, 0.5, self.wheel_spacing],
            [0            , -1 , self.wheel_spacing],
            [ np.sqrt(3)/2, 0.5, self.wheel_spacing]
        ])* (1/self.wheel_radius))
        # 得到速度向量
        velocity_vector = convert_matrix_inverse @ dphi / np.pi
        vx_new, vy_new, angular_z = velocity_vector.flatten()

        # 将新坐标系下的速度转换回原始坐标系
        linear_x = (np.cos(_theta) * vx_new + np.sin(_theta) * vy_new)
        # linear_y = -np.sin(_theta) * vx_new + np.cos(_theta) * vy_new
        linear_y = (-np.sin(_theta) * vx_new + np.cos(_theta) * vy_new)

        return (linear_x, linear_y, angular_z*np.pi)


    def inverse(self, linear_x, linear_y, angular_z):
        # """逆运动学

        # Args:
        #     linear_x (float): 机器人在 X 方向的线速度 (m/s)
        #     linear_y (float): 机器人在 Y 方向的线速度 (m/s)
        #     angular_z (float): 机器人绕 Z 轴的角速度 (rad/s)

        # Returns:
        #     (dpm1, dpm2, dpm3): 三个轮子的转速 (DPM，度每秒)
        # """

        #_theta = -np.pi/3
        _theta = 0
        # vx_new = np.cos(_theta)*(-linear_x)- np.sin(_theta)*linear_y
        # vy_new = np.sin(_theta)*(-linear_x) + np.cos(_theta)*linear_y
        vx_new = np.cos(_theta)*(linear_x)- np.sin(_theta)*(linear_y)
        vy_new = np.sin(_theta)*(linear_x) + np.cos(_theta)*(linear_y)
        # convert_matrix = np.array([
        #     [0,             1,     self.wheel_radius],
        #     [-np.sqrt(3)/2, -0.5,  self.wheel_radius],
        #     [+np.sqrt(3)/2, -0.5,  self.wheel_radius]
        # ])
        convert_matrix = np.array([
            [-np.sqrt(3)/2, 0.5,  self.wheel_spacing],
            [0, -1,  self.wheel_spacing],
            [np.sqrt(3)/2, 0.5,  self.wheel_spacing]
        ])
        #dphi = np.pi*(1/self.wheel_spacing)*convert_matrix@np.array([[vx_new],[vy_new],[angular_z]])
        dphi = np.pi*(1/self.wheel_radius)*convert_matrix@np.array([[vx_new],[vy_new],[angular_z/np.pi]])
        dpms = dphi.reshape(-1).tolist()

        return (dpms[0], dpms[1], dpms[2])


class BaseControllerROS2(Node):
    def __init__(self):
        super().__init__('base_controller')
        self.get_logger().info("Base controller started successfully")

        # 初始化轮径和轮间距参数
        self.declare_parameter('wheel_radius', 0.0375)  # 轮径为0.0375米
        self.declare_parameter('wheel_spacing', 0.175)    # 轮距为0.175米
        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        wheel_spacing = self.get_parameter('wheel_spacing').get_parameter_value().double_value

        # 初始化运动学模型
        self.kinematics = Omni3WheelRobotKinematics(wheel_radius, wheel_spacing)

        # 轮速指令消息初始化
        self.wheel_speed_msg = Float32MultiArray()
        self.wheel_speed_msg.data = [0.0, 0.0, 0.0]
        self.pub_timer = self.create_timer(0.05, self.pub_task)

        # 订阅命令速度
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/chassis/command/cmd_vel',
            self.listener_command_cmd_vel,
            qos_profile_sensor_data)

        # 发布轮速指令
        self.publisher_wheel_speed = self.create_publisher(
            Float32MultiArray,
            '/chassis/command/wheelspeed',
            qos_profile_sensor_data)

        # 订阅实际轮速
        self.subscription_actual_wheel_speed = self.create_subscription(
            Float32MultiArray,
            '/chassis/actual/wheelspeed',
            self.listener_actual_wheel_speed,
            qos_profile_sensor_data)

        # 发布实际速度命令
        self.publisher_actual_cmd_vel = self.create_publisher(
            TwistWithCovarianceStamped,
            '/chassis/actual/cmd_vel',
            qos_profile_sensor_data)

    def pub_task(self):
        # 定时发布轮速指令
        self.publisher_wheel_speed.publish(self.wheel_speed_msg)
        

    def listener_command_cmd_vel(self, msg):
        # 使用逆运动学将线速度和角速度转换为三个轮子的速度 (DPM)
        dpm1, dpm2, dpm3 = self.kinematics.inverse(msg.linear.x, msg.linear.y, msg.angular.z)
        #dpm1, dpm2, dpm3 = self.kinematics.inverse(-0.1, 0, 0)
        wheel_speed_msg = Float32MultiArray()
        wheel_speed_msg.data = [dpm1, dpm2, dpm3]
        self.wheel_speed_msg = wheel_speed_msg

    def listener_actual_wheel_speed(self, msg):
        # 使用正运动学将实际轮速转换为线速度和角速度
        linear_x, linear_y, angular_z = self.kinematics.forward(msg.data[0], msg.data[1], msg.data[2])
        #linear_x, linear_y, angular_z = self.kinematics.forward(7.255, 0, -7.255)
        # 创建 TwistWithCovarianceStamped 消息
        cmd_vel_msg = TwistWithCovarianceStamped()
        # 填充 header 信息
        cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_msg.header.frame_id = "base_link"
        # 设置线速度和角速度
        cmd_vel_msg.twist.twist.linear.x = linear_x
        cmd_vel_msg.twist.twist.linear.y = linear_y
        cmd_vel_msg.twist.twist.linear.z = 0.0
        cmd_vel_msg.twist.twist.angular.x = 0.0
        cmd_vel_msg.twist.twist.angular.y = 0.0
        cmd_vel_msg.twist.twist.angular.z = angular_z
        # 设置协方差矩阵
        covariance_matrix = [0.0] * 36  # 初始化为零矩阵
        covariance_matrix[0] = 0.0001 if linear_x == 0 else 0.2
        covariance_matrix[7] = 0.0001 if linear_y == 0 else 0.2
        covariance_matrix[35] = 0.0001 if angular_z == 0 else 0.2
        cmd_vel_msg.twist.covariance = covariance_matrix
        # 发布消息
        self.publisher_actual_cmd_vel.publish(cmd_vel_msg)

    def run(self):
        if rclpy.ok():
            rclpy.spin(self)


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
