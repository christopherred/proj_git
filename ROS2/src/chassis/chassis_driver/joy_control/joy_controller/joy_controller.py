import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        # 从参数服务器获取按钮序号
        self.auto_button     = self.declare_parameter('auto_button',     1).get_parameter_value().integer_value
        self.joystick_button = self.declare_parameter('joystick_button', 2).get_parameter_value().integer_value
        self.keyboard_button = self.declare_parameter('keyboard_button', 3).get_parameter_value().integer_value
        self.stop_button     = self.declare_parameter('stop_button',     4).get_parameter_value().integer_value

        # 订阅话题
        self.joy_subscription = self.create_subscription(
            Joy,
            '/controller/joystick/joy',
            self.joy_callback,
            10)

        # 从参数服务器获取axes索引
        self.vx_axis     = self.declare_parameter('vx_axis',     0).get_parameter_value().integer_value
        self.vy_axis     = self.declare_parameter('vy_axis',     1).get_parameter_value().integer_value
        self.vtheta_axis = self.declare_parameter('vtheta_axis', 2).get_parameter_value().integer_value

        # 从参数服务器获取缩放因子
        self.vx_scale     = self.declare_parameter('vx_scale',     1.0).get_parameter_value().double_value
        self.vy_scale     = self.declare_parameter('vy_scale',     1.0).get_parameter_value().double_value
        self.vtheta_scale = self.declare_parameter('vtheta_scale', 1.0).get_parameter_value().double_value

        # 创建客户端以调用服务
        self.auto_control_client     = self.create_client(Empty, '/set_auto_control')
        self.joystick_control_client = self.create_client(Empty, '/set_joystick_control')
        self.keyboard_control_client = self.create_client(Empty, '/set_keyboard_control')
        self.stop_client             = self.create_client(Empty, '/set_stop')

        # 等待服务可用
        self.auto_control_client.wait_for_service()
        self.joystick_control_client.wait_for_service()
        self.keyboard_control_client.wait_for_service()
        self.stop_client.wait_for_service()

        # 初始化速度发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, '/controller/joystick/cmd_vel', 10)

    def joy_callback(self, msg):
        # 检查四个按钮是否按下
        if msg.buttons[self.auto_button]     == 1:
            self.call_service(self.auto_control_client)
        if msg.buttons[self.joystick_button] == 1:
            self.call_service(self.joystick_control_client)
        if msg.buttons[self.keyboard_button] == 1:
            self.call_service(self.keyboard_control_client)
        if msg.buttons[self.stop_button]     == 1:
            self.call_service(self.stop_client)


        # 创建并发布Twist消息
        twist = Twist()
        twist.linear.x  = msg.axes[self.vx_axis]     * self.vx_scale
        twist.linear.y  = msg.axes[self.vy_axis]     * self.vy_scale
        twist.angular.z = msg.axes[self.vtheta_axis] * self.vtheta_scale
        self.cmd_vel_publisher.publish(twist)

    def call_service(self, client):
        # 创建一个空的请求并调用服务
        request = Empty.Request()
        client.call_async(request)


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
joy_controller = JoyController()

def main():
    joy_controller.run()

if __name__ == '__main__':
    main()
