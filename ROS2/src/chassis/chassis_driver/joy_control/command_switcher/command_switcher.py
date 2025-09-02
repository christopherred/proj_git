import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import Empty

STOP     = 0
JOYSTICK = 1
AUTO     = 2
KEYBOARD = 3

class CommandSwitcher(Node):
    def __init__(self):
        super().__init__('command_switcher')

        # 初始化订阅者
        self.joystick_sub = self.create_subscription(Twist, '/controller/joystick/cmd_vel', self.joystick_callback, 10)
        self.auto_sub     = self.create_subscription(Twist, '/controller/auto/cmd_vel',     self.auto_callback,     10)
        self.keyboard_sub = self.create_subscription(Twist, '/controller/keyboard/cmd_vel', self.keyboard_callback, 10)

        # 初始化发布者
        self.cmd_pub  = self.create_publisher(Twist, '/chassis/command/cmd_vel', 10)
        self.mode_pub = self.create_publisher(Int32, '/chassis/mode', 10)

        # 初始化服务
        self.joystick_srv = self.create_service(Empty, 'set_joystick_control', self.set_joystick_control_callback)
        self.auto_srv     = self.create_service(Empty, 'set_auto_control',     self.set_auto_control_callback)
        self.keyboard_srv = self.create_service(Empty, 'set_keyboard_control', self.set_keyboard_control_callback)
        self.STOP_srv     = self.create_service(Empty, 'set_stop',             self.set_stop_callback)

        # 初始化控制模式状态
        self.control_mode = STOP
        self.get_logger().info('Initialized with stop')

        # 以10Hz频率发布mode
        self.timer = self.create_timer(0.1, self.publish_mode)

    def set_joystick_control_callback(self, request, response):
        self.control_mode = JOYSTICK
        self.get_logger().info('Switched to joystick control')
        return response

    def set_auto_control_callback(self, request, response):
        self.control_mode = AUTO
        self.get_logger().info('Switched to auto control')
        return response

    def set_keyboard_control_callback(self, request, response):
        self.control_mode = KEYBOARD
        self.get_logger().info('Switched to keyboard control')
        return response

    def set_stop_callback(self, request, response):
        self.control_mode = STOP
        self.get_logger().info('STOP !!!')
        return response

    def joystick_callback(self, msg):
        if self.control_mode == JOYSTICK:
            self.cmd_pub.publish(msg)

    def auto_callback(self, msg):
        if self.control_mode == AUTO:
            self.cmd_pub.publish(msg)

    def keyboard_callback(self, msg):
        if self.control_mode == KEYBOARD:
            self.cmd_pub.publish(msg)

    def stop_cmd_pub(self):
        if self.control_mode == STOP:
            msg = Twist()
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            msg.linear.x  = 0.0
            msg.linear.y  = 0.0
            msg.linear.z  = 0.0
            self.cmd_pub.publish(msg)

    def publish_mode(self):
        mode_msg = Int32()
        mode_msg.data = self.control_mode
        self.mode_pub.publish(mode_msg)
        self.stop_cmd_pub()


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
command_switcher = CommandSwitcher()


def main():
    command_switcher.run()


if __name__ == "__main__":
    main()
