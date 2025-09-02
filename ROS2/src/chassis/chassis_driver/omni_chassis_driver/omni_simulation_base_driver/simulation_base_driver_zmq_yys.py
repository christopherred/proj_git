import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from threading import Thread, Lock
import time
import csv
import os
import signal
import sys
import keyboard  # 需要安装: pip install keyboard
import threading
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from rclpy.qos import qos_profile_sensor_data

class Robot(Thread):
    def __init__(self, sim, clientID, name):
        super().__init__()
        self.sim = sim
        self.clientID = clientID
        self.lock = Lock()  # 用于线程安全
        self.robot = self.sim.getObject(f'/{name}')
        self.joint1 = self.sim.getObject(f'/{name}/joint1')
        self.joint2 = self.sim.getObject(f'/{name}/joint2')
        self.joint3 = self.sim.getObject(f'/{name}/joint3')
        print(f'Robot: {self.robot}, joint1: {self.joint1}, joint2: {self.joint2}, joint3: {self.joint3}')
        self.cmd_wheel_speed = [0.0, 0.0, 0.0]
        self.odom = None
        
        self.actual_wheel_speed = [1.0, 1.0, 1.0]
        self.actual_wheel_torque = [0.0, 0.0, 0.0]
        
        # 添加停止标志
        self.running = True

    def _get_odom(self):
        pose = self.sim.getObjectPose(self.robot)
        linearVelocity, angularVelocity = self.sim.getVelocity(self.robot)
        self.odom = (pose, linearVelocity, angularVelocity)

    def get_odom(self):
        return self.odom

    def _get_wheels_speed_and_torque(self):
        self.actual_wheel_speed = [
            self.sim.getJointVelocity(self.joint1),
            self.sim.getJointVelocity(self.joint2),
            self.sim.getJointVelocity(self.joint3)
        ]
        self.actual_wheel_torque = [
            self.sim.getJointForce(self.joint1),
            self.sim.getJointForce(self.joint2),
            self.sim.getJointForce(self.joint3)
        ]

    def get_wheels_speed_and_torque(self):
        return (self.actual_wheel_speed, self.actual_wheel_torque)

    def set_joints_vel(self, vel1, vel2, vel3):
        self.cmd_wheel_speed = [float(vel1), float(vel2), float(vel3)]

    def _set_joints_vel(self):
        self.sim.setJointTargetVelocity(self.joint1, self.cmd_wheel_speed[0])
        self.sim.setJointTargetVelocity(self.joint2, self.cmd_wheel_speed[1])
        self.sim.setJointTargetVelocity(self.joint3, self.cmd_wheel_speed[2])
    
    def stop(self):
        self.running = False

    def run(self):
        start_time = time.time()
        counter    = 0

        while self.running:
            counter += 1
            if time.time() - start_time >= 20:  # 运行满20秒
                print(f"VrepApi freq: {counter*3} Hz")
                counter = 0
                start_time = time.time()

            self._get_odom()
            self._get_wheels_speed_and_torque()
            self._set_joints_vel()


class ROS2Robot(Node):
    def __init__(self):
        super().__init__('ros2_robot')
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.clientID = self.sim.startSimulation()

        # 创建机器人实例
        self.robot = Robot(self.sim, self.clientID, 'robot1')
        self.robot.start()

        # 创建发布者
        self.pub_wheelspeed = self.create_publisher(Float32MultiArray, '/chassis/actual/wheelspeed', 10)
        self.pub_wheeltorque = self.create_publisher(Float32MultiArray, '/chassis/actual/wheeltorque', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        # 创建TF发布器
        self.tfbroadcaster = TransformBroadcaster(self)

        # 创建订阅者
        self.sub_wheelspeed_cmd = self.create_subscription(Float32MultiArray, '/chassis/command/wheelspeed', self.update_wheel_speed, qos_profile_sensor_data)

        # 创建两个定时器
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 添加轨迹记录
        self.trajectory = []
        self.trajectory_timestamps = []
        self.start_time = time.time()
        
        # 注册信号处理器，用于优雅地处理Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 添加运行标志和键盘监听线程
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True  # 设为守护线程，主线程结束时自动结束
        self.keyboard_thread.start()
        
        self.get_logger().info("ROS2Robot 初始化完成，将记录轨迹数据")
        self.get_logger().info("按'S'键保存轨迹数据，按'Q'键退出程序")

    def keyboard_listener(self):
        """监听键盘事件的线程"""
        while self.running:
            try:
                # 监听S键保存数据
                if keyboard.is_pressed('s'):
                    self.get_logger().info("检测到'S'键被按下，保存轨迹数据...")
                    self.export_trajectory()
                    # 等待按键释放，避免多次触发
                    while keyboard.is_pressed('s'):
                        time.sleep(0.1)
                
                # 监听Q键退出程序
                if keyboard.is_pressed('q'):
                    self.get_logger().info("检测到'Q'键被按下，正在保存数据并退出...")
                    self.export_trajectory()
                    self.cleanup()
                    self.running = False
                    # 使用os._exit直接退出，避免被ROS2拦截
                    import os
                    os._exit(0)
                    
                time.sleep(0.1)  # 减少CPU使用率
            except Exception as e:
                self.get_logger().error(f"键盘监听线程出错: {e}")
                time.sleep(1)  # 发生错误时暂停一下

    def signal_handler(self, sig, frame):
        self.get_logger().info("接收到终止信号，导出轨迹数据并关闭节点")
        self.export_trajectory()
        self.cleanup()
        sys.exit(0)

    def timer_callback(self):
        # 发布轮子数据
        self.pub_wheel_data()
        # 发布里程计和TF
        self.publish_odom_and_tf()
        # 记录轨迹
        self.record_trajectory()

    def pub_wheel_data(self):
        try:
            # 获取速度和转矩
            wheel_speeds, wheel_torques = self.robot.get_wheels_speed_and_torque()

            # 确保获取到有效数据
            if wheel_speeds is None or wheel_torques is None:
                self.get_logger().error("Failed to get wheel data")
                return

            # 发布速度消息
            wheel_speed_msg = Float32MultiArray()
            wheel_speed_msg.data = list(wheel_speeds)  # 转为列表，确保符合 ROS 消息类型要求
            self.pub_wheelspeed.publish(wheel_speed_msg)

            # 发布转矩消息
            wheel_torque_msg = Float32MultiArray()
            wheel_torque_msg.data = list(wheel_torques)  # 转为列表，确保符合 ROS 消息类型要求
            self.pub_wheeltorque.publish(wheel_torque_msg)

        except Exception as e:
            self.get_logger().error(f"Error in pub_wheel_data: {e}")

    def record_trajectory(self):
        """记录机器人轨迹数据"""
        with self.robot.lock:
            if self.robot.odom:
                pose, linear_velocity, angular_velocity = self.robot.odom
                current_time = time.time() - self.start_time
                
                # 记录位置和时间戳
                x, y = pose[0], pose[1]
                self.trajectory.append((x, y))
                self.trajectory_timestamps.append(current_time)
    
    def export_trajectory(self):
        """导出轨迹数据到CSV文件"""
        try:
            # 检查是否有轨迹数据
            if len(self.trajectory) == 0:
                self.get_logger().warn("轨迹数据为空，无法导出")
                return
                
            # 尝试不同的保存位置
            paths_to_try = [
                ("当前目录", os.getcwd()),
                ("桌面(中文)", os.path.join(os.path.expanduser('~'), '桌面')),
                ("桌面(英文)", os.path.join(os.path.expanduser('~'), 'Desktop')),
                ("用户目录", os.path.expanduser('~')),
                ("/tmp目录", "/tmp")  # Linux系统通常都有权限写入此目录
            ]
            
            # 创建文件名
            filename = f'robot_trajectory_{time.strftime("%Y%m%d_%H%M%S")}.csv'
            
            for path_name, path in paths_to_try:
                try:
                    # 检查目录是否存在且可写
                    if os.path.exists(path) and os.access(path, os.W_OK):
                        file_path = os.path.join(path, filename)
                        
                        self.get_logger().info(f"尝试保存到 {path_name}: {file_path}")
                        
                        with open(file_path, 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            # 写入标题
                            csv_writer.writerow(['timestamp', 'x', 'y'])
                            # 写入数据
                            for i, (x, y) in enumerate(self.trajectory):
                                csv_writer.writerow([self.trajectory_timestamps[i], x, y])
                                
                        self.get_logger().info(f"轨迹数据已成功导出到: {file_path}")
                        return  # 保存成功，退出函数
                except Exception as e:
                    self.get_logger().error(f"尝试保存到 {path_name} 时出错: {e}")
            
            # 如果所有尝试都失败
            self.get_logger().error("所有保存位置尝试均失败")
        except Exception as e:
            self.get_logger().error(f"导出轨迹数据时出错: {e}")

    def publish_odom_and_tf(self):
        with self.robot.lock:
            if self.robot.odom:
                pose, linear_velocity, angular_velocity = self.robot.odom
                current_time = self.get_clock().now().to_msg()

                # 发布Odometry
                odom_msg = Odometry()
                odom_msg.header.stamp = current_time
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"
                odom_msg.pose.pose.position.x = pose[0]
                odom_msg.pose.pose.position.y = pose[1]
                odom_msg.pose.pose.position.z = pose[2]
                odom_msg.pose.pose.orientation = Quaternion(x=pose[3], y=pose[4], z=pose[5], w=pose[6])
                odom_msg.twist.twist.linear.x = linear_velocity[0]
                odom_msg.twist.twist.linear.y = linear_velocity[1]
                odom_msg.twist.twist.angular.z = angular_velocity[2]
                self.pub_odom.publish(odom_msg)

                # 发布TF
                t = TransformStamped()
                t.header.stamp = current_time
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                t.transform.translation.x = pose[0]
                t.transform.translation.y = pose[1]
                t.transform.translation.z = pose[2]
                t.transform.rotation.x = pose[3]
                t.transform.rotation.y = pose[4]
                t.transform.rotation.z = pose[5]
                t.transform.rotation.w = pose[6]
                self.tfbroadcaster.sendTransform(t)

    def update_wheel_speed(self, msg: Float32MultiArray):
        with self.robot.lock:
            self.robot.cmd_wheel_speed = msg.data
    
    def cleanup(self):
        """清理资源并停止仿真"""
        if not hasattr(self, '_cleanup_called'):  # 防止多次调用
            self._cleanup_called = True
            self.get_logger().info("停止仿真并清理资源...")
            self.running = False  # 停止键盘监听线程
            self.robot.stop()
            self.sim.stopSimulation()
            self.robot.join(timeout=1.0)  # 确保线程安全退出
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    ros2robot = ROS2Robot()
    
    try:
        # 提示用户如何操作
        print("\n==========================================")
        print("按 'S' 键: 随时保存当前轨迹数据")
        print("按 'Q' 键: 保存数据并退出程序")
        print("Ctrl+C:   可能导致数据丢失，建议使用'Q'退出")
        print("==========================================\n")
        
        rclpy.spin(ros2robot)
    except KeyboardInterrupt:
        print("接收到键盘中断，尝试保存数据...")
        ros2robot.export_trajectory()  # 尝试保存，尽管可能被拦截
    finally:
        # 清理资源
        ros2robot.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()