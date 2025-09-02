import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
#from zmqRemoteApi import RemoteAPIClient
from threading import Thread, Lock
import time
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
        # self.sim.setJointTargetVelocity(self.joint1, 1)
        # self.sim.setJointTargetVelocity(self.joint2, 0)
        # self.sim.setJointTargetVelocity(self.joint3, -1)

    def run(self):
        start_time = time.time()
        counter    = 0

        while True:
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

        # 创建两个定时器xx
        self.timer = self.create_timer(0.05, self.timer_callback)
        

    def timer_callback(self):
        # 发布轮子数据
        self.pub_wheel_data()
        # 发布里程计和TF
        self.publish_odom_and_tf()

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

            # 日志记录
            #self.get_logger().info(f"Published wheel speeds: {wheel_speeds}")
            #self.get_logger().info(f"Published wheel torques: {wheel_torques}")

        except Exception as e:
            self.get_logger().error(f"Error in pub_wheel_data: {e}")



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


def main(args=None):
    rclpy.init(args=args)
    ros2robot = ROS2Robot()
    rclpy.spin(ros2robot)
    ros2robot.robot.join()  # 确保线程安全退出
    ros2robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()