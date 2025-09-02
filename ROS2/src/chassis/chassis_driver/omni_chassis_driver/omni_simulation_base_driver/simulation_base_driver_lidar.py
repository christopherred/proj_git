#!/usr/bin/env python
#coding=UTF-8


'''底盘控制测试版 其中激光雷达部分代码未完善，rviz中不能正确显示周围环境，同时对于控制方式的同步与异步也有待改进'''


import time
import sched
import math
import random
import numpy as np

import sim
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node

import tf2_ros
from sensor_msgs.msg import JointState, PointCloud2, PointField, NavSatFix, Imu, LaserScan
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2

from geometry_msgs.msg import Quaternion, Twist, Pose, TransformStamped
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from threading import Lock
from std_msgs.msg import Float32MultiArray, Float64, Header  # 修改话题类型为 Float32MultiArray

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from threading import Lock
lock = Lock()


"""
注：

1. Vrep中的欧拉角指的是Tait-Bryan角，即 R = Rx(alpha) · Ry(beta) · Rz(gamma)，
分别表示绝对参考系绕 Z、Y 和 X 轴（按顺序）的基本旋转，单位为弧度，
参考：https://manual.coppeliarobotics.com/index.html中【On positions and orientations】


"""
def euler_to_quaternion(alpha, beta, gamma):
    cy = math.cos(gamma * 0.5)
    sy = math.sin(gamma * 0.5)
    cp = math.cos(beta * 0.5)
    sp = math.sin(beta * 0.5)
    cr = math.cos(alpha * 0.5)
    sr = math.sin(alpha * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]

class Robot():    
    def __init__(self, name, clientID):
        # 获取机器人及其关节的句柄
        self.sim             = sim
        self.clientID        = clientID
        self.robot           = sim.simxGetObjectHandle(clientID, '/robot1',            sim.simx_opmode_blocking)[1]
        self.joint1          = sim.simxGetObjectHandle(clientID, "/robot1/joint1",     sim.simx_opmode_blocking)[1]
        self.joint2          = sim.simxGetObjectHandle(clientID, "/robot1/joint2",     sim.simx_opmode_blocking)[1]
        self.joint3          = sim.simxGetObjectHandle(clientID, "/robot1/joint3",     sim.simx_opmode_blocking)[1]
        self.point_clouds    = sim.simxGetObjectHandle(clientID, "/robot1/SickTIM310", sim.simx_opmode_blocking)[1]        
        # 打印获取的关节句柄，确认初始化成功
        print(f'Robot: {self.robot}, joint1: {self.joint1}, joint2: {self.joint2}, joint3: {self.joint3}, SickTIM310: {self.point_clouds}')
    # 初始化机器人对象
    
class ROSRobot(Robot, Node):    
    def __init__(self, name, clientID):
        # 初始化 Node 父类
        Node.__init__(self, name)

        # 初始化 Robot 父类
        Robot.__init__(self, name, clientID)
        
        # 创建发布者
        self.pub_wheelspeed  = self.create_publisher(Float32MultiArray, '/chassis/actual/wheelspeed', 10)
        self.pub_wheeltorque = self.create_publisher(Float32MultiArray, '/chassis/actual/wheeltorque', 10)
        self.puber_laser     = self.create_publisher(LaserScan, 'robot1/laser/scan', 10)
        self.puber_odom            = self.create_publisher(Odometry, 'robot1/filtered/odom', 10)
        self.tfbroadcaster         = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster    = tf2_ros.StaticTransformBroadcaster(self)
        # Publish static transforms from ground to map1 and from map1 to odom1
        self.publish_static_transform("ground", "map1",  [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        self.publish_static_transform("map1",   "odom1", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        self.publish_static_transform("robot1_base_link_filter","robot1_laser_link", [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0])
        # 创建订阅者
        self.sub_wheelspeed_cmd = self.create_subscription(Float32MultiArray, '/chassis/command/wheelspeed', self.update_wheel_speed, 10)

        # 定时器，每隔0.1秒执行一次 
        timer_period = 1/20  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
        # 将时钟发布器添加到 ROS 2 节点
        # self.clock_pub = self.create_publisher(Float64, '/clock', 10)
        # self.sim_time = 0  # 初始化模拟时间变量

        # 确保仿真的同步模式
        sim.simxSynchronous(self.clientID, True)  # 启用同步模式
        sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot)  # 开始仿真

        # 获取仿真的时间步长（以秒为单位）
        _, self.time_step = sim.simxGetFloatingParameter(self.clientID, sim.sim_floatparam_simulation_time_step, sim.simx_opmode_blocking)
    

    def get_odom_and_tf(self):
        current_time = self.get_clock().now().to_msg()
        position= self.get_position()
        orientation= self.get_orientation()

        # Create an Odometry message
        msg = Odometry()
        msg.header.stamp = current_time
        msg.header.frame_id = "odom1"
        msg.child_frame_id = "robot1_base_link_filter"

        msg.pose.pose.position.x  = position[0]
        msg.pose.pose.position.y  = position[1]
        msg.pose.pose.position.z  = position[2]-position[2]
        msg.pose.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        linear_velocity, angular_velocity = self.get_speed()
        msg.twist.twist.linear.x  = linear_velocity[0]
        msg.twist.twist.linear.y  = linear_velocity[1]
        msg.twist.twist.linear.z  = linear_velocity[2]
        msg.twist.twist.angular.x = angular_velocity[0]
        msg.twist.twist.angular.y  = angular_velocity[1]
        msg.twist.twist.angular.z  = angular_velocity[2]

        # Publish the Odometry message
        self.puber_odom.publish(msg)
        # Publish dynamic TF
        t = TransformStamped()
        t.header.stamp            = current_time
        t.header.frame_id         = "odom1"
        t.child_frame_id          = "robot1_base_link_filter"
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]-position[2]
        t.transform.rotation.x    = orientation[0]
        t.transform.rotation.y    = orientation[1]
        t.transform.rotation.z    = orientation[2]
        t.transform.rotation.w    = orientation[3]
        self.tfbroadcaster.sendTransform(t)

    def publish_static_transform(self, parent_frame, child_frame, translation, rotation):
        t = TransformStamped()
        t.header.stamp            = self.get_clock().now().to_msg()
        t.header.frame_id         = parent_frame
        t.child_frame_id          = child_frame
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x    = float(rotation[0])
        t.transform.rotation.y    = float(rotation[1])
        t.transform.rotation.z    = float(rotation[2])
        t.transform.rotation.w    = float(rotation[3])

        self.static_broadcaster.sendTransform(t)
    
    def get_position(self):
        # 获取机器人的位置，返回的是一个三维坐标 (x, y, z)
        return_code,position = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_blocking)
        return position  # 确保位置是浮点数列表
    
    def get_orientation(self):
        # 获取机器人的姿态，
        return_code,orientation = sim.simxGetObjectOrientation(self.clientID, self.robot, -1, sim.simx_opmode_blocking)
        if return_code != sim.simx_return_ok:
            raise ValueError("Failed to get orientation from simulator.")
        q = euler_to_quaternion(orientation[0], orientation[1], orientation[2])
    # 确保返回的旋转是浮动类型的四元数
        return q
    
    def get_laser(self):

        #with lock:
        result=[]
        returnCode, point_cloud = sim.simxGetStringSignal(self.clientID, "laser_signal", sim.simx_opmode_blocking)
        returnCode, data = sim.simxGetStringSignal(self.clientID, "laser_signal2", sim.simx_opmode_blocking)
        # 将 bytearray 转换为字符串
        data_str = data.decode('utf-8')
        # 将字符串分割为列表，并过滤掉空字符串
        data_list = data_str.split(',') 
        formatted_list=[]
        for num in data_list:
        # 移除前后空白字符并忽略空字符串
            cleaned_num = num.strip()
            if cleaned_num:
                try:
                    formatted_list.append(float(cleaned_num))
                except ValueError:
            # 如果依然出现转换错误，作进一步处理，比如记录日志
                    print(f"无法转换为浮点数: {cleaned_num}")
        #print(formatted_list)

        # 假设 point_cloud 是一个 bytearray，表示一个逗号分隔的浮动数值字符串
        point_cloud_str = point_cloud.decode('utf-8')  # 将 bytearray 转换为字符串
        coordinates = point_cloud_str.split(',')  # 按逗号分割字符串

        # 创建一个新的列表，用来存储解码后的点
        points = []
        for i in range(0, len(coordinates) -2, 3):
            # 将每三个元素 (x, y, z) 转换为浮动数值
            x = float(coordinates[i])
            y = float(coordinates[i+1])
    
            points.append((x, y))  # 存储 (x, y, z) 坐标

        # print(points)

    # 现在 points 是一个包含 (x, y, z) 元组的列表
        current_time = self.get_clock().now().to_msg()
        position= self.get_position()

        if points:
                # 创建 LaserScan 消息
            laser_scan_msg = LaserScan()
            laser_scan_msg.header.stamp = current_time
            laser_scan_msg.header.frame_id = "robot1_laser_link"
        
            # 计算激光雷达的距离和角度
            distances = []
            angles = []
            # 生成假设的距离数据，通常你会从雷达传感器获取这些数据
            # 假设点云数据按激光的角度顺序排列
            for i, point in enumerate(points):
                # 计算从原点到该点的距离
                distance = math.sqrt((point[0])**2 + (point[1])**2)
                distances.append(distance)
            #print(distances)
                # 计算该点的角度
            #    angle = math.atan2(point[1], point[0])  # 计算该点的角度（rad）
            #    angles.append(angle)

            # 设置LaserScan消息的各个字段
            laser_scan_msg.angle_min =  -135 * math.pi / 180  # 假设第一个点是最小角度
            laser_scan_msg.angle_max = 135 * math.pi / 180  # 假设最后一个点是最大角度
            laser_scan_msg.angle_increment =  (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / (512 - 1)
            laser_scan_msg.time_increment = 1/20/512  # 如果不需要，保持为 0
            laser_scan_msg.scan_time = 0.0  # 需要的话，可以设置扫描周期
            laser_scan_msg.range_min = 0.05  # 激光雷达的最小有效距离
            laser_scan_msg.range_max = 8.0  # 激光雷达的最大有效距离
            laser_scan_msg.ranges = distances # 距离数据
            #laser_scan_msg.ranges = [0.393093, 0.393844, 0.394612, 0.395396, 0.396197]  #测试数据
            #laser_scan_msg.ranges = formatted_list
        
            #laser_scan_msg.intensities = [1.0] * len(distances)  # 激光强度（默认值为1）
            laser_scan_msg.intensities =[]

            # 发布 LaserScan 消息
            self.puber_laser.publish(laser_scan_msg)
            #print(f"Laser scan ranges: {laser_scan_msg.ranges}")
    

    def transform_points(self, points, position):
        transformed_points = []
        for point in points:
            #transformed_points.append((point[0] + position[0], point[1] + position[1], point[2] + position[2]))
            transformed_points.append((point[0] , point[1], point[2]))
        return transformed_points
    
    def pub_wheel_data(self):

        # 获取各轮子的速度
        result, joint1_vel = sim.simxGetObjectFloatParameter(clientID, self.joint1, 2012, sim.simx_opmode_blocking)
        result, joint2_vel = sim.simxGetObjectFloatParameter(clientID, self.joint2, 2012, sim.simx_opmode_blocking)
        result, joint3_vel = sim.simxGetObjectFloatParameter(clientID, self.joint3, 2012, sim.simx_opmode_blocking)

        # 获取转矩
        result, torque1 = sim.simxGetJointForce(clientID, self.joint1, sim.simx_opmode_blocking)
        result, torque2 = sim.simxGetJointForce(clientID, self.joint2, sim.simx_opmode_blocking)
        result, torque3 = sim.simxGetJointForce(clientID, self.joint3, sim.simx_opmode_blocking)
        
        # 发布轮子速度
        wheel_speed_msg = Float32MultiArray()
        wheel_speed_msg.data = [joint1_vel, joint2_vel, joint3_vel]  # 各轮子的速度
        self.pub_wheelspeed.publish(wheel_speed_msg)

        # 发布轮子扭矩
        wheel_torque_msg = Float32MultiArray()
        wheel_torque_msg.data = [torque1, torque2, torque3]  # 各轮子的扭矩
        self.pub_wheeltorque.publish(wheel_torque_msg)


    def set_speed(self, msg: Twist):
        # 解析Twist消息并使用逆运动学计算每个关节的速度
        self.vel_x = msg.linear.x
        self.vel_y = msg.linear.y
        self.ang_vel = msg.angular.z

    def update_wheel_speed(self, msg: Float32MultiArray):
        # 根据接收到的每个轮子的目标速度指令来分别设置关节的速度
        joint1_cmd_vel, joint2_cmd_vel, joint3_cmd_vel = msg.data

        # 更新每个轮子的目标速度
        sim.simxSetJointTargetVelocity(self.clientID, self.joint1, joint1_cmd_vel, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.joint2, joint2_cmd_vel, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.joint3, joint3_cmd_vel, sim.simx_opmode_oneshot)    

    def timer_callback(self):
        self.pub_wheel_data()  # 定时发布轮子数据
        self.get_laser()
        self.get_odom_and_tf()
        # 更新仿真时间
        # self.sim_time += self.time_step  # 按步长递增模拟时间
        # clock_msg = Float64()
        # clock_msg.data = self.sim_time  # 发布当前模拟时间
        # self.clock_pub.publish(clock_msg)

        # 触发同步仿真步骤
        sim.simxSynchronousTrigger(self.clientID)
    
    def get_speed(self):
        with lock:
            _,linear_velocity, angular_velocity = self.sim.simxGetObjectVelocity(self.clientID, self.robot, sim.simx_opmode_blocking )
        return (linear_velocity, angular_velocity)
        
if __name__ == "__main__":
    rclpy.init()  # ROS系统初始化
    sim.simxFinish(-1)  # 关闭所有连接
    clientID = sim.simxStart('127.0.0.1', -3, True, True, 5000, 5)  # 启动仿真连接
    synchronous = False    
    newrobot = ROSRobot('robot1',clientID)  # 创建RosRobot类实例
    rclpy.spin(newrobot)  # ROS节点进入循环
    rclpy.shutdown()  # 在退出程序时清理ROS
    
    