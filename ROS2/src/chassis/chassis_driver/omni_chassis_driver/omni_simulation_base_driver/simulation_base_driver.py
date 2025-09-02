#!/usr/bin/env python
#coding=UTF-8


'''在不添加激光雷达下的底盘控制'''


import time
import sched
import math
import random
import numpy as np


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
from std_msgs.msg import Float32MultiArray, Float64, Header

import sys
sys.path.append('/opt/vrepRemoteApi')
import sim

lock = Lock()

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
        # 打印获取的关节句柄，确认初始化成功
        print(f'Robot: {self.robot}, joint1: {self.joint1}, joint2: {self.joint2}, joint3: {self.joint3}')
    
class ROSRobot(Robot, Node):    
    def __init__(self, name, clientID):
        # 初始化 Node 父类
        Node.__init__(self, name)

        # 初始化 Robot 父类
        Robot.__init__(self, name, clientID)
        
        # 创建发布者
        self.pub_wheelspeed  = self.create_publisher(Float32MultiArray, '/chassis/actual/wheelspeed', 10)
        self.pub_wheeltorque = self.create_publisher(Float32MultiArray, '/chassis/actual/wheeltorque', 10)
        self.puber_odom            = self.create_publisher(Odometry, 'odom', 10)
        self.tfbroadcaster         = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster    = tf2_ros.StaticTransformBroadcaster(self)
        # Publish static transforms from ground to map1 and from map1 to odom1
        # self.publish_static_transform("ground", "map1",  [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        # self.publish_static_transform("map1",   "odom1", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
       
        # 创建订阅者
        self.sub_wheelspeed_cmd = self.create_subscription(Float32MultiArray, '/chassis/command/wheelspeed', self.update_wheel_speed, 10)

        # 定时器，每隔0.1秒执行一次 
        timer_period = 1/20  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
        # 取消同步模式，使用异步模式
        # sim.simxSynchronous(self.clientID, False)  # 禁用同步模式
        sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot)  # 开始仿真

        # 获取仿真的时间步长（以秒为单位）
        # _, self.time_step = sim.simxGetFloatingParameter(self.clientID, sim.sim_floatparam_simulation_time_step, sim.simx_opmode_blocking)

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
    
    def transform_points(self, points, position):
        transformed_points = []
        for point in points:
            #transformed_points.append((point[0] + position[0], point[1] + position[1], point[2] + position[2]))
            transformed_points.append((point[0] , point[1], point[2]))
        return transformed_points

    def pub_wheel_data(self):
        # 获取各轮子的速度
        result, joint1_vel = sim.simxGetObjectFloatParameter(self.clientID, self.joint1, 2012, sim.simx_opmode_blocking)
        result, joint2_vel = sim.simxGetObjectFloatParameter(self.clientID, self.joint2, 2012, sim.simx_opmode_blocking)
        result, joint3_vel = sim.simxGetObjectFloatParameter(self.clientID, self.joint3, 2012, sim.simx_opmode_blocking)

        # 获取转矩
        result, torque1 = sim.simxGetJointForce(self.clientID, self.joint1, sim.simx_opmode_blocking)
        result, torque2 = sim.simxGetJointForce(self.clientID, self.joint2, sim.simx_opmode_blocking)
        result, torque3 = sim.simxGetJointForce(self.clientID, self.joint3, sim.simx_opmode_blocking)
        
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
    
    def get_speed(self):
        with lock:
            _,linear_velocity, angular_velocity = self.sim.simxGetObjectVelocity(self.clientID, self.robot, sim.simx_opmode_blocking )
        return (linear_velocity, angular_velocity)

    def update_wheel_speed(self, msg: Float32MultiArray):
        # 根据接收到的每个轮子的目标速度指令来分别设置关节的速度
        joint1_cmd_vel, joint2_cmd_vel, joint3_cmd_vel = msg.data

        # 更新每个轮子的目标速度
        sim.simxSetJointTargetVelocity(self.clientID, self.joint1, joint1_cmd_vel, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.joint2, joint2_cmd_vel, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.joint3, joint3_cmd_vel, sim.simx_opmode_oneshot)    

    def timer_callback(self):
        # 发布轮子数据
        # self.get_odom_and_tf()
        self.pub_wheel_data()
        

def main():
    rclpy.init()  # ROS系统初始化
    sim.simxFinish(-1)  # 关闭所有连接
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # 启动仿真连接
    
    if clientID != -1:
        newrobot = ROSRobot('robot1', clientID)  # 创建RosRobot类实例
        
        try:
            rclpy.spin(newrobot)  # ROS节点进入循环
        finally:
            # 确保正确关闭仿真和ROS
            sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
            sim.simxFinish(clientID)
            newrobot.destroy_node()
            rclpy.shutdown()  # 在退出程序时清理ROS
    else:
        print("Failed to connect to CoppeliaSim")

if __name__ == "__main__":
    main()