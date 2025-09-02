#!/usr/bin/env python3
# coding=UTF-8
import time
import sched
import math
import random
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import tf2_ros
from sensor_msgs.msg import JointState, PointCloud2, PointField, NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import sensor_msgs_py.point_cloud2 as pc2

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 包含雷达的参考代码 


from threading import Lock
lock   = Lock()

# SIM_IP = '192.168.1.231'
SIM_IP = 'localhost'

class VrepOmniRobot:
    def __init__(self, sim) -> None:
        self.sim = sim
        self.robot = sim.getObject('/robot1')
        self.joint1 = sim.getObject("/robot1/joint1")
        self.joint2 = sim.getObject("/robot1/joint2")
        self.joint3 = sim.getObject("/robot1/joint3")
        self.point_clouds = sim.getObject("/robot1/VPL16_2D/ptCloud")

        self.ref = sim.getObject('/robot1/reference')
        self.mass_object = sim.getObject('/robot1/mass')
        self.sensor = sim.getObject('/robot1/forceSensor')
        self.mass = sim.getObjectFloatParam(self.mass_object, sim.shapefloatparam_mass)
        self.old_transformation_matrix = sim.getObjectMatrix(self.ref)
        self.last_time = sim.getSimulationTime()
        self.refloc = [39.9636188935, 116.3050734262, 50.35]  # Reference location (latitude, longitude, altitude)

    def set_joint(self, v1, v2, v3):
        with lock:
            self.sim.setJointTargetVelocity(self.joint1, v1)
            self.sim.setJointTargetVelocity(self.joint2, v2)
            self.sim.setJointTargetVelocity(self.joint3, v3)

    def get_pose(self):
        with lock:
            pose = self.sim.getObjectPose(self.robot)
            position = [float(p) for p in pose[0:3]]  # Ensure position is a list of floats
            quaternion = pose[3:]
        return (position, quaternion)

    def get_speed(self):
        with lock:
            linear_velocity, angular_velocity = self.sim.getVelocity(self.robot)
        return (linear_velocity, angular_velocity)

    def get_laser(self):
        with lock:
            point_cloud = self.sim.getPointCloudPoints(self.point_clouds)
        # Assume point_cloud is a flat list of floats, with every three floats representing a point (x, y, z)
        points = []
        for i in range(0, len(point_cloud), 3):
            points.append((point_cloud[i], point_cloud[i+1], point_cloud[i+2]))
        return points


    def get_gps(self):
        position = self.sim.getObjectPosition(self.ref, -1)

        # Add noise to make it more realistic
        position[0] += 2 * (random.random() - 0.5) * 0.005
        position[1] += 2 * (random.random() - 0.5) * 0.005
        position[2] += 2 * (random.random() - 0.5) * 0.005

        enu_x = position[0]
        enu_y = position[1]
        enu_z = -position[2]

        # Convert to GPS coordinates
        latitude, longitude = self.convert_to_gps(enu_x, enu_y)
        altitude = self.refloc[2] + enu_z

        return latitude, longitude, altitude

    def convert_to_gps(self, dx, dy):
        """Convert relative position to GPS coordinates based on the reference location."""
        earth_radius = 6378137.0  # Radius of Earth in meters

        # Calculate latitude
        dlat = dx / earth_radius  # Note that dx is used for latitude in NED
        latitude = self.refloc[0] + (dlat * 180.0 / math.pi)

        # Calculate longitude
        dlon = dy / (earth_radius * math.cos(math.pi * self.refloc[0] / 180.0))  # Note that dy is used for longitude in NED
        longitude = self.refloc[1] + (dlon * 180.0 / math.pi)

        return latitude, longitude

    def get_gyro(self):
        transformation_matrix = self.sim.getObjectMatrix(self.ref)
        old_inverse = self.sim.copyTable(self.old_transformation_matrix)
        old_inverse = self.sim.getMatrixInverse(old_inverse)
        m = self.sim.multiplyMatrices(old_inverse, transformation_matrix)
        euler = self.sim.getEulerAnglesFromMatrix(m)
        current_time = self.sim.getSimulationTime()
        gyro_data = [0.0, 0.0, 0.0]
        dt = current_time - self.last_time
        if dt != 0:
            gyro_data = [euler[i] / dt for i in range(3)]

        self.old_transformation_matrix = self.sim.copyTable(transformation_matrix)
        self.last_time = current_time

        return gyro_data

    def get_accel(self):
        result, force, _ = self.sim.readForceSensor(self.sensor)
        if result > 0:
            accel = [force[0] / self.mass, force[1] / self.mass, force[2] / self.mass]
        else:
            accel = [0.0, 0.0, 0.0]
        return accel

class VrepOmniRosRobot(Node):
    def __init__(self, index, sim):
        super().__init__(f"robot{index}_vrep_sim")
        self.get_logger().info(f"robot{index}_vrep_sim node online")
        self.index                 = index
        self.sim                   = sim
        self.vrobot                = VrepOmniRobot(sim)
        self.lister_motors         = self.create_subscription(JointState, f"robot{self.index}/motor/cmds", self.__set_vrobot_joints, qos_profile_system_default)
        self.tfbroadcaster         = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster    = tf2_ros.StaticTransformBroadcaster(self)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.puber_odom            = self.create_publisher(Odometry, f"robot{self.index}/filtered/odom", qos_profile)
        self.puber_laser           = self.create_publisher(PointCloud2, f"robot{self.index}/laser/scan", qos_profile)
        self.puber_gps             = self.create_publisher(NavSatFix, f"robot{self.index}/sensors/gps/fix", qos_profile)
        self.puber_imu             = self.create_publisher(Imu, f"robot{self.index}/sensors/imu/data", qos_profile)
        # self.timer_get_odom_and_tf = self.create_timer(0.05, self.get_odom_and_tf)
        # self.timer_get_laser       = self.create_timer(0.05, self.get_laser)
        # self.timer_get_gps         = self.create_timer(0.05, self.get_gps)
        # self.timer_get_imu         = self.create_timer(0.05, self.get_imu)

        # initial position
        self.initial_position, self.initial_orientation = self.vrobot.get_pose()

        # Publish static transforms from ground to map1 and from map1 to odom1
        self.publish_static_transform("ground", "map1",  [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        self.publish_static_transform("map1",   "odom1", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

    def __set_vrobot_joints(self, joint_velocity: JointState):
        self.vrobot.set_joint(-joint_velocity.velocity[0], -joint_velocity.velocity[1], -joint_velocity.velocity[2])

    def get_odom_and_tf(self):
        current_time = self.get_clock().now().to_msg()
        # Create an Odometry message
        msg = Odometry()
        msg.header.stamp = current_time
        msg.header.frame_id = "odom1"
        msg.child_frame_id = f"robot{self.index}_base_link_filter"

        position, orientation     = self.vrobot.get_pose()
        msg.pose.pose.position.x  = position[0]
        msg.pose.pose.position.y  = position[1]
        msg.pose.pose.position.z  = position[2]-position[2]
        msg.pose.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        linear_velocity, angular_velocity = self.vrobot.get_speed()
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
        t.child_frame_id          = f"robot{self.index}_base_link_filter"
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]-position[2]
        t.transform.rotation.x    = orientation[0]
        t.transform.rotation.y    = orientation[1]
        t.transform.rotation.z    = orientation[2]
        t.transform.rotation.w    = orientation[3]
        self.tfbroadcaster.sendTransform(t)

    def get_laser(self):
        current_time = self.get_clock().now().to_msg()
        point_cloud = self.vrobot.get_laser()
        position, _ = self.vrobot.get_pose()

        if point_cloud:
            header = self.create_header(current_time, f"robot{self.index}_base_link_filter")
            transformed_points = self.transform_points(point_cloud, position)
            point_cloud_msg = self.create_point_cloud_msg(header, transformed_points)
            self.puber_laser.publish(point_cloud_msg)

    def create_header(self, stamp, frame_id):
        header = PointCloud2().header
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    def create_point_cloud_msg(self, header, points):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        cloud_data = []
        for point in points:
            cloud_data.append([point[0], point[1], point[2]-point[2]])

        cloud_data = np.array(cloud_data, dtype=np.float32)
        point_cloud_msg = pc2.create_cloud(header, fields, cloud_data)

        return point_cloud_msg

    def transform_points(self, points, position):
        transformed_points = []
        for point in points:
            #transformed_points.append((point[0] + position[0], point[1] + position[1], point[2] + position[2]))
            transformed_points.append((point[0] , point[1], point[2]))
        return transformed_points

    def get_gps(self):
        current_time = self.get_clock().now().to_msg()
        latitude, longitude, altitude = self.vrobot.get_gps()

        gps_msg = NavSatFix()
        gps_msg.header.stamp = current_time
        gps_msg.header.frame_id = 'ground'
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = altitude

        self.puber_gps.publish(gps_msg)

    def get_imu(self):
        current_time = self.get_clock().now().to_msg()
        accel_data = self.vrobot.get_accel()
        gyro_data = self.vrobot.get_gyro()
        _, orientation = self.vrobot.get_pose()

        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = f"robot{self.index}_base_link_filter"

        imu_msg.linear_acceleration.x = accel_data[0]
        imu_msg.linear_acceleration.y = accel_data[1]
        imu_msg.linear_acceleration.z = accel_data[2]

        imu_msg.angular_velocity.x = gyro_data[0]
        imu_msg.angular_velocity.y = gyro_data[1]
        imu_msg.angular_velocity.z = gyro_data[2]

        imu_msg.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        self.puber_imu.publish(imu_msg)


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

    def terminate(self):
        # self.timer_get_odom_and_tf.cancel()
        # self.timer_get_laser.cancel()
        # self.timer_get_gps.cancel()
        # self.timer_get_imu.cancel()
        pass

class VrepOmniRobotDriver:
    def __init__(self):
        self.sim            = RemoteAPIClient(host=SIM_IP).require('sim')
        self.sim.setStepping(True)
        self.sim.startSimulation()
        self.robot1_node    = VrepOmniRosRobot(1, self.sim)
        self.scheduler      = sched.scheduler(time.time, time.sleep)

    def task(self):
        for _ in range(5):
            self.sim.step()
            self.robot1_node.get_odom_and_tf()
            self.robot1_node.get_imu()
            self.robot1_node.get_gps()
        self.robot1_node.get_laser()


    def run(self):
        try:
            while True:
                self.task()
        except KeyboardInterrupt:
            pass
        finally:
            self.robot1_node.terminate()
            if rclpy.ok():
                rclpy.shutdown()
            with lock:
                self.sim = RemoteAPIClient(host=SIM_IP).require('sim')
                self.sim.stopSimulation()



def main():
    rclpy.init()
    driver = VrepOmniRobotDriver()
    driver.run()

if __name__ == "__main__":
    main()
