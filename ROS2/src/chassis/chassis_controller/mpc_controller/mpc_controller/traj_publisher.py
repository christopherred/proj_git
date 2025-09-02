#!/usr/bin/env python3

import numpy as np
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrajPublisher(Node):
    def __init__(self):
        super().__init__('vrep_simulation')

        # 创建轨迹发布者
        self.traj_pub = self.create_publisher(Path, '/designed_path', 10)

        # 定时器：20Hz 发布频率
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)
        #self.timer = self.create_timer(1.0 / 5.0, self.timer_callback)

        # 加载轨迹数据
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.x_ref = np.loadtxt(os.path.join(current_dir, 'square_x.csv'), delimiter=',')
        self.y_ref = np.loadtxt(os.path.join(current_dir, 'square_y.csv'), delimiter=',')
        self.theta_ref = np.loadtxt(os.path.join(current_dir, 'square_theta.csv'), delimiter=',')

        self.data_len = len(self.x_ref)
        self.get_logger().info(f"Loaded trajectory data of length: {self.data_len}")

        # 起始索引
        self.time_index = 0

    def euler_to_quaternion(self, yaw):
        """
        欧拉角到四元数的转换，仅支持绕 Z 轴旋转（2D 平面运动）。
        """
        half_yaw = yaw / 2.0
        q_x = 0.0
        q_y = 0.0
        q_z = np.sin(half_yaw)
        q_w = np.cos(half_yaw)
        return q_x, q_y, q_z, q_w

    def create_pose_stamped(self, x, y, yaw):
        """
        将 (x, y, yaw) 转换为 ROS2 的 PoseStamped 消息。
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"  # 设置坐标系
        pose_stamped.header.stamp = self.get_clock().now().to_msg()  # 时间戳

        # 设置位置
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0

        # 设置方向
        q_x, q_y, q_z, q_w = self.euler_to_quaternion(yaw)
        pose_stamped.pose.orientation.x = q_x
        pose_stamped.pose.orientation.y = q_y
        pose_stamped.pose.orientation.z = q_z
        pose_stamped.pose.orientation.w = q_w

        return pose_stamped

    def timer_callback(self):
        """
        定时器回调函数：每次发布一个滑动窗口内的轨迹段。
        """
        if self.time_index >= self.data_len - 1:  # 确保不会超出轨迹长度
            self.get_logger().info("Trajectory publishing complete.")
            rclpy.shutdown()
            return

        # 创建 Path 消息
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        # 从当前索引发布 40 个点（滑动窗口）
        for i in range(self.time_index, min(self.time_index + 40, self.data_len)):
            path.poses.append(self.create_pose_stamped(self.x_ref[i], self.y_ref[i], self.theta_ref[i]))

        # 发布轨迹
        self.traj_pub.publish(path)

        # 更新索引（每次滑动一个点）
        self.time_index += 1
        #self.get_logger().info(f"published : {self.time_index}")


def main(args=None):
    rclpy.init(args=args)
    traj_pub = TrajPublisher()
    rclpy.spin(traj_pub)
    traj_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
