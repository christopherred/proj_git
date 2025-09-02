#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from tf2_ros import Buffer, TransformListener


from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import time

class TfToPoseNode(Node):
    def __init__(self):
        super().__init__('tf_to_pose_node')

        # 设置帧ID和发布频率
        self.parent_frame = 'map'
        self.child_frame = 'base_link'
        self.publish_rate = 10.0  # Hz

        # 初始化TF缓冲区和监听器，设置较大的缓存时间
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 设置QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # 创建PoseStamped发布器
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/cartographer/pose',
            qos_profile
        )

        # 等待tf树建立
        self.get_logger().info(f"等待从 {self.parent_frame} 到 {self.child_frame} 的变换建立...")

        # 创建定时器，定期发布姿态
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_pose)

        self.get_logger().info(
            f'TF转换到姿态节点已启动，'
            f'将从 {self.parent_frame} 到 {self.child_frame} 的TF转换为姿态，'
            f'发布到话题: /cartographer/pose，频率: {self.publish_rate} Hz'
        )

    def publish_pose(self):
        """发布从TF获取的姿态信息"""
        try:
            # 使用当前时间的稍早时刻查询变换
            now = self.get_clock().now()
            # 延迟50ms，给TF系统时间发布最新的变换
            past_time = now - Duration(seconds=0.05)

            # 查询变换，添加超时
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )

            # 创建PoseStamped消息
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = now.to_msg()
            pose_msg.header.frame_id = self.parent_frame

            # 从变换填充姿态
            pose_msg.pose.pose.position.x = transform.transform.translation.x
            pose_msg.pose.pose.position.y = transform.transform.translation.y
            pose_msg.pose.pose.position.z = transform.transform.translation.z
            pose_msg.pose.pose.orientation = transform.transform.rotation

            # 设置协方差矩阵（默认为对角线为小值的矩阵，表示高置信度）
            # 协方差矩阵是6x6的，按顺序为 [x, y, z, rotation about X, rotation about Y, rotation about Z]
            covariance = [0.0] * 36  # 初始化为全零

            # 设置位置协方差（x, y）
            covariance[0] = 0.001  # x方差
            covariance[7] = 0.001  # y方差

            # 设置旋转协方差（围绕Z轴的旋转）
            covariance[35] = 0.001  # 旋转Z方差

            pose_msg.pose.covariance = covariance

            # 发布姿态
            self.pose_publisher.publish(pose_msg)


        except Exception as e:
            self.get_logger().warn(f"{str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TfToPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.timer.destroy()
        node.destroy_node()

if __name__ == '__main__':
    main()
