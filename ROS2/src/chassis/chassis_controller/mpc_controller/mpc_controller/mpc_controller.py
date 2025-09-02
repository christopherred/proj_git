#!/usr/bin/env python3

import numpy as np
import pandas as pd
import math
import time
import datetime
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, PoseArray, Pose
sys.path.append('/home/robot_code/ros2_ws/src/chassis_controller/mpc_controller/mpc_controller')
from mpc import MPC

def quart_to_rpy(x, y, z, w):
    """ 四元数转rpy """
    r = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    p = math.asin(2*(w*y - z*x))
    yy = math.atan2(2*(w*z + x*y), 1 - 2*(z*z + y*y))
    return r, p, yy


def find_min_distance(curr_state, designed_path):
    """ 找到和当前位置最接近的点 """
    designed_path = np.array(designed_path)
    curr_state    = np.array(curr_state)
    # 只取前两维计算每个点与curr_state的欧几里得距离
    distances     = np.linalg.norm(designed_path[:, :2] - curr_state[:2], axis=1)
    # 找到最小距离的索引
    closest_point_index = np.argmin(distances)
    return closest_point_index


def adjust_yaw(yaw, car_yaw):
    """ 调整 yaw 使得其位于 [car_yaw - pi, car_yaw + pi] 之间 """
    delta_yaw = yaw - car_yaw
    delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi
    adjusted_yaw = delta_yaw + car_yaw
    return adjusted_yaw


class MPCController:
    def __init__(self, replan_period, control_period, step_len) -> None:
        self.replan_period  = replan_period
        self.control_period = control_period
        self.step_len       = step_len
        self.curr_state     = [0, 0, 0]
        self.designed_path  = [[0, 0, 0]] * step_len
        self.u_res          = [[0, 0, 0]] * step_len
        self.have_dpath     = False
        self.have_odom      = True
        self.xdata = np.zeros(1)
        self.ydata = np.zeros(1)
        self.speedx = np.zeros(1)
        self.speedy = np.zeros(1)
        self.speedz = np.zeros(1)
        self.times = 0
        self.robotState = 0
        self.saved = 0

        self._init_ros(replan_period, control_period)
        self._init_mpc()

    def _init_mpc(self):
        """ 构建 MPC 问题 """
        self.mpc = MPC()
        self.mpc.set_type(self.mpc.DIFF_CAR)
        self.mpc.set_time(
            T        = self.control_period,
            step_len = self.step_len
        )
        self.mpc.set_QR(
            Q = np.array([[-50000.0, 0.0, 0.0], [0.0, -50000.0, 0.0], [0.0, 0.0, 1.0]]),
            R = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        )
        self.mpc.set_max_speed(
            vx_min    = -0.082,
            vx_max    = 0.082,
            vy_min    = -0.082,
            vy_max    = 0.082,
            omega_min = -0.1,
            omega_max = 0.1
        )

    def _ros_log_info(self, data):
        self.node.get_logger().info(data)

    def _run_mpc(self):
        """求解 MPC 问题"""
        try:
            if not self.have_dpath:
                self._ros_log_info('Waiting for new designed path.')
                return
            if not self.have_odom:
                self._ros_log_info('Waiting for new odom.')
                return
        
            # 求解 MPC
            start_time = datetime.datetime.now()
            self.mpc.set_opti()
            self.mpc.set_init(self.curr_state)
            self.mpc.set_goal(self.designed_path)
            _, u_res = self.mpc.run()
            end_time = datetime.datetime.now()
        
            #self._ros_log_info(f'MPC solved in {(end_time - start_time).total_seconds()} sec')
        
            # 更新控制指令
            self.u_res = u_res.T.tolist()
            self.have_dpath = False
            self.have_odom = False
    
        except Exception as e:
            self._ros_log_info(f'MPC encountered an error: {e}')
            # 清空控制指令，确保停止
            self.u_res = [[0.0, 0.0, 0.0]] * self.step_len
            self.robotState = 1
            self.u_res = [[0.0, 0.0, 0.0]] * self.step_len

    def _init_ros(self, replan_period, control_period):
        """ 初始化 ROS 节点 """
        self.node = rclpy.create_node('mpc_controller')
        

        self._timer_mpc_run = self.node.create_timer(replan_period, self._run_mpc)
        self._timer_pub_cmd = self.node.create_timer(control_period, self._pub_cmd)

        #self._suber_odom = self.node.create_subscription(Odometry, '/odom', self._sub_odom, 10)
        self._suber_odom = self.node.create_subscription(Odometry, '/cartographer/filtered_odom', self._sub_odom, 10)
        self._suber_path = self.node.create_subscription(Path, '/designed_path', self._sub_path, 10)
        #self._puber_cmd = self.node.create_publisher(Twist, '/chassis/command/cmd_vel', 10)
        self._puber_cmd = self.node.create_publisher(Twist, '/controller/auto/cmd_vel', 10)
        self._puber_state = self.node.create_publisher(Int32, '/robot_state', 10)

    def _pub_cmd(self):
        """ 发送 cmd_vel """
        #self.node.get_logger().info(f"当前控制队列长度: {len(self.u_res)}")
        state_msg = Int32()
        state_msg.data = self.robotState
        self._puber_state.publish(state_msg)
        if len(self.u_res) > 0:
            control_cmd = self.u_res.pop(0)
            control_cmd_msg = Twist()
            control_cmd_msg.linear.x = float(control_cmd[0])
            control_cmd_msg.linear.y = float(control_cmd[1])
            control_cmd_msg.angular.z = float(control_cmd[2])
            self.speedx = np.append(self.speedx, control_cmd[0])
            self.speedy = np.append(self.speedy, control_cmd[1])
            self.speedz = np.append(self.speedz, control_cmd[2])
            self._puber_cmd.publish(control_cmd_msg)
        else:
            control_cmd_msg = Twist()
            control_cmd_msg.linear.x = 0.0
            control_cmd_msg.linear.y = 0.0
            control_cmd_msg.angular.z = 0.0
            self._puber_cmd.publish(control_cmd_msg)
            #self.node.get_logger().info("机器人停止，发布空指令 cmd_vel: 0, 0, 0")

    def _sub_odom(self, odom_msg: Odometry):
        """处理机器人的里程位置"""
        self.curr_state[0] = odom_msg.pose.pose.position.x
        self.curr_state[1] = odom_msg.pose.pose.position.y

        self.xdata = np.append(self.xdata, self.curr_state[0])
        self.ydata = np.append(self.ydata, self.curr_state[1])

        self.times += 1
        self._ros_log_info(f'times: {self.times}')
        if self.robotState == 1 and self.saved == 0:
            pd.DataFrame(self.xdata).to_csv('xres.csv', header=False, index=False)
            pd.DataFrame(self.ydata).to_csv('yres.csv', header=False, index=False)
            pd.DataFrame(self.speedx).to_csv('speedx.csv', header=False, index=False)
            pd.DataFrame(self.speedy).to_csv('speedy.csv', header=False, index=False)
            pd.DataFrame(self.speedz).to_csv('speedz.csv', header=False, index=False)
            self.saved = 1
            self.node.get_logger().info("saved.")



        _, _, self.curr_state[2] = quart_to_rpy(
            odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w
        )
        #仿真的yaw角与真实的yaw角相差180度
        if self.curr_state[2] <= 0:
            self.curr_state[2] += math.pi
        else:
            self.curr_state[2] -= math.pi

        #self._ros_log_info(f'x:{self.curr_state[0]}, y:{self.curr_state[1]}, yaw: {self.curr_state[2]}')
        self.have_odom = True

    def _sub_path(self, path_msg: Path):
        """ 处理期望轨迹 """
        #self.node.get_logger().info("sub.")
        designed_points = path_msg.poses
        #if len(designed_points) != self.step_len:
            #self.node.get_logger().warning(f"期望轨迹序列长度应为{self.step_len}，实际为{len(designed_points)}")

        designed_path = []
        for point in designed_points:
            x = point.pose.position.x
            y = point.pose.position.y
            _, _, yaw = quart_to_rpy(
                point.pose.orientation.x, point.pose.orientation.y, point.pose.orientation.z, point.pose.orientation.w
            )
            car_yaw = self.curr_state[2]
            designed_path.append([x, y, adjust_yaw(yaw, car_yaw)])

        self._process_path(designed_path)

    def _process_path(self, designed_path):
        """ 更新期望轨迹 """
        self.designed_path = designed_path
        self.have_dpath = True
        #self.node.get_logger().info("Path received.")

    def stop(self):
        self._timer_mpc_run.cancel()
        self._timer_pub_cmd.cancel()

    def run(self):
        rclpy.spin(self.node)
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    mpc_controller = MPCController(1/10, 1/20, 40)
    #mpc_controller = MPCController(1/2.5, 1/5, 40)
    mpc_controller.run()


if __name__ == "__main__":
    main()
