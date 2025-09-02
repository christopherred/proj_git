from dataclasses import dataclass, field
from typing import List, Dict
from enum import Enum
from multiprocessing import shared_memory
import pickle
import os
import time
import subprocess

# ROS2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

class RosState(Enum):
    """ 传感器状态枚举 """
    UNKNOWN       = 0
    INITIALIZING  = 1
    RUNNING       = 2
    ERROR         = 3

class ControlMode(Enum):
    """ 控制模式枚举 """
    JOY           = 0
    AUTONOMOUS    = 1
    STOP          = 2

class RosAction(Enum):
    NOACTION      = 0
    START         = 1
    CLOSE         = 2

@dataclass
class RobotData:
    # 系统状态
    robot_state: dict = field(default_factory=lambda: {
        "odom_position": [-0.0, -0.0, -0.0],
        "odom_velocity": [-0.0, -0.0, -0.0],
        "cmd_velocity":  [-0.0, -0.0, -0.0],
        "control_mode":  ControlMode.STOP,
    })
    # 动作指令
    ros_action: dict = field(default_factory=lambda: {
        "sensors":              RosAction.NOACTION,
        "camera":               RosAction.NOACTION,
        "chassis1":             RosAction.NOACTION,
        "chassis2":             RosAction.NOACTION,
        "joy":                  RosAction.NOACTION,
        'cartographer':         RosAction.NOACTION,
        'cartographer_pose':    RosAction.NOACTION,
    })
    # 传感器状态
    ros_state: dict = field(default_factory=lambda: {
        "sensors":              RosState.UNKNOWN,
        "camera":               RosState.UNKNOWN,
        "chassis1":             RosState.UNKNOWN,
        "chassis2":             RosState.UNKNOWN,
        "joy":                  RosState.UNKNOWN,
        "cartographer":         RosState.UNKNOWN,
        "cartographer_pose":    RosState.UNKNOWN
    })

class RobotDataManager:
    """ 机器人数据管理器 """
    def __init__(self, need_create=False):
        self.need_create = need_create
        self.data        = RobotData()
        buffer_size      = len(pickle.dumps(self.data))
        try:
            if need_create:
                self._shared_memory = shared_memory.SharedMemory(name='robot_data', create=True, size=10*buffer_size)
            else:
                self._shared_memory = shared_memory.SharedMemory(name='robot_data', create=False)
        except FileExistsError:
            # 如果共享内存已存在但需要创建，先尝试连接并删除它
            if need_create:
                temp = shared_memory.SharedMemory(name='robot_data', create=False)
                temp.close()
                temp.unlink()
                self._shared_memory = shared_memory.SharedMemory(name='robot_data', create=True, size=10*buffer_size)


    def update(self):
        """ 更新数据到共享内存 """
        data_bytes = pickle.dumps(self.data)
        self._shared_memory.buf[:len(data_bytes)] = data_bytes

    def read_from_memory(self):
        """ 从共享内存读取数据 """
        data_bytes = bytes(self._shared_memory.buf)
        self.data = pickle.loads(data_bytes)
        return self.data

    def close(self):
        # 关闭共享内存
        if self.need_create:
            self._shared_memory.close()
            self._shared_memory.unlink()
        else:
            self._shared_memory.close()



def check_sensors_log_conditions(log_file="log/sensors.log"):
    try:
        with open(log_file, 'r') as file:
            content = file.read()

            # 检查所有需要的条件是否都存在
            condition1 = "current scan mode" in content
            condition2 = "Successfully connected" in content
            condition3 = "ahrsBringup::processLoop: start" in content

            all_conditions_met = condition1 and condition2 and condition3

            # 可以打印各个条件的状态用于调试
            print(f"'current scan mode' 存在: {condition1}")
            print(f"'Successfully connected' 存在: {condition2}")
            print(f"'ahrsBringup::processLoop: start' 存在: {condition3}")

            return all_conditions_met
    except FileNotFoundError:
        print(f"找不到日志文件: {log_file}")
        return False
    except Exception as e:
        print(f"读取日志时出错: {e}")
        return False



class RobotManager(Node):

    def __init__(self):
        super().__init__('robot_manager')
        self.get_logger().info("Robot manager started successfully")
        self.data_manager = RobotDataManager()
        self.create_subscription(
            Odometry,
            '/cartographer/filtered_odom',
            self._get_robot_odom,
            10
        )
        self.create_subscription(
            Int32,
            '/chassis/mode',
            self._get_robot_mode,
            10
        )
        self.create_subscription(
            Twist,
            '/chassis/command/cmd_vel',
            self._get_robot_cmdvel,
            10
        )
        self.create_timer(0.1, self._manager)

    def _manager(self):
        # 接收信号，启动ros launch，维护状态
        self.data_manager.read_from_memory()

        # Sensors
        if str(self.data_manager.data.ros_action['sensors']) == str(RosAction.START) and str(self.data_manager.data.ros_state['sensors']) in [str(RosState.UNKNOWN), str(RosState.ERROR)] :
            print("启动传感器")
            self.sensors_process = subprocess.Popen(
                "ros2 launch sensors_driver start.launch.py",
                shell=True,
                stdout=open("log/sensors.log", "w"),
                stderr=subprocess.STDOUT,  # 合并stderr到stdout
                text=True
            )
            self.data_manager.data.ros_action['sensors'] = RosAction.NOACTION
            self.data_manager.data.ros_state['sensors']  = RosState.INITIALIZING
            self.data_manager.update()

        if str(self.data_manager.data.ros_state['sensors'])  == str(RosState.INITIALIZING):
            if check_sensors_log_conditions():
                self.data_manager.data.ros_state['sensors']  = RosState.RUNNING
                self.data_manager.update()


        # Chassis1
        if str(self.data_manager.data.ros_action['chassis1']) == str(RosAction.START) and str(self.data_manager.data.ros_state['chassis1']) in [str(RosState.UNKNOWN), str(RosState.ERROR)] :
            print("以局部坐标系模式启动底盘")
            self.sensors_process = subprocess.Popen(
                "ros2 launch omni_chassis_driver actual.launch.py",
                shell=True,
                stdout=open("log/chassis1.log", "w"),
                stderr=subprocess.STDOUT,  # 合并stderr到stdout
                text=True
            )
            self.data_manager.data.ros_action['chassis1'] = RosAction.NOACTION
            self.data_manager.data.ros_state['chassis1']  = RosState.INITIALIZING
            self.data_manager.update()

        # Chassis2
        if str(self.data_manager.data.ros_action['chassis2']) == str(RosAction.START) and str(self.data_manager.data.ros_state['chassis2']) in [str(RosState.UNKNOWN), str(RosState.ERROR)] :
            print("以全局坐标系模式启动底盘")
            self.sensors_process = subprocess.Popen(
                "ros2 launch omni_chassis_driver actual.launch.py use_global_controller:=true",
                shell=True,
                stdout=open("log/chassis2.log", "w"),
                stderr=subprocess.STDOUT,  # 合并stderr到stdout
                text=True
            )
            self.data_manager.data.ros_action['chassis2'] = RosAction.NOACTION
            self.data_manager.data.ros_state['chassis2']  = RosState.INITIALIZING
            self.data_manager.update()

        # joy
        if str(self.data_manager.data.ros_action['joy']) == str(RosAction.START) and str(self.data_manager.data.ros_state['joy']) in [str(RosState.UNKNOWN), str(RosState.ERROR)] :
            print("启动手柄")
            self.sensors_process = subprocess.Popen(
                "ros2 launch  joy_control             start.launch.py",
                shell=True,
                stdout=open("log/joy.log", "w"),
                stderr=subprocess.STDOUT,  # 合并stderr到stdout
                text=True
            )
            self.data_manager.data.ros_action['joy'] = RosAction.NOACTION
            self.data_manager.data.ros_state['joy']  = RosState.INITIALIZING
            self.data_manager.update()

        # cartographer
        if str(self.data_manager.data.ros_action['cartographer']) == str(RosAction.START) and str(self.data_manager.data.ros_state['cartographer']) in [str(RosState.UNKNOWN), str(RosState.ERROR)] :
            print("启动手柄")
            self.sensors_process = subprocess.Popen(
                "ros2 launch  cartographer cartographer.launch.py",
                shell=True,
                stdout=open("log/cartographer.log", "w"),
                stderr=subprocess.STDOUT,  # 合并stderr到stdout
                text=True
            )
            self.data_manager.data.ros_action['cartographer'] = RosAction.NOACTION
            self.data_manager.data.ros_state['cartographer']  = RosState.INITIALIZING
            self.data_manager.update()

        # cartographer_pose
        if str(self.data_manager.data.ros_action['cartographer_pose']) == str(RosAction.START) and str(self.data_manager.data.ros_state['cartographer_pose']) in [str(RosState.UNKNOWN), str(RosState.ERROR)] :
            print("启动定位")
            self.sensors_process = subprocess.Popen(
                "ros2 launch  cartographer_pose cartographer_pose.launch.py",
                shell=True,
                stdout=open("log/cartographer_pose.log", "w"),
                stderr=subprocess.STDOUT,  # 合并stderr到stdout
                text=True
            )
            self.data_manager.data.ros_action['cartographer_pose'] = RosAction.NOACTION
            self.data_manager.data.ros_state['cartographer_pose']  = RosState.INITIALIZING
            self.data_manager.update()

    def close_process(self):
        print("清理进程")
        try:
            self.sensors_process.terminate()
        except:
            pass
        finally:
            self.sensors_process.kill()



    def _get_robot_odom(self, msg: Odometry):
        """ 获取机器人里程计数据 """
        # 提取位置
        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        # 提取线速度
        velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z
        )
        # 更新机器人状态
        self.data_manager.data.robot_state["odom_position"][0] = position[0]
        self.data_manager.data.robot_state["odom_position"][1] = position[1]
        self.data_manager.data.robot_state["odom_position"][2] = euler_from_quaternion(orientation)[2]
        self.data_manager.data.robot_state["odom_velocity"][0] = velocity[0]
        self.data_manager.data.robot_state["odom_velocity"][1] = velocity[1]
        self.data_manager.data.robot_state["odom_velocity"][2] = velocity[2]
        self.data_manager.update()

    def _get_robot_mode(self, msg: Int32):
        """ 获取机器人控制模式 """
        # 更新机器人状态
        if msg.data == 0:
            self.data_manager.data.robot_state["control_mode"] = ControlMode.STOP
        elif msg.data == 1:
            self.data_manager.data.robot_state["control_mode"] = ControlMode.JOY
        elif msg.data == 2:
            self.data_manager.data.robot_state["control_mode"] = ControlMode.AUTONOMOUS
        else:
            pass
        self.data_manager.update()

    def _get_robot_cmdvel(self, msg: Twist):
        """ 获取机器人命令速度 """
        # 提取线速度
        cmd_velocity = (
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        )
        # 更新机器人状态
        self.data_manager.data.robot_state["cmd_velocity"][0] = cmd_velocity[0]
        self.data_manager.data.robot_state["cmd_velocity"][1] = cmd_velocity[1]
        self.data_manager.data.robot_state["cmd_velocity"][2] = cmd_velocity[2]
        self.data_manager.update()


if __name__ == '__main__':
    data_manager = RobotDataManager(need_create=True)
    data_manager.update()
    data_manager.read_from_memory()
    data_manager.update()
    rclpy.init()
    robot_manager = RobotManager()
    try:
        rclpy.spin(robot_manager)
    except KeyboardInterrupt:
        pass
    finally:
        data_manager.close()
        robot_manager.close_process()
        if rclpy.ok():
            rclpy.shutdown()






