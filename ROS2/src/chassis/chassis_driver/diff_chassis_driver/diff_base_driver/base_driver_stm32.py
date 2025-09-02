import serial
import time
import struct
import sys
import threading
import signal
from datetime import datetime
import numpy as np

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return ((crc & 0xFF) << 8) | (crc >> 8)


class ChassisSerial(threading.Thread):
    def __init__(self, port, band):
        super().__init__()
        # 打开串口
        self.ser    = serial.Serial(port, band)
        # 设置运行标志位
        self.is_run = True
        self.info   = None
        # 实际解析出的左轮速度和右轮速度
        self.actual_left_speed  = 0.0
        self.actual_right_speed = 0.0
        self.last_left_number   = 0
        self.last_right_number  = 0
        self.last_time          = datetime.now()

    def stop(self):
        self.is_run = False
        time.sleep(0.1)
        self.ser.close()

    def read_frame(self, ser):
        buffer = bytearray()    # 用于存储读取到的数据
        frame_length = 63       # 数据帧长度
        start_byte = 0xAA       # 帧头字节

        while self.is_run:
            byte = ser.read(1)  # 每次读取一个字节
            if not byte:
                continue        # 如果没有读取到数据，继续等待

            # 检测帧头
            if byte[0] == start_byte:
                buffer.append(byte[0])
                buffer += ser.read(frame_length - 1)  # 读取剩余的字节

                if len(buffer) == frame_length:
                    return buffer

    def parse_chassis_data(self, data):
        # 定义数据帧格式
        format_str = (
            '<'
            'B'  # head_data (uint8_t) 数据帧头
            'B'  # version (uint8_t) 版本号
            'B'  # error (uint8_t) 报错状态
            'B'  # robot_type (uint8_t) 机器人类型
            'h'  # left_speed (short) 左轮速度
            'h'  # right_speed (short) 右轮速度
            'l'  # left_number (int32_t) 左轮编码器
            'l'  # right_number (int32_t) 右轮编码器
            '8H' # ult[8] (uint16_t[8]) 超声传感器数据
            'H'  # charge_left (uint16_t) 充电左超声
            'H'  # charge_right (uint16_t) 充电右超声
            'B'  # alarm (uint8_t) 距离警报状态
            'H'  # power (uint16_t) 电池电压
            'B'  # drop_front (uint8_t) 前跌落状态
            'B'  # drop_behind (uint8_t) 后跌落状态
            'B'  # avoid_mode (uint8_t) 避障模式
            'B'  # mode (uint8_t) 工作模式
            '3B' # door[3] (uint8_t[3]) 快递或售卖功能状态
            'B'  # wash_mode (uint8_t) 清扫模式
            'B'  # wash_spray (uint8_t) 喷雾开关状态
            'B'  # wash_brush_up_down (uint8_t) 刷子升降
            'B'  # wash_brush_start (uint8_t) 刷子开关
            'B'  # wash_water (uint8_t) 清水水量百分比
            '3B' # rgb_led[3] (uint8_t[3]) RGB灯色状态
            'B'  # charge_state (uint8_t) 充电状态
            'B'  # temperature (uint8_t) 工作温度
            'B'  # wet (uint8_t) 湿度
            'B'  # body_sensor (uint8_t) 人体感应状态
            'H'  # head (uint16_t) 底盘朝向角度
            'B'  # command (uint8_t) 共用指令
            'H'  # crc (uint16_t) 校验码
        )

        # 使用struct.unpack解析数据
        parsed_data = struct.unpack(format_str, data)

        # 将解析后的数据映射到字段
        chassis_info = {
            "head_data": parsed_data[0],
            "version": parsed_data[1],
            "error": parsed_data[2],
            "robot_type": parsed_data[3],
            "left_speed": parsed_data[4],
            "right_speed": -parsed_data[5],     # 需要加个-才能保存正确
            "left_number": parsed_data[6],
            "right_number": parsed_data[7],
            "ult": parsed_data[8:16],
            "charge_left": parsed_data[16],
            "charge_right": parsed_data[17],
            "alarm": parsed_data[18],
            "power": parsed_data[19],
            "drop_front": parsed_data[20],
            "drop_behind": parsed_data[21],
            "avoid_mode": parsed_data[22],
            "mode": parsed_data[23],
            "door": parsed_data[24:27],
            "wash_mode": parsed_data[27],
            "wash_spray": parsed_data[28],
            "wash_brush_up_down": parsed_data[29],
            "wash_brush_start": parsed_data[30],
            "wash_water": parsed_data[31],
            "rgb_led": parsed_data[32:35],
            "charge_state": parsed_data[35],
            "temperature": parsed_data[36],
            "wet": parsed_data[37],
            "body_sensor": parsed_data[38],
            "head": parsed_data[39],
            "command": parsed_data[40],
            "crc": parsed_data[41],
        }

        return chassis_info

    def run(self):
        # 循环运行
        while self.is_run:
            # 当读取到有效数据的时候，进行数据处理
            if (serial_data := self.read_frame(self.ser) ) is not None:
                # CRC正确
                info = self.parse_chassis_data(serial_data)
                if ( crc16(serial_data[:-2]) == info['crc'] ):
                    self.info = info
                    # 解析速度
                    current_time = datetime.now()
                    interval     = 0.5*( 100 + (current_time - self.last_time).total_seconds() * 1000 )  # 将时间间隔转换为毫秒，由于固定频率100ms，减小误差
                    actual_left_speed  = 1000 * 60 * (info['left_number'] - self.last_left_number)   / (16384 * interval)
                    actual_right_speed = 1000 * 60 * (info['right_number'] - self.last_right_number) / (16384 * interval)
                    self.last_time          = current_time
                    self.last_left_number   = info['left_number']
                    self.last_right_number  = info['right_number']
                    if np.abs(actual_left_speed) < 100 and np.abs(actual_right_speed) < 100:
                        self.actual_left_speed  = actual_left_speed
                        self.actual_right_speed = actual_right_speed


    def get_info(self):
        return self.info

    def send_cmd(self, left_speed, right_speed, command=8):
        # 使用struct来打包数据，'B'代表一个无符号字符，'h'代表一个有符号短整数
        cmd = struct.pack('=BhhB', 0xAA, round(left_speed), round(right_speed), command)
        self.ser.write(cmd)



class ChassisDriver(object):

    def _get_serial_and_port_from_ros(self):
        """ 从 ROS 参数服务器当中获取串口信息 """
        raise NotImplementedError

    def _puber_set_chassis_actual_wheelspeed(self):
        """ 设置 /chassis/actual/wheelspeed 的 topic 发送器 """
        raise NotImplementedError

    def _pub_chassis_actual_wheelspeed(self, left_speed:int, right_speed:int):
        """ 发送 /chassis/actual/wheelspeed """
        raise NotImplementedError

    def _suber_set_chassis_command_wheelspeed(self):
        """ 设置 /chassis/command/wheelspeed 的 topic 接收器 """
        raise NotImplementedError

    def _suber_cb_chassis_command_wheelspeed(self, msg):
        """  /chassis/command/wheelspeed 的接收回调函数 """
        raise NotImplementedError

    def _timer_set_pub_task(self):
        """  构建10Hz的定时器，运行发送任务 """
        raise NotImplementedError

    def _timer_stop_pub_task(self):
        """  构建10Hz的定时器，运行发送任务 """
        raise NotImplementedError

    def _ros_log_debug(self, log_data):
        raise NotImplementedError

    def _ros_log_info(self, log_data):
        raise NotImplementedError

    def _ros_log_warn(self, log_data):
        raise NotImplementedError

    def _ros_log_error(self, log_data):
        raise NotImplementedError

    def __init__(self) -> None:
        # Step1：从ROS参数服务器中获取通讯参数
        self._get_serial_and_port_from_ros()
        # Step2：打开串口
        try:
            self.chassis_serial = ChassisSerial(self.serial_port, self.serial_band)
            self._ros_log_info(f'serial {self.serial_port} open successfully!')
        except:
            self._ros_log_error(f'serial {self.serial_port} do not open!')
            sys.exit(0)
        # Step3：新建一个线程用于处理串口数据
        self.chassis_serial.start()
        # Step4：ROS相关
        self._puber_set_chassis_actual_wheelspeed()
        self._suber_set_chassis_command_wheelspeed()
        self._timer_set_pub_task()

    def pub_task(self, timer_triger=None):
        info = self.chassis_serial.get_info()
        self._ros_log_debug(info)
        if info is not None:
            self._pub_chassis_actual_wheelspeed(self.chassis_serial.actual_left_speed, self.chassis_serial.actual_right_speed)
            self._ros_log_debug(f"actual wheel speed: {self.chassis_serial.actual_left_speed}, {self.chassis_serial.actual_right_speed}")
        print(info)

    def stop(self):
        # self._ros_log_info(f'stoping')
        self._timer_stop_pub_task()
        self.chassis_serial.stop()





import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ChassisDriverROS2(ChassisDriver, Node):

    def __init__(self):
        Node.__init__(self, 'chassis_driver_stm32_node')
        ChassisDriver.__init__(self)


    def _get_serial_and_port_from_ros(self):
        """ 从 ROS 参数服务器当中获取串口信息 """
        self.declare_parameter('port', '/dev/base_driver_d')
        self.declare_parameter('baud', 115200)
        self.serial_port = self.get_parameter('port').get_parameter_value().string_value
        self.serial_band = self.get_parameter('baud').get_parameter_value().integer_value


    def _puber_set_chassis_actual_wheelspeed(self):
        """ 设置 /chassis/actual/wheelspeed 的 topic 发送器 """
        self._puber_chassis_actual_wheelspeed = self.create_publisher(Float32MultiArray, '/chassis/actual/wheelspeed', 10)

    def _pub_chassis_actual_wheelspeed(self, left_speed:int, right_speed:int):
        """ 发送 /chassis/actual/wheelspeed """
        wheelspeed_msg = Float32MultiArray()
        wheelspeed_msg.data = [left_speed, right_speed]
        self._puber_chassis_actual_wheelspeed.publish(wheelspeed_msg)

    def _suber_set_chassis_command_wheelspeed(self):
        """ 设置 /chassis/command/wheelspeed 的 topic 接收器 """
        self.create_subscription(Float32MultiArray, '/chassis/command/wheelspeed', self._suber_cb_chassis_command_wheelspeed, 10)

    def _suber_cb_chassis_command_wheelspeed(self, msg):
        """  /chassis/command/wheelspeed 的接收回调函数 """
        self.chassis_serial.send_cmd(msg.data[0], msg.data[1])

    def _timer_set_pub_task(self):
        """  构建10Hz的定时器，运行发送任务 """
        self.pub_timer = self.create_timer(0.1, self.pub_task)

    def _timer_stop_pub_task(self):
        self.pub_timer.cancel()

    def _ros_log_debug(self, log_data):
        self.get_logger().debug(str(log_data))

    def _ros_log_info(self, log_data):
        self.get_logger().info(str(log_data))

    def _ros_log_warn(self, log_data):
        self.get_logger().warn(str(log_data))

    def _ros_log_error(self, log_data):
        self.get_logger().error(str(log_data))

    def run(self):
        if rclpy.ok():
            rclpy.spin(self)



def signal_handler(sig, frame):
    chassis_driver.stop()
    time.sleep(0.1)
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
rclpy.init()
chassis_driver = ChassisDriverROS2()


def main():
    chassis_driver.run()

if __name__ == "__main__":
    main()
