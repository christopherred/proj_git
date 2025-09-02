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

def crc16_modbus(data):
    crc = 0xFFFF  # Initial value for CRC register
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
        low_byte = (crc & 0xFF00) >> 8
        high_byte = crc & 0x00FF
    return high_byte,low_byte

def crc16_check(data:bytes):
        temp = data[:-2]
        high_byte_true, low_byte_true = crc16_modbus(temp)
        high_byte = data[-1]
        low_byte = data[-2]
        if (high_byte==high_byte_true)and(low_byte==low_byte_true):
            return True
        else:
            return False

def split_int(n):
    n = round(n)
    # 获取高8位和低8位
    high_byte = (n >> 8) & 0xFF    # 确保高字节在0-255之间
    low_byte  = n & 0xFF           # 确保低字节在0-255之间
    return high_byte, low_byte

class ChassisSerial(threading.Thread):

    def __init__(self, port, band):
        super().__init__()
        # 打开串口
        self.ser            = serial.Serial(port, band)
        # 设置运行标志位
        self.is_run         = True
        self.is_init        = False
        self.need_set_speed = False
        # 实际解析出的左轮速度和右轮速度
        self.actual_left_speed   = 0.0
        self.actual_right_speed  = 0.0
        self.command_left_speed  = 0.0
        self.command_right_speed = 0.0
        # 初始化电机
        self.motor_init()


    def terminate(self):
        self.is_run = False
        self.motor_stop()
        time.sleep(0.1)
        self.ser.close()


    def motor_init(self, left_acctime = 250, right_acctime = 500, left_redtime = 250, right_redtime = 500):
        """ 初始化电机指令 """

        # 设置速度模式命令:01 06 20 0D 00 03 53 C8
        self.ser.write(struct.pack('=BBBBBBBB', 0x01, 0x06, 0x20, 0x0D, 0x00, 0x03, 0x53, 0xC8))
        time.sleep(0.1)

        # 设置左电机S型加速时间500ms命令:01 06 20 80 01 F4 83 F5
        left_acctime1, left_acctime2    = split_int(left_acctime)
        init_cmd_Lmotoracctime          = struct.pack('=BBBBBB', 0x01, 0x06, 0x20, 0x80, left_acctime1,left_acctime2)
        crc_high_byte, crc_low_byte     = crc16_modbus(init_cmd_Lmotoracctime)
        init_cmd_Lmotoracctime          = struct.pack('=BBBBBBBB', 0x01, 0x06, 0x20, 0x80, left_acctime1, left_acctime2, crc_high_byte, crc_low_byte)
        #self.ser.write(init_cmd_Lmotoracctime)
        time.sleep(0.1)


        # 设置左电机S型减速时间500ms命令:01 06 20 82 01 F4 22 35
        left_redtime1,left_redtime2     = split_int(left_redtime)
        init_cmd_Lmotorredtime          = struct.pack('=BBBBBB', 0x01, 0x06, 0x20, 0x82, left_redtime1, left_redtime2)
        high_byte, low_byte             = crc16_modbus(init_cmd_Lmotorredtime)
        init_cmd_Lmotorredtime          = struct.pack('=BBBBBBBB', 0x01, 0x06, 0x20, 0x82, left_redtime1,left_redtime2,high_byte,low_byte)
        #self.ser.write(init_cmd_Lmotorredtime)
        time.sleep(0.1)

        # 设置右电机S型加速时间500ms命令: 01 06 20 81 01 F4 D2 35
        right_acctime1,right_acctime2   = split_int(right_acctime)
        init_cmd_Rmotoracctime          = struct.pack('=BBBBBB', 0x01, 0x06, 0x20, 0x81, right_acctime1,right_acctime2)
        high_byte, low_byte             = crc16_modbus(init_cmd_Rmotoracctime)
        init_cmd_Rmotoracctime          = struct.pack('=BBBBBBBB', 0x01, 0x06, 0x20, 0x81, right_acctime1,right_acctime2,high_byte,low_byte)
        #self.ser.write(init_cmd_Rmotoracctime)
        time.sleep(0.1)


        # 设置右电机S型减速时间500ms命令: 01 06 20 83 01 F4 73 F5
        right_redtime1,right_redtime2   = split_int(right_redtime)
        init_cmd_Rmotorredtime          = struct.pack('=BBBBBB', 0x01, 0x06, 0x20, 0x83, right_redtime1,right_redtime2)
        high_byte, low_byte             = crc16_modbus(init_cmd_Rmotorredtime)
        init_cmd_Rmotorredtime          = struct.pack('=BBBBBBBB', 0x01, 0x06, 0x20, 0x83, right_redtime1,right_redtime2,high_byte,low_byte)
        #self.ser.write(init_cmd_Rmotorredtime)
        time.sleep(0.1)

        #使能命令：01 06 20 0E 00 08 E2 0F
        init_cmd_Enable                 = struct.pack('=BBBBBBBB', 0x01, 0x06, 0x20, 0x0E, 0x00, 0x08, 0xE2, 0x0F)
        self.ser.write(init_cmd_Enable)

        # 清空输出缓冲区
        time.sleep(1)
        self.ser.reset_input_buffer()


    def motor_stop(self):
        #停机命令：01 06 20 0E 00 07 A2 0B
        self.ser.reset_input_buffer()
        stop_cmd_stop = struct.pack('=BBBBBBBB', 0x01, 0x06, 0x20, 0x0E, 0x00, 0x07, 0xA2, 0x0B)
        self.ser.write(stop_cmd_stop)
        time.sleep(0.01)


    def motor_read_speed(self):
        self.ser.reset_input_buffer()
        read_speed_cmd = struct.pack('=BBBBBBBB', 0x01, 0x03, 0x20, 0xAB, 0x00, 0x02, 0xBE, 0x2B)
        self.ser.write(read_speed_cmd)
        buffer = self.ser.read(9)
        print(' '.join(f"0x{byte:02X}" for byte in buffer))
        parsed_data = struct.unpack(('>''B''B''B''h''h''B''B'), buffer)
        self.actual_left_speed  = parsed_data[3]/10
        self.actual_right_speed = -(parsed_data[4]/10)
        print(f'left: {self.actual_left_speed}, right: {self.actual_right_speed}')
        time.sleep(0.01)


    def motor_set_speed(self):
        self.ser.reset_input_buffer()
        left_speed1,  left_speed2  = split_int(self.command_left_speed)
        right_speed1, right_speed2 = split_int(-self.command_right_speed)
        #01 10 20 88 00 02 04 00 64 00 64 23 9C
        speed_cmd = struct.pack('=BBBBBBBBBBB', 0x01, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04, left_speed1, left_speed2, right_speed1, right_speed2)
        crc_high_byte, crc_low_byte = crc16_modbus(speed_cmd)
        speed_cmd = struct.pack('=BBBBBBBBBBBBB', 0x01, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04, left_speed1, left_speed2, right_speed1, right_speed2, crc_high_byte, crc_low_byte)
        self.ser.write(speed_cmd)
        time.sleep(0.01)

    def asyn_get_speed(self):
        return (self.actual_left_speed, self.actual_right_speed)

    def asyn_set_speed(self, left_speed, right_speed):
        self.command_left_speed  = left_speed
        self.command_right_speed = right_speed



    def run(self):
        while self.is_run:
            self.motor_read_speed()
            self.motor_set_speed()
            time.sleep(0.01)



import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ChassisDriverROS2(Node):

    def __init__(self):
        Node.__init__(self, 'diff_base_driver')
        # Step1：从ROS参数服务器中获取通讯参数
        self.declare_parameter('port', '/dev/ttyCH341USB0')
        self.declare_parameter('baud', 115200)
        self.serial_port = self.get_parameter('port').get_parameter_value().string_value
        self.serial_band = self.get_parameter('baud').get_parameter_value().integer_value
        # Step2：打开串口
        try:
            self.chassis_serial = ChassisSerial(self.serial_port, self.serial_band)
            self.get_logger().info(f'serial {self.serial_port} open successfully!')
        except:
            self.get_logger().error(f'serial {self.serial_port} do not open!')
            sys.exit(0)
        # Step3：新建一个线程用于处理串口数据
        self.chassis_serial.start()
        # Step4：ROS相关
        self.actual_wheelspeed_puber  = self.create_publisher(Float32MultiArray, '/chassis/actual/wheelspeed', 10)
        self.create_subscription(Float32MultiArray, '/chassis/command/wheelspeed', self.command_wheelspeed_callback, 10)
        self.run_timer = self.create_timer(0.1, self.pub_task)



    def pub_task(self, timer_triger=None):
        left_speed, right_speed = self.chassis_serial.asyn_get_speed()
        wheelspeed_msg          = Float32MultiArray()
        wheelspeed_msg.data     = [float(left_speed), float(right_speed)]
        self.actual_wheelspeed_puber.publish(wheelspeed_msg)


    def command_wheelspeed_callback(self, msg):
        """  /chassis/command/wheelspeed 的接收回调函数 """
        self.chassis_serial.asyn_set_speed(msg.data[0], msg.data[1])


    def terminate(self):
        self.run_timer.cancel()
        self.chassis_serial.terminate()



def main():
    rclpy.init()
    motor_node = ChassisDriverROS2()
    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_node.terminate()
        time.sleep(0.1)
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":

    main()
