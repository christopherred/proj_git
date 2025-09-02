import serial
from scipy.interpolate import interp1d
import numpy as np
import struct
import rclpy
import rclpy.logging
from rclpy.node import Node
import time
from collections import deque
import threading
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Float32MultiArray  # 修改话题类型为 Float32MultiArray
import signal
import warnings
import sys
from rclpy.qos import qos_profile_sensor_data
warnings.filterwarnings('ignore')
rclpy.init()
logger = rclpy.logging.get_logger("base_driver")

motor_filter_states = [(0, 0), (0, 0), (0, 0)]


class Motor(threading.Thread):

    def __init__(self, ser, id) -> None:
        super().__init__()
        self.running = True
        self.ser = ser
        self.id = id
        _currents = np.array([50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750])
        _torques = np.array([0.084, 0.155, 0.228, 0.303, 0.380, 0.451, 0.524, 0.597, 0.669, 0.743, 0.817, 0.893, 0.973, 1.050, 1.126])
        currents = np.hstack((-_currents, _currents))
        torques = np.hstack((-_torques, _torques))
        self.current2torque = interp1d(currents, torques, 'cubic')
        self.torque2current = interp1d(torques, currents, 'cubic')
        self.open()
        self.speed_filter = deque(maxlen=5)
        self.torque_filter = deque(maxlen=5)
        self.expect_speed = 0
        self.need_set_speed = False

    def _open(self):
        error = True
        data2send = bytearray(5)
        data2send[0] = 0x3E
        data2send[1] = 0x88
        data2send[2] = self.id
        data2send[3] = 0x00
        data2send[4] = (data2send[0] + data2send[1] + data2send[2] + data2send[3]) % 256
        self.ser.write(data2send)
        time.sleep(0.1)
        data_receive = self.ser.read_all()
        try:
            if data_receive[4] == data2send[4]:
                error = False
            else:
                error = True
        except:
            error = True
        return error

    def open(self):
        error = self._open()
        while error:
            try:
                logger.error("Open motor " + str(self.id) + " failed!")
                error = self._open()
                time.sleep(1)
            except:
                break
        logger.info("Open motor " + str(self.id) + " succeeded!")

    def _read_speed_and_torque(self):
        error = False
        data2send = bytearray(5)
        data2send[0] = 0x3E
        data2send[1] = 0x9C
        data2send[2] = self.id
        data2send[3] = 0x00
        data2send[4] = (data2send[0] + data2send[1] + data2send[2] + data2send[3]) % 256
        self.ser.write(data2send)
        time.sleep(0.005)
        data_receive = self.ser.read_all()
        try:
            if not data_receive[0] == data2send[0] and data_receive[1] == data2send[1] and data_receive[2] == data2send[2] and data_receive[3] == data2send[3]:
                error = True
                return (0, 0, error)
            if not (data_receive[5] + data_receive[6] + data_receive[7] + data_receive[8] + data_receive[9] + data_receive[10] + data_receive[11]) % 256 == data_receive[12]:
                error = True
                return (0, 0, error)
        except:
            error = True
            return (0, 0, error)

        current_bytes = bytearray([data_receive[6], data_receive[7]])
        current = struct.unpack('<h', current_bytes)[0]
        torque = float(self.current2torque(current))
        speed_bytes = bytearray([data_receive[8], data_receive[9]])
        speed = struct.unpack('<h', speed_bytes)[0]
        return (torque, speed, error)

    def read_speed_and_torque(self):
        (torque, speed, error) = self._read_speed_and_torque()
        while error:
            try:
                logger.error("Read motor " + str(self.id) + " failed!")
                (torque, speed, error) = self._read_speed_and_torque()
            except:
                pass

        self.speed_filter.append(speed)
        self.torque_filter.append(torque)
        global motor_filter_states
        motor_filter_states[self.id - 1] = self.cal_state()

    def _set_speed(self, expect_speed: float):
        error = False
        int_speed = int(expect_speed * 100)
        bytes_speed = struct.pack('<i', int_speed)
        data2send = bytearray(10)
        data2send[0] = 0x3E
        data2send[1] = 0xA2
        data2send[2] = self.id
        data2send[3] = 0x04
        data2send[4] = (data2send[0] + data2send[1] + data2send[2] + data2send[3]) % 256
        data2send[5] = bytes_speed[0]
        data2send[6] = bytes_speed[1]
        data2send[7] = bytes_speed[2]
        data2send[8] = bytes_speed[3]
        data2send[9] = (data2send[5] + data2send[6] + data2send[7] + data2send[8]) % 256
        self.ser.write(data2send)
        time.sleep(0.005)
        data_receive = self.ser.read_all()
        try:
            if not data_receive[0] == data2send[0] and data_receive[1] == data2send[1] and data_receive[2] == data2send[2] and data_receive[3] == data2send[3]:
                error = True
                return (0, 0, error)
            if not (data_receive[5] + data_receive[6] + data_receive[7] + data_receive[8] + data_receive[9] + data_receive[10] + data_receive[11]) % 256 == data_receive[12]:
                error = True
                return (0, 0, error)
        except:
            error = True
            return (0, 0, error)
        current_bytes = bytearray([data_receive[6], data_receive[7]])
        current = struct.unpack('<h', current_bytes)[0]
        torque = float(self.current2torque(current))
        speed_bytes = bytearray([data_receive[8], data_receive[9]])
        speed = struct.unpack('<h', speed_bytes)[0]
        return (torque, speed, error)

    def set_speed(self):
        (torque, speed, error) = self._set_speed(self.expect_speed)
        while error:
            logger.info("Set motor " + str(self.id) + " speed failed!")
            (torque, speed, error) = self._set_speed(self.expect_speed)
            self.speed_filter.append(speed)
            self.torque_filter.append(torque)
        self.need_set_speed = False

    def asyn_set_speed(self, expect_speed):
        # logger.info(f"Motor {self.id} expected speed set to: {expect_speed}")
        self.expect_speed = expect_speed
        self.need_set_speed = True

    def cal_state(self):
        speed_array = np.array(self.speed_filter)
        speed = np.mean(speed_array[np.logical_and(speed_array != np.max(speed_array), speed_array != np.min(speed_array))])
        if np.isnan(speed):
            speed = np.mean(speed_array)
        torque_array = np.array(self.torque_filter)
        torque = np.mean(torque_array[np.logical_and(torque_array != np.max(torque_array), torque_array != np.min(torque_array))])
        if np.isnan(torque):
            torque = np.mean(torque_array)
        return (speed, torque)

    def run(self):
        while self.running:
            if self.need_set_speed:
                self.set_speed()
            else:
                self.read_speed_and_torque()
            time.sleep(0.02)

    def stop(self):
        self._set_speed(0)
        self._set_speed(0)
        self._set_speed(0)
        logger.info("Motor " + str(self.id) + " stop!")

    def terminate(self):
        self.running = False
        self.stop()
        logger.info("Motor " + str(self.id) + " terminate!")


class MotorControlNode(Node):
    def __init__(self, motor1, motor2, motor3):
        Node.__init__(self, "base_driver")
        self.__create_ros_communication()
        self.motor1, self.motor2, self.motor3 = motor1, motor2, motor3
        self.get_logger().info("Motor control node started successfully!")
        time.sleep(1)
        self.pub_timer = self.create_timer(0.05, self.pub_task)

    def __create_ros_communication(self):
        # 发布电机的速度和扭矩
        self.publisher_wheelspeed = self.create_publisher(
            Float32MultiArray, '/chassis/actual/wheelspeed', qos_profile_sensor_data)
        self.publisher_wheeltorque = self.create_publisher(
            Float32MultiArray, '/chassis/actual/wheeltorque', qos_profile_sensor_data)

        # 订阅 '/chassis/command/wheelspeed' 以接收电机速度指令
        self.subscription_wheelspeed = self.create_subscription(
            Float32MultiArray, '/chassis/command/wheelspeed', self.__wheelspeed_callback, qos_profile_sensor_data)

        self.running = True

    def __wheelspeed_callback(self, msg: Float32MultiArray):
        if self.running:
            self.motor1.asyn_set_speed(float(np.rad2deg(-msg.data[0])))
            self.motor2.asyn_set_speed(float(np.rad2deg(-msg.data[1])))
            self.motor3.asyn_set_speed(float(np.rad2deg(-msg.data[2])))



    def pub_task(self):
        if self.running:
            wheelspeed_msg = Float32MultiArray()
            wheeltorque_msg = Float32MultiArray()

            global motor_filter_states
            speed1, torque1 = motor_filter_states[0]
            speed2, torque2 = motor_filter_states[1]
            speed3, torque3 = motor_filter_states[2]

            # 发布电机的实际线速度
            wheelspeed_msg.data = [
                float(np.deg2rad(-speed1)),
                float(np.deg2rad(-speed2)),
                float(np.deg2rad(-speed3))
            ]
            self.publisher_wheelspeed.publish(wheelspeed_msg)

            # 发布电机的实际扭矩
            wheeltorque_msg.data = [
                float(-torque1),
                float(-torque2),
                float(-torque3)
            ]
            self.publisher_wheeltorque.publish(wheeltorque_msg)


    def terminate(self):
        self.running = False
        self.motor1.terminate()
        self.motor2.terminate()
        self.motor3.terminate()
        time.sleep(0.1)



def main():
    # 启动串口
    baudrate = 2000000
    port1 = "/dev/omni_motor1"
    port2 = "/dev/omni_motor2"
    port3 = "/dev/omni_motor3"
    ser1 = serial.Serial(port1, baudrate)
    ser2 = serial.Serial(port2, baudrate)
    ser3 = serial.Serial(port3, baudrate)
    logger.info("Motor starting... port:" + port1 + "," + port2 + "," + port3 + " , baudrate:" + str(baudrate))

    # 初始化电机
    motor1 = Motor(ser1, 1)
    motor2 = Motor(ser2, 2)
    motor3 = Motor(ser3, 3)
    motor1.start()
    motor2.start()
    motor3.start()


    # 启动ROS通讯
    motor_node = MotorControlNode(motor1, motor2, motor3)
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
