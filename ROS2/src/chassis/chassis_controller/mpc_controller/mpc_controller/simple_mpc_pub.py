import numpy as np
from mpc import MPC
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

'''使用当前文件夹中的路径文件x_ref.csv等进行简单的mpc控制，并发布话题/chassis/command/cmd_vel'''

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')  # 初始化 ROS 2 节点
        self.get_logger().info("Initializing MPC Controller Node...")

        # 创建发布器
        self.cmd_vel_publisher = self.create_publisher(Twist, '/chassis/command/cmd_vel', 10)

        # 加载参考路径数据
        self.get_logger().info('Loading reference path data...')
        try:
            self.x_ref = np.loadtxt('/home/robot_code/ros2_ws/src/chassis_controller/mpc_controller/mpc_controller/x_ref.csv', delimiter=',')
            self.y_ref = np.loadtxt('/home/robot_code/ros2_ws/src/chassis_controller/mpc_controller/mpc_controller/y_ref.csv', delimiter=',')
            self.theta_ref = np.loadtxt('/home/robot_code/ros2_ws/src/chassis_controller/mpc_controller/mpc_controller/theta_ref.csv', delimiter=',')
        except Exception as e:
            self.get_logger().error(f"Error loading reference data: {e}")
            raise e

        # 初始化 MPC
        self.get_logger().info('Setting up MPC...')
        self.mpc = MPC()
        self.mpc.set_type(self.mpc.DIFF_CAR)
        self.mpc.set_time(T=1/20, step_len=40)
        self.mpc.set_QR(
            Q=np.array([[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 10.0]]),
            R=np.array([[0.5, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.4]])
        )
        self.mpc.set_max_speed(
            vx_max=1.5,
            vx_min=-0.2,
            vy_max=0.0,
            vy_min=0.0,
            omega_max=1.0,
            omega_min=-1.0
        )

        # 设置初始状态和目标路径
        self.get_logger().info('Configuring MPC optimization...')
        self.mpc.set_opti()
        self.mpc.set_init(np.vstack((self.x_ref, self.y_ref, self.theta_ref)).T[400])
        self.mpc.set_goal(np.vstack((self.x_ref, self.y_ref, self.theta_ref)).T[400:440])

        # 运行 MPC
        self.get_logger().info('Running MPC...')
        try:
            self.state, self.u = self.mpc.run()
            self.get_logger().info(f"Control Commands (u): {self.u}")
        except Exception as e:
            self.get_logger().error(f"Error running MPC: {e}")
            raise e

        # 初始化定时器
        self.get_logger().info('Starting timer to publish control commands...')
        self.u_index = 0
        self.timer = self.create_timer(0.5, self.publish_cmd)  # 每 0.1 秒发布一次

    def publish_cmd(self):
        """发布控制量到 /chassis/command/cmd_vel"""
        if self.u_index >= self.u.shape[1]:
            self.get_logger().info('All control commands have been published.')
            self.timer.cancel()  # 停止定时器
            return

        # 创建并发布 Twist 消息
        cmd = Twist()
        cmd.linear.x = self.u[0, self.u_index]  # 线速度 vx
        cmd.linear.y = self.u[1, self.u_index]  # 线速度 vy
        cmd.angular.z = self.u[2, self.u_index]  # 角速度 omega
        self.cmd_vel_publisher.publish(cmd)

        self.get_logger().info(f"Published Command {self.u_index + 1}: vx={cmd.linear.x:.2f}, vy={cmd.linear.y:.2f}, omega={cmd.angular.z:.2f}")
        self.u_index += 1


def main(args=None):
    """主函数"""
    rclpy.init(args=args)  # 初始化 ROS 2
    node = MPCController()  # 创建节点实例

    try:
        rclpy.spin(node)  # 阻塞执行,使节点持续运行
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user, shutting down...")
    finally:
        node.destroy_node()  # 关闭节点
        rclpy.shutdown()  # 关闭 ROS 2 系统

if __name__ == '__main__':
    main()
