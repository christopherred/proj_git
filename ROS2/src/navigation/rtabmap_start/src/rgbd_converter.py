import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from rtabmap_msgs.msg import RGBDImage
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class RGBDConverter(Node):
    def __init__(self):
        super().__init__('rgbd_converter')
        
        # 订阅realsense的RGBD消息
        self.subscription = self.create_subscription(
            RGBD,
            '/camera/camera/rgbd',
            self.rgbd_callback,
            10)
            
        # 发布RTAB-Map格式的RGBD消息
        self.publisher = self.create_publisher(
            RGBDImage,
            '/camera/camera/rgbd_converted',
            10)
            
        self.bridge = CvBridge()

    def rgbd_callback(self, msg):
        rgbd_msg = RGBDImage()
        
        # 转换时间戳和头信息
        rgbd_msg.header = msg.header
        rgbd_msg.rgb_camera_info = msg.rgb_camera_info
        rgbd_msg.depth_camera_info = msg.depth_camera_info
        
        # 转换图像数据
        rgbd_msg.rgb = msg.rgb
        rgbd_msg.depth = msg.depth
        
        # 发布转换后的消息
        self.publisher.publish(rgbd_msg)

def main(args=None):
    rclpy.init(args=args)
    converter = RGBDConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()