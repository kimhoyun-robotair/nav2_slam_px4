#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageFrameTransformer(Node):
    def __init__(self):
        super().__init__('image_frame_transformer')

        # Color image 구독 및 변환 퍼블리셔
        self.image_sub = self.create_subscription(
            Image, '/camera', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/camera/image', 10)

        # Depth image 구독 및 변환 퍼블리셔
        self.depth_sub = self.create_subscription(
            Image, '/depth_camera', self.depth_callback, 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth_image', 10)

    def image_callback(self, msg):
        new_msg = Image()
        new_msg.header = msg.header
        new_msg.header.frame_id = "camera_link_optical"
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.encoding = msg.encoding
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.step = msg.step
        new_msg.data = msg.data

        self.image_pub.publish(new_msg)

    def depth_callback(self, msg):
        new_msg = Image()
        new_msg.header = msg.header
        new_msg.header.frame_id = "depth_camera_link_optical"
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.encoding = msg.encoding
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.step = msg.step
        new_msg.data = msg.data

        self.depth_pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageFrameTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
