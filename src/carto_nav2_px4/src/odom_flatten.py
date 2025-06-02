#!/usr/bin/env python3
# odom_flatten_node.py
#
#  /odom  (6-DoF) -------------------->  /filtered_odom  (2-D)
#
#       z, roll, pitch  모두 0 으로 고정
#
#  ROS 2 Foxy‒Humble 호환

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class OdomFlattenNode(Node):
    def __init__(self):
        super().__init__('odom_flatten_node')

        # QoS: Odometry는 대부분 best-effort 이므로 기본값 사용
        self.sub = self.create_subscription(
            Odometry, '/drone_odom', self.cb_odom, 10)

        self.pub = self.create_publisher(
            Odometry, '/odom', 10)

        self.get_logger().info('🟢 Odom flatten node started')

    # ─────────────────────────────────────────
    def cb_odom(self, msg: Odometry):
        # 1) Pose 복사 후 z·roll·pitch 제거
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        q_flat = quaternion_from_euler(0.0, 0.0, yaw)

        odom2d = Odometry()
        odom2d.header          = msg.header       # 시간·frame_id 그대로
        odom2d.child_frame_id  = msg.child_frame_id or 'base_footprint'

        # 위치 (x, y) 유지, z = 0
        odom2d.pose.pose.position.x = msg.pose.pose.position.x
        odom2d.pose.pose.position.y = msg.pose.pose.position.y
        odom2d.pose.pose.position.z = 0.0

        odom2d.pose.pose.orientation.x = q_flat[0]
        odom2d.pose.pose.orientation.y = q_flat[1]
        odom2d.pose.pose.orientation.z = q_flat[2]
        odom2d.pose.pose.orientation.w = q_flat[3]

        # 2) Twist 복사 후 z·roll·pitch 속도 제거
        odom2d.twist.twist.linear.x  = msg.twist.twist.linear.x
        odom2d.twist.twist.linear.y  = msg.twist.twist.linear.y
        odom2d.twist.twist.linear.z  = 0.0

        odom2d.twist.twist.angular.x = 0.0     # roll 속도 제거
        odom2d.twist.twist.angular.y = 0.0     # pitch 속도 제거
        odom2d.twist.twist.angular.z = msg.twist.twist.angular.z  # yaw만 유지

        # (선택) z·roll·pitch 공분산값을 크게 만들어 Planner가 무시하도록 할 수도 있음
        # idx: 0-5 x,y,z,r,p,y
        # odom2d.pose.covariance[14] = 1e3   # z
        # odom2d.pose.covariance[21] = 1e3   # roll
        # odom2d.pose.covariance[28] = 1e3   # pitch

        self.pub.publish(odom2d)

# ───────────────────────────────────────────────
def main():
    rclpy.init()
    node = OdomFlattenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
