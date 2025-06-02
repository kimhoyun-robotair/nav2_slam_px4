#!/usr/bin/env python3

"""
Copyright 2024 Lorenzo Grandi

GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007

Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
Everyone is permitted to copy and distribute verbatim copies
of this license document, but changing it is not allowed.

**Modifications**
- Remove 2nd PX4 TF functions
- Modify the TF frame names: map -> odom
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitude, VehicleGlobalPosition
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np
np.float = float
from tf_transformations import (
    quaternion_about_axis, quaternion_multiply,
    quaternion_from_euler, euler_from_quaternion
)

class OdometryToTransformNode(Node):
    def __init__(self):
        super().__init__('odometry_to_transform_node')

        self.loc_pos = None
        self.att = None

        self.qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        
        # Create a subscriber to the /px4_1/fmu/out/vehicle_local_position topic
        self.loc_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.loc_pos_callback,
            self.qos)


        # Create a subscriber to the /px4_1/fmu/out/vehicle_attitude topic
        self.att_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.att_callback,
            self.qos)
        
        # Create TransformStamped message
        self.t1 = TransformStamped()

        # Create TransformBroadcaster to publish transforms
        self.tf_broadcaster_1 = tf2_ros.TransformBroadcaster(self)

    def loc_pos_callback(self, msg):
        #self.get_logger().info('Loc pos 1 received')
        self.loc_pos = msg
        self.pub_transf()


    def att_callback(self, msg):
        #self.get_logger().info('Att 1 received')
        self.att = msg
        self.pub_transf()


    def pub_transf(self):
        if self.loc_pos is not None and self.att is not None:
        
            self.t1.header.stamp = self.get_clock().now().to_msg()
            self.t1.header.frame_id = 'x500_rtab_0/odom'
            self.t1.child_frame_id = 'x500_rtab_0/base_footprint'

            # Set the translation from the local position message
            self.t1.transform.translation.x = float(self.loc_pos.y)
            self.t1.transform.translation.y = float(self.loc_pos.x)
            self.t1.transform.translation.z = 0.0  # Apply z-axis lower limit
            
            #   a. PX4(ENU) → NED 변환
            q_enu = self.att.q
            q_ned = quaternion_multiply(
                q_enu, quaternion_about_axis(np.pi/2, (1, 0, 0)))

            #   b. Euler 로 변환 후 roll·pitch 제거
            roll, pitch, yaw = euler_from_quaternion(q_ned)
            q_flat = quaternion_from_euler(0.0, 0.0, yaw)

            self.t.transform.rotation.x = q_flat[0]
            self.t.transform.rotation.y = q_flat[1]
            self.t.transform.rotation.z = q_flat[2]
            self.t.transform.rotation.w = q_flat[3]
            
            # Publish the transform
            self.tf_broadcaster_1.sendTransform(self.t1)
            #self.get_logger().info("Map BaseLink transform 1 published")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryToTransformNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()