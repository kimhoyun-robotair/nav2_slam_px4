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
- Remove unused imports
- Modify codes for publishing odometry topic
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np
np.float = float
from tf_transformations import quaternion_about_axis,  quaternion_multiply

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
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',   # 원하는 토픽명으로 변경 가능
            10
        )
        
        self.odom_msg = Odometry()
        self.timer = self.create_timer(0.02, self.pub_odom)

    def loc_pos_callback(self, msg):
        #self.get_logger().info('Loc pos 1 received')
        self.loc_pos = msg


    def att_callback(self, msg):
        #self.get_logger().info('Att 1 received')
        self.att = msg


    def pub_odom(self):
        if self.loc_pos is not None and self.att is not None:
        
            self.odom_msg.header.stamp = self.get_clock().now().to_msg()
            self.odom_msg.header.frame_id = 'map'
            self.odom_msg.child_frame_id = 'base_link'

            # Set the translation from the local position message
            self.odom_msg.pose.pose.position.x = float(self.loc_pos.y)
            self.odom_msg.pose.pose.position.y = float(self.loc_pos.x)
            self.odom_msg.pose.pose.position.z = max(-float(self.loc_pos.z), 0.0)  # Apply z-axis lower limit
            
            # Set orientation (Quaternion)
            q_ENU_1 = self.att.q
            q_NED_1 = quaternion_multiply(q_ENU_1, quaternion_about_axis(np.pi/2, (1, 0, 0)))
            q = Quaternion()
            q.w = q_NED_1[0]
            q.x = q_NED_1[1]
            q.y = -q_NED_1[2]
            q.z = -q_NED_1[3]
            self.odom_msg.pose.pose.orientation = q
            
            # Publish the transform
            self.odom_pub.publish(self.odom_msg)
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