#!/usr/bin/env python3
"""
Bridge ROS2 geometry_msgs/Twist (base_link-ENU, FLU) → PX4 TrajectorySetpoint (body-NED, FRD).
Both world-frame (ENU↔NED) and body-frame (FLU↔FRD) conversions are done explicitly so that
all sign flips are easy to audit.

Layout
------
• class **TwistToTrajectoryNode - main rclpy.Node
    ├─ _ros_to_frd()   - FLU → FRD (body frame)
    ├─ _frd_to_ned_world() - rotate body-FRD → world-NED using current attitude
    ├─ _ros_yaw_to_px4()   - angular.z conversion (FLU→FRD, ENU→NED)
    └─ timers / subs / pubs as usual
"""

from __future__ import annotations
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import Twist
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude, VehicleLocalPosition

# -----------------------------------------------------------------------------
# Helper – quaternion → yaw (NED convention, yaw about Down ‑Z)
# PX4 attitude msg stores q = (w,x,y,z) already in **FRD‑NED** frame.
# -----------------------------------------------------------------------------

def quat_to_yaw_ned(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TwistToTrajectoryNode(Node):
    """Subscribe /cmd_vel and publish TrajectorySetpoint."""

    def __init__(self):
        super().__init__("twist_to_traj_node")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # internal state -----------------------------------------------------------------
        self._body_yaw_ned = 0.0  # current yaw angle of vehicle in NED (rad)
        self._cmd_lin_flu = np.zeros(3)  # latest linear velocity in FLU/ENU (m/s)
        self._cmd_yaw_flu = 0.0  # latest angular z in FLU/ENU (rad/s)

        # subscribers --------------------------------------------------------------------
        self.create_subscription(Twist, "/cmd_vel", self._twist_cb, 10)
        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._attitude_cb,
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_cb,
            qos,
        )

        # publisher ----------------------------------------------------------------------
        self._pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )

        # timer --------------------------------------------------------------------------
        self.create_timer(0.02, self._publish_setpoint)  # 50 Hz

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------
    def _twist_cb(self, msg: Twist):
        """Store incoming cmd_vel (base_link, FLU, ENU)."""
        self._cmd_lin_flu = np.array(
            [msg.linear.x, msg.linear.y, msg.linear.z], dtype=float
        )
        self._cmd_yaw_flu = msg.angular.z

    def _attitude_cb(self, msg: VehicleAttitude):
        self._body_yaw_ned = quat_to_yaw_ned(msg.q)

    def _local_cb(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    # ---------------------------------------------------------------------
    # Conversion helpers
    # ---------------------------------------------------------------------
    @staticmethod
    def _ros_to_frd(v_flu: np.ndarray) -> np.ndarray:
        """FLU → FRD (body frame sign flips)."""
        #   FLU (x fwd, y left, z up)  →  FRD (x fwd, y right, z down)
        x, y, z = v_flu
        return np.array([x, -y, -z])

    def _frd_to_ned_world(self, v_frd: np.ndarray) -> np.ndarray:
        """Rotate body‑FRD velocity into world‑NED using current yaw."""
        cy = math.cos(self._body_yaw_ned)
        sy = math.sin(self._body_yaw_ned)
        rot = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        return rot @ v_frd

    @staticmethod
    def _ros_yaw_to_px4(yaw_rate_flu: float) -> float:
        """Angular z : +CCW about +Z (FLU) equals +CCW about −Z (FRD).
        The numeric sign is therefore preserved.
        """
        return -yaw_rate_flu

    # ---------------------------------------------------------------------
    # Main loop – build & publish TrajectorySetpoint
    # ---------------------------------------------------------------------
    def _publish_setpoint(self):
        # 1) Body‑frame conversion --------------------------------------------------------
        v_frd = self._ros_to_frd(self._cmd_lin_flu)

        # 2) World‑frame rotation ---------------------------------------------------------
        v_ned = self._frd_to_ned_world(v_frd)

        # 3) Pack PX4 TrajectorySetpoint --------------------------------------------------
        msg = TrajectorySetpoint()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position[:] = [float('nan'), float('nan'), float('nan')]
        msg.acceleration[:] = [float("nan")] * 3
        msg.velocity[:] = v_ned.astype(float)
        msg.yaw = float("nan")  # keep current heading
        msg.yawspeed = self._ros_yaw_to_px4(self._cmd_yaw_flu)

        self._pub.publish(msg)


# --------------------------------------------------------------------------------------
#   main
# --------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
