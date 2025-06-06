#!/usr/bin/env python3
"""
ROS 2 node: subscribe to

  • /cmd_vel                        (geometry_msgs/Twist)
  • /fmu/in/trajectory_setpoint     (px4_msgs/TrajectorySetpoint)
  • /fmu/out/vehicle_local_position (px4_msgs/VehicleLocalPosition)

and print  
   1) 현재 cmd_vel (선형 속도)  
   2) TrajectorySetpoint 위치·속도  
   3) VehicleLocalPosition 위치(x,y)·고도(z)

in one compact block every 0.2 s (5 Hz).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


class TopicMonitor(Node):
    def __init__(self):
        super().__init__('px4_topic_monitor')

        # 최신 메시지를 저장할 내부 변수
        self._cmd_vel      = (0.0, 0.0, 0.0)
        self._traj_vel     = (float('nan'),) * 3
        self._traj_pos     = (float('nan'),) * 3
        self._local_pos    = (float('nan'), float('nan'), float('nan'))

        # QoS: PX4 uORB → ROS 2 브리지 기본값과 맞추기
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 구독자 생성 ---------------------------------------------------------
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_cb,
            10,
        )
        self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self._traj_cb,
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._loc_cb,
            qos,
        )

        # 5 Hz 타이머 → 표준출력
        self.create_timer(0.2, self._print_status)

    # 콜백 ---------------------------------------------------------------
    def _cmd_cb(self, msg: Twist):
        self._cmd_vel = (msg.linear.x, msg.linear.y, msg.linear.z)

    def _traj_cb(self, msg: TrajectorySetpoint):
        self._traj_vel = tuple(msg.velocity)
        self._traj_pos = tuple(msg.position)

    def _loc_cb(self, msg: VehicleLocalPosition):
        self._local_pos = (msg.x, msg.y, msg.z)

    # 출력 ---------------------------------------------------------------
    def _print_status(self):
        cv = self._cmd_vel
        tv = self._traj_vel
        tp = self._traj_pos
        lp = self._local_pos

        self.get_logger().info(
            f"\ncmd_vel            : ({cv[0]:5.2f}, {cv[1]:5.2f}, {cv[2]:5.2f}) m/s\n"
            f"trajectory position: ({tp[0]:6.2f}, {tp[1]:6.2f}, {tp[2]:6.2f}) m\n"
            f"trajectory velocity: ({tv[0]:5.2f}, {tv[1]:5.2f}, {tv[2]:5.2f}) m/s\n"
            f"local position     : ({lp[0]:7.2f}, {lp[1]:7.2f}, {lp[2]:7.2f}) m\n"
            + "-" * 46
        )

def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
