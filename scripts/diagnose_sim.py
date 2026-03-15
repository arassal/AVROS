#!/usr/bin/env python3
"""Diagnostic telemetry script for AVROS Webots simulation.

Subscribes to all key topics and prints a structured report every 2s:
- IMU orientation (yaw in degrees)
- GNSS position
- EKF local + global odometry
- TF map→base_link
- cmd_vel from Nav2

Usage:
    ros2 run avros_sim diagnose_sim  # if installed
    python3 scripts/diagnose_sim.py  # standalone
"""

import math
import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf2_ros


def quat_to_yaw(q):
    """Extract yaw (rad) from quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('avros_diagnostics', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True),
        ])

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        self.imu = None
        self.gnss = None
        self.odom_local = None
        self.odom_global = None
        self.odom_gps = None
        self.cmd_vel = None

        self.create_subscription(Imu, '/imu/data', self._imu_cb, qos)
        self.create_subscription(NavSatFix, '/gnss', self._gnss_cb, qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_local_cb, qos)
        self.create_subscription(Odometry, '/odometry/global', self._odom_global_cb, qos)
        self.create_subscription(Odometry, '/odometry/gps', self._odom_gps_cb, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(2.0, self._report)
        self.get_logger().info('AVROS Diagnostics started — reporting every 2s')

    def _imu_cb(self, msg):
        self.imu = msg

    def _gnss_cb(self, msg):
        self.gnss = msg

    def _odom_local_cb(self, msg):
        self.odom_local = msg

    def _odom_global_cb(self, msg):
        self.odom_global = msg

    def _odom_gps_cb(self, msg):
        self.odom_gps = msg

    def _cmd_vel_cb(self, msg):
        self.cmd_vel = msg

    def _report(self):
        lines = ['\n' + '=' * 60, 'AVROS DIAGNOSTIC REPORT', '=' * 60]

        # IMU
        if self.imu:
            yaw = quat_to_yaw(self.imu.orientation)
            lines.append(f'IMU yaw:       {math.degrees(yaw):+7.1f}° ({yaw:+.4f} rad)')
            lines.append(f'  quat:        x={self.imu.orientation.x:.4f} y={self.imu.orientation.y:.4f} '
                         f'z={self.imu.orientation.z:.4f} w={self.imu.orientation.w:.4f}')
            lines.append(f'  gyro:        x={self.imu.angular_velocity.x:.3f} '
                         f'y={self.imu.angular_velocity.y:.3f} z={self.imu.angular_velocity.z:.3f}')
            lines.append(f'  accel:       x={self.imu.linear_acceleration.x:.3f} '
                         f'y={self.imu.linear_acceleration.y:.3f} z={self.imu.linear_acceleration.z:.3f}')
        else:
            lines.append('IMU:           NO DATA')

        # GNSS
        if self.gnss:
            lines.append(f'GNSS:          lat={self.gnss.latitude:.6f} lon={self.gnss.longitude:.6f} '
                         f'alt={self.gnss.altitude:.1f}')
        else:
            lines.append('GNSS:          NO DATA')

        # Local EKF odometry
        if self.odom_local:
            p = self.odom_local.pose.pose.position
            yaw = quat_to_yaw(self.odom_local.pose.pose.orientation)
            lines.append(f'EKF local:     x={p.x:+8.2f} y={p.y:+8.2f} yaw={math.degrees(yaw):+7.1f}°')
        else:
            lines.append('EKF local:     NO DATA')

        # Global EKF odometry
        if self.odom_global:
            p = self.odom_global.pose.pose.position
            yaw = quat_to_yaw(self.odom_global.pose.pose.orientation)
            lines.append(f'EKF global:    x={p.x:+8.2f} y={p.y:+8.2f} yaw={math.degrees(yaw):+7.1f}°')
        else:
            lines.append('EKF global:    NO DATA')

        # GPS odometry (from navsat_transform)
        if self.odom_gps:
            p = self.odom_gps.pose.pose.position
            yaw = quat_to_yaw(self.odom_gps.pose.pose.orientation)
            lines.append(f'navsat odom:   x={p.x:+8.2f} y={p.y:+8.2f} yaw={math.degrees(yaw):+7.1f}°')
        else:
            lines.append('navsat odom:   NO DATA')

        # TF map → base_link
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            p = t.transform.translation
            yaw = quat_to_yaw(t.transform.rotation)
            lines.append(f'TF map→base:   x={p.x:+8.2f} y={p.y:+8.2f} yaw={math.degrees(yaw):+7.1f}°')
        except Exception:
            lines.append('TF map→base:   NOT AVAILABLE')

        # cmd_vel
        if self.cmd_vel:
            lines.append(f'cmd_vel:       linear.x={self.cmd_vel.linear.x:+.3f} '
                         f'angular.z={self.cmd_vel.angular.z:+.3f}')
        else:
            lines.append('cmd_vel:       NO DATA')

        lines.append('=' * 60)

        # Expected values for car at rotation 0 0 1 0.3439
        lines.append('EXPECTED (spawn): IMU yaw ≈ +19.7° (+0.3439 rad)')
        lines.append('                  GNSS ≈ 34.0593, -117.8221')
        lines.append('                  Map pos ≈ (-107, 5)')
        lines.append('')

        self.get_logger().info('\n'.join(lines))


def main():
    rclpy.init()
    node = DiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
