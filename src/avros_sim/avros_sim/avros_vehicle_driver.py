"""AVROS Webots vehicle driver — cmd_vel to wheel motors + combined IMU.

Uses direct Robot API motor control (no Driver API / wbu_driver_*).
The Car PROTO exposes front wheel motors and steering motors that can
be controlled directly, avoiding the Driver API's double-stepping issue.

Front wheels: left_front_wheel, right_front_wheel (velocity control)
Steering: left_steer, right_steer (position control)
Rear wheels: passive (no motors, just sensors/brakes)

Note: Webots Car PROTO steering sign is inverted from ROS convention —
positive motor position turns RIGHT, negative turns LEFT. The steering
angle is negated before applying to motors to match ROS cmd_vel convention
(positive angular.z = left turn).
"""

import rclpy
import rclpy.parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from math import atan2

WHEELBASE = 1.23
TRACK_FRONT = 0.9
MAX_STEERING_RAD = 0.489
WHEEL_RADIUS = 0.36  # Car PROTO default wheel radius


class AvrosVehicleDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())

        # Steering motors (position control)
        self.__left_steer = self.__robot.getDevice('left_steer')
        self.__right_steer = self.__robot.getDevice('right_steer')

        # Drive motors (velocity control) — front-wheel drive
        self.__left_motor = self.__robot.getDevice('left_front_wheel')
        self.__right_motor = self.__robot.getDevice('right_front_wheel')

        # Set drive motors to velocity mode
        for motor in [self.__left_motor, self.__right_motor]:
            if motor:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)

        # IMU devices
        self.__inertial_unit = self.__robot.getDevice('imu_inertial')
        self.__gyro = self.__robot.getDevice('imu_gyro')
        self.__accel = self.__robot.getDevice('imu_accel')
        self.__inertial_unit.enable(timestep)
        self.__gyro.enable(timestep)
        self.__accel.enable(timestep)

        rclpy.init(args=None)
        self.__node = rclpy.create_node(
            'avros_vehicle_driver',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', value=True),
            ],
        )
        self.__node.create_subscription(
            Twist, 'cmd_vel', self.__cmd_vel_callback, 1
        )
        self.__imu_pub = self.__node.create_publisher(Imu, '/imu/data', 10)

        self.__speed = 0.0
        self.__steering = 0.0

    def __cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        self.__speed = v

        if abs(v) > 0.01:
            steering_rad = atan2(omega * WHEELBASE, abs(v))
        elif abs(omega) > 0.01:
            steering_rad = MAX_STEERING_RAD if omega > 0 else -MAX_STEERING_RAD
        else:
            steering_rad = 0.0

        steering_rad = max(-MAX_STEERING_RAD,
                           min(MAX_STEERING_RAD, steering_rad))
        self.__steering = steering_rad

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        speed = self.__speed
        steering = self.__steering

        # Negate steering for Webots Car PROTO convention (positive = right)
        steer_cmd = -steering
        if self.__left_steer and self.__right_steer:
            self.__left_steer.setPosition(steer_cmd)
            self.__right_steer.setPosition(steer_cmd)

        # Set wheel velocities (rad/s)
        wheel_vel = speed / WHEEL_RADIUS
        if self.__left_motor:
            self.__left_motor.setVelocity(wheel_vel)
        if self.__right_motor:
            self.__right_motor.setVelocity(wheel_vel)

        # Publish combined IMU
        self.__publish_imu()

    def __publish_imu(self):
        # getQuaternion() returns [x, y, z, w] in Webots ENU frame (R2025a default)
        # which maps directly to ROS ENU convention — no frame conversion needed
        q = self.__inertial_unit.getQuaternion()
        gyro = self.__gyro.getValues()
        accel = self.__accel.getValues()

        msg = Imu()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self.__imu_pub.publish(msg)
