#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.wheel_radius = 0.04  # meters
        self.wheel_base = 0.138  # distance between wheels, meters

        self.left_joint_name = "left"
        self.right_joint_name = "right"

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

    def joint_state_callback(self, msg: JointState):
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            self.get_logger().error("Time Delta is either zero or negative")
            return

        try:
            left_idx = msg.name.index(self.left_joint_name)
            right_idx = msg.name.index(self.right_joint_name)
        except ValueError:
            self.get_logger().warn("Wheel joint names not found in /joint_states")
            return

        # get velocity in rad/s, since I publish degrees
        left_wheel_rad_s = msg.velocity[left_idx] * (math.pi / 180.0)
        right_wheel_rad_s = msg.velocity[right_idx] * (math.pi / 180.0)

        v_left = left_wheel_rad_s * self.wheel_radius
        v_right = right_wheel_rad_s * self.wheel_radius

        linear_velocity = (v_right + v_left) / 2.0
        angular_velocity = (v_right - v_left) / self.wheel_base

        delta_theta = angular_velocity * dt

        # Update position and orientation
        # Use average heading during time interval to get more accurate end position
        self.x += linear_velocity * math.cos(self.theta + delta_theta / 2.0) * dt
        self.y += linear_velocity * math.sin(self.theta + delta_theta / 2.0) * dt
        self.theta += delta_theta

        # Normalize theta to be between -pi and pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.publish_odom(now, linear_velocity, angular_velocity)

    def publish_odom(self, now, linear_velocity, angular_velocity):
        q = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = q

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
