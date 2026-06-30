import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from nav_msgs.msg import Odometry
from und_edubot_interfaces.srv import Move

from geometry_msgs.msg import Twist, Vector3
import numpy as np


def vector32Numpy(vector: Vector3) -> np.ndarray:
    # note that z doesn't actually matter when
    # A. we don't actually ever update z for positin
    # this is a ground robot, and there is no real way for it to intentionally manipulate its z position
    return np.array([vector.x, vector.y, vector.z])


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller")

        self.declare_parameter("wheel_radius", 0.04)
        self.declare_parameter("wheel_base", 0.138)

        self.odometry_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odometry_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.movement_server = self.create_service(
            Move, "/move", self.move_cb, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_rate = self.create_rate(5)

        self.odom = None

        self.get_logger().info("Motion Controller created")

    def odometry_cb(self, msg: Odometry):
        self.odom = msg

    def move_cb(self, req: Move.Request, res: Move.Response) -> Move.Response:
        self.get_logger().info("Movement requested")

        # For now, assume that this is a copy
        starting_pose = self.odom.pose.pose

        # Turn
        self.get_logger().info("Turning")
        # Implement later

        # Go forward
        self.get_logger().info("Driving forward")
        while (
            (dist_traveled := np.linalg.norm(
                vector32Numpy(self.odom.pose.pose.position)
                - vector32Numpy(starting_pose.position)
            ))
            < req.distance
        ):
            twist = Twist()
            twist.linear.x = 1.0
            self.get_logger().info(
                f"Not quite there yet, driving forward, Traveled {dist_traveled} out of {req.distance}"
            )
            self.pub_rate.sleep()

        # Should calculate error from an expected ending distance

        return res


def main(args=None):
    rclpy.init(args=args)

    motion_controller = MotionController()
    executor = MultiThreadedExecutor()

    executor.add_node(motion_controller)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        motion_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
