import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from nav_msgs.msg import Odometry
from und_edubot_interfaces.srv import Move

from geometry_msgs.msg import Twist, Vector3
import numpy as np


def position_to_numpy(position) -> np.ndarray:
    return np.array([position.x, position.y, position.z])


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
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_rate = self.create_rate(5)

        self.odom = None

        self.get_logger().info("Motion Controller created")

    def odometry_cb(self, msg: Odometry):
        self.odom = msg

    def move_cb(self, req: Move.Request, res: Move.Response) -> Move.Response:
        self.get_logger().info("Movement requested")

        while self.odom is None:
            self.get_logger().info("Waiting for odometry before moving")
            self.pub_rate.sleep()

        starting_position = position_to_numpy(self.odom.pose.pose.position)

        # Turn
        self.get_logger().info("Turning")
        # Implement later

        # Go forward
        self.get_logger().info("Driving forward")
        while (
            dist_traveled := np.linalg.norm(
                position_to_numpy(self.odom.pose.pose.position) - starting_position
            )
        ) < req.distance:
            twist = Twist()
            twist.linear.x = 1.0
            self.get_logger().info(
                f"Not quite there yet, driving forward, Traveled {dist_traveled} out of {req.distance}"
            )
            self.cmd_vel_pub.publish(twist)
            self.pub_rate.sleep()

        # stop motors
        self.cmd_vel_pub.publish(Twist())

        delta = position_to_numpy(self.odom.pose.pose.position) - starting_position
        res.delta_xy = Vector3(x=float(delta[0]), y=float(delta[1]), z=0.0)

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
