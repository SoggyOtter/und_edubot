# This node will calculate the variance of a single point in the scan data. I.e. take x number of measurements for a single point, then publish the variance

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from tf2_msgs.msg import TFMessage


import typing as t
import numpy as np
import argparse

import csv
from pathlib import Path
from dataclasses import dataclass

@dataclass
class Sample:
    scans: np.ndarray
    count: int = 0

    def add_measurement(self, value: float):
        # if we have one to many scans, that's ok
        self.scans[self.count % len(self.scans)] = value
        self.count += 1

def find_nearest_pt(scan: np.ndarray, angle=0.0):
    angles = scan[0, :]
    nearest_angle_idx = np.argmin(np.abs(angles - angle))
    return scan[:, nearest_angle_idx]


def parse_args(args: t.List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("range", type=float, help="Range where samples were taken")
    parser.add_argument("num_scans", type=int, default=100, help="Number of scans to use for variance calculation")
    parser.add_argument("--angle", type=float, default=0.0, help="Angle to measure variance for")
    parser.add_argument("--output", type=Path, default="variance.csv", help="Output file for the variance data")

    return parser.parse_args(args)

csv_fieldnames = ["range", "angle", "num_scans", "variance", "avg_distance"]

def write_variance(output_file: Path, row_entry: t.Dict[str, t.Any]):
    # initialize the output file
    if not output_file.parent.exists():
        output_file.parent.mkdir(parents=True)

    if not output_file.exists():
        with open(output_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=csv_fieldnames)
            writer.writeheader()
        
    with open(output_file, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=csv_fieldnames)
        writer.writerow(row_entry)


class VarianceNode(Node):
    def __init__(self, num_scans: int, angle: float):
        super().__init__("variance_node")
        # Initialize any necessary parameters or subscribers here
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Define QoS matching /tf_static (Transient Local)
        static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.tf_subscription = self.create_subscription(
            TFMessage,
            "tf_static",
            self.tf_callback,
            static_qos
        )
    
        # publish distance as if it were a time-of-flight sensor just to make debuggin easier
        self.distance_publisher = self.create_publisher(Float64, "distance", 10)
        
        # transformation from laser frame to body frame, so that we select the correct point
        self.transform: t.Optional[TFMessage] = None

        self.get_logger().info("Variance node started")

        self.sample = Sample(scans = np.zeros(num_scans))



    def scan_callback(self, msg: LaserScan):
        # Process the scan data and calculate variance
        # Need to transform into the body frame
        
        if self.transform is None:
            self.get_logger().warn("Transform is not set yet", throttle_duration_sec=2.0)
            return
        
        # update angles w.r.t the ranges in base_link frame
        # technically, we can use np.roll to roll the array in half, which would result in the desired behavior, I think
        # should use actual TF at some point
        angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))])
        angles = np.roll(angles, len(angles) // 2)

        scan = np.array([angles, msg.ranges])
        self.get_logger().info(f"scan shape is {scan.shape}", once=True)
        
        # find the point that corresponds to front of robot
        self.get_logger().info(f"angles are {angles}", once=True)
        pt_in_front = find_nearest_pt(scan, angle=0.0)


        distance = Float64()
        # need to select appropriate range
        distance.data =  pt_in_front[1] # get range 
        self.distance_publisher.publish(distance) # should reject np.inf readings

        # reject np.inf readings
        if np.isinf(distance.data):
            self.get_logger().warn("Inf distance detected, skipping", throttle_duration_sec=2.0)
            return

        self.sample.add_measurement(distance.data)
                
    def tf_callback(self, msg: TFMessage):
        self.get_logger().info("Transform received")
        # Process the transform data
        self.transform = msg


def main(args=None):
    rclpy.init(args=args)

    arglist = remove_ros_args(args)[1:]

    args = parse_args(arglist)
    node = VarianceNode(num_scans = args.num_scans, angle = args.angle)
    while(node.sample.count < args.num_scans):
        rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f"Current count: {node.sample.count}", throttle_duration_sec=1.0)
    
    variance = node.sample.scans.var()
    print(f"Variance of the selected point: {variance}")

    row_entry = {
        "range": args.range,
        "angle": args.angle,
        "num_scans": args.num_scans,
        "variance": f"{variance:.12f}",
        "avg_distance": f"{node.sample.scans.mean():.12f}"
    }
    write_variance(args.output, row_entry)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()