#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path
import typing as t

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from und_edubot_interfaces.srv import Move


csv_fieldnames = [
    "distance",
    "angle_d",
    "delta_x",
    "delta_y",
    "delta_z",
    "left_wheel_delta",
    "right_wheel_delta",
]


class MovementRecorder(Node):
    def __init__(self) -> None:
        super().__init__("movement_recorder")
        self.client = self.create_client(Move, "/move")

    def move(
        self, distance: float, angle_d: float, timeout_sec: float
    ) -> Move.Response:
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("Timed out waiting for /move service")

        request = Move.Request()
        request.distance = float(distance)
        request.angle_d = float(angle_d)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            raise RuntimeError("Timed out waiting for /move response")

        response = future.result()
        if response is None:
            raise RuntimeError("Move service call failed")

        return response


def parse_args(args: t.List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Call /move and write the movement deltas to a CSV file."
    )
    parser.add_argument("distance", type=float, help="Distance to travel in meters")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default="move_output.csv",
        help="CSV file to append the move result to",
    )
    parser.add_argument(
        "--angle",
        type=float,
        default=0.0,
        help="Turn angle in degrees to send with the move request",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for the service and response",
    )
    return parser.parse_args(args)


def write_result(output_file: Path, row_entry: t.Dict[str, t.Any]) -> None:
    if not output_file.parent.exists():
        output_file.parent.mkdir(parents=True)

    if not output_file.exists() or output_file.stat().st_size == 0:
        with output_file.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=csv_fieldnames)
            writer.writeheader()

    with output_file.open("a", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=csv_fieldnames)
        writer.writerow(row_entry)


def main(args=None) -> None:
    arglist = remove_ros_args(args=args)[1:]
    parsed_args = parse_args(arglist)

    rclpy.init(args=args)
    node = MovementRecorder()

    try:
        response = node.move(
            parsed_args.distance,
            parsed_args.angle,
            parsed_args.timeout,
        )
        if len(response.wheel_delta) != 2:
            raise RuntimeError(
                "Expected left and right wheel deltas in the /move response"
            )

        row_entry = {
            "distance": parsed_args.distance,
            "angle_d": parsed_args.angle,
            "delta_x": response.delta_xy.x,
            "delta_y": response.delta_xy.y,
            "delta_z": response.delta_xy.z,
            "left_wheel_delta": response.wheel_delta[0],
            "right_wheel_delta": response.wheel_delta[1],
        }
        write_result(parsed_args.output, row_entry)
        node.get_logger().info(
            "Wrote move result to "
            f"{parsed_args.output}: delta_xy="
            f"({response.delta_xy.x}, {response.delta_xy.y}, {response.delta_xy.z}), "
            f"wheel_delta=({response.wheel_delta[0]}, {response.wheel_delta[1]})"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
