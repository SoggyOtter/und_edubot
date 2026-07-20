#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from und_edubot_interfaces.srv import Move


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


def parse_args(args) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Call /move and write the returned delta_xy to a CSV file."
    )
    parser.add_argument("distance", type=float, help="Distance to travel in meters")
    parser.add_argument(
        "-o",
        "--output",
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
    return parser.parse_args(remove_ros_args(args=args)[1:])


def write_result(
    path: Path, distance: float, angle_d: float, response: Move.Response
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists() or path.stat().st_size == 0

    with path.open("a", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "distance",
                "angle_d",
                "delta_x",
                "delta_y",
                "delta_z",
            ],
        )
        if write_header:
            writer.writeheader()
        writer.writerow(
            {
                "distance": distance,
                "angle_d": angle_d,
                "delta_x": response.delta_xy.x,
                "delta_y": response.delta_xy.y,
                "delta_z": response.delta_xy.z,
            }
        )


def main(args=None) -> None:
    parsed_args = parse_args(args)

    rclpy.init(args=args)
    node = MovementRecorder()

    try:
        response = node.move(
            parsed_args.distance,
            parsed_args.angle,
            parsed_args.timeout,
        )
        write_result(
            Path(parsed_args.output),
            parsed_args.distance,
            parsed_args.angle,
            response,
        )
        node.get_logger().info(
            "Wrote move result to "
            f"{parsed_args.output}: delta_xy="
            f"({response.delta_xy.x}, {response.delta_xy.y}, {response.delta_xy.z})"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
