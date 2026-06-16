#import os
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import typing as t

from .utils import write_to_csv, write_to_npz
import numpy as np

from pathlib import Path

# TODO[s]
# add option to save deg
# add option to save as x,y
import queue

class ScanSaver(Node):
    def __init__(self):
        super().__init__("scan_saver")
        self.save_scan_server = self.create_service(Trigger, "save_scan", self.save_scan)
        self._set_last_scan_sub = self.create_subscription(LaserScan, "scan", callback=self.set_last_scan, qos_profile=qos_profile_sensor_data)
        
        # implementing a service for this is technically cleaner
        self.declare_parameter("output_directory", value="output")
        self.declare_parameter("format", value="csv")
        self.declare_parameter("num_scans", value=1 )
        self.num_scans = self.get_parameter('num_scans').value
        self.format = self.get_parameter('format').value
        # self.get_logger().info(f"constructing a queue of {self.num_scans}")
        # self._last_scan: queue.Queue[LaserScan] = queue.Queue(self.num_scans)
        self._last_scan: t.Optional[LaserScan] = None

        
    @property
    def output_dir(self) -> t.Tuple[Path, int]:
        out_dir = Path(self.get_parameter("output_directory").value)
        self.get_logger().info(f"output dir is {out_dir}")
        # this will initialize the directory and scan count
        
        # every single time we will check the number of .csv in the dir, since this is on a manual trigger cmd
        
        if not out_dir.exists():
            out_dir.mkdir(parents=True)
            
        self.get_logger().info(f"output dir is {out_dir}")

        scan_count = len(list(out_dir.glob(f"laser_scan_[0-9]*.{self.format}")))
        return (out_dir, scan_count)
    
    
    def save_scan(self, _: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self._last_scan is None:
            # note, should check if topic even exists
            self.get_logger().error("Could not save a scan becase we do not have any scan to save, try running lidar driver")
            res.success = False
            return res

        self.format = self.get_parameter("format").value
        acceptable_formats = ["csv"]
        # acceptable_formats = ["csv", "npz"]
        if self.format not in acceptable_formats:
            res.success = False
            err_msg = f"Format type {self.format} not supported. Supported types are {acceptable_formats}"
            res.message = err_msg
            self.get_logger().warn(err_msg)
            return res

        # this will handle some more complex bs
        # should update
        out_dir, scan_count = self.output_dir 
        filename = out_dir/ f"laser_scan_{scan_count}.{self.format}"

        if self.format == "csv":
            self._save_csv(filename)
        else:
            write_to_npz(self._last_scan, filename)
        
        # write_to_csv(self._last_scan, filename)
        # self.get_logger().info(f"Saved scan of length {self._last_scan.qsize()} to {filename}")
        self.get_logger().info(f"Saved scan to {filename}")

        res.success = True
        return res
        
    def set_last_scan(self, msg: LaserScan):
        self._last_scan = msg
    
    def _save_csv(self, filename):
        # Saving only the last scan is supported for csv right now
        scan = self._last_scan
        write_to_csv(scan, filename)
        
        
def main(args = None):
    rclpy.init(args=args)
    
    scan_saver = ScanSaver()
    rclpy.spin(scan_saver)
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    