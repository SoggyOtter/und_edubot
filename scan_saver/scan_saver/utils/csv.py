import csv
import typing as t
from queue import Queue
from pathlib import Path

from sensor_msgs.msg import LaserScan
import numpy as np

def write_to_csv(scan: LaserScan, output_file: t.Union[Path, str]):
    if isinstance(output_file, str):
        output_file = Path(output_file)
    
    # if output_file.suffix == 'csv':
        # raise RuntimeError(f"We do not support saving to the file type {output_file}")


    with open(output_file, "w", newline='') as csvfile:
            writer = csv.writer(csvfile)
            # technically, this should be calculated using max-min, and start at min+step
            for i,range in enumerate(scan.ranges):
                writer.writerow([scan.angle_min + i*scan.angle_increment, range])

def write_to_npz(scans: t.Union[Queue[LaserScan], t.Sequence[LaserScan]], output_file: t.Union[Path, str]):
    if isinstance(output_file, str):
        output_file = Path(output_file)
    
    if isinstance(scans, Queue):
        num_scans = scans.qsize()
        num_points = len(scans.queue[0].ranges)
    else:
        num_scans = len(scans)
        num_points = len(scan[0].ranges)
    
    # Get array of (num_scans, len(scan))
    output_array = np.zeros((num_scans, num_points))
    for i, scan in enumerate(scans.queue):
        output_array[i] = np.asarray(scan.ranges)
    
    np.savez_compressed(output_file, output_array)
        
    