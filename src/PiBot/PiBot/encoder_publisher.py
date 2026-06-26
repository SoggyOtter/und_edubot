import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import numpy as np
import smbus2
import time

from collections import deque


# Define I2C address and bus
AS5600_ADDR = 0x36
ANGLE_REG = 0x0E


def read_angle_L():
    bus = smbus2.SMBus(4)
    # Read two bytes from the angle register
    raw_data = bus.read_i2c_block_data(AS5600_ADDR, ANGLE_REG, 2)
    angle = (raw_data[0] << 8) | raw_data[1]  # Combine MSB and LSB
    angle = angle & 0x0FFF  # Mask to 12 bits
    return (angle / 4096.0) * 360.0  # Convert to degrees


def read_angle_R():
    bus = smbus2.SMBus(1)
    # Read two bytes from the angle register
    raw_data = bus.read_i2c_block_data(AS5600_ADDR, ANGLE_REG, 2)
    angle = (raw_data[0] << 8) | raw_data[1]  # Combine MSB and LSB
    angle = angle & 0x0FFF  # Mask to 12 bits
    return (angle / 4096.0) * 360.0  # Convert to degrees


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.angular_velocity_cb)
        self.joint_names = ['left', 'right']
        self.ang_L_ring = deque(maxlen=3)
        self.ang_R_ring = deque(maxlen=3)
        
        self.ang_R_prev = read_angle_R()
        self.time_R_prev = time.time()
        self.ang_L_prev = read_angle_L()
        self.time_L_prev = time.time()
        
        
        
        self.prev_time = time.time()
        
        
        self.get_logger().info('Joint State Publisher Node has been started.')
        
    def angular_velocity_cb(self):
        # Note, before publishing, should probably apply a low pass filter on the absolute rate
        try:
            msg = JointState()
            msg.name = self.joint_names
            ang_L = read_angle_L()
            # Calculate difference in degrees
            # do -1 to get robot frame
            now = time.time()
            angular_diff_deg_L = (ang_L - self.ang_L_prev + 180.0) % 360. -180
            angular_vel_deg_L = angular_diff_deg_L / (now - self.time_L_prev)
            
            ang_R = read_angle_R()
            angular_diff_deg_R = -1* (ang_R - self.ang_R_prev + 180.0) % 360. -180
            angular_vel_deg_R = angular_diff_deg_R / (now - self.time_R_prev)
            
            
            self.ang_R_ring.append(angular_vel_deg_R)
            self.ang_L_ring.append(angular_vel_deg_L)
            
            # Update values for calculating velocity and time
            self.ang_L_prev = ang_L
            self.ang_R_prev = ang_R
            # should just use same time, should specify high precision clock for better results
            self.time_L_prev = now
            self.time_R_prev = now
            
            msg.position = [ang_L, ang_R]
            msg.velocity = [angular_vel_deg_L, angular_vel_deg_R]
            self.publisher_.publish(msg)
        
        except OSError as e:
            self.get_logger().warn(f"I2C read failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in encoder_callback: {e}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = JointStatePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# try:
#     while True:
#         angleL = read_angle_L()
#         angleR = read_angle_R()
#         print(f"Angle: {angleL:.2f} degrees")
#         print(f"Angle: {angleR:.2f} degrees")
#         time.sleep(0.05)
# except KeyboardInterrupt:
#     print("Exiting...")
