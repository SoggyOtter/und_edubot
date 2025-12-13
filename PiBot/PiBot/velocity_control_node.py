import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gpiozero import Motor
from time import sleep

MAX_RPM = 163.0
MAX_RPS = MAX_RPM / 60.0  # ≈ 2.7167 rev/s


class VelocityController(Node):
    def __init__(self):
        super().__init__("node")
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.set_motor_speed, 10
        )
        
        self.motor_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.velocity_sub = self.create_subscription(JointState, "/joint_states", self.calculate_error, 10, callback_group=self.motor_cb_group)
        self.motor_1_err_pub = self.create_publisher(Float64, "motor_1/velocity_error", 10)
        self.motor_2_err_pub = self.create_publisher(Float64, "motor_2/velocity_error", 10)
        
        self.motor_1 = Motor(25, 18)
        self.motor_2 = Motor(14, 15)
        
        self.err_m1 = 0
        self.err_m2 = 0
        
        self.m1vel_pwm = 0
        self.m1vel_target = 0
        self.m2vel_pwm = 0
        self.m2vel_target = 0
        
        self.m1_err_int = 0.0
        self.m2_err_int = 0.0

        self.m1_prev_vel = 0.0
        self.m2_prev_vel = 0.0

        self.last_pid_time = self.get_clock().now()
        
        # set speed every 1 second
        self.set_motor_speed_timer = self.create_timer(1./20., self.do_drive_motor)
        self.declare_params()
    
    def declare_params(self):
        self.declare_parameter("motor1.kp", 0.37)
        self.declare_parameter("motor1.ki", 0.001)
        self.declare_parameter("motor1.kd", 0.00001)

        self.declare_parameter("motor2.kp", 0.35)
        self.declare_parameter("motor2.ki", 0.001)
        self.declare_parameter("motor2.kd", 0.00001)

        
    
    def calculate_error(self, msg: JointState):
        now = self.get_clock().now()
        dt = (now - self.last_pid_time).nanoseconds * 1e-9
        self.last_pid_time = now

        if dt <= 0.0:
            return

        # --- Measured velocity (rev/s) ---
        vel_L_rps = msg.velocity[0] / 360.0
        vel_R_rps = msg.velocity[1] / 360.0

        # --- Errors ---
        err1 = self.m1vel_target - vel_L_rps
        err2 = self.m2vel_target - vel_R_rps

        # --- Integral ---
        self.m1_err_int += err1 * dt
        self.m2_err_int += err2 * dt

        # anti-windup (integrator clamp)
        self.m1_err_int = max(-1.0, min(1.0, self.m1_err_int))
        self.m2_err_int = max(-1.0, min(1.0, self.m2_err_int))

        # --- Derivative (on measurement) ---
        dvel1 = (vel_L_rps - self.m1_prev_vel) / dt
        dvel2 = (vel_R_rps - self.m2_prev_vel) / dt

        self.m1_prev_vel = vel_L_rps
        self.m2_prev_vel = vel_R_rps

        # --- Gains ---
        kp1 = self.get_parameter("motor1.kp").value
        ki1 = self.get_parameter("motor1.ki").value
        kd1 = self.get_parameter("motor1.kd").value

        kp2 = self.get_parameter("motor2.kp").value
        ki2 = self.get_parameter("motor2.ki").value
        kd2 = self.get_parameter("motor2.kd").value

        # --- PID Output (PWM) ---
        self.m1vel_pwm = (
            kp1 * err1 +
            ki1 * self.m1_err_int -
            kd1 * dvel1
        )

        self.m2vel_pwm = (
            kp2 * err2 +
            ki2 * self.m2_err_int -
            kd2 * dvel2
        )

        # --- Clamp output ---
        self.m1vel_pwm = max(-1.0, min(1.0, self.m1vel_pwm))
        self.m2vel_pwm = max(-1.0, min(1.0, self.m2vel_pwm))

        # --- Debug ---
        self.motor_1_err_pub.publish(Float64(data=err1))
        self.motor_2_err_pub.publish(Float64(data=err2))

            

    def do_drive_motor(self):
        # self.motor_2.forward()
        self.get_logger().info(f"PWMs are M1: {self.m1vel_pwm}, M2:{self.m2vel_pwm}")
        if abs(self.m1vel_pwm) < .1:
            self.motor_1.forward(0)
        elif self.m1vel_pwm >= 0:
            self.motor_1.forward(min(1.0,abs(self.m1vel_pwm)))
        else:
            self.motor_1.backward(min(1.0, abs(self.m1vel_pwm)))
            
        if abs(self.m2vel_pwm) < .1:
            self.motor_2.forward(0)
        elif self.m2vel_pwm >= 0:
            self.motor_2.forward(min(1.0, abs(self.m2vel_pwm)))
        else:
            self.motor_2.backward(min(1.0,abs(self.m2vel_pwm)))
        
    def set_motor_speed(self, msg: Twist):
        linearvel = msg.linear.x
        
        angularvel = msg.angular.z
        if not linearvel == 0:
            self.m1vel_target = linearvel
            self.m2vel_target = linearvel
        elif not angularvel == 0:
            self.m1vel_target = -angularvel
            self.m2vel_target = angularvel



def main(args=None):
    rclpy.init(args=args)

    controller = VelocityController()
    rclpy.spin(controller)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
