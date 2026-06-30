import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gpiozero import Motor


MAX_RPM = 163.0
MAX_RPS = MAX_RPM / 60.0


class VelocityController(Node):
    def __init__(self):
        super().__init__("velocity_controller")

        # Subscriptions
        self.create_subscription(Twist, "/cmd_vel", self.set_motor_speed, 10)
        self.create_subscription(
            JointState,
            "/joint_states",
            self.calculate_error,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Publishers
        self.motor_1_err_pub = self.create_publisher(
            Float64, "motor_1/velocity_error", 10
        )
        self.motor_2_err_pub = self.create_publisher(
            Float64, "motor_2/velocity_error", 10
        )

        # Motors
        self.motor_1 = Motor(25, 18)
        self.motor_2 = Motor(14, 15)

        # Targets
        self.m1vel_target = 0.0
        self.m2vel_target = 0.0

        # PID state
        self.m1_err_int = 0.0
        self.m2_err_int = 0.0
        self.m1_prev_vel = 0.0
        self.m2_prev_vel = 0.0

        self.dvel1_filt = 0.0
        self.dvel2_filt = 0.0

        self.m1vel_pwm = 0.0
        self.m2vel_pwm = 0.0
        self.m1vel_pwm_prev = 0.0
        self.m2vel_pwm_prev = 0.0

        # Timing
        self.last_pid_time = self.get_clock().now()

        # Smoothing
        self.max_pwm_rate = 2.0  # PWM units per second
        self.d_alpha = 0.2  # derivative low-pass
        self.deadband = 0.03

        self.declare_params()

        # Motor update timer
        self.create_timer(1.0 / 20.0, self.do_drive_motor)

    # -----------------------------------------------------

    def declare_params(self):
        self.declare_parameter("motor1.kp", 0.37)
        self.declare_parameter("motor1.ki", 0.15)
        self.declare_parameter("motor1.kd", 0.005)

        self.declare_parameter("motor2.kp", 0.35)
        self.declare_parameter("motor2.ki", 0.15)
        self.declare_parameter("motor2.kd", 0.005)

    # -----------------------------------------------------

    def set_motor_speed(self, msg: Twist):
        self.m1vel_target = msg.linear.x
        self.m2vel_target = msg.linear.x

        if msg.angular.z != 0:
            self.m1vel_target = -msg.angular.z
            self.m2vel_target = msg.angular.z

    # -----------------------------------------------------

    def rate_limit(self, desired, current, dt):
        max_delta = self.max_pwm_rate * dt
        return max(current - max_delta, min(current + max_delta, desired))

    def apply_deadband(self, u):
        if abs(u) < self.deadband:
            return 0.0
        return u

    # -----------------------------------------------------

    def calculate_error(self, msg: JointState):
        now = self.get_clock().now()
        dt = (now - self.last_pid_time).nanoseconds * 1e-9
        self.last_pid_time = now
        if dt <= 0.0:
            return

        vel_L_rps = msg.velocity[0] / 360.0
        vel_R_rps = msg.velocity[1] / 360.0

        err1 = self.m1vel_target - vel_L_rps
        err2 = self.m2vel_target - vel_R_rps

        self.motor_1_err_pub.publish(Float64(data=err1))
        self.motor_2_err_pub.publish(Float64(data=err2))

        # Integral
        self.m1_err_int += err1 * dt
        self.m2_err_int += err2 * dt
        self.m1_err_int = max(-1.0, min(1.0, self.m1_err_int))
        self.m2_err_int = max(-1.0, min(1.0, self.m2_err_int))

        # Derivative (filtered, on measurement)
        raw_d1 = (vel_L_rps - self.m1_prev_vel) / dt
        raw_d2 = (vel_R_rps - self.m2_prev_vel) / dt
        self.m1_prev_vel = vel_L_rps
        self.m2_prev_vel = vel_R_rps

        self.dvel1_filt += self.d_alpha * (raw_d1 - self.dvel1_filt)
        self.dvel2_filt += self.d_alpha * (raw_d2 - self.dvel2_filt)

        # Gains
        kp1 = self.get_parameter("motor1.kp").value
        ki1 = self.get_parameter("motor1.ki").value
        kd1 = self.get_parameter("motor1.kd").value

        kp2 = self.get_parameter("motor2.kp").value
        ki2 = self.get_parameter("motor2.ki").value
        kd2 = self.get_parameter("motor2.kd").value

        # PID output
        pid1 = kp1 * err1 + ki1 * self.m1_err_int - kd1 * self.dvel1_filt
        pid2 = kp2 * err2 + ki2 * self.m2_err_int - kd2 * self.dvel2_filt

        # Rate limit
        pid1 = self.rate_limit(pid1, self.m1vel_pwm_prev, dt)
        pid2 = self.rate_limit(pid2, self.m2vel_pwm_prev, dt)

        # Deadband
        self.m1vel_pwm = self.apply_deadband(pid1)
        self.m2vel_pwm = self.apply_deadband(pid2)

        # Clamp
        self.m1vel_pwm = max(-1.0, min(1.0, self.m1vel_pwm))
        self.m2vel_pwm = max(-1.0, min(1.0, self.m2vel_pwm))

        self.m1vel_pwm_prev = self.m1vel_pwm
        self.m2vel_pwm_prev = self.m2vel_pwm

    # -----------------------------------------------------

    def do_drive_motor(self):
        # basically if both are 0, we should exit before processing PWM
        if self.m1vel_target == self.m2vel_target == 0:
            self.motor_1.forward(0)
            self.motor_2.forward(0)
            return

        if self.m1vel_pwm >= 0:
            self.motor_1.forward(abs(self.m1vel_pwm))
        else:
            self.motor_1.backward(abs(self.m1vel_pwm))

        if self.m2vel_pwm >= 0:
            self.motor_2.forward(abs(self.m2vel_pwm))
        else:
            self.motor_2.backward(abs(self.m2vel_pwm))


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
