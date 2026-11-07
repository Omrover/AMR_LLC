import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Float64, String

from . import exercises, exercises_solution
from .protocol import encoder_sample


class Monitor(Node):
    def __init__(self):
        super().__init__('nxp_encoder_monitor')
        self.declare_parameter('ticks_per_rev', 600)
        self.declare_parameter('use_solution', False)
        self.declare_parameter('demo_pwm', 0.0)
        self.tpr = self.get_parameter('ticks_per_rev').value
        if self.tpr <= 0:
            raise ValueError('ticks_per_rev must be positive')
        self.exercise = (
            exercises_solution if self.get_parameter('use_solution').value else exercises)
        self.demo = self.get_parameter('demo_pwm').value
        self.previous = None
        self.last_seen = None
        self.todo_reported = False
        self.speed_pub = self.create_publisher(Float64, '/nxp/wheel_rad_s', 10)
        self.pwm_pub = self.create_publisher(
            Float32MultiArray, '/nxp/wheel_pwm', qos_profile_sensor_data)
        self.create_subscription(
            String, '/nxp/encoder_sample', self.sample, qos_profile_sensor_data)
        self.create_timer(1.0, self.health)
        if self.demo != 0.0:
            self.exercise.pwm_pair(self.demo)
            self.create_timer(0.1, self.command)

    def sample(self, msg):
        try:
            current = encoder_sample(msg.data)
        except ValueError:
            return
        self.last_seen = time.monotonic()
        try:
            speed = self.exercise.wheel_speed(self.previous, current, self.tpr)
        except NotImplementedError as exc:
            if not self.todo_reported:
                self.get_logger().warning(str(exc))
                self.todo_reported = True
            speed = None
        self.previous = current
        if speed is not None and math.isfinite(speed):
            self.speed_pub.publish(Float64(data=float(speed)))

    def health(self):
        stale = self.last_seen is None or time.monotonic() - self.last_seen > 1.0
        if stale:
            self.previous = None
            self.get_logger().warning('STALE encoder: no sample within 1 s')

    def command(self):
        # Optional demonstration is deliberately bounded to the first 3 seconds.
        if not hasattr(self, 'demo_start'):
            self.demo_start = time.monotonic()
        value = self.demo if time.monotonic() - self.demo_start < 3.0 else 0.0
        self.pwm_pub.publish(Float32MultiArray(data=self.exercise.pwm_pair(value)))


def main(args=None):
    # Ctrl-C raises KeyboardInterrupt, so cleanup runs on a valid context.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
