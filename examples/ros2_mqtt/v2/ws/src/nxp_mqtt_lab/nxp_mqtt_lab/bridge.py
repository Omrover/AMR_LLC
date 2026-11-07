import json
import queue
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Int64, String

from .mqtt_support import make_client
from .protocol import encode_command, encoder_sample


class Bridge(Node):
    def __init__(self):
        super().__init__('nxp_mqtt_bridge')
        for name, value in (
            ('broker_host', '127.0.0.1'), ('broker_port', 18883),
            ('mqtt_prefix', 'class10/sim'), ('allow_pwm', False),
        ):
            self.declare_parameter(name, value)
        self.prefix = self.get_parameter('mqtt_prefix').value.rstrip('/')
        if not self.prefix or '+' in self.prefix or '#' in self.prefix:
            raise ValueError('mqtt_prefix must be a nonempty literal topic prefix')
        self.allow_pwm = self.get_parameter('allow_pwm').value
        self.samples = queue.Queue(maxsize=32)
        self.boot_id = None
        self.last_sample = None
        self.last_command = None
        self.pending_command = None
        self.bad_packets = 0
        self.ticks_pub = self.create_publisher(
            Int64, '/nxp/encoder_ticks', qos_profile_sensor_data)
        self.sample_pub = self.create_publisher(
            String, '/nxp/encoder_sample', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/nxp/status', 10)
        self.create_subscription(
            Float32MultiArray, '/nxp/wheel_pwm', self.command,
            qos_profile_sensor_data)
        self.client = make_client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect_async(
            self.get_parameter('broker_host').value,
            self.get_parameter('broker_port').value, keepalive=10)
        self.client.loop_start()
        self.create_timer(0.02, self.drain)
        self.create_timer(0.1, self.send_command)
        self.create_timer(1.0, self.status)
        self.get_logger().info(
            f'MQTT {self.prefix}; PWM forwarding={self.allow_pwm}')

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(f'{self.prefix}/encoder', qos=0)

    def on_message(self, client, userdata, msg):
        if msg.retain:
            return
        try:
            sample = encoder_sample(msg.payload)
        except ValueError:
            self.bad_packets += 1
            return
        try:
            self.samples.put_nowait((time.monotonic(), sample))
        except queue.Full:
            pass

    def drain(self):
        while True:
            try:
                arrived, sample = self.samples.get_nowait()
            except queue.Empty:
                return
            if time.monotonic() - arrived > 0.5:
                continue
            if self.boot_id != sample['boot_id']:
                self.pending_command = None
                self.last_command = None
            self.boot_id = sample['boot_id']
            self.last_sample = arrived
            self.ticks_pub.publish(Int64(data=sample['ticks']))
            self.sample_pub.publish(String(data=json.dumps(sample)))

    def command(self, msg):
        if not self.allow_pwm:
            return
        try:
            if len(msg.data) != 2:
                raise ValueError('expected [left, right]')
            encode_command(*msg.data, self.boot_id)
        except ValueError as exc:
            self.pending_command = None
            self.last_command = None
            self.get_logger().warning(str(exc))
            return
        self.pending_command = tuple(msg.data)
        self.last_command = time.monotonic()

    def send_command(self):
        if not self.allow_pwm or self.boot_id is None:
            return
        now = time.monotonic()
        fresh = (
            self.client.is_connected() and self.last_sample is not None
            and now - self.last_sample < 0.5
            and self.last_command is not None and now - self.last_command < 0.2)
        values = self.pending_command if fresh else None
        if values is None:
            self.pending_command = None
            self.last_command = None
        payload = encode_command(*(values or (0.0, 0.0)), self.boot_id)
        self.client.publish(f'{self.prefix}/wheel_pwm', payload, qos=0, retain=False)

    def status(self):
        age = None if self.last_sample is None else time.monotonic() - self.last_sample
        state = 'LIVE' if age is not None and age < 1.0 else 'STALE'
        self.status_pub.publish(String(data=json.dumps({
            'state': state, 'mqtt_connected': self.client.is_connected(),
            'age_s': None if age is None else round(age, 2),
            'invalid_packets': self.bad_packets,
        })))

    def close(self):
        if self.allow_pwm and self.boot_id is not None:
            info = self.client.publish(
                f'{self.prefix}/wheel_pwm',
                encode_command(0.0, 0.0, self.boot_id), qos=0, retain=False)
            try:
                info.wait_for_publish(timeout=0.2)
            except RuntimeError:
                pass
        self.client.disconnect()
        self.client.loop_stop()


def main(args=None):
    # Ctrl-C raises KeyboardInterrupt, so cleanup runs on a valid context.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
