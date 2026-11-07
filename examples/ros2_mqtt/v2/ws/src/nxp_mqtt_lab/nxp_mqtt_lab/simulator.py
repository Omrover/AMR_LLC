"""MQTT-only device model. No ROS imports and no hardware access."""
import argparse
import json
import threading
import time
import uuid

from .mqtt_support import make_client
from .protocol import pwm_command


class Simulator:
    def __init__(self, host, port, prefix, hand_spin):
        self.prefix = prefix.rstrip('/')
        self.hand_spin = hand_spin
        self.boot_id = uuid.uuid4().hex
        self.lock = threading.Lock()
        self.left = self.right = 0.0
        self.deadline = 0.0
        self.client = make_client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.client.connect_async(host, port, keepalive=10)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(f'{self.prefix}/wheel_pwm', qos=0)

    def on_disconnect(self, client, userdata, rc):
        with self.lock:
            self.left = self.right = 0.0
            self.deadline = 0.0

    def on_message(self, client, userdata, msg):
        if msg.retain:
            return
        try:
            left, right, timeout = pwm_command(msg.payload, self.boot_id)
        except ValueError:
            return
        with self.lock:
            self.left, self.right = left, right
            self.deadline = time.monotonic() + timeout / 1000.0

    def run(self):
        self.client.loop_start()
        ticks = 0.0
        start = previous = time.monotonic()
        next_publish = start
        try:
            while True:
                now = time.monotonic()
                dt, previous = now - previous, now
                with self.lock:
                    if now >= self.deadline:
                        self.left = self.right = 0.0
                    left, right = self.left, self.right
                ticks += (120.0 if self.hand_spin else left * 1200.0) * dt
                if now >= next_publish:
                    self.client.publish(
                        f'{self.prefix}/encoder',
                        json.dumps({
                            'ticks': int(ticks), 'uptime_ms': int((now - start) * 1000),
                            'boot_id': self.boot_id,
                        }), qos=0, retain=False)
                    self.client.publish(
                        f'{self.prefix}/applied_pwm',
                        json.dumps({'left': left, 'right': right}),
                        qos=0, retain=False)
                    next_publish = now + 0.1
                time.sleep(0.005)
        except KeyboardInterrupt:
            pass
        finally:
            self.on_disconnect(self.client, None, 0)
            self.client.disconnect()
            self.client.loop_stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=18883)
    parser.add_argument('--prefix', default='class10/sim')
    parser.add_argument('--hand-spin', action='store_true',
                        help='Generate 120 ticks/s independent of simulated PWM')
    args = parser.parse_args()
    Simulator(args.host, args.port, args.prefix, args.hand_spin).run()
