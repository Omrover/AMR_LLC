"""Opt-in real broker + ROS processes. No mocks, robot or shared broker."""
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('RUN_MQTT_INTEGRATION') != '1',
    reason='Set RUN_MQTT_INTEGRATION=1 for broker/ROS integration',
)


def test_live_bridge_speed_loss_reboot_and_pwm(tmp_path):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float32MultiArray, Float64, Int64, String
    from nxp_mqtt_lab.mqtt_support import make_client
    from nxp_mqtt_lab.protocol import encode_command

    broker = os.environ.get('MOSQUITTO_BIN') or shutil.which('mosquitto')
    assert broker, 'Install mosquitto or set MOSQUITTO_BIN'
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    conf = tmp_path / 'mosquitto.conf'
    conf.write_text(f'listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n')
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '179'
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = env['ROS_AUTOMATIC_DISCOVERY_RANGE']
    processes = []
    logs = []
    client = None
    node = None

    def start(name, argv):
        logfile = (tmp_path / f'{name}.log').open('w+')
        process = subprocess.Popen(
            argv, env=env, stdout=logfile, stderr=subprocess.STDOUT)
        processes.append(process)
        logs.append(logfile)
        return process

    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)

    def program(module, *args):
        return [sys.executable, '-c',
                f'from nxp_mqtt_lab.{module} import main; main()', *args]

    def wait_for(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0.03)
            else:
                time.sleep(0.03)
        for log in logs:
            log.flush()
        details = '\n'.join(p.read_text() for p in tmp_path.glob('*.log'))
        pytest.fail(f'Condition timed out\n{details}')

    def broker_ready():
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=0.1):
                return True
        except OSError:
            return False

    try:
        start('broker', [broker, '-c', str(conf)])
        wait_for(broker_ready)
        applied, mqtt_samples = [], []
        client = make_client()

        def on_connect(client, userdata, flags, rc):
            client.subscribe('class10/test/#')

        def on_message(client, userdata, msg):
            if msg.topic.endswith('/applied_pwm'):
                applied.append(json.loads(msg.payload))
            elif msg.topic.endswith('/encoder'):
                try:
                    value = json.loads(msg.payload)
                    if 'boot_id' in value:
                        mqtt_samples.append(value)
                except ValueError:
                    pass

        client.on_connect, client.on_message = on_connect, on_message
        client.connect('127.0.0.1', port, keepalive=10)
        client.loop_start()
        wait_for(client.is_connected)
        rclpy.init()
        node = rclpy.create_node('class10_integration_observer')
        ticks, speeds, statuses = [], [], []
        node.create_subscription(
            Int64, '/nxp/encoder_ticks', lambda msg: ticks.append(msg.data),
            qos_profile_sensor_data)
        node.create_subscription(
            Float64, '/nxp/wheel_rad_s', lambda msg: speeds.append(msg.data), 10)
        node.create_subscription(
            String, '/nxp/status', lambda msg: statuses.append(json.loads(msg.data)), 10)
        pub = node.create_publisher(
            Float32MultiArray, '/nxp/wheel_pwm', qos_profile_sensor_data)
        sim = start('sim-hand', program(
            'simulator', '--port', str(port), '--prefix', 'class10/test', '--hand-spin'))
        bridge = start('bridge-readonly', program(
            'bridge', '--ros-args', '-p', f'broker_port:={port}',
            '-p', 'mqtt_prefix:=class10/test'))
        monitor = start('monitor', program(
            'monitor', '--ros-args', '-p', 'use_solution:=true'))
        wait_for(lambda: len(ticks) >= 10 and len(speeds) >= 5)
        assert ticks[-1] > ticks[0]
        assert speeds[-1] == pytest.approx(1.2566, abs=0.25)
        old_boot = mqtt_samples[-1]['boot_id']
        # Default bridge must not forward wheel commands.
        until = time.monotonic() + 0.5
        while time.monotonic() < until:
            pub.publish(Float32MultiArray(data=[0.1, 0.1]))
            rclpy.spin_once(node, timeout_sec=0.03)
        assert all(value['left'] == 0 for value in applied)
        client.publish('class10/test/encoder', b'not-json', retain=False)
        wait_for(lambda: any(s.get('invalid_packets', 0) > 0 for s in statuses))
        stop(sim)
        wait_for(lambda: statuses and statuses[-1]['state'] == 'STALE')
        stop(bridge)
        sim = start('sim-pwm', program(
            'simulator', '--port', str(port), '--prefix', 'class10/test'))
        bridge = start('bridge-pwm', program(
            'bridge', '--ros-args', '-p', f'broker_port:={port}',
            '-p', 'mqtt_prefix:=class10/test', '-p', 'allow_pwm:=true'))
        wait_for(lambda: mqtt_samples[-1]['boot_id'] != old_boot)
        wait_for(lambda: statuses[-1]['state'] == 'LIVE')
        until = time.monotonic() + 1
        while time.monotonic() < until:
            pub.publish(Float32MultiArray(data=[0.1, 0.1]))
            rclpy.spin_once(node, timeout_sec=0.03)
        wait_for(lambda: applied[-1]['left'] == pytest.approx(0.1))
        # Stop just the command source: bridge must zero after its host timeout.
        wait_for(lambda: applied[-1]['left'] == 0.0, timeout=2)
        stop(monitor)
        start('monitor-demo', program(
            'monitor', '--ros-args', '-p', 'use_solution:=true',
            '-p', 'demo_pwm:=0.1'))
        wait_for(lambda: applied[-1]['left'] == pytest.approx(0.1))
        wait_for(lambda: applied[-1]['left'] == 0.0, timeout=5)
        until = time.monotonic() + 0.5
        while time.monotonic() < until:
            pub.publish(Float32MultiArray(data=[0.1, 0.1]))
            rclpy.spin_once(node, timeout_sec=0.03)
        wait_for(lambda: applied[-1]['left'] == pytest.approx(0.1))
        # Abrupt loss of bridge: no graceful zero can mask the device watchdog.
        bridge.kill()
        bridge.wait(timeout=3)
        wait_for(lambda: applied[-1]['left'] == 0.0, timeout=2)
        # Explicit retained and wrong-boot commands must never move outputs.
        topic = 'class10/test/wheel_pwm'
        boot = mqtt_samples[-1]['boot_id']
        payload = encode_command(0.1, 0.1, boot)
        client.publish(topic, payload, qos=0, retain=True).wait_for_publish(1)
        # MQTT marks retained messages only on delivery to a new subscription.
        stop(sim)
        applied.clear()
        sim = start('sim-retain', program(
            'simulator', '--port', str(port), '--prefix', 'class10/test'))
        wait_for(lambda: len(applied) >= 4)
        assert all(value['left'] == 0 for value in applied)
        client.publish(topic, b'', retain=True).wait_for_publish(1)
        client.publish(topic, encode_command(0.1, 0.1, 'old-boot'), retain=False)
        count = len(applied)
        wait_for(lambda: len(applied) >= count + 4)
        assert all(value['left'] == 0 for value in applied)
        assert all(abs(speed) < 20 for speed in speeds)
        assert all(process.poll() is None for process in (sim, processes[0]))
    finally:
        for process in reversed(processes):
            stop(process)
        if client is not None:
            client.disconnect()
            client.loop_stop()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for log in logs:
            log.close()
