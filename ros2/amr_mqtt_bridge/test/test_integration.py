"""Integración: Mosquitto real + emulador del MCU + bridge, cada uno en su proceso
(como en el robot), observados desde un nodo sonda en el proceso del test.

Se salta si faltan mosquitto, paho-mqtt o rclpy (p. ej. fuera de la NUC).
"""
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time

import pytest

pytest.importorskip('paho.mqtt.client')
rclpy = pytest.importorskip('rclpy')
MOSQUITTO = shutil.which('mosquitto') or (
    '/usr/sbin/mosquitto' if os.path.exists('/usr/sbin/mosquitto') else None)
pytestmark = pytest.mark.skipif(MOSQUITTO is None, reason='mosquitto no instalado')


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


@pytest.fixture(scope='module')
def broker(tmp_path_factory):
    port = free_port()
    conf = tmp_path_factory.mktemp('mosq') / 'test.conf'
    conf.write_text(f'listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n')
    proc = subprocess.Popen([MOSQUITTO, '-c', str(conf)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    for _ in range(50):
        try:
            socket.create_connection(('127.0.0.1', port), timeout=0.1).close()
            break
        except OSError:
            time.sleep(0.1)
    yield port
    proc.terminate()
    proc.wait(timeout=5)


class Emulator:
    """MCU emulado en otro proceso. kill() usa SIGKILL: el SO cierra el socket sin
    DISCONNECT y el broker publica el LWT, igual que un reset del MCU real."""

    def __init__(self, port, prefix):
        self.proc = subprocess.Popen(
            [sys.executable, '-u', '-m', 'amr_mqtt_bridge.mcu_emulator',
             '--port', str(port), '--prefix', prefix],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        first = self.proc.stdout.readline()
        assert 'boot_id=' in first, first
        self.boot_id = first.split('boot_id=')[1].split()[0]

    def kill(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait(timeout=5)


@pytest.fixture(scope='module')
def ros(broker):
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    from rclpy.executors import MultiThreadedExecutor

    prefix = f'test/{os.getpid()}'
    bridge = subprocess.Popen(
        [sys.executable, '-u', '-m', 'amr_mqtt_bridge.bridge_node', '--ros-args',
         '-p', f'broker_port:={broker}', '-p', f'mqtt_prefix:={prefix}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    probe = rclpy.create_node('integration_probe')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield {'probe': probe, 'port': broker, 'prefix': prefix}
    bridge.send_signal(signal.SIGINT)
    try:
        bridge.wait(timeout=5)
    except subprocess.TimeoutExpired:
        bridge.kill()
    executor.shutdown()
    rclpy.shutdown()


def wait_for(predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def call(client, request, timeout=3.0):
    assert client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(request)
    assert wait_for(future.done, timeout)
    return future.result()


def test_end_to_end(ros):
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu
    from std_srvs.srv import SetBool
    from amr_interfaces.msg import LlcStatus
    from amr_interfaces.srv import ArmCommand, SetGripper
    from amr_mqtt_bridge.bridge_node import LATCHED_QOS

    # Colas profundas en la sonda: mide lo que publica el bridge, no la latencia de
    # este executor de Python (con depth=5 best-effort descarta ráfagas).
    reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    best_effort = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)

    probe = ros['probe']
    odom, imu, status = [], [], []
    probe.create_subscription(Odometry, 'odom', odom.append, reliable)
    probe.create_subscription(Imu, 'imu/data', imu.append, best_effort)
    probe.create_subscription(LlcStatus, 'llc/status', status.append, LATCHED_QOS)
    cmd_pub = probe.create_publisher(Twist, 'cmd_vel', 1)

    def live_with(boot):
        return bool(status) and status[-1].link_live and status[-1].boot_id == boot

    emu = Emulator(ros['port'], ros['prefix'])
    try:
        assert wait_for(lambda: live_with(emu.boot_id), 10.0)

        # Tasas: /odom >= 45 Hz e /imu/data >= 90 Hz.
        odom.clear()
        imu.clear()
        time.sleep(2.0)
        assert len(odom) / 2.0 >= 45.0
        assert len(imu) / 2.0 >= 90.0

        # cmd_vel mueve la base; al dejar de publicar se detiene (timeout del bridge).
        twist = Twist()
        twist.linear.x = 0.3
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            cmd_pub.publish(twist)
            time.sleep(0.05)
        assert odom[-1].twist.twist.linear.x == pytest.approx(0.3, abs=0.02)
        stop_t = time.monotonic()
        assert wait_for(lambda: abs(odom[-1].twist.twist.linear.x) < 1e-3, 1.5)
        assert time.monotonic() - stop_t < 0.2 + 0.3 + 0.3   # timeout + rampa + margen

        # Servicios con ack del MCU.
        arm = probe.create_client(ArmCommand, 'llc/arm_command')
        res = call(arm, ArmCommand.Request(op=ArmCommand.Request.ENABLE))
        assert res.accepted, res.message
        res = call(arm, ArmCommand.Request(op=ArmCommand.Request.GOAL,
                                           positions=[0.5, 0.2, -0.3, 0.1]))
        assert res.accepted, res.message
        res = call(arm, ArmCommand.Request(op=ArmCommand.Request.GOAL,
                                           positions=[0.0, 3.0, 0.0, 0.0]))
        assert not res.accepted and 'límites' in res.message
        grip = probe.create_client(SetGripper, 'llc/set_gripper')
        res = call(grip, SetGripper.Request(position=1.0, max_current_a=0.8))
        assert res.accepted, res.message
        estop = probe.create_client(SetBool, 'llc/set_estop')
        assert call(estop, SetBool.Request(data=True)).success
        assert wait_for(lambda: status and status[-1].estop, 2.0)
        assert call(estop, SetBool.Request(data=False)).success

        # Reset del MCU: LWT -> offline/stale y luego LIVE con un boot_id nuevo.
        old_boot = emu.boot_id
        emu.kill()
        assert wait_for(lambda: status and status[-1].state in ('offline', 'stale'), 6.0)
        emu = Emulator(ros['port'], ros['prefix'])
        assert emu.boot_id != old_boot
        assert wait_for(lambda: live_with(emu.boot_id), 5.0)
    finally:
        emu.kill()
