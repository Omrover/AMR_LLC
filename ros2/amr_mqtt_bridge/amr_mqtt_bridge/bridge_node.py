"""Bridge ROS 2 <-> MQTT del controlador de bajo nivel (Tlaloc S32K344).

Telemetría MQTT (contrato v1) -> /odom, /tf, /imu/data, /joint_states,
/battery_state, /llc/status, /arm/status.
ROS -> MQTT: /cmd_vel (reenviado a cmd_rate_hz con valid_ms), servicios
/llc/arm_command, /llc/set_gripper, /llc/set_estop, /llc/reset_odometry
(esperan el ack del MCU).

Mismo esqueleto que examples/ros2_mqtt/v2 (cola desde el hilo de paho +
timer de drenado, STALE/LIVE, reset por boot_id, stop al cerrar).
"""
import math
import os
import queue
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions

from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from amr_interfaces.msg import ArmStatus, LlcStatus
from amr_interfaces.srv import ArmCommand, ResetOdometry, SetGripper

from . import protocol as p
from .arm_units import ArmUnits
from .battery import battery_level, soc_from_voltage
from .mqtt_support import make_client
from .time_sync import TimeSync

ODOM_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
CMD_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)

ARM_OPS = {ArmCommand.Request.GOAL: 'goal', ArmCommand.Request.ENABLE: 'enable',
           ArmCommand.Request.DISABLE: 'disable', ArmCommand.Request.CLEAR_FAULTS: 'clear',
           ArmCommand.Request.HOME: 'home'}


def stamp_msg(seconds):
    sec = int(math.floor(seconds))
    return TimeMsg(sec=sec, nanosec=int((seconds - sec) * 1e9))


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def diag6(values):
    cov = [0.0] * 36
    for i, v in enumerate(values):
        cov[i * 7] = float(v)
    return cov


class LlcBridge(Node):
    def __init__(self):
        super().__init__('amr_mqtt_bridge')
        d = self.declare_parameter
        d('broker_host', '127.0.0.1')
        d('broker_port', 18883)
        d('mqtt_prefix', 'omrover/mm01')
        d('stale_timeout', 1.0)
        d('ack_timeout', 0.5)
        d('time_sync_window', 5.0)
        # Base
        d('odom_frame', 'odom')
        d('base_frame', 'base_footprint')
        d('imu_frame', 'imu_link')
        d('publish_tf', True)
        d('use_stamped_cmd_vel', False)
        d('cmd_rate_hz', 20.0)
        d('cmd_vel_timeout', 0.2)
        d('cmd_valid_ms', 250)
        d('max_linear', 1.0)
        d('max_angular', 2.0)
        d('wheel_joints', ['wheel_fl_joint', 'wheel_rl_joint', 'wheel_rr_joint', 'wheel_fr_joint'])
        d('unused_covariance', 1e6)
        d('imu_orientation_variance', 1e-3)
        d('imu_gyro_variance', 1e-4)
        d('imu_accel_variance', 1e-2)
        # Brazo (config/arm.yaml)
        d('arm_joints', ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4'])
        d('arm_counts_per_rev', 16384)
        d('arm_gear_ratio', 1.0)
        d('arm_zero_offset', [0, 0, 0, 0])
        d('arm_sign', [1, 1, 1, 1])
        d('arm_lower', [-math.pi, -math.pi / 2, -2.5, -2.5])
        d('arm_upper', [math.pi, math.pi / 2, 2.5, 2.5])
        d('arm_default_accel', 2.0)
        d('arm_status_rate_hz', 10.0)
        d('gripper_joint', 'gripper_finger_joint')
        d('gripper_travel', 0.035)
        # Batería LiPo 6S
        d('battery_cells', 6)
        d('battery_capacity_ah', 6.0)
        d('battery_low_volt', 21.6)
        d('battery_critical_volt', 21.0)

        g = self.get_parameter
        self.prefix = g('mqtt_prefix').value.rstrip('/')
        if not self.prefix or '+' in self.prefix or '#' in self.prefix:
            raise ValueError('mqtt_prefix debe ser un prefijo literal no vacío')
        self.stale_timeout = float(g('stale_timeout').value)
        self.ack_timeout = float(g('ack_timeout').value)
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.imu_frame = g('imu_frame').value
        self.publish_tf = bool(g('publish_tf').value)
        self.cmd_vel_timeout = float(g('cmd_vel_timeout').value)
        self.cmd_valid_ms = int(g('cmd_valid_ms').value)
        self.max_linear = min(float(g('max_linear').value), p.MAX_LINEAR)
        self.max_angular = min(float(g('max_angular').value), p.MAX_ANGULAR)
        self.wheel_joints = list(g('wheel_joints').value)
        self.unused_cov = float(g('unused_covariance').value)
        self.imu_cov = (float(g('imu_orientation_variance').value),
                        float(g('imu_gyro_variance').value),
                        float(g('imu_accel_variance').value))
        self.arm_joints = list(g('arm_joints').value)
        self.arm = ArmUnits(int(g('arm_counts_per_rev').value), float(g('arm_gear_ratio').value),
                            list(g('arm_zero_offset').value), list(g('arm_sign').value),
                            list(g('arm_lower').value), list(g('arm_upper').value))
        self.arm_default_accel = float(g('arm_default_accel').value)
        self.arm_status_period = 1.0 / max(float(g('arm_status_rate_hz').value), 0.1)
        self.gripper_joint = g('gripper_joint').value
        self.gripper_travel = float(g('gripper_travel').value)
        self.battery_cells = int(g('battery_cells').value)
        self.battery_capacity = float(g('battery_capacity_ah').value)
        self.battery_low = float(g('battery_low_volt').value)
        self.battery_critical = float(g('battery_critical_volt').value)

        # Estado compartido con el hilo de paho
        self.samples = queue.Queue(maxsize=256)
        self.lock = threading.Lock()
        self.boot_id = None
        self.last_rx = None            # time.monotonic() de la última telemetría
        self.invalid_packets = 0
        self.mcu_status = None
        self.seq = int(time.time()) & 0xFFFF   # arranque distinto en cada ejecución
        self.pending = {}              # seq -> [Event, ack|None, cmd]
        self.cmd = None                # (vx, vy, wz, monotonic)
        self.time_sync = TimeSync(float(g('time_sync_window').value))
        self.rx_count = {t: 0 for t in p.TELEMETRY}
        self.rate = {t: 0.0 for t in p.TELEMETRY}
        self.last_rate_t = time.monotonic()
        self.last_arm_status = 0.0
        self.battery_state = 'ok'
        self.wheel_errors = []

        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, 'odom', ODOM_QOS)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile_sensor_data)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', LATCHED_QOS)
        self.status_pub = self.create_publisher(LlcStatus, 'llc/status', LATCHED_QOS)
        self.arm_status_pub = self.create_publisher(ArmStatus, 'arm/status', LATCHED_QOS)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None

        # Entradas ROS
        timers = MutuallyExclusiveCallbackGroup()
        services = ReentrantCallbackGroup()
        if g('use_stamped_cmd_vel').value:
            self.create_subscription(TwistStamped, 'cmd_vel',
                                     lambda m: self.on_cmd_vel(m.twist), CMD_QOS,
                                     callback_group=timers)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, CMD_QOS,
                                     callback_group=timers)
        self.create_service(ArmCommand, 'llc/arm_command', self.srv_arm,
                            callback_group=services)
        self.create_service(SetGripper, 'llc/set_gripper', self.srv_gripper,
                            callback_group=services)
        self.create_service(SetBool, 'llc/set_estop', self.srv_estop, callback_group=services)
        self.create_service(ResetOdometry, 'llc/reset_odometry', self.srv_reset_odom,
                            callback_group=services)

        # Timers en reloj monotónico: no dependen de /clock (con use_sim_time, un timer
        # en reloj ROS se reevalúa con cada mensaje de /clock y dispara el CPU).
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.01, self.drain, callback_group=timers, clock=steady)
        self.create_timer(1.0 / float(g('cmd_rate_hz').value), self.send_cmd_vel,
                          callback_group=timers, clock=steady)
        self.create_timer(1.0, self.publish_link_status, callback_group=timers, clock=steady)

        self.diag = None
        try:
            from diagnostic_updater import Updater
            self.diag = Updater(self)
            self.diag.setHardwareID(f'tlaloc:{self.prefix}')
            self.diag.add('llc_link', self.diagnose)
        except ImportError:
            self.get_logger().warning('diagnostic_updater no disponible; sin /diagnostics')

        self.client = make_client(client_id=f'amr_bridge_{os.getpid()}')
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect_async(g('broker_host').value, int(g('broker_port').value),
                                  keepalive=5)
        self.client.loop_start()
        self.get_logger().info(
            f"MQTT {g('broker_host').value}:{g('broker_port').value} prefix {self.prefix}")

    # ------------------------------------------------------------- hilo paho
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(f'{self.prefix}/llc/#', qos=1)
            self.get_logger().info('conectado al broker')
        else:
            self.get_logger().error(f'conexión MQTT rechazada rc={rc}')

    def on_message(self, client, userdata, msg):
        # Reloj del nodo: tiempo de pared en el robot, /clock con use_sim_time
        # (el MCU simulado en Gazebo publica t_ms en tiempo de simulación).
        rx = self.get_clock().now().nanoseconds * 1e-9
        topic = msg.topic[len(self.prefix) + 1:]
        if topic not in p.TELEMETRY or not msg.payload:
            return                       # payload vacío = el MCU borra su LWT retenido
        try:
            data = p.decode_telemetry(topic, msg.payload)
        except p.ProtocolError as exc:
            self.invalid_packets += 1
            self.get_logger().warning(f'{topic}: {exc}', throttle_duration_sec=5.0)
            return
        if msg.retain and data.get('state') != 'offline':
            return                       # solo el LWT puede venir retenido
        if topic == 'llc/ack':
            self.resolve_ack(data)
            return
        try:
            self.samples.put_nowait((rx, time.monotonic(), topic, data))
        except queue.Full:
            pass

    def resolve_ack(self, ack):
        with self.lock:
            waiter = self.pending.get(ack['seq'])
            if waiter is not None and waiter[2] == ack['cmd']:
                waiter[1] = ack
                waiter[0].set()

    # ------------------------------------------------------------ drenado ROS
    def drain(self):
        while True:
            try:
                rx, mono, topic, data = self.samples.get_nowait()
            except queue.Empty:
                return
            if data.get('state') == 'offline':
                self.on_offline()
                continue
            if time.monotonic() - mono > 0.5:
                continue                 # muestra vieja: no se publica tarde
            if data['boot_id'] != self.boot_id:
                self.on_new_boot(data['boot_id'])
            self.last_rx = mono
            self.rx_count[topic] += 1
            stamp = self.time_sync.stamp(data['boot_id'], data['t_ms'], rx, stream=topic)
            if stamp is None:
                continue
            handler = getattr(self, 'on_' + topic.split('/')[1])
            handler(data, stamp_msg(stamp))

    def on_new_boot(self, boot_id):
        if self.boot_id is not None:
            self.get_logger().warning(f'MCU reiniciado: boot_id {self.boot_id} -> {boot_id}')
        else:
            self.get_logger().info(f'MCU boot_id {boot_id}')
        with self.lock:
            self.boot_id = boot_id
            self.cmd = None              # no reenviar comandos previos al reset
            for waiter in self.pending.values():
                waiter[0].set()          # libera servicios en espera (sin ack)

    def on_offline(self):
        self.get_logger().warning('MCU offline (LWT)')
        self.mcu_status = {'state': 'offline'}
        self.last_rx = None
        self.publish_link_status()

    def on_odom(self, d, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = d['x']
        msg.pose.pose.position.y = d['y']
        q = yaw_quaternion(d['yaw'])
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q
        msg.twist.twist.linear.x = d['vx']
        msg.twist.twist.linear.y = d['vy']
        msg.twist.twist.angular.z = d['wz']
        c, u = d['cov'], self.unused_cov
        msg.pose.covariance = diag6([c[0], c[1], u, u, u, c[2]])
        msg.twist.covariance = diag6([c[3], c[4], u, u, u, c[5]])
        self.odom_pub.publish(msg)
        if self.tf_pub is not None:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = d['x']
            tf.transform.translation.y = d['y']
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf_pub.sendTransform(tf)

    def on_imu(self, d, stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        w, x, y, z = d['q']
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = d['gyro']
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = d['acc']
        o, gy, a = self.imu_cov
        msg.orientation_covariance = [o, 0.0, 0.0, 0.0, o, 0.0, 0.0, 0.0, o]
        msg.angular_velocity_covariance = [gy, 0.0, 0.0, 0.0, gy, 0.0, 0.0, 0.0, gy]
        msg.linear_acceleration_covariance = [a, 0.0, 0.0, 0.0, a, 0.0, 0.0, 0.0, a]
        self.imu_pub.publish(msg)

    def on_wheels(self, d, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.wheel_joints
        msg.position = [float(v) for v in d['pos']]
        msg.velocity = [float(v) for v in d['vel']]
        self.joint_pub.publish(msg)
        self.wheel_errors = d['rc_err']

    def on_arm(self, d, stamp):
        grip = d['grip']
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.arm_joints + [self.gripper_joint]
        msg.position = self.arm.counts_to_rad(d['pos']) + [(1.0 - grip['pos']) * self.gripper_travel]
        msg.velocity = self.arm.rpm_to_rad_s(d['vel']) + [0.0]
        msg.effort = list(d['cur']) + [grip['cur']]
        self.joint_pub.publish(msg)

        now = time.monotonic()
        if now - self.last_arm_status < self.arm_status_period:
            return
        self.last_arm_status = now
        st = ArmStatus()
        st.header.stamp = stamp
        st.enabled = d['enabled']
        st.in_position = d['in_pos']
        st.motor_faults = d['fault']
        st.temperature_c = d['temp']
        st.current_a = d['cur']
        st.gripper_position = grip['pos']
        st.gripper_current_a = grip['cur']
        st.gripper_grasped = grip['grasped']
        self.arm_status_pub.publish(st)

    def on_battery(self, d, stamp):
        volt = d['volt']
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = volt
        msg.percentage = soc_from_voltage(volt, self.battery_cells)
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = self.battery_capacity
        msg.temperature = math.nan
        msg.cell_voltage = [volt / self.battery_cells] * self.battery_cells
        msg.cell_temperature = []
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        msg.location = 'base'
        self.battery_pub.publish(msg)
        level = battery_level(volt, self.battery_low, self.battery_critical)
        if level != self.battery_state:
            logger = self.get_logger()
            log = {'critical': logger.error, 'low': logger.warning}.get(level, logger.info)
            log(f'batería {level}: {volt:.2f} V')
            self.battery_state = level

    def on_status(self, d, stamp):
        self.mcu_status = d
        if d['faults']:
            self.get_logger().warning(f"fallas MCU: {p.fault_names(d['faults'])}",
                                      throttle_duration_sec=5.0)
        self.publish_link_status(stamp)

    # --------------------------------------------------------- estado enlace
    def link_age(self):
        return None if self.last_rx is None else time.monotonic() - self.last_rx

    def link_live(self):
        age = self.link_age()
        return age is not None and age < self.stale_timeout

    def publish_link_status(self, stamp=None):
        if stamp is None:
            # El timer de 1 Hz solo publica si el MCU calla (si no, lo hace on_status).
            if self.link_live() and self.mcu_status and self.mcu_status.get('state') != 'offline':
                return
            stamp = self.get_clock().now().to_msg()
        st = LlcStatus()
        st.header.stamp = stamp
        mcu = self.mcu_status or {}
        live = self.link_live()
        st.state = mcu.get('state', 'unknown') if live else (
            'offline' if mcu.get('state') == 'offline' else 'stale')
        st.boot_id = self.boot_id or ''
        st.mqtt_connected = self.client.is_connected()
        st.link_live = live
        age = self.link_age()
        st.link_age_s = -1.0 if age is None else float(age)
        st.invalid_packets = self.invalid_packets
        st.estop = bool(mcu.get('estop', False))
        st.watchdog_tripped = bool(mcu.get('wd', False))
        st.faults = int(mcu.get('faults', 0))
        st.cpu_load = float(mcu.get('cpu', 0.0))
        st.roboclaw_errors = [int(v) for v in self.wheel_errors]
        self.status_pub.publish(st)

    def diagnose(self, stat):
        now = time.monotonic()
        dt = now - self.last_rate_t
        if dt > 0.5:
            for t in self.rx_count:
                self.rate[t] = self.rx_count[t] / dt
                self.rx_count[t] = 0
            self.last_rate_t = now
        if not self.client.is_connected():
            stat.summary(DiagnosticStatus.ERROR, 'sin conexión al broker MQTT')
        elif not self.link_live():
            stat.summary(DiagnosticStatus.ERROR, 'MCU sin telemetría (STALE/offline)')
        elif self.mcu_status and self.mcu_status.get('state') not in ('ok', 'init'):
            stat.summary(DiagnosticStatus.WARN, f"MCU en estado {self.mcu_status.get('state')}")
        elif self.rate['llc/odom'] < 40.0:
            stat.summary(DiagnosticStatus.WARN, f"odometría lenta: {self.rate['llc/odom']:.1f} Hz")
        else:
            stat.summary(DiagnosticStatus.OK, 'enlace LIVE')
        stat.add('boot_id', str(self.boot_id))
        stat.add('invalid_packets', str(self.invalid_packets))
        for t, r in self.rate.items():
            stat.add(f'rate {t}', f'{r:.1f}')
        return stat

    # -------------------------------------------------------------- cmd_vel
    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    def send_cmd_vel(self):
        if self.boot_id is None or not self.client.is_connected() or not self.link_live():
            return                       # el watchdog del MCU detiene la base
        cmd = self.cmd
        if cmd is None or time.monotonic() - cmd[3] > self.cmd_vel_timeout:
            vx = vy = wz = 0.0
        else:
            vx, vy, wz = cmd[:3]
            if not all(math.isfinite(v) for v in (vx, vy, wz)):
                self.get_logger().error('cmd_vel no finito: enviando cero')
                vx = vy = wz = 0.0
        vx = max(-self.max_linear, min(self.max_linear, vx))
        vy = max(-self.max_linear, min(self.max_linear, vy))
        wz = max(-self.max_angular, min(self.max_angular, wz))
        payload = p.encode_command('cmd/vel', self.boot_id, self.next_seq(),
                                   vx=vx, vy=vy, wz=wz, valid_ms=self.cmd_valid_ms)
        self.client.publish(f'{self.prefix}/cmd/vel', payload, qos=0, retain=False)

    # ------------------------------------------------------------ servicios
    def next_seq(self):
        with self.lock:
            self.seq = (self.seq + 1) & p.UINT32_MAX
            return self.seq

    def send_and_wait(self, topic, **fields):
        """Publica un comando QoS 1 y espera el ack. Devuelve (ok, seq, mensaje)."""
        boot = self.boot_id
        if boot is None or not self.client.is_connected() or not self.link_live():
            return False, 0, 'MCU no disponible (sin telemetría o broker desconectado)'
        seq = self.next_seq()
        try:
            payload = p.encode_command(topic, boot, seq, **fields)
        except p.ProtocolError as exc:
            return False, seq, f'comando inválido: {exc}'
        event = threading.Event()
        with self.lock:
            self.pending[seq] = [event, None, p.ACK_NAME[topic]]
        self.client.publish(f'{self.prefix}/{topic}', payload, qos=1, retain=False)
        event.wait(self.ack_timeout)
        with self.lock:
            ack = self.pending.pop(seq)[1]
        if ack is None:
            return False, seq, 'sin ack del MCU (timeout o reinicio)'
        ok = ack['result'] in ('ok', 'duplicate')
        return ok, seq, ack['result'] + (f": {ack['msg']}" if ack.get('msg') else '')

    def srv_arm(self, req, res):
        op = ARM_OPS.get(req.op)
        if op is None:
            res.accepted, res.message = False, f'op desconocida {req.op}'
            return res
        fields = {'op': op}
        if op == 'goal':
            q = list(req.positions)
            if len(q) != self.arm.n:
                res.accepted, res.message = False, f'se esperaban {self.arm.n} posiciones'
                return res
            bad = self.arm.check_limits(q)
            if bad:
                res.accepted = False
                res.message = f'fuera de límites articulares: {[self.arm_joints[i] for i in bad]}'
                return res
            acc = list(req.accelerations) or [self.arm_default_accel] * self.arm.n
            if len(acc) != self.arm.n or any(a <= 0 for a in acc):
                res.accepted, res.message = False, 'accelerations inválidas'
                return res
            rpm_s = self.arm.rad_s2_to_rpm_s(acc)
            fields.update(pos=self.arm.rad_to_counts(q), acc=rpm_s, dec=rpm_s)
        res.accepted, res.seq, res.message = self.send_and_wait('cmd/arm', **fields)
        return res

    def srv_gripper(self, req, res):
        res.accepted, res.seq, res.message = self.send_and_wait(
            'cmd/gripper', pos=float(req.position), max_cur=float(req.max_current_a))
        return res

    def srv_estop(self, req, res):
        res.success, _, res.message = self.send_and_wait('cmd/estop', active=bool(req.data))
        return res

    def srv_reset_odom(self, req, res):
        yaw = math.atan2(math.sin(req.yaw), math.cos(req.yaw))
        res.success, _, res.message = self.send_and_wait(
            'cmd/odom_reset', x=float(req.x), y=float(req.y), yaw=yaw)
        return res

    # ----------------------------------------------------------------- cierre
    def close(self):
        if self.boot_id is not None and self.client.is_connected():
            info = self.client.publish(
                f'{self.prefix}/cmd/vel',
                p.encode_command('cmd/vel', self.boot_id, self.next_seq(),
                                 vx=0.0, vy=0.0, wz=0.0, valid_ms=self.cmd_valid_ms),
                qos=0, retain=False)
            try:
                info.wait_for_publish(timeout=0.2)
            except (RuntimeError, ValueError):
                pass
        self.client.disconnect()
        self.client.loop_stop()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LlcBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
