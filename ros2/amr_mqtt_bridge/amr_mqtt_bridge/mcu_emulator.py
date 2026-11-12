"""Emulador del firmware de Tlaloc (lado MCU del contrato MQTT v1).

Sirve para SIL sin hardware: integra la cinemática omni a partir de cmd/vel,
emula IMU, brazo (perfil trapezoidal por junta), gripper con corriente,
batería, e-stop, watchdog de 250 ms, boot_id y LWT. El núcleo `EmulatedMcu`
no depende de MQTT ni de ROS para poder probarlo directamente.

    ros2 run amr_mqtt_bridge mcu_emulator --port 18883 --prefix omrover/mm01
"""
import argparse
import json
import math
import random
import secrets
import signal
import threading
import time

from . import protocol as p
from .omni import MecanumKinematics, integrate_pose


class Trapezoid:
    """Movimiento punto a punto de un eje con aceleración, deceleración y velocidad máxima."""

    def __init__(self, pos=0.0):
        self.pos = float(pos)
        self.vel = 0.0
        self.target = float(pos)
        self.acc = 1.0
        self.dec = 1.0
        self.vmax = 1.0

    def command(self, target, acc, dec, vmax):
        self.target, self.acc, self.dec, self.vmax = float(target), acc, dec, vmax

    def step(self, dt):
        err = self.target - self.pos
        if abs(err) < 1e-6 and abs(self.vel) < 1e-6:
            self.pos, self.vel = self.target, 0.0
            return
        direction = math.copysign(1.0, err)
        stop_dist = self.vel * self.vel / (2.0 * self.dec)
        if direction * self.vel > 0 and stop_dist >= abs(err):
            self.vel -= math.copysign(min(self.dec * dt, abs(self.vel)), self.vel)
        else:
            self.vel += direction * self.acc * dt
            self.vel = max(-self.vmax, min(self.vmax, self.vel))
        new_pos = self.pos + self.vel * dt
        if (self.target - new_pos) * direction < 0:     # cruzó el objetivo
            new_pos, self.vel = self.target, 0.0
        self.pos = new_pos

    @property
    def in_position(self):
        return abs(self.target - self.pos) < 2.0 and abs(self.vel) < 1.0


class EmulatedMcu:
    COUNTS_PER_REV = 16384

    def __init__(self, wheel_radius=0.0762, half_length=0.1583, half_width=0.1734,
                 battery_volt=25.0,
                 drain_v_per_s=0.002, grip_object_at=0.6, arm_vmax_rpm=30.0,
                 noise=0.0, boot_id=None, clock=time.monotonic):
        self.kin = MecanumKinematics(wheel_radius, half_length, half_width)
        self.boot_id = boot_id or secrets.token_hex(8)
        self.clock = clock
        self.t0 = clock()
        self.noise = noise
        self.rng = random.Random(0)
        # Base
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_until_ms = 0
        self.cmd_active = False
        self.wd_tripped = False
        self.wheel_pos = [0.0] * p.N_WHEELS
        self.wheel_vel = [0.0] * p.N_WHEELS
        self.lin_acc, self.ang_acc = 1.0, 3.0
        # Brazo y gripper
        self.arm_enabled = False
        self.joints = [Trapezoid() for _ in range(p.N_JOINTS)]
        self.arm_vmax = arm_vmax_rpm * self.COUNTS_PER_REV / 60.0
        self.grip_pos = 0.0
        self.grip_target = 0.0
        self.grip_max_cur = 1.0
        self.grip_cur = 0.0
        self.grasped = False
        self.grip_object_at = grip_object_at
        # Sistema
        self.battery = battery_volt
        self.drain = drain_v_per_s
        self.estop = False
        self.last_seq = {}

    # ----------------------------------------------------------------- tiempo
    def now_ms(self):
        return int((self.clock() - self.t0) * 1000.0)

    # --------------------------------------------------------------- comandos
    def handle(self, topic, payload, retain=False):
        """Procesa un comando. Devuelve la lista de (topic, payload, qos, retain) a publicar."""
        try:
            if retain:
                raise p.ProtocolError('comando con retain')
            cmd = p.decode_command(topic, payload, self.boot_id)
        except p.ProtocolError as exc:
            if topic == 'cmd/vel':
                self.cmd_active = False            # inválido nunca deja en movimiento
                return []
            seq = _loose_seq(payload)
            if seq is None or topic not in p.ACK_NAME:
                return []
            return [self._ack(seq, p.ACK_NAME[topic], 'rejected', str(exc))]

        if topic == 'cmd/vel':
            self.cmd = (cmd['vx'], cmd['vy'], cmd['wz'])
            self.cmd_until_ms = self.now_ms() + cmd['valid_ms']
            self.cmd_active = not self.estop
            self.wd_tripped = False
            return []

        name = p.ACK_NAME[topic]
        if self.last_seq.get(topic) == cmd['seq']:
            return [self._ack(cmd['seq'], name, 'duplicate')]
        self.last_seq[topic] = cmd['seq']
        result, msg = getattr(self, '_cmd_' + name)(cmd)
        return [self._ack(cmd['seq'], name, result, msg)]

    def _cmd_arm(self, cmd):
        op = cmd['op']
        if op == 'enable':
            self.arm_enabled = True
        elif op == 'disable':
            self.arm_enabled = False
            for j in self.joints:
                j.command(j.pos, 1, 1, 1)
                j.vel = 0.0
        elif op == 'clear':
            pass
        elif op in ('goal', 'home'):
            if not self.arm_enabled:
                return 'rejected', 'brazo deshabilitado'
            if self.estop:
                return 'rejected', 'e-stop activo'
            targets = cmd['pos'] if op == 'goal' else [0] * p.N_JOINTS
            accs = cmd.get('acc', [100.0] * p.N_JOINTS)
            decs = cmd.get('dec', [100.0] * p.N_JOINTS)
            k = self.COUNTS_PER_REV / 60.0              # rpm -> counts/s
            for j, t, a, d in zip(self.joints, targets, accs, decs):
                j.command(t, a * k, d * k, self.arm_vmax)
        return 'ok', ''

    def _cmd_gripper(self, cmd):
        if self.estop:
            return 'rejected', 'e-stop activo'
        self.grip_target = cmd['pos']
        self.grip_max_cur = cmd['max_cur']
        if cmd['pos'] < self.grip_pos:
            self.grasped = False
        return 'ok', ''

    def _cmd_estop(self, cmd):
        self.estop = cmd['active']
        if self.estop:
            self.cmd_active = False
            for j in self.joints:
                j.command(j.pos, 1, 1, 1)
                j.vel = 0.0
        return 'ok', ''

    def _cmd_odom_reset(self, cmd):
        self.x, self.y, self.yaw = cmd['x'], cmd['y'], cmd['yaw']
        return 'ok', ''

    def _ack(self, seq, name, result, msg=''):
        fields = {'seq': seq, 'cmd': name, 'result': result}
        if msg:
            fields['msg'] = msg[:64]
        return ('llc/ack', p.encode_telemetry('llc/ack', self.boot_id, self.now_ms(), **fields),
                1, False)

    # ----------------------------------------------------------------- física
    def step(self, dt):
        now = self.now_ms()
        if self.cmd_active and now >= self.cmd_until_ms:
            self.cmd_active = False
            self.wd_tripped = True
        target = self.cmd if (self.cmd_active and not self.estop) else (0.0, 0.0, 0.0)
        self.vx = _approach(self.vx, target[0], self.lin_acc * dt)
        self.vy = _approach(self.vy, target[1], self.lin_acc * dt)
        self.wz = _approach(self.wz, target[2], self.ang_acc * dt)
        self.wheel_vel = self.kin.inverse(self.vx, self.vy, self.wz)
        self.wheel_pos = [q + w * dt for q, w in zip(self.wheel_pos, self.wheel_vel)]
        self.x, self.y, self.yaw = integrate_pose(
            self.x, self.y, self.yaw, self.vx, self.vy, self.wz, dt)

        if self.arm_enabled and not self.estop:
            for j in self.joints:
                j.step(dt)

        grip_rate = 1.0 * dt
        if self.grasped or self.estop:
            self.grip_cur = self.grip_max_cur * 0.9 if self.grasped else 0.0
        elif abs(self.grip_target - self.grip_pos) > 1e-3:
            closing = self.grip_target > self.grip_pos
            self.grip_pos = _approach(self.grip_pos, self.grip_target, grip_rate)
            self.grip_cur = 0.15
            if closing and self.grip_object_at is not None and self.grip_pos >= self.grip_object_at:
                self.grip_pos = self.grip_object_at
                self.grip_cur = self.grip_max_cur + 0.05
                self.grasped = True
        else:
            self.grip_cur = 0.02

        moving = abs(self.vx) + abs(self.vy) + abs(self.wz) > 1e-3
        self.battery = max(18.0, self.battery - self.drain * dt * (3.0 if moving else 1.0))

    # ------------------------------------------------------------- telemetría
    def faults(self):
        mask = 0
        if self.battery <= 19.8:
            mask |= 1 << 6
        return mask

    def telemetry(self, topic):
        n = self.noise
        g = (lambda: self.rng.gauss(0.0, n)) if n > 0 else (lambda: 0.0)
        t = self.now_ms()
        if topic == 'llc/odom':
            fields = dict(x=self.x, y=self.y, yaw=self.yaw, vx=self.vx + g(), vy=self.vy + g(),
                          wz=self.wz + g(), cov=[1e-3, 1e-3, 2e-3, 4e-4, 4e-4, 1e-3])
        elif topic == 'llc/imu':
            half = 0.5 * self.yaw
            fields = dict(q=[math.cos(half), 0.0, 0.0, math.sin(half)],
                          gyro=[g(), g(), self.wz + g()], acc=[g(), g(), 9.81 + g()],
                          acc_status=3)
        elif topic == 'llc/wheels':
            fields = dict(pos=self.wheel_pos, vel=self.wheel_vel, rc_err=[0, 0])
        elif topic == 'llc/arm':
            k = 60.0 / self.COUNTS_PER_REV                 # counts/s -> rpm
            fields = dict(
                enabled=self.arm_enabled,
                pos=[int(round(j.pos)) for j in self.joints],
                vel=[j.vel * k for j in self.joints],
                cur=[0.1 + abs(j.vel) * 1e-4 for j in self.joints],
                temp=[32.0] * p.N_JOINTS,
                in_pos=[j.in_position for j in self.joints],
                fault=[0] * p.N_JOINTS,
                grip={'pos': self.grip_pos, 'cur': self.grip_cur, 'grasped': self.grasped})
        elif topic == 'llc/battery':
            fields = dict(volt=round(self.battery, 2), logic_volt=5.0)
        elif topic == 'llc/status':
            faults = self.faults()
            state = 'estop' if self.estop else ('fault' if faults else 'ok')
            fields = dict(state=state, estop=self.estop, faults=faults,
                          wd=self.wd_tripped, cpu=0.12)
        else:
            raise ValueError(topic)
        return p.encode_telemetry(topic, self.boot_id, t, **fields)


RATES_HZ = {'llc/odom': 50.0, 'llc/imu': 100.0, 'llc/wheels': 20.0, 'llc/arm': 50.0,
            'llc/battery': 1.0, 'llc/status': 2.0}


def _approach(value, target, step):
    if value < target:
        return min(value + step, target)
    return max(value - step, target)


def _loose_seq(payload):
    try:
        seq = json.loads(payload).get('seq')
    except (ValueError, AttributeError, TypeError):
        return None
    return seq if type(seq) is int and 0 <= seq <= p.UINT32_MAX else None


class MqttMcuRunner:
    """Conecta un EmulatedMcu a Mosquitto con el mismo comportamiento del firmware."""

    def __init__(self, mcu, host='127.0.0.1', port=18883, prefix='omrover/mm01',
                 rates=None, period_s=0.005):
        from .mqtt_support import make_client
        self.mcu = mcu
        self.prefix = prefix.rstrip('/')
        self.rates = dict(rates or RATES_HZ)
        self.period = period_s
        self.connected = threading.Event()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.client = make_client(
            client_id=self.prefix,
            will=(f'{self.prefix}/llc/status', p.encode_lwt(), 1, True))
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = lambda *a: self.connected.clear()
        self.client.on_message = self._on_message
        self.host, self.port = host, port

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            # Borra el LWT retenido de una sesión anterior (payload vacío + retain).
            client.publish(f'{self.prefix}/llc/status', b'', qos=1, retain=True)
            client.subscribe(f'{self.prefix}/cmd/#', qos=1)
            self.connected.set()

    def _on_message(self, client, userdata, msg):
        topic = msg.topic[len(self.prefix) + 1:]
        with self.lock:
            out = self.mcu.handle(topic, msg.payload, retain=bool(msg.retain))
        for t, payload, qos, retain in out:
            client.publish(f'{self.prefix}/{t}', payload, qos=qos, retain=retain)

    def run(self, duration=None):
        self.client.connect_async(self.host, self.port, keepalive=2)
        self.client.loop_start()
        next_pub = {t: 0.0 for t in self.rates}
        last = time.monotonic()
        end = None if duration is None else last + duration
        try:
            while not self.stop_event.is_set() and (end is None or time.monotonic() < end):
                now = time.monotonic()
                with self.lock:
                    self.mcu.step(now - last)
                last = now
                if self.connected.is_set():
                    for topic, hz in self.rates.items():
                        if now >= next_pub[topic]:
                            with self.lock:
                                payload = self.mcu.telemetry(topic)
                            self.client.publish(f'{self.prefix}/{topic}', payload,
                                                qos=1 if topic == 'llc/status' else 0)
                            next_pub[topic] = max(next_pub[topic] + 1.0 / hz, now)
                time.sleep(self.period)
        finally:
            self.client.loop_stop()
            self.client.disconnect()

    def kill(self):
        """Cierra el socket sin DISCONNECT: el broker publica el LWT (como un reset)."""
        self.stop_event.set()
        try:
            self.client.socket().close()
        except Exception:  # noqa: BLE001
            pass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=18883)
    ap.add_argument('--prefix', default='omrover/mm01')
    ap.add_argument('--battery-volt', type=float, default=25.0)
    ap.add_argument('--drain', type=float, default=0.002, help='V/s en reposo (x3 en movimiento)')
    ap.add_argument('--grip-object-at', type=float, default=0.6,
                    help='posición 0..1 donde el gripper toca un objeto (<0 sin objeto)')
    ap.add_argument('--wheel-radius', type=float, default=0.0762)
    ap.add_argument('--half-length', type=float, default=0.1583)
    ap.add_argument('--half-width', type=float, default=0.1734)
    ap.add_argument('--noise', type=float, default=0.0)
    ap.add_argument('--duration', type=float, default=None)
    args, _ = ap.parse_known_args(argv)   # tolera --ros-args al lanzarse con ros2 run

    mcu = EmulatedMcu(wheel_radius=args.wheel_radius, half_length=args.half_length,
                      half_width=args.half_width,
                      battery_volt=args.battery_volt, drain_v_per_s=args.drain,
                      grip_object_at=args.grip_object_at if args.grip_object_at >= 0 else None,
                      noise=args.noise)
    runner = MqttMcuRunner(mcu, args.host, args.port, args.prefix)
    signal.signal(signal.SIGTERM, lambda *a: runner.stop_event.set())
    print(f'MCU emulado boot_id={mcu.boot_id} -> {args.host}:{args.port} prefix {args.prefix}',
          flush=True)
    try:
        runner.run(args.duration)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
