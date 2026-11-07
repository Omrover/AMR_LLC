"""Comportamiento del lado MCU (EmulatedMcu) frente al contrato, sin MQTT."""
import json

import pytest

from amr_mqtt_bridge import protocol as p
from amr_mqtt_bridge.mcu_emulator import EmulatedMcu


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


@pytest.fixture
def mcu():
    clock = FakeClock()
    m = EmulatedMcu(boot_id='testboot', clock=clock)
    m.fake_clock = clock
    return m


def run(m, seconds, dt=0.005):
    for _ in range(int(round(seconds / dt))):
        m.fake_clock.t += dt
        m.step(dt)


def vel(m, seq, vx, vy=0.0, wz=0.0, valid_ms=250):
    return m.handle('cmd/vel', p.encode_command('cmd/vel', m.boot_id, seq, vx=vx, vy=vy,
                                                wz=wz, valid_ms=valid_ms))


def ack_of(out):
    assert len(out) == 1 and out[0][0] == 'llc/ack'
    return p.decode_telemetry('llc/ack', out[0][1])


def test_velocity_command_moves_and_watchdog_stops(mcu):
    for i in range(20):                       # 1 s a 20 Hz
        vel(mcu, i, 0.3)
        run(mcu, 0.05)
    assert mcu.vx == pytest.approx(0.3)
    assert mcu.x > 0.15
    run(mcu, 0.25 + 0.3 + 0.05)               # expira valid_ms y frena con rampa
    assert mcu.vx == 0.0
    assert mcu.wd_tripped
    status = p.decode_telemetry('llc/status', mcu.telemetry('llc/status'))
    assert status['wd'] is True


def test_invalid_velocity_stops_immediately(mcu):
    vel(mcu, 1, 0.3)
    run(mcu, 0.1)
    bad = json.dumps({'v': 1, 'boot_id': mcu.boot_id, 'seq': 2, 'vx': 9.0, 'vy': 0,
                      'wz': 0, 'valid_ms': 250})
    assert mcu.handle('cmd/vel', bad) == []
    assert not mcu.cmd_active


def test_retained_and_foreign_boot_commands_rejected(mcu):
    payload = p.encode_command('cmd/vel', mcu.boot_id, 1, vx=0.3, vy=0, wz=0, valid_ms=250)
    mcu.handle('cmd/vel', payload, retain=True)
    assert not mcu.cmd_active
    foreign = p.encode_command('cmd/estop', 'otherboot', 5, active=True)
    a = ack_of(mcu.handle('cmd/estop', foreign))
    assert a['result'] == 'rejected' and not mcu.estop


def test_duplicate_seq_not_executed_twice(mcu):
    cmd = p.encode_command('cmd/odom_reset', mcu.boot_id, 9, x=1.0, y=2.0, yaw=0.5)
    assert ack_of(mcu.handle('cmd/odom_reset', cmd))['result'] == 'ok'
    mcu.x = 5.0
    assert ack_of(mcu.handle('cmd/odom_reset', cmd))['result'] == 'duplicate'
    assert mcu.x == 5.0


def test_arm_requires_enable_and_reaches_goal(mcu):
    goal = p.encode_command('cmd/arm', mcu.boot_id, 1, op='goal', pos=[4096, -2048, 1000, 0],
                            acc=[200.0] * 4, dec=[200.0] * 4)
    assert ack_of(mcu.handle('cmd/arm', goal))['result'] == 'rejected'
    enable = p.encode_command('cmd/arm', mcu.boot_id, 2, op='enable')
    assert ack_of(mcu.handle('cmd/arm', enable))['result'] == 'ok'
    goal = p.encode_command('cmd/arm', mcu.boot_id, 3, op='goal', pos=[4096, -2048, 1000, 0],
                            acc=[200.0] * 4, dec=[200.0] * 4)
    assert ack_of(mcu.handle('cmd/arm', goal))['result'] == 'ok'
    run(mcu, 3.0)
    arm = p.decode_telemetry('llc/arm', mcu.telemetry('llc/arm'))
    assert arm['pos'] == [4096, -2048, 1000, 0]
    assert all(arm['in_pos'])


def test_gripper_detects_grasp_by_current(mcu):
    cmd = p.encode_command('cmd/gripper', mcu.boot_id, 1, pos=1.0, max_cur=0.8)
    assert ack_of(mcu.handle('cmd/gripper', cmd))['result'] == 'ok'
    run(mcu, 1.5)
    arm = p.decode_telemetry('llc/arm', mcu.telemetry('llc/arm'))
    assert arm['grip']['grasped'] is True
    assert arm['grip']['pos'] == pytest.approx(0.6)


def test_estop_blocks_motion(mcu):
    ack_of(mcu.handle('cmd/estop', p.encode_command('cmd/estop', mcu.boot_id, 1, active=True)))
    vel(mcu, 2, 0.3)
    run(mcu, 0.2)
    assert mcu.vx == 0.0
    status = p.decode_telemetry('llc/status', mcu.telemetry('llc/status'))
    assert status['state'] == 'estop' and status['estop']


def test_all_telemetry_is_valid(mcu):
    vel(mcu, 1, 0.2, 0.1, 0.5)
    run(mcu, 0.5)
    for topic in ('llc/odom', 'llc/imu', 'llc/wheels', 'llc/arm', 'llc/battery', 'llc/status'):
        data = p.decode_telemetry(topic, mcu.telemetry(topic))
        assert data['boot_id'] == 'testboot'
