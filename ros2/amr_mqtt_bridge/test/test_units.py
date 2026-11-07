"""Pruebas de time_sync, battery, omni y arm_units (sin ROS ni MQTT)."""
import math
import random

import pytest

from amr_mqtt_bridge.arm_units import ArmUnits
from amr_mqtt_bridge.battery import battery_level, soc_from_voltage
from amr_mqtt_bridge.omni import OmniKinematics, integrate_pose
from amr_mqtt_bridge.time_sync import TimeSync


# ------------------------------------------------------------------ time_sync
def test_time_sync_uses_minimum_latency():
    ts = TimeSync(window_s=5.0)
    base = 1000.0
    rng = random.Random(1)
    stamps = []
    for i in range(200):
        t_ms = i * 20
        latency = 0.002 + rng.random() * 0.030       # 2..32 ms de jitter
        stamps.append(ts.stamp('boot', t_ms, base + t_ms / 1000.0 + latency, 'odom'))
    # Tras el transitorio el offset converge a la latencia mínima (~2 ms).
    assert ts.offset == pytest.approx(base + 0.002, abs=0.002)
    assert all(b > a for a, b in zip(stamps, stamps[1:]))


def test_time_sync_never_in_future_and_resets_on_boot():
    ts = TimeSync()
    assert ts.stamp('a', 5000, 100.0) <= 100.0
    assert ts.stamp('a', 4000, 100.1) is None          # t_ms retrocede: muestra vieja
    s = ts.stamp('b', 10, 101.0)                        # nuevo arranque: uptime reinicia
    assert s == pytest.approx(101.0)


def test_time_sync_streams_are_independent():
    ts = TimeSync()
    assert ts.stamp('a', 100, 10.0, 'odom') is not None
    assert ts.stamp('a', 90, 10.001, 'imu') is not None
    assert ts.stamp('a', 90, 10.002, 'imu') is None


# -------------------------------------------------------------------- battery
@pytest.mark.parametrize('volt,expected', [(25.2, 1.0), (30.0, 1.0), (19.8, 0.0),
                                           (10.0, 0.0), (6 * 3.82, 0.5)])
def test_soc(volt, expected):
    assert soc_from_voltage(volt, 6) == pytest.approx(expected, abs=1e-9)


def test_soc_monotonic():
    values = [soc_from_voltage(19.0 + i * 0.05, 6) for i in range(140)]
    assert all(b >= a for a, b in zip(values, values[1:]))


def test_battery_level():
    assert battery_level(24.0, 21.6, 21.0) == 'ok'
    assert battery_level(21.5, 21.6, 21.0) == 'low'
    assert battery_level(21.0, 21.6, 21.0) == 'critical'


# ----------------------------------------------------------------------- omni
def test_omni_roundtrip():
    kin = OmniKinematics(0.05, 0.25)
    rng = random.Random(3)
    for _ in range(100):
        v = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-2, 2))
        assert kin.forward(kin.inverse(*v)) == pytest.approx(v, abs=1e-9)


def test_omni_x_configuration_signs():
    kin = OmniKinematics(0.05, 0.25)
    s = math.sqrt(0.5) * 1.0 / 0.05
    # Avance puro en x: ruedas fl/rl negativas, rr/fr positivas (v_i = -sin(a_i) vx).
    assert kin.inverse(1.0, 0.0, 0.0) == pytest.approx([-s, -s, s, s])
    # Giro puro: todas iguales (L wz / r).
    assert kin.inverse(0.0, 0.0, 1.0) == pytest.approx([5.0] * 4)


def test_integrate_pose_circle():
    x = y = yaw = 0.0
    dt = 0.001
    for _ in range(int(2 * math.pi / dt)):
        x, y, yaw = integrate_pose(x, y, yaw, 1.0, 0.0, 1.0, dt)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-3)


# ------------------------------------------------------------------ arm_units
def make_units(**kw):
    args = dict(counts_per_rev=16384, gear_ratio=1.0, zero_offset=[0, 100, -50, 0],
                sign=[1, -1, 1, 1], lower=[-3.2] * 4, upper=[3.2] * 4)
    args.update(kw)
    return ArmUnits(**args)


def test_arm_counts_rad_roundtrip():
    u = make_units()
    q = [0.1, -0.5, 1.2, -3.0]
    assert u.counts_to_rad(u.rad_to_counts(q)) == pytest.approx(q, abs=2 * math.pi / 16384)
    assert u.rad_to_counts([math.pi / 2, 0, 0, 0])[0] == 4096


def test_arm_gear_ratio_and_accel():
    u = make_units(gear_ratio=8.0)
    assert u.rad_to_counts([2 * math.pi, 0, 0, 0])[0] == 8 * 16384
    assert u.rad_s2_to_rpm_s([2 * math.pi] * 4)[0] == pytest.approx(8 * 60.0)
    assert u.rpm_to_rad_s([60.0, 60.0, 0, 0])[:2] == pytest.approx([2 * math.pi / 8, -2 * math.pi / 8])


def test_arm_limits():
    assert make_units().check_limits([0, 0, 3.3, -3.3]) == [2, 3]
