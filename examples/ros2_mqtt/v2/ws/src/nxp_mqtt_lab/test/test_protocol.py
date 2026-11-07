import json
import math

from nxp_mqtt_lab import exercises_solution as solution
from nxp_mqtt_lab.protocol import encode_command, encoder_sample, pwm_command
import pytest


def sample(ticks, uptime_ms, boot_id='boot-a'):
    return {'ticks': ticks, 'uptime_ms': uptime_ms, 'boot_id': boot_id}


def test_speed_and_recovery_boundaries():
    a, b = sample(10, 100), sample(70, 200)
    assert solution.wheel_speed(a, b, 600) == pytest.approx(2 * math.pi)
    assert solution.wheel_speed(None, b, 600) is None
    assert solution.wheel_speed(a, sample(70, 100), 600) is None
    assert solution.wheel_speed(a, sample(70, 90), 600) is None
    assert solution.wheel_speed(a, sample(0, 200, 'boot-b'), 600) is None
    assert solution.wheel_speed(a, sample(-50, 200), 600) == pytest.approx(-2 * math.pi)


@pytest.mark.parametrize('payload', [
    b'bad', b'\xff', b'[]', b'{}',
    b'{"ticks": true, "uptime_ms": 1, "boot_id": "a"}',
    b'{"ticks": 1, "uptime_ms": -1, "boot_id": "a"}',
    json.dumps(sample(2**63, 1)).encode(),
    json.dumps(sample(1, 1, '')).encode(),
    b' ' * 1025,
])
def test_bad_telemetry(payload):
    with pytest.raises(ValueError):
        encoder_sample(payload)


def test_command_bounds_and_boot():
    payload = encode_command(0.1, -0.1, 'a')
    assert pwm_command(payload, 'a') == (0.1, -0.1, 250)
    with pytest.raises(ValueError):
        pwm_command(payload, 'b')
    for value in (float('nan'), float('inf'), 0.21, True, 10**400):
        with pytest.raises(ValueError):
            solution.pwm_pair(value)
    obj = json.loads(payload)
    obj['left'] = 10**400
    with pytest.raises(ValueError):
        pwm_command(json.dumps(obj), 'a')
    for timeout in (0, 251, True):
        obj = json.loads(payload)
        obj['valid_ms'] = timeout
        with pytest.raises(ValueError):
            pwm_command(json.dumps(obj), 'a')


def test_student_contract_when_requested():
    import os
    if os.environ.get('CHECK_STUDENT') != '1':
        pytest.skip('Set CHECK_STUDENT=1 after completing exercises.py')
    from nxp_mqtt_lab import exercises
    a, b = sample(0, 100), sample(60, 200)
    for previous, current in (
        (None, b), (a, b), (b, a), (a, sample(0, 100)),
        (a, sample(0, 200, 'new')), (b, sample(0, 300)),
    ):
        actual = exercises.wheel_speed(previous, current, 600)
        expected = solution.wheel_speed(previous, current, 600)
        if expected is None:
            assert actual is None
        else:
            assert actual == pytest.approx(expected)
