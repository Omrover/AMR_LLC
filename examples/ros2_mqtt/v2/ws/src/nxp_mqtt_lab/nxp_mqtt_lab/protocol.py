"""The proposed lab contract; adapt at this boundary for existing firmware."""
import json
import math

MAX_DUTY = 0.2
WATCHDOG_MS = 250


def decode_object(payload):
    if len(payload) > 1024:
        raise ValueError('payload exceeds 1024 bytes')
    try:
        value = json.loads(payload)
    except (ValueError, UnicodeError) as exc:
        raise ValueError('invalid JSON') from exc
    if not isinstance(value, dict):
        raise ValueError('expected JSON object')
    return value


def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError('integer outside allowed range')
    return value


def encoder_sample(payload):
    obj = decode_object(payload)
    try:
        ticks = integer(obj['ticks'], -(2**63), 2**63 - 1)
        uptime_ms = integer(obj['uptime_ms'], 0, 2**63 - 1)
        boot_id = obj['boot_id']
    except KeyError as exc:
        raise ValueError('missing encoder field') from exc
    if not isinstance(boot_id, str) or not 1 <= len(boot_id) <= 64:
        raise ValueError('invalid boot_id')
    return {'ticks': ticks, 'uptime_ms': uptime_ms, 'boot_id': boot_id}


def duty(value):
    if (type(value) not in (int, float) or abs(value) > MAX_DUTY
            or not math.isfinite(value)):
        raise ValueError('duty must be finite and within [-0.2, 0.2]')
    return float(value)


def pwm_command(payload, boot_id):
    obj = decode_object(payload)
    try:
        if obj['boot_id'] != boot_id:
            raise ValueError('wrong device boot')
        left, right = duty(obj['left']), duty(obj['right'])
        timeout = integer(obj['valid_ms'], 1, WATCHDOG_MS)
    except KeyError as exc:
        raise ValueError('missing command field') from exc
    return left, right, timeout


def encode_command(left, right, boot_id):
    return json.dumps({
        'left': duty(left), 'right': duty(right),
        'valid_ms': WATCHDOG_MS, 'boot_id': boot_id,
    }, allow_nan=False)


def angular_speed(previous, current, ticks_per_rev):
    if ticks_per_rev <= 0:
        raise ValueError('ticks_per_rev must be positive')
    if previous is None or current['boot_id'] != previous['boot_id']:
        return None
    dt = (current['uptime_ms'] - previous['uptime_ms']) / 1000.0
    if dt <= 0:
        return None
    return (current['ticks'] - previous['ticks']) * 2 * math.pi / ticks_per_rev / dt
