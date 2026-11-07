"""Contrato MQTT v1 Tlaloc <-> NUC (ver AMR_LLC/protocol/CONTRACT.md).

Módulo sin dependencias de ROS: lo usan el bridge (decodifica telemetría y
codifica comandos), el emulador del MCU (el lado contrario) y las pruebas.
"""
import json
import math
import re

VERSION = 1
MAX_PAYLOAD = 1024
MAX_VALID_MS = 250
MAX_LINEAR = 2.0          # m/s, límite del contrato (el MCU puede limitar más)
MAX_ANGULAR = 4.0         # rad/s
MAX_ARM_ACCEL = 10000.0   # rpm/s
MAX_GRIP_CURRENT = 5.0    # A
N_WHEELS = 4
N_JOINTS = 4
UINT32_MAX = 2 ** 32 - 1
INT32_MIN, INT32_MAX = -2 ** 31, 2 ** 31 - 1

WHEEL_ORDER = ('fl', 'rl', 'rr', 'fr')
STATES = ('init', 'ok', 'estop', 'fault')
ARM_OPS = ('goal', 'enable', 'disable', 'clear', 'home')
ACK_CMDS = ('arm', 'gripper', 'estop', 'odom_reset')
ACK_RESULTS = ('ok', 'rejected', 'busy', 'duplicate')

FAULT_BITS = {
    0: 'roboclaw_128', 1: 'roboclaw_129', 2: 'imu', 3: 'can_bus',
    4: 'arm_motor', 5: 'gripper_overcurrent', 6: 'battery_critical', 7: 'estop_hw',
}

_BOOT_ID = re.compile(r'^[0-9A-Za-z_-]{1,64}$')


class ProtocolError(ValueError):
    """Payload que no cumple el contrato."""


# --------------------------------------------------------------- validadores

def _is_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def number(low=-math.inf, high=math.inf):
    def check(value):
        if not _is_number(value) or not low <= value <= high:
            raise ProtocolError(f'número fuera de [{low}, {high}]: {value!r}')
        return float(value)
    return check


def integer(low, high):
    def check(value):
        if type(value) is not int or not low <= value <= high:
            raise ProtocolError(f'entero fuera de [{low}, {high}]: {value!r}')
        return value
    return check


def boolean(value):
    if type(value) is not bool:
        raise ProtocolError(f'se esperaba bool: {value!r}')
    return value


def choice(options):
    def check(value):
        if value not in options:
            raise ProtocolError(f'valor no permitido {value!r}; opciones {options}')
        return value
    return check


def text(max_len):
    def check(value):
        if not isinstance(value, str) or len(value) > max_len:
            raise ProtocolError('texto inválido')
        return value
    return check


def array(item, length):
    def check(value):
        if not isinstance(value, list) or len(value) != length:
            raise ProtocolError(f'se esperaba un arreglo de {length} elementos')
        return [item(v) for v in value]
    return check


def boot_id(value):
    if not isinstance(value, str) or not _BOOT_ID.match(value):
        raise ProtocolError('boot_id inválido')
    return value


def obj(spec, optional=()):
    def check(value):
        if not isinstance(value, dict):
            raise ProtocolError('se esperaba un objeto')
        return _fields(value, spec, optional)
    return check


def _fields(value, spec, optional=()):
    out = {}
    for key, validator in spec.items():
        if key not in value:
            if key in optional:
                continue
            raise ProtocolError(f'falta el campo {key!r}')
        try:
            out[key] = validator(value[key])
        except ProtocolError as exc:
            raise ProtocolError(f'{key}: {exc}') from None
    unknown = set(value) - set(spec)
    if unknown:
        raise ProtocolError(f'campos desconocidos: {sorted(unknown)}')
    return out


# ------------------------------------------------------------------ esquemas

_ANY = number(-1e6, 1e6)
_POS = number(-1e4, 1e4)
_YAW = number(-math.pi - 1e-3, math.pi + 1e-3)

TELEMETRY = {
    'llc/odom': {
        'x': _POS, 'y': _POS, 'yaw': _YAW,
        'vx': number(-10, 10), 'vy': number(-10, 10), 'wz': number(-20, 20),
        'cov': array(number(0, 1e6), 6),
    },
    'llc/imu': {
        'q': array(number(-1.1, 1.1), 4),
        'gyro': array(number(-50, 50), 3),
        'acc': array(number(-200, 200), 3),
        'acc_status': integer(0, 3),
    },
    'llc/wheels': {
        'pos': array(_ANY, N_WHEELS),
        'vel': array(number(-1000, 1000), N_WHEELS),
        'rc_err': array(integer(0, UINT32_MAX), 2),
    },
    'llc/arm': {
        'enabled': boolean,
        'pos': array(integer(INT32_MIN, INT32_MAX), N_JOINTS),
        'vel': array(number(-1e5, 1e5), N_JOINTS),
        'cur': array(number(-100, 100), N_JOINTS),
        'temp': array(number(-50, 200), N_JOINTS),
        'in_pos': array(boolean, N_JOINTS),
        'fault': array(integer(0, 255), N_JOINTS),
        'grip': obj({'pos': number(0, 1), 'cur': number(-20, 20), 'grasped': boolean}),
    },
    'llc/battery': {'volt': number(0, 60), 'logic_volt': number(0, 60)},
    'llc/status': {
        'state': choice(STATES), 'estop': boolean, 'faults': integer(0, UINT32_MAX),
        'wd': boolean, 'cpu': number(0, 1),
    },
    'llc/ack': {
        'seq': integer(0, UINT32_MAX), 'cmd': choice(ACK_CMDS),
        'result': choice(ACK_RESULTS), 'msg': text(64),
    },
}
_TELEMETRY_OPTIONAL = {'llc/battery': ('logic_volt',), 'llc/ack': ('msg',)}

COMMANDS = {
    'cmd/vel': {
        'vx': number(-MAX_LINEAR, MAX_LINEAR), 'vy': number(-MAX_LINEAR, MAX_LINEAR),
        'wz': number(-MAX_ANGULAR, MAX_ANGULAR), 'valid_ms': integer(1, MAX_VALID_MS),
    },
    'cmd/arm': {
        'op': choice(ARM_OPS),
        'pos': array(integer(INT32_MIN, INT32_MAX), N_JOINTS),
        'acc': array(number(1e-9, MAX_ARM_ACCEL), N_JOINTS),
        'dec': array(number(1e-9, MAX_ARM_ACCEL), N_JOINTS),
    },
    'cmd/gripper': {'pos': number(0, 1), 'max_cur': number(1e-9, MAX_GRIP_CURRENT)},
    'cmd/estop': {'active': boolean},
    'cmd/odom_reset': {'x': _POS, 'y': _POS, 'yaw': number(-math.pi, math.pi)},
}
_GOAL_ONLY = ('pos', 'acc', 'dec')
ACK_NAME = {'cmd/arm': 'arm', 'cmd/gripper': 'gripper', 'cmd/estop': 'estop',
            'cmd/odom_reset': 'odom_reset'}


# ------------------------------------------------------------------- parsing

def _reject_constant(name):
    raise ProtocolError(f'constante JSON no permitida: {name}')


def _no_duplicates(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ProtocolError(f'clave duplicada {key!r}')
        out[key] = value
    return out


def decode_object(payload):
    if isinstance(payload, str):
        payload = payload.encode('utf-8')
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f'payload de {len(payload)} bytes excede {MAX_PAYLOAD}')
    try:
        value = json.loads(payload, parse_constant=_reject_constant,
                           object_pairs_hook=_no_duplicates)
    except ProtocolError:
        raise
    except (ValueError, UnicodeError) as exc:
        raise ProtocolError('JSON inválido') from exc
    if not isinstance(value, dict):
        raise ProtocolError('se esperaba un objeto JSON')
    if value.get('v') != VERSION or type(value.get('v')) is not int:
        raise ProtocolError(f'versión de contrato no soportada: {value.get("v")!r}')
    return value


def _dumps(obj_):
    text_ = json.dumps(obj_, separators=(',', ':'), allow_nan=False)
    if len(text_.encode('utf-8')) > MAX_PAYLOAD:
        raise ProtocolError('payload excede el máximo del contrato')
    return text_


def decode_telemetry(topic, payload):
    """Valida un mensaje MCU->NUC. Devuelve dict con boot_id, t_ms y los campos.

    El LWT (`llc/status` con state=offline) se devuelve como {'state': 'offline'}.
    """
    if topic not in TELEMETRY:
        raise ProtocolError(f'topic de telemetría desconocido: {topic}')
    value = decode_object(payload)
    if topic == 'llc/status' and value.get('state') == 'offline':
        if set(value) != {'v', 'state'}:
            raise ProtocolError('LWT con campos extra')
        return {'state': 'offline'}
    body = dict(value)
    body.pop('v')
    try:
        header = {'boot_id': boot_id(body.pop('boot_id')),
                  't_ms': integer(0, 2 ** 63 - 1)(body.pop('t_ms'))}
    except KeyError as exc:
        raise ProtocolError(f'falta el campo {exc}') from None
    fields = _fields(body, TELEMETRY[topic], _TELEMETRY_OPTIONAL.get(topic, ()))
    return {**header, **fields}


def encode_telemetry(topic, boot, t_ms, **fields):
    """Lado MCU (emulador/pruebas): construye y valida un mensaje de telemetría."""
    message = {'v': VERSION, 'boot_id': boot, 't_ms': int(t_ms), **fields}
    text_ = _dumps(message)
    decode_telemetry(topic, text_)
    return text_


def encode_lwt():
    return _dumps({'v': VERSION, 'state': 'offline'})


def decode_command(topic, payload, current_boot_id):
    """Lado MCU: valida un comando NUC->MCU contra el boot_id vigente."""
    if topic not in COMMANDS:
        raise ProtocolError(f'topic de comando desconocido: {topic}')
    value = decode_object(payload)
    body = dict(value)
    body.pop('v')
    try:
        boot = boot_id(body.pop('boot_id'))
        seq = integer(0, UINT32_MAX)(body.pop('seq'))
    except KeyError as exc:
        raise ProtocolError(f'falta el campo {exc}') from None
    if boot != current_boot_id:
        raise ProtocolError('boot_id distinto al del MCU (comando de otro arranque)')
    spec = COMMANDS[topic]
    optional = ()
    if topic == 'cmd/arm':
        if body.get('op') != 'goal':
            if any(k in body for k in _GOAL_ONLY):
                raise ProtocolError('pos/acc/dec solo se permiten con op=goal')
            optional = _GOAL_ONLY
    return {'boot_id': boot, 'seq': seq, **_fields(body, spec, optional)}


def encode_command(topic, boot, seq, **fields):
    """Lado NUC: construye y valida un comando. Lanza ProtocolError si no cumple."""
    message = {'v': VERSION, 'boot_id': boot, 'seq': int(seq), **fields}
    text_ = _dumps(message)
    decode_command(topic, text_, boot)
    return text_


def fault_names(mask):
    return [name for bit, name in FAULT_BITS.items() if mask & (1 << bit)]
