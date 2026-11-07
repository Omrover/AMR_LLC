import json
import math
import pathlib

import pytest

from amr_mqtt_bridge import protocol as p

FIXTURES = pathlib.Path(__file__).resolve().parents[3] / 'protocol' / 'fixtures' / 'commands.txt'
BOOT = 'b00710c0ffee0001'


def load_fixtures():
    if not FIXTURES.exists():
        return []
    rows = []
    for line in FIXTURES.read_text(encoding='utf-8').splitlines():
        if not line.strip() or line.startswith('#'):
            continue
        topic, verdict, payload = line.split('\t', 2)
        rows.append(pytest.param(topic, verdict, payload, id=f'{topic}:{len(rows)}'))
    return rows


@pytest.mark.skipif(not FIXTURES.exists(), reason='fixtures del contrato no encontrados')
@pytest.mark.parametrize('topic,verdict,payload', load_fixtures())
def test_command_fixtures(topic, verdict, payload):
    if verdict == 'valid':
        cmd = p.decode_command(topic, payload, BOOT)
        assert cmd['boot_id'] == BOOT
    else:
        with pytest.raises(p.ProtocolError):
            p.decode_command(topic, payload, BOOT)


def test_encode_command_roundtrip():
    text = p.encode_command('cmd/vel', BOOT, 7, vx=0.1, vy=-0.2, wz=0.3, valid_ms=250)
    assert json.loads(text) == {'v': 1, 'boot_id': BOOT, 'seq': 7, 'vx': 0.1, 'vy': -0.2,
                                'wz': 0.3, 'valid_ms': 250}
    assert p.decode_command('cmd/vel', text, BOOT)['vy'] == -0.2


@pytest.mark.parametrize('fields', [
    dict(vx=3.0, vy=0.0, wz=0.0, valid_ms=250),
    dict(vx=math.nan, vy=0.0, wz=0.0, valid_ms=250),
    dict(vx=0.0, vy=0.0, wz=0.0, valid_ms=300),
    dict(vx=0.0, vy=0.0, wz=0.0),
    dict(vx=0.0, vy=0.0, wz=0.0, valid_ms=250, extra=1),
])
def test_encode_command_rejects(fields):
    with pytest.raises((p.ProtocolError, ValueError)):
        p.encode_command('cmd/vel', BOOT, 1, **fields)


def test_arm_non_goal_rejects_goal_fields():
    payload = json.dumps({'v': 1, 'boot_id': BOOT, 'seq': 1, 'op': 'enable', 'pos': [0, 0, 0, 0]})
    with pytest.raises(p.ProtocolError):
        p.decode_command('cmd/arm', payload, BOOT)


TELEMETRY_SAMPLES = {
    'llc/odom': dict(x=1.0, y=-2.0, yaw=0.5, vx=0.1, vy=0.0, wz=0.2, cov=[0.001] * 6),
    'llc/imu': dict(q=[1.0, 0.0, 0.0, 0.0], gyro=[0.0, 0.0, 0.1], acc=[0.0, 0.0, 9.81],
                    acc_status=3),
    'llc/wheels': dict(pos=[1.0, 2.0, 3.0, 4.0], vel=[0.1, 0.2, 0.3, 0.4], rc_err=[0, 0]),
    'llc/arm': dict(enabled=True, pos=[0, 100, -100, 16384], vel=[0.0] * 4, cur=[0.1] * 4,
                    temp=[30.0] * 4, in_pos=[True] * 4, fault=[0] * 4,
                    grip={'pos': 0.5, 'cur': 0.2, 'grasped': False}),
    'llc/battery': dict(volt=24.1),
    'llc/status': dict(state='ok', estop=False, faults=0, wd=False, cpu=0.2),
    'llc/ack': dict(seq=5, cmd='arm', result='ok'),
}


@pytest.mark.parametrize('topic', sorted(TELEMETRY_SAMPLES))
def test_telemetry_roundtrip(topic):
    text = p.encode_telemetry(topic, BOOT, 1234, **TELEMETRY_SAMPLES[topic])
    data = p.decode_telemetry(topic, text)
    assert data['boot_id'] == BOOT and data['t_ms'] == 1234
    for key, value in TELEMETRY_SAMPLES[topic].items():
        assert data[key] == value


def test_telemetry_rejects_unknown_and_missing_fields():
    good = json.loads(p.encode_telemetry('llc/battery', BOOT, 1, volt=24.0))
    with pytest.raises(p.ProtocolError):
        p.decode_telemetry('llc/battery', json.dumps({**good, 'foo': 1}))
    del good['volt']
    with pytest.raises(p.ProtocolError):
        p.decode_telemetry('llc/battery', json.dumps(good))


def test_telemetry_rejects_bad_boot_id_and_version():
    with pytest.raises(p.ProtocolError):
        p.decode_telemetry('llc/battery', '{"v":1,"boot_id":"bad id!","t_ms":1,"volt":24}')
    with pytest.raises(p.ProtocolError):
        p.decode_telemetry('llc/battery', '{"v":true,"boot_id":"abc","t_ms":1,"volt":24}')
    with pytest.raises(p.ProtocolError):
        p.decode_telemetry('llc/battery', '{"v":1,"boot_id":"abc","t_ms":-1,"volt":24}')


def test_lwt():
    assert p.decode_telemetry('llc/status', p.encode_lwt()) == {'state': 'offline'}
    with pytest.raises(p.ProtocolError):
        p.decode_telemetry('llc/status', '{"v":1,"state":"offline","x":1}')


def test_payload_size_limit():
    with pytest.raises(p.ProtocolError):
        p.decode_object(b'{"v":1,"pad":"' + b'x' * 1100 + b'"}')


def test_fault_names():
    assert p.fault_names(0b1000_0101) == ['roboclaw_128', 'imu', 'estop_hw']
