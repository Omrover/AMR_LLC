"""Valida con el decodificador Python la telemetría formateada por amr_contract (C)."""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]          # AMR_LLC/
sys.path.insert(0, str(ROOT / 'ros2' / 'amr_mqtt_bridge'))
from amr_mqtt_bridge import protocol as p  # noqa: E402


def main(path):
    lines = [ln for ln in pathlib.Path(path).read_text().splitlines() if ln]
    failures = 0
    for line in lines:
        topic, payload = line.split('\t', 1)
        try:
            data = p.decode_telemetry(topic, payload)
        except p.ProtocolError as exc:
            print(f'FAIL {topic}: {exc}\n  {payload}')
            failures += 1
            continue
        if data.get('state') != 'offline' and data['t_ms'] != 123456:
            print(f'FAIL {topic}: t_ms inesperado')
            failures += 1
    if len(lines) < 8:
        print(f'FAIL: se esperaban 8 mensajes, hay {len(lines)}')
        failures += 1
    print(f'telemetría C -> Python: {len(lines) - failures}/{len(lines)} válidos')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
