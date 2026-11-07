"""Leemos el texto de la NXP sin abrir puertos ni cargar la interfaz."""
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
import csv
import math
import re

NUMBER = r"(?:NA|[+-]?\d+)"
INT32_MIN = -2147483648
INT32_MAX = 2147483647
DEFAULT_COUNTS_PER_REV = 3332
LINE = re.compile(
    rf"^n=(\d+) pos=({NUMBER}) counts speed=({NUMBER}) counts/s "
    r"Vbat=(NA|\d+(?:\.\d+)?) V status\(pos,speed,V\)=(\d+),(\d+),(\d+)"
    r"(?P<extra>(?: [a-z_]+=(?:NA|[+-]?\d+))*)$"
)


@dataclass(frozen=True)
class Sample:
    n: int
    position_counts: int | None
    speed_cps: int | None
    battery_v: float | None
    position_status: int
    speed_status: int
    voltage_status: int
    target_cps: int | None = None
    reference_cps: int | None = None
    error_cps: int | None = None
    duty_pct: float | None = None
    dt_us: int | None = None
    armed: int | None = None
    duty_max_pct: int | None = None
    position_target_counts: int | None = None
    position_error_counts: int | None = None
    position_reached: int | None = None
    encoder_raw_counts: int | None = None


def number(value):
    return None if value is None or value == "NA" else int(value)


def counts_to_rpm(speed_cps, counts_per_rev=DEFAULT_COUNTS_PER_REV):
    """RPM del eje cuya vuelta corresponde al factor de conteos indicado."""
    if not math.isfinite(counts_per_rev) or counts_per_rev <= 0:
        raise ValueError("Los conteos por vuelta deben ser positivos y finitos.")
    if speed_cps is None:
        return None
    rpm = speed_cps * 60.0 / counts_per_rev
    if not math.isfinite(rpm):
        raise ValueError("La conversión a RPM queda fuera del rango numérico.")
    return rpm


def parse_line(line: str) -> Sample | None:
    # Normalizamos las etiquetas admitidas antes de interpretar la telemetría.
    for current, previous in ((" conteos velocidad=", " counts speed="),
                              (" conteos/s voltaje=", " counts/s Vbat="),
                              (" estado(pos,vel,V)=", " status(pos,speed,V)="),
                              (" consigna=", " target="),
                              (" duty_milesimas=", " duty_milli_pct="),
                              (" intervalo_us=", " dt_us=")):
        line = line.replace(current, previous)
    match = LINE.fullmatch(line.strip())
    if not match:
        return None
    n, position, speed, voltage, ps, ss, vs = match.groups()[:7]
    extra = dict(item.split("=", 1) for item in match.group("extra").split())
    duty = number(extra.get("duty_milli_pct"))
    return Sample(
        int(n), number(position) if ps == "1" else None,
        number(speed) if ss == "1" else None,
        float(voltage) if vs == "1" and voltage != "NA" else None,
        int(ps), int(ss), int(vs), number(extra.get("target")),
        number(extra.get("ref")), number(extra.get("error")),
        None if duty is None else duty / 1000.0, number(extra.get("dt_us")),
        number(extra.get("armado")),
        number(extra.get("duty_maximo")),
        number(extra.get("destino_pos")), number(extra.get("error_pos")) if ps == "1" else None,
        number(extra.get("en_posicion")) if ps == "1" and ss == "1" else None,
        number(extra.get("encoder_bruto")) if ps == "1" else None,
    )


CSV_NAMES = {
    "utc": "fecha_utc", "elapsed_s": "tiempo_s", "n": "muestra",
    "position_counts": "posicion_conteos", "speed_cps": "velocidad_conteos_s",
    "battery_v": "voltaje_v", "position_status": "estado_posicion",
    "speed_status": "estado_velocidad", "voltage_status": "estado_voltaje",
    "target_cps": "consigna_conteos_s", "reference_cps": "referencia_conteos_s",
    "error_cps": "error_conteos_s", "duty_pct": "duty_porcentaje",
    "dt_us": "intervalo_us", "armed": "armado",
    "duty_max_pct": "duty_maximo_porcentaje",
    "position_target_counts": "destino_posicion_conteos", "position_error_counts": "error_posicion_conteos",
    "position_reached": "en_posicion", "encoder_raw_counts": "encoder_bruto_conteos",
}
FIELDS = list(CSV_NAMES.values())


class CsvLog:
    """Cada ensayo va en un archivo nuevo; no sobrescribimos mediciones."""
    def __init__(self, path):
        self.file = open(path, "x", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()

    def write(self, sample, elapsed):
        values = {"utc": datetime.now(timezone.utc).isoformat(),
                  "elapsed_s": round(elapsed, 6), **asdict(sample)}
        self.writer.writerow({CSV_NAMES[key]: value for key, value in values.items()})
        self.file.flush()

    def close(self):
        self.file.close()


def read_csv(path):
    """Leemos los formatos CSV admitidos. Un dato vacío sigue siendo un hueco."""
    result = []
    with open(path, newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        spanish = "tiempo_s" in (reader.fieldnames or [])
        names = CSV_NAMES if spanish else {key: key for key in CSV_NAMES}
        required = {names[key] for key in ("utc", "elapsed_s", "n", "position_counts", "speed_cps",
                    "battery_v", "position_status", "speed_status", "voltage_status", "target_cps",
                    "reference_cps", "error_cps", "duty_pct", "dt_us")}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("El CSV no corresponde a la consola S32K312.")
        for row in reader:
            values = {}
            for key in Sample.__dataclass_fields__:
                value = row.get(names[key], "")
                if not value:
                    values[key] = None
                elif key in ("battery_v", "duty_pct"):
                    values[key] = float(value)
                else:
                    values[key] = int(value)
            elapsed = float(row[names["elapsed_s"]])
            if not math.isfinite(elapsed) or any(
                isinstance(v, float) and not math.isfinite(v) for v in values.values()
            ):
                raise ValueError("Valores no finitos en CSV.")
            result.append((elapsed, Sample(**values)))
    return result


def normalize_command(text):
    """Validamos antes de enviar: una línea, parámetros enteros y rangos."""
    if "\n" in text or "\r" in text:
        raise ValueError("Enviar un solo comando por vez.")
    parts = text.strip().upper().split()
    if not parts:
        return None
    name = parts[0]
    arity = {"ARM": 0, "STOP": 0, "HELP": 0, "INFO": 0,
             "VEL": 1, "PID": 3, "WINDOW": 2, "LIMITES": 2, "DUTYMAX": 1,
             "ZERO": 0, "POS": 1, "PPID": 2, "TOL": 1}
    if name not in arity or len(parts) != arity[name] + 1:
        raise ValueError("Usar INFO, HELP, ARM, STOP, VEL, PID, LIMITES, DUTYMAX, ZERO, WINDOW, POS, PPID o TOL.")
    try:
        values = [int(part) for part in parts[1:]]
    except ValueError as exc:
        raise ValueError("Los parametros deben ser enteros; PID usa milesimas.") from exc
    if any(v < INT32_MIN or v > INT32_MAX for v in values):
        raise ValueError("Parametro fuera de int32.")
    if name == "PID" and any(v < 0 or v > 1000 for v in values):
        raise ValueError("Ganancias PID: 0..1000, divididas entre 1000 en NXP.")
    if name == "DUTYMAX" and not 0 <= values[0] <= 100:
        raise ValueError("DUTYMAX requiere un porcentaje entero entre 0 y 100.")
    if name == "PPID" and any(v < 0 or v > 100000 for v in values):
        raise ValueError("PPID: ganancias 0..100000 milésimas (0 a 100).")
    if name == "TOL" and values[0] <= 0:
        raise ValueError("TOL requiere conteos positivos.")
    if name == "WINDOW" and values[0] >= values[1]:
        raise ValueError("WINDOW requiere min < max.")
    if name == "LIMITES" and not (values[0] <= 0 <= values[1] and values[0] < values[1]):
        raise ValueError("LIMITES: minimo <= 0 <= maximo; minimo < maximo; enteros int32.")
    return " ".join([name, *map(str, values)])
