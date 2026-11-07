"""Recibimos datos en otro hilo para no frenar los botones ni las gráficas."""
from dataclasses import dataclass
from pathlib import Path
import queue
import threading
import time
from telemetry import CsvLog, parse_line


@dataclass(frozen=True)
class Received:
    elapsed: float
    line: str


def list_ports():
    from serial.tools.list_ports import comports
    for port in comports():
        print(f"{port.device}: {port.description}")


class SerialLink:
    def __init__(self, port, baud=115200, log=None, *, connection=None):
        self.log = None
        self.serial_log = None
        self._log_lock = threading.Lock()
        try:
            if log:
                csv_path = Path(log)
                self.serial_log_path = csv_path.with_suffix(".serial.txt")
                # Ninguno de los dos registros puede reemplazar un ensayo existente.
                for path in (csv_path, self.serial_log_path):
                    if path.exists():
                        raise FileExistsError(f"Ya existe el registro: {path}")
                self.log = CsvLog(csv_path)
                self.serial_log = self.serial_log_path.open("x", encoding="utf-8")
                self.serial_log.write("tiempo_s sentido linea\n")
                self.serial_log.flush()
            if connection is None:
                import serial
                connection = serial.Serial(port, baud, timeout=0.05, write_timeout=0.2)
            self.port = connection
        except Exception:
            if self.log:
                self.log.close()
            if self.serial_log:
                self.serial_log.close()
            raise
        self.events = queue.Queue(maxsize=8192)
        self.error = None
        self.dropped = 0
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._closed = False
        self.started = time.monotonic()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _record_line(self, elapsed, direction, line):
        if self.serial_log:
            with self._log_lock:
                self.serial_log.write(f"{elapsed:.6f} {direction} {line}\n")
                self.serial_log.flush()

    def _reader(self):
        buffer = bytearray()
        try:
            while not self._stop.is_set():
                buffer.extend(self.port.read(min(self.port.in_waiting or 1, 4096)))
                while b"\n" in buffer:
                    raw, _, rest = buffer.partition(b"\n")
                    buffer = bytearray(rest)
                    if len(raw) > 4096:
                        raise RuntimeError("Linea serial demasiado larga.")
                    line = raw.decode("ascii", errors="replace").strip()
                    elapsed = time.monotonic() - self.started
                    # Guardamos también fallos y acuses que no son muestras del CSV.
                    self._record_line(elapsed, "RECIBIDO", line)
                    sample = parse_line(line)
                    if sample and self.log:
                        self.log.write(sample, elapsed)
                    try:
                        self.events.put_nowait(Received(elapsed, line))
                    except queue.Full:
                        self.dropped += 1  # El CSV ya guardó el dato, aunque no se muestre.
                if len(buffer) > 4096:
                    raise RuntimeError("Serial sin newline (fin de linea) o trama corrupta.")
        except Exception as exc:
            if not self._stop.is_set():
                self.error = str(exc)

    def send(self, line):
        if not line or len(line) > 63 or "\n" in line or "\r" in line:
            raise ValueError("Comando ASCII unico de 1..63 caracteres.")
        data = (line + "\n").encode("ascii")
        with self._write_lock:
            if self.port.write(data) != len(data):
                raise OSError("Escritura serial incompleta.")
            self._record_line(time.monotonic() - self.started, "ENVIADO", line)

    def close(self, *, stop=False):
        """Intentamos STOP al cerrar; eso no confirma que el motor se detuvo."""
        if self._closed:
            return None
        self._closed = True
        warning = None
        if stop:
            try:
                self.send("STOP")
            except Exception as exc:
                warning = f"No se pudo enviar STOP: {exc}. Usar SW2/parada fisica."
        self._stop.set()
        self.thread.join(timeout=1.0)
        self.port.close()
        self.thread.join(timeout=1.0)
        if self.log:
            self.log.close()
        if self.serial_log:
            self.serial_log.close()
        return warning
