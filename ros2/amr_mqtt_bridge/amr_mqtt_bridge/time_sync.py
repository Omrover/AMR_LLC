"""Mapea el uptime del MCU (t_ms) a tiempo de la NUC.

offset = min(t_rx - t_mcu) en una ventana deslizante. El mínimo corresponde a
la muestra con menor latencia de red, así el jitter de TCP/MQTT no entra en
los timestamps. La ventana permite seguir la deriva lenta entre relojes. Todo
se reinicia con cada boot_id, porque el uptime vuelve a cero tras un reset.

El offset es común a todos los streams (todos usan el mismo reloj del MCU).
La monotonicidad se exige por stream (odom, imu, ...).
"""
from collections import deque


class TimeSync:
    def __init__(self, window_s=5.0):
        self.window_s = float(window_s)
        self.boot_id = None
        self._samples = deque()       # (rx_time_s, offset_s)
        self._last = {}               # stream -> (t_mcu_s, stamp_s)

    def reset(self):
        self._samples.clear()
        self._last.clear()

    @property
    def offset(self):
        return min(o for _, o in self._samples) if self._samples else None

    def stamp(self, boot_id, t_ms, rx_time_s, stream='default'):
        """Devuelve el timestamp de la muestra en segundos del reloj de la NUC.
        Devuelve None si la muestra es vieja (t_ms retrocede) dentro del mismo arranque."""
        if boot_id != self.boot_id:
            self.boot_id = boot_id
            self.reset()
        t_mcu = t_ms / 1000.0
        last = self._last.get(stream)
        if last is not None and t_mcu <= last[0]:
            return None
        self._samples.append((rx_time_s, rx_time_s - t_mcu))
        while rx_time_s - self._samples[0][0] > self.window_s:
            self._samples.popleft()
        stamp = min(t_mcu + self.offset, rx_time_s)   # nunca en el futuro
        if last is not None and stamp <= last[1]:
            stamp = last[1] + 1e-6                     # monotónico por stream
        self._last[stream] = (t_mcu, stamp)
        return stamp
