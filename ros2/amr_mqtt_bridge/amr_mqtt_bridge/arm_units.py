"""Conversión entre unidades nativas del GIM6010-8 (counts, rpm) y SI (rad, rad/s).

counts_per_rev y gear_ratio describen el eje en el que el motor reporta la
posición: si los 16384 counts/rev ya son del eje de salida, gear_ratio = 1.
"""
import math


class ArmUnits:
    def __init__(self, counts_per_rev, gear_ratio, zero_offset, sign, lower, upper):
        n = len(zero_offset)
        if not (len(sign) == len(lower) == len(upper) == n):
            raise ValueError('zero_offset, sign, lower y upper deben tener el mismo largo')
        if counts_per_rev <= 0 or gear_ratio <= 0:
            raise ValueError('counts_per_rev y gear_ratio deben ser positivos')
        if any(s not in (-1, 1) for s in sign):
            raise ValueError('sign debe ser +1 o -1')
        self.n = n
        self.counts_per_rad = counts_per_rev * gear_ratio / (2.0 * math.pi)
        self.gear_ratio = float(gear_ratio)
        self.zero = [int(z) for z in zero_offset]
        self.sign = [int(s) for s in sign]
        self.lower = [float(v) for v in lower]
        self.upper = [float(v) for v in upper]

    def counts_to_rad(self, counts):
        return [s * (c - z) / self.counts_per_rad
                for c, z, s in zip(counts, self.zero, self.sign)]

    def rad_to_counts(self, rad):
        return [int(round(z + s * q * self.counts_per_rad))
                for q, z, s in zip(rad, self.zero, self.sign)]

    def rpm_to_rad_s(self, rpm):
        return [s * r * 2.0 * math.pi / 60.0 / self.gear_ratio for r, s in zip(rpm, self.sign)]

    def rad_s2_to_rpm_s(self, acc):
        """Aceleración articular (magnitud) -> rpm/s en el eje del motor."""
        return [abs(a) * self.gear_ratio * 60.0 / (2.0 * math.pi) for a in acc]

    def check_limits(self, rad):
        bad = [i for i, q in enumerate(rad) if not self.lower[i] <= q <= self.upper[i]]
        return bad
