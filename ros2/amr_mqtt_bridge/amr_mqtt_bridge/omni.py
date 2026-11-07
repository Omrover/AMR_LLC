"""Cinemática de la base omnidireccional de 4 ruedas (configuración X).

Rueda i en (L cos a_i, L sin a_i) con eje radial y tracción tangencial
u_i = (-sin a_i, cos a_i). Velocidad de superficie de la rueda:

    v_i = -sin(a_i) vx + cos(a_i) vy + L wz,     w_i = v_i / r

La directa usa mínimos cuadrados (pseudo-inversa), válida para cualquier
conjunto de ángulos. Misma convención que firmware (AMR_localization/amr_odom).
"""
import math


class OmniKinematics:
    def __init__(self, wheel_radius, center_distance, angles_deg=(45.0, 135.0, 225.0, 315.0)):
        if wheel_radius <= 0 or center_distance <= 0:
            raise ValueError('wheel_radius y center_distance deben ser positivos')
        self.r = float(wheel_radius)
        self.L = float(center_distance)
        self.angles = [math.radians(a) for a in angles_deg]
        # Jacobiano J (n x 3) en velocidad de superficie: v = J [vx, vy, wz]
        self.J = [(-math.sin(a), math.cos(a), self.L) for a in self.angles]
        jtj = [[sum(row[i] * row[j] for row in self.J) for j in range(3)] for i in range(3)]
        self._jtj_inv = _inv3(jtj)

    def inverse(self, vx, vy, wz):
        """Velocidad del robot (base_footprint) -> velocidad angular de cada rueda [rad/s]."""
        return [(jx * vx + jy * vy + jw * wz) / self.r for jx, jy, jw in self.J]

    def forward(self, wheel_rad_s):
        """Velocidades angulares de rueda -> (vx, vy, wz) por mínimos cuadrados."""
        if len(wheel_rad_s) != len(self.J):
            raise ValueError('número de ruedas incorrecto')
        v = [w * self.r for w in wheel_rad_s]
        jtv = [sum(self.J[k][i] * v[k] for k in range(len(v))) for i in range(3)]
        return tuple(sum(self._jtj_inv[i][j] * jtv[j] for j in range(3)) for i in range(3))


def _inv3(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise ValueError('configuración de ruedas singular')
    return [
        [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
        [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
        [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
    ]


def integrate_pose(x, y, yaw, vx, vy, wz, dt):
    """Integra una velocidad en el frame del robot (punto medio en yaw)."""
    mid = yaw + 0.5 * wz * dt
    x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
    y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
    yaw = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))
    return x, y, yaw
