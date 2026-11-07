"""Estado de carga de la batería LiPo 6S 6000 mAh a partir de la tensión.

La LUT es de tensión por celda en reposo/carga ligera; bajo carga la tensión
cae y el porcentaje se subestima (conservador). Los umbrales de decisión del
BT y del bridge se fijan en voltios (bridge.yaml) para no depender de la LUT.
"""
from bisect import bisect_left

# (V por celda, fracción 0..1)
LIPO_CELL_LUT = (
    (3.30, 0.00), (3.50, 0.05), (3.68, 0.10), (3.73, 0.20), (3.77, 0.30),
    (3.79, 0.40), (3.82, 0.50), (3.87, 0.60), (3.92, 0.70), (3.97, 0.80),
    (4.06, 0.90), (4.20, 1.00),
)


def soc_from_voltage(volt, cells=6, lut=LIPO_CELL_LUT):
    """Interpolación lineal de la LUT; satura en 0 y 1."""
    if cells <= 0:
        raise ValueError('cells debe ser positivo')
    cell = volt / cells
    xs = [p[0] for p in lut]
    if cell <= xs[0]:
        return lut[0][1]
    if cell >= xs[-1]:
        return lut[-1][1]
    i = bisect_left(xs, cell)
    (x0, y0), (x1, y1) = lut[i - 1], lut[i]
    return y0 + (y1 - y0) * (cell - x0) / (x1 - x0)


def battery_level(volt, low_volt, critical_volt):
    """'critical', 'low' o 'ok' según los umbrales en voltios."""
    if volt <= critical_volt:
        return 'critical'
    if volt <= low_volt:
        return 'low'
    return 'ok'
