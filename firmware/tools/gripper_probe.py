"""Muestrea en vivo el estado interno del gripper en la Tlaloc (pyocd en modo attach:
no detiene ni reinicia la MCU). Imprime cada cambio y un resumen por estado al final.

    ~/.mcuxpressotools/.mcux-venv-3.12/bin/python ~/gripper_probe.py [segundos] [zephyr.elf]

pin_nFAULT es el pin PTE2 leído directo (no la falla filtrada del firmware): si baja
mientras el motor empuja, el DRV8833 está limitando por sobrecorriente.
No usar mientras corre west flash/debug (el probe es uno solo)."""
import os
import re
import struct
import subprocess
import sys
import time

from pyocd.core.helpers import ConnectHelper

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30
elf = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser(
    '~/amr_ws/src/AMR_LLC/firmware/build/zephyr/zephyr.elf')
gdb = os.path.expanduser('~/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb')
fields = ['state', 'pos', 'cur_a', 'duty', 'zero_mv', 'grasped', 'overcurrent', 'driver_fault']
cmd = [gdb, '-batch', '-nx', elf] + [a for f in fields for a in ('-ex', f'p/x &llc.grip.{f}')]
out = subprocess.run(cmd, capture_output=True, text=True).stdout
addr = dict(zip(fields, [int(x, 16) for x in re.findall(r'= (0x[0-9a-f]+)', out)]))
base = min(addr.values())
size = max(addr.values()) - base + 4
# PGPDI del puerto E bajo; PTE2 = bit 13 (la S32K3 numera los bits al revés).
PGPDI_E_L, NFAULT_BIT = 0x40291752, 1 << 13
NAMES = {0: 'IDLE', 1: 'OPENING', 2: 'CLOSING', 3: 'HOLDING', 4: 'BACK_OFF'}
stats = {}
with ConnectHelper.session_with_chosen_probe(
        target_override='s32k344', options={'connect_mode': 'attach'}) as s:
    t = s.target
    t0 = time.time()
    last = None
    while time.time() - t0 < dur:
        b = bytes(t.read_memory_block8(base, size))
        pin_low = not (t.read16(PGPDI_E_L) & NFAULT_BIT)

        def g(f, fmt):
            return struct.unpack_from(fmt, b, addr[f] - base)[0]

        st, pos, cur, duty, zero = (g('state', '<I'), g('pos', '<f'), g('cur_a', '<f'),
                                    g('duty', '<f'), g('zero_mv', '<f'))
        gr, oc, df = (b[addr['grasped'] - base], b[addr['overcurrent'] - base],
                      b[addr['driver_fault'] - base])
        name = NAMES.get(st, str(st))
        s_ = stats.setdefault(name, [0, 0.0, 9.9, 0])
        s_[0] += 1
        s_[1] = max(s_[1], cur)
        s_[2] = min(s_[2], cur)
        s_[3] += pin_low
        key = (st, round(duty, 2), round(cur, 2), gr, oc, df, pin_low)
        if key != last:
            print(f'{time.time() - t0:6.2f}s {name:8s} pos={pos:4.2f} duty={duty:+.2f} '
                  f'I={cur:5.3f}A cero={zero:6.1f}mV grasped={gr} overcurrent={oc} '
                  f'falla_nFAULT={df} pin_nFAULT={"BAJO" if pin_low else "alto"}', flush=True)
            last = key
        time.sleep(0.003)
print('\nresumen (muestras, I max, I min, muestras con el pin nFAULT en bajo):')
for k, v in stats.items():
    print(f'  {k:8s} {v[0]:6d}  {v[1]:.3f} A  {v[2]:.3f} A  {v[3]:5d}')
