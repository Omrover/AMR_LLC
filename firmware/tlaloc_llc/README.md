# tlaloc_llc — controlador de bajo nivel (Zephyr 4.5, S32K344)

Firmware de la Tlaloc (FRDM-A-S32K344). Habla el contrato MQTT v1
(`AMR_LLC/protocol/CONTRACT.md`) con la NUC y controla:

| Subsistema | Hardware | Pines (ver `boards/omrover/tlaloc/tlaloc-pinctrl.dtsi`) |
|---|---|---|
| Tracción + encoders | 2 RoboClaw (128 frente: ruedas 1 y 4; 129 atrás: ruedas 2 y 3), Packet Serial Multi-Unit 115200 | LPUART3 TX PTD2 / RX PTD3 |
| IMU | BNO080 (SH-2, Game Rotation Vector + gyro + accel) | LPI2C0 SDA PTD13 / SCL PTD14, INT PTA21 (EIRQ3) |
| Gripper | Servo PWM 50 Hz + ACS712 | eMIOS1 ch10 PTA27, ADC0 precisión ch0 PTD1 |
| Brazo (M5) | 4 GIM6010-8 | FlexCAN0 RX PTA6 / TX PTA7 |
| SBC | FS26 (watchdog) | LPSPI0 PCS0 PTB0, SOUT PTB1, SCK PTC8, SIN PTC9, INTB PTE18 (WKPU) |
| Red | KSZ8091RNA RMII → `eno1` de la NUC | EMAC0 (pines a VERIFICAR) |
| Consola | OpenSDA | LPUART6 RX PTA15 / TX PTA16, 115200 |

## Estructura

```
src/llc.c, llc.h        núcleo portable: contrato, watchdog, tracción, EKF, IMU, gripper, batería
src/gripper.c           servo + ACS712 con detección de agarre por corriente
src/main.c              hilos (control 20 ms, IMU por INT) y cliente MQTT
src/app_config.c        Kconfig -> configuración del núcleo
src/hal_tlaloc.c        drivers Zephyr de la Tlaloc
src/hal_sim.c           native_sim con RoboClaw y BNO080 simuladas byte a byte
tests/host_test.c       lazo cerrado del núcleo en host (lo corre lib/run_host_tests.sh)
../lib/                 roboclaw, amr_drive, bno08x, amr_contract (+ simuladores)
../../../AMR_localization/amr_odom   cinemática omni X + EKF
../boards/omrover/tlaloc             board out-of-tree
```

## Pruebas en host (sin Zephyr)

```bash
bash AMR_LLC/firmware/lib/run_host_tests.sh            # gcc + ASan/UBSan (Linux/NUC)
CC="zig cc" SANITIZE="" bash AMR_LLC/firmware/lib/run_host_tests.sh   # Windows
```

## Compilar

```bash
source ~/repos/zephyr/zephyr-env.sh        # o el entorno de west
cd ~/amr_ws/src/AMR_LLC/firmware
# SIL: el firmware como proceso Linux contra el broker local
west build -p -b native_sim/native/64 tlaloc_llc -d build_sim
./build_sim/zephyr/zephyr.exe
# Hardware
west build -p -b tlaloc tlaloc_llc -d build_tlaloc
west flash -d build_tlaloc            # pyocd (OpenSDA en modo CMSIS-DAP) o --runner jlink
```

Con el SIL corriendo, `ros2 launch amr_bringup llc_bridge.launch.py` levanta el
bridge real contra este firmware (mismo flujo que con el emulador Python).

Opciones útiles (`-- -DCONFIG_...=...` o `prj.conf`):
`TLALOC_ALLOW_MOTION`, `TLALOC_COUNTS_PER_REV`, `TLALOC_MAX_QPPS`,
`TLALOC_SIGN_WHEEL{1..4}_*`, `TLALOC_BNO080_ADDR`, `TLALOC_IMU_Z_DOWN`,
`TLALOC_GRIP_PULSE_{OPEN,CLOSED}_US`, `TLALOC_ACS712_MV_PER_A_X10`,
`TLALOC_FS26_WATCHDOG`.

## Bring-up en hardware (en este orden)

1. **FS26.** Está en modo normal: el driver debe cerrar INIT_FS en < 256 ms o el
   SBC reinicia al MCU. Si la consola muestra reinicios en bucle, poner el FS26
   en modo debug y revisar LPSPI0. Para depurar con breakpoints:
   `-DCONFIG_TLALOC_FS26_WATCHDOG=n` o FS26 en modo debug.
2. **Consola y red.** 115200 en el puerto OpenSDA. En la NUC:
   `nmcli con add type ethernet ifname eno1 ip4 192.168.50.1/24`, luego
   `ping 192.168.50.20`. Si no hay enlace, revisar los pines EMAC y la
   dirección PHY (`tlaloc.dts`).
3. **Encoders sin motores** (`-DCONFIG_TLALOC_ALLOW_MOTION=n`): empujar el robot
   1.00 m hacia adelante con cinta métrica. `/odom` debe marcar x ≈ 1.00.
   Si marca 0.5 o 2.0, la relación 1:2 está al revés: `TLALOC_COUNTS_PER_REV`
   = 4000 o 16000.
4. **Signos de ruedas, con las ruedas en el aire.** Publicar `wz = +0.3`. Cada
   rueda debe girar en el sentido que, por sí sola, haría rotar al robot
   antihorario visto desde arriba. La que gire al revés lleva
   `TLALOC_SIGN_WHEELn = -1`. Si la rueda se dispara o vibra, el problema es el
   PID de la RoboClaw (encoder invertido respecto al motor), no el firmware.
5. **En el piso, a baja velocidad.** `vx = +0.1` avanza, `vy = +0.1` va a la
   izquierda, `wz = +0.3` gira antihorario.
6. **IMU.** Girando antihorario, `/imu/data` debe dar `angular_velocity.z > 0`.
   Si no, `TLALOC_IMU_Z_DOWN=y`. Una vuelta completa a mano debe dar Δyaw ≈ 2π
   en `/odom`.
7. **Gripper.** Revisar en la consola el cero del ACS712 y ajustar los pulsos
   abierto/cerrado del servo.

## Parámetros físicos (C:/Users/Ivani/routing, nov 2026)

- Rueda omni de 6" → r = 0.0762 m. Ruedas en (±0.1583, ±0.1734) m a 45°, por lo
  que el brazo de palanca de wz es L = (Lx + Ly)/√2 = 0.2345 m.
- E6B2-CWZ de 1000 PPR ×4 en la salida del motorreductor, rueda:motor = 1:2:
  8000 cuentas por vuelta de rueda (a verificar en el paso 3).
- QPPS máximo 5000 → 3.93 rad/s de rueda → 0.30 m/s en la superficie. Eso da
  vx máx ≈ 0.42 m/s (eje puro) y wz máx ≈ 1.27 rad/s. Usar estos límites en
  Nav2 (M4).
- RoboClaw con serial timeout de 3 s: se recomienda 0.5 s. El firmware envía a
  50 Hz y, si el MCU muere, las ruedas se detendrían en medio segundo en vez de
  tres.
