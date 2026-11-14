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
| Red | KSZ8091RND RMII (50 MHz de Y4) → `eno1` de la NUC | EMAC0 (pines del `.mex`), reset PHY PTE21 |
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
../../../AMR_localization/amr_odom   cinemática mecanum + EKF
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
   Si marca 0.5 o 2.0, `TLALOC_COUNTS_PER_REV` (2000) está mal por la
   relación 1:2: probar 1000 o 4000.
4. **Signos de ruedas, con las ruedas en el aire.** Publicar `vx = +0.1`: las
   cuatro ruedas mecanum deben girar como si el robot avanzara. La que gire al
   revés lleva `TLALOC_SIGN_WHEELn = -1`. Luego `wz = +0.3`: las del lado
   izquierdo (1 y 2) hacia atrás y las del derecho (3 y 4) hacia adelante. Si la rueda se dispara o vibra, el problema es el
   PID de la RoboClaw (encoder invertido respecto al motor), no el firmware.
5. **En el piso, a baja velocidad.** `vx = +0.1` avanza, `vy = +0.1` va a la
   izquierda, `wz = +0.3` gira antihorario. Si `vy` va a la derecha o el robot
   gira al pedir `vy`, los rodillos están en "O" y no en "X" vistos desde
   arriba: intercambiar las ruedas de cada lado (fl con rl y fr con rr).
6. **IMU.** Girando antihorario, `/imu/data` debe dar `angular_velocity.z > 0`.
   Si no, `TLALOC_IMU_Z_DOWN=y`. Una vuelta completa a mano debe dar Δyaw ≈ 2π
   en `/odom`.
7. **Gripper.** Revisar en la consola el cero del ACS712 y ajustar los pulsos
   abierto/cerrado del servo.
8. **Brazo (CAN, GIM6010-8 en las direcciones 1 a 4).** Con el brazo apoyado o
   sostenido, sin `enable`: `llc/arm` debe mostrar `fault` en 0 (255 = esa junta
   no responde: revisar CANH/CANL, la terminación de 120 Ω y la dirección del
   motor). Mover cada junta a mano y anotar `pos`: confirma el sentido y cuántos
   counts da una vuelta de la junta (16384 si es por vuelta de salida). Apagar y
   encender en otra posición y comparar: así se ve si la posición de 0xA4 es
   absoluta o arranca en cero. Luego `enable` (no debe moverse nada) y un `goal`
   pequeño de una sola junta con `acc`/`dec` bajos (50 rpm/s).

## Parámetros físicos (C:/Users/Ivani/routing, nov 2026)

- Ruedas mecanum de 6" rectas, con rodillos a 45° en "X" vistos desde arriba →
  r = 0.0762 m. Ruedas en (±Lx, ±Ly) = (±0.1583, ±0.1734) m, por lo que el
  brazo de palanca de wz es Lx + Ly = 0.3317 m.
- E6B2-CWZ de 500 PPR ×4 en el eje de la rueda (una vuelta de rueda = una del
  encoder = dos del motor): 2000 cuentas por vuelta de rueda.
- Motor de 165 rpm → 82.5 rpm de rueda = 8.64 rad/s ≈ 2750 QPPS. Manda sobre
  el límite de 5000 QPPS de las RoboClaw. Con 10 % de margen, 7.7 rad/s de
  rueda: vx o vy máx ≈ 0.59 m/s (eje puro) y wz máx ≈ 1.77 rad/s. Nav2 usa límites
  más bajos (M4).
- **QPPS de las RoboClaw.** Con 2000 cuentas/vuelta, la velocidad máxima real
  es ~2750 QPPS, no 5000. El PID de velocidad usa el QPPS como término de
  avance: volver a medirlo en Motion Studio (motor al 100 % con la rueda en el
  aire) y reajustar el PID con ese valor.
- Timeout serie de las RoboClaw: el firmware programa 0.5 s al arrancar y tras
  cada reconexión (`TLALOC_RC_SERIAL_TIMEOUT_DS`, comando 14). Si el MCU deja
  de enviar, las ruedas paran en medio segundo.
- Ethernet: pines RMII y reset del PHY (PTE21) tomados del `.mex` de la FRDM-A.
  PHY KSZ8091RND con 50 MHz en XI; dirección 0 por strap (R538 a GND).

- ACS712-05B (185 mV/A) con divisor 2/3 hacia el ADC de 3.3 V: 10k en serie y
  20k a GND (el ACS712 pide ≥ 4.7k de carga). Si el divisor es 1/2 (10k/10k),
  `TLALOC_ACS712_MV_PER_A_X10=925`.

### Ethernet y MQTT: qué se resolvió (nov 2026)

- **PHY:** `tlaloc.dts` usa el driver MII genérico (`ethernet-phy`). Con
  `microchip,ksz8081` el KSZ8091RND nunca completaba la autonegociación (BMSR
  0x7849) y quedaba así hasta un reset de placa; con el genérico enlaza a
  100 Mbps full duplex en ~3 s. El ejemplo RTD de NXP, que no toca el PHY,
  también enlaza.
- **TX RMII:** el grupo TXD0/TXD1/TX_EN de `tlaloc-pinctrl.dtsi` necesita
  `output-enable` (en S32K3 el OBE solo se pone así).
- **CAN sin motores:** `can_send()` sin callback espera el fin de la trama sin
  límite, y sin ACK nunca termina. El hilo de control se quedaba con
  `core_lock` y MQTT se conectaba pero no publicaba (el broker lo tiraba por
  keepalive). `can_tx()` ahora envía con callback y no encola si el
  controlador está en error-passive/bus-off; el brazo queda offline (fault 255).

### Si no hay enlace Ethernet

LEDs del RJ45 de la Tlaloc: los dos cuelgan de la misma señal LED0 del PHY.
**Naranja = hay enlace** (parpadea con tráfico). **Verde = sin enlace o PHY
en reset.**

1. Dejar la NUC en autonegociación: `sudo ethtool -s eno1 autoneg on`.
   Revisar con `ethtool eno1` o `cat /sys/class/net/eno1/carrier`.
2. Compilar con diagnóstico:
   `west build -b tlaloc -p -- -DEXTRA_CONF_FILE=debug_eth.conf`
   (shell, `net`, `mdio`, logs DBG de PHY y Ethernet; sin motores).
3. Consola OpenSDA a 115200 con **un solo** programa abierto en
   `/dev/ttyACM0` (`picocom -b 115200 /dev/ttyACM0`); dos lectores a la vez
   se reparten los bytes y los logs salen cortados.
   - Debe aparecer `PHY (0) Link speed 100 Mb, full duplex`.
   - `net iface`, `net ping 192.168.50.1`, `kernel thread list`.
   - `mdio read mdio 0 <reg>` (registro en **hex**, sin `0x`): 0 BMCR,
     1 BMSR (bit 2 enlace, bit 5 AN completa), 4 ANAR, 5 lo que anunció el
     otro extremo, 1e (bit 4 = hay señal en RX), 1f (bit 7 = 1, modo 50 MHz).
4. Hardware (si el ejemplo RTD tampoco enlaza): JP5 puesto (P3V3_ENET),
   3.3 V en VDDA_3V3 (pin 2 de U14), 1.2 V en VDD_1V2 (pin 1), 50 MHz en TP65.
5. Para anunciar solo 100 Mbps: en el nodo `phy` de `tlaloc.dts`,
   `default-speeds = "100BASE Full-Duplex", "100BASE Half-Duplex";`.
