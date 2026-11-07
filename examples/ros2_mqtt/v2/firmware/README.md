# S32K344 wheel node (Zephyr) — encoder → MQTT, MQTT → wheel PWM

A complete firmware application for the **NXP S32K344**, built with Zephyr
v4.5.0. Its default target is the **NXP MR-CANHUBK3** board, which reaches the
NUC over 100BASE-T1 Ethernet through an RDDRONE-T1ADAPT media converter. It
uses the same MQTT/JSON contract as the Python simulator and the ROS bridge
in `../ws`.

```text
qdec0 (eMIOS+LCU) ─► student_ticks_from_rotation() ─► {"ticks","uptime_ms","boot_id"} ─► <prefix>/encoder
<prefix>/wheel_pwm ─► lab_parse_pwm() ─► 250 ms watchdog ─► student_hbridge_pulses() ─► eMIOS PWM
```

| File | Role |
|---|---|
| `s32k344_wheel_node/src/student.c` | **Student TODOs** (`## TODO:: add code here`) |
| `s32k344_wheel_node/src/student_solution.c` | Reference, `-DCONFIG_LAB_USE_SOLUTION=y` |
| `s32k344_wheel_node/src/main.c` | MQTT client thread + 5 ms local control/watchdog thread |
| `s32k344_wheel_node/src/lab_logic.c` | Portable watchdog and JSON contract (host-tested) |
| `s32k344_wheel_node/src/hal_s32k344.c` | Real hardware: Zephyr qdec sensor + eMIOS PWM |
| `s32k344_wheel_node/src/hal_sim.c` | `native_sim` wheel model: same firmware runs as a Linux process |
| `s32k344_wheel_node/boards/mr_canhubk3.overlay` | **Pins and calibration — check against your robot** |
| `s32k344_wheel_node/boards/mr_canhubk3.conf` | Static IP `192.168.50.20/24` |
| `s32k344_wheel_node/tests/run_host_tests.sh` | Unit tests, gcc only (no Zephyr) |
| `setup_zephyr.sh` | One-time toolchain install on the NUC (no sudo) |

## Verified status

| Item | Status |
|---|---|
| `west build -b mr_canhubk3` (student and solution) | Builds: ~145 KB flash, ~64 KB RAM, with real NXP qdec, eMIOS PWM, GMAC and TJA1103 drivers |
| Same firmware on `native_sim` ↔ Mosquitto ↔ ROS bridge ↔ monitor | Run end-to-end: 10 Hz, 1.257 rad/s, STALE/recovery with new boot_id, PWM pulse, stops 0.2–0.3 s after the bridge is killed |
| Host unit tests | Pass with the solution; fail on the stub until the TODOs are done |
| **Flashing and running on a physical board** | **Not tested here (no board available).** Do the bring-up checklist below before class |

## 1. One-time setup on each NUC

```bash
cd firmware
./setup_zephyr.sh                  # ~/zephyr-class10, ~4.5 GB, no sudo
source ~/zephyr-class10/env.sh     # in every firmware terminal
```

To flash you also need the debug-probe software: SEGGER J-Link, which is the
board default (`west flash`), or `pip install pyocd` for `west flash -r pyocd`.

## 2. Build, flash, observe

```bash
source ~/zephyr-class10/env.sh
cd firmware/s32k344_wheel_node

# Student build (read-only: PWM disabled)
west build -p -b mr_canhubk3 .
west flash                          # or: west flash -r pyocd

# Serial console (LPUART2, 115200 8N1), e.g.:
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
```

Expected console output:

```text
<inf> hal: encoder 600 ticks/rev, PWM period 1000000 ns
<inf> wheel_node: student self-test: PASS        (FAIL until student.c is done)
<inf> wheel_node: broker 192.168.50.1:18883 prefix class10/robot01, wheel PWM disabled (read-only)
<inf> wheel_node: MQTT connected, boot_id 3f9c0a1e00a41b27
```

Per-robot settings are passed at build time and don't need file edits:

```bash
west build -p -b mr_canhubk3 . -- \
  -DCONFIG_LAB_MQTT_PREFIX=\"class10/robot02\" \
  -DCONFIG_NET_CONFIG_MY_IPV4_ADDR=\"192.168.50.22\" \
  -DCONFIG_LAB_BROKER_ADDR=\"192.168.50.1\" -DCONFIG_LAB_BROKER_PORT=18883
```

Other options: `-DCONFIG_LAB_ALLOW_PWM=y` (wheels lifted only) and
`-DCONFIG_LAB_USE_SOLUTION=y`.

### No board / no probe: run the identical firmware on the NUC

```bash
west build -p -b native_sim/native/64 . -- -DCONFIG_LAB_SIM_HAND_SPIN=y
./build/zephyr/zephyr.exe           # MQTT to 127.0.0.1:18883, prefix class10/fw
```

Leave out `LAB_SIM_HAND_SPIN` and add `-DCONFIG_LAB_ALLOW_PWM=y` for the PWM
exercise. The simulated left wheel turns at 1200 ticks/s per unit duty.

## 3. Instructor bring-up checklist (once per robot, before class)

The overlay is written for the documented MR-CANHUBK3 pinout. Your robot's
wiring must be confirmed against it:

1. **Network.** Set the NUC's robot-facing Ethernet port to `192.168.50.1/24`
   and start `mosquitto -c mosquitto-board.conf` (from `v2/`). The board pings
   from the NUC once it boots: `ping 192.168.50.20`. If your robots already
   use another broker, pass its address and port as shown above.
2. **Encoder.** The A/B phases go to **PTB2/PTB3** (board `qdec0`, eMIOS0
   ch6/7 via LCU1+TRGMUX), at **3.3 V logic only**. Turn the wheel by hand: ticks
   must change, and forward must be positive (otherwise swap A/B). Mark the
   wheel, turn exactly 10 revolutions and set
   `micro-ticks-per-rev = <ticks/rev × 1000000>` in the overlay. Use a whole
   number of ticks, because the NXP driver uses integer division.
3. **Motors.** The overlay drives a **2-input H-bridge per wheel** (DRV8833 or
   DRV8871 style) with eMIOS0 PWM ch0..3 on **PTB12..PTB15** at 1 kHz: left
   IN1/IN2, then right IN1/IN2. If your driver is PWM+DIR or uses other pins,
   change the overlay and `set_wheel()` in `hal_s32k344.c`. Measure each pin
   with a scope or LED **with motor power off** before connecting the driver.
4. **Watchdog.** Wheels off the ground and an accessible power switch. Build
   with `CONFIG_LAB_ALLOW_PWM=y`, run the 0.1 duty demo, then unplug the
   Ethernet cable mid-pulse: the wheel must stop within about 0.3 s.
5. **Reset.** Press reset: `/nxp/status` goes STALE, then LIVE with a new
   `boot_id`, and there is no speed spike.

The duty limit of ±0.2 and the 250 ms watchdog are lab limits. They are not a
certified safety function, and they don't replace a hardware motor-disable or
emergency stop.

## Design notes

- **Local responsibilities.** The control thread runs every 5 ms whatever the
  MQTT state is. A command expires after `valid_ms` (≤ 250), and the outputs
  drop to zero at once on MQTT disconnect, on an invalid or retained command,
  or on a wrong `boot_id`.
- **Boot ID.** It is created after the first CONNACK, using random bits plus
  the cycle counter. Link-up timing makes it differ on every reset. It only
  tells resets apart; it is not a security token.
- **One encoder.** MR-CANHUBK3 exposes one hardware quadrature decoder. The
  bridge contract and simulator model one (left) wheel encoder. A second one
  would need another eMIOS/LCU instance in the overlay.
- **micro-ROS comparison.** This firmware publishes *application JSON* over
  MQTT, and the NUC bridge maps it to ROS 2. With micro-ROS, the same
  `main.c` would instead call `rclc_publisher_init_*` and an XRCE-DDS Agent
  would replace both the broker and the bridge.
