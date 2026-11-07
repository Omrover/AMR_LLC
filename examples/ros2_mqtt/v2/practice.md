# Practice v2: S32K344 wheel node → MQTT → ROS 2

**40 minutes.** Your team writes the **firmware** half (C, on the NXP
S32K344) and the **ROS 2** half (Python, on the NUC) of one encoder pipeline:

```text
encoder ─► S32K344 (Zephyr) ─► MQTT <prefix>/encoder ─► Mosquitto (NUC) ─► bridge ─► /nxp/encoder_sample ─► monitor ─► /nxp/wheel_rad_s
                 ▲                                                                                         │
       motors ◄──┘◄───────────────── MQTT <prefix>/wheel_pwm ◄───────── bridge (allow_pwm) ◄── /nxp/wheel_pwm┘ (optional)
```

There are three TODOs, each marked `## TODO:: add code here`:

| # | File | Function |
|---|---|---|
| 1 | `firmware/s32k344_wheel_node/src/student.c` | `student_ticks_from_rotation()`: qdec angle → ticks |
| 2 | `firmware/s32k344_wheel_node/src/student.c` | `student_hbridge_pulses()`: signed duty → H-bridge IN1/IN2 |
| 3 | `ws/src/nxp_mqtt_lab/nxp_mqtt_lab/exercises.py` | `wheel_speed()`: ticks → rad/s (+ optional `pwm_pair()`) |

Your instructor tells you which **device path** your team uses. All three
speak the same MQTT contract:

| Path | Device | Broker config | `mqtt_prefix` |
|---|---|---|---|
| **B: Board** | Real S32K344 robot, read-only by default | `mosquitto-board.conf` | `class10/robot01` (yours) |
| **F: Firmware sim** | Same firmware as a Linux process (`native_sim`) | `mosquitto.conf` | `class10/fw` |
| **P: Python sim** | `ros2 run nxp_mqtt_lab simulator` (TODOs 1–2 host-tested only) | `mosquitto.conf` | `class10/sim` |

## Terminal setup (every terminal, from the `v2` directory)

```bash
source /opt/ros/jazzy/setup.bash
source ws/install/setup.bash
export ROS_DOMAIN_ID=42                 # your team's number
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# Firmware terminals also need the toolchain:
source ~/zephyr-class10/env.sh
```

Set `PREFIX` for your path, e.g. `export PREFIX=class10/robot01`.

## 0–5 min: start the pipeline and see the bug

```bash
# Terminal A: broker (board: mosquitto-board.conf; F/P: mosquitto.conf)
mosquitto -c mosquitto-board.conf
# Terminal B: bridge
ros2 run nxp_mqtt_lab bridge --ros-args -p mqtt_prefix:=$PREFIX
# Terminal C: raw MQTT
mosquitto_sub -h 127.0.0.1 -p 18883 -t "$PREFIX/#" -v
```

The device is still running the student stub:

- **Board:** it is already flashed with the student build. The serial console
  shows `student self-test: FAIL`, and the encoder always reports
  `"ticks":0`, however you turn the wheel.
- **Firmware sim:** build and run the stub yourself:

  ```bash
  cd firmware/s32k344_wheel_node
  west build -p -b native_sim/native/64 -d build_sim . -- -DCONFIG_LAB_SIM_HAND_SPIN=y
  ./build_sim/zephyr/zephyr.exe
  ```

- **Python sim:** run `ros2 run nxp_mqtt_lab simulator --hand-spin`. Ticks
  already increase; do TODOs 1–2 with the host tests only.

Payload at about 10 Hz: `{"ticks":0,"uptime_ms":3100,"boot_id":"3f9c0a1e00a41b27"}`.

## 5–17 min: firmware TODOs (`student.c`)

The NXP `qdec` driver (eMIOS + LCU on PTB2/PTB3) does not give you ticks. It
gives the **cumulative wheel angle in radians**:
`rotation_rad = ticks · 2π / ticks_per_rev`.

1. `student_ticks_from_rotation()`: invert that formula and round with `llround`.
2. `student_hbridge_pulses()`: for a 2-input H-bridge (DRV8833-style), a
   positive duty drives IN1 and a negative duty drives IN2. Both inputs stay at
   0 for 0, NaN or |duty| > 1. The two inputs must never both be high.

Test on the NUC without hardware (about 1 second):

```bash
firmware/s32k344_wheel_node/tests/run_host_tests.sh      # until: "PASSED (0 failures)"
```

Then rebuild and run your firmware:

```bash
cd firmware/s32k344_wheel_node
# Board (PWM stays disabled in this build):
west build -p -b mr_canhubk3 . && west flash
# Firmware sim (Ctrl-C the old zephyr.exe first):
west build -d build_sim && ./build_sim/zephyr/zephyr.exe
```

Checkpoint: the log shows `student self-test: PASS`. On the board, turning the
wheel by hand changes `ticks`; forward should be positive. The firmware sim
counts about 12 ticks per sample.

## 17–29 min: ROS TODO (`exercises.py` → `wheel_speed`)

1. Return `None` for the first sample, a changed `boot_id`, or dt ≤ 0.
2. dt comes from the **device** `uptime_ms`, not from arrival time.
3. `omega = Δticks · 2π / ticks_per_rev / dt` (signed, rad/s).

```bash
ros2 run nxp_mqtt_lab monitor --ros-args -p ticks_per_rev:=600   # board: your measured value
ros2 topic echo /nxp/wheel_rad_s
CHECK_STUDENT=1 python3 -m pytest -q ws/src/nxp_mqtt_lab/test/test_protocol.py
```

Restart the monitor after each edit; no rebuild is needed. Checkpoint for the
sims: **≈ 1.257 rad/s** (12 ticks / 0.1 s at 600 ticks/rev). On the board,
the sign follows the direction you turn the wheel. If you are stuck, use
`-p use_solution:=true` for the ROS side, or build the firmware with
`-- -DCONFIG_LAB_USE_SOLUTION=y`.

## 29–34 min: break the link

```bash
ros2 topic echo /nxp/status
```

1. **Board:** unplug the T1 Ethernet cable for 5 s, then plug it back in.
   **Sims:** Ctrl-C the device process.
   → Within 1–2 s the status goes `STALE`. No speed is published, because
   *no data is not zero speed*.
2. Reconnect, or restart the device: status returns to `LIVE`.
3. Press **reset** on the board, or restart the sim: the `boot_id` changes, and
   your `wheel_speed` must show **no spike**.

## 34–38 min (optional): wheel PWM — wheels lifted, power switch in reach

The board must be flashed by the instructor with `-DCONFIG_LAB_ALLOW_PWM=y`.
The firmware sim is built without `LAB_SIM_HAND_SPIN`:

```bash
west build -p -b native_sim/native/64 -d build_sim . -- -DCONFIG_LAB_ALLOW_PWM=y
./build_sim/zephyr/zephyr.exe
```

```bash
# Bridge with command forwarding:
ros2 run nxp_mqtt_lab bridge --ros-args -p mqtt_prefix:=$PREFIX -p allow_pwm:=true
# 3 s pulse at duty 0.1 (TODO: pwm_pair in exercises.py, or use_solution:=true):
ros2 run nxp_mqtt_lab monitor --ros-args -p demo_pwm:=0.1
```

Watch `$PREFIX/applied_pwm` in terminal C. It should hold about 0.1 for 3 s,
then 0, and the wheel turns (sim: about 1.257 rad/s). Now repeat and, mid-pulse,
freeze the bridge so that it cannot send a stop: press **Ctrl-Z** in its
terminal, then run `kill -9 %1`. Alternatively, unplug Ethernet. The firmware's 5 ms control
thread stops the wheel within about **0.25 s**, even though nobody sent a stop
command. Duty is not speed, and |duty| ≤ 0.2 is enforced in the firmware.

## 38–40 min: evidence

Show `student self-test: PASS`, raw ticks, `/nxp/wheel_rad_s`, and one
STALE→LIVE recovery with a new `boot_id`. Submit `student.c`,
`exercises.py`, and short answers:

- Why is this bridge **not** a micro-ROS Agent? What would replace it with micro-ROS?
- Why doesn't MQTT QoS 1 guarantee that a wheel command is fresh or safe?
- Which code keeps running on the S32K344 when the NUC disappears, and why must it be there?
