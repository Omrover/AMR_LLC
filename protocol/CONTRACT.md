# Contrato MQTT v1: Tlaloc (S32K344) ↔ NUC

Fuente de verdad del enlace entre el firmware (`firmware/lib/amr_contract`) y el
bridge ROS 2 (`ros2/amr_mqtt_bridge/amr_mqtt_bridge/protocol.py`). Extiende el
contrato del ejemplo `examples/ros2_mqtt/v2`.

Los dos lados se prueban contra los mismos fixtures (`fixtures/commands.txt`):

| Lado | Prueba |
|---|---|
| Python | `ros2/amr_mqtt_bridge/test/test_protocol.py` |
| C | `firmware/lib/amr_contract/tests/run_host_tests.sh` |

Además, la telemetría formateada en C se valida con el decodificador de Python.

## Transporte

| Elemento | Valor |
|---|---|
| Broker | Mosquitto en la NUC, puerto **18883** (`AMR_master/amr_bringup/config/mosquitto/`) |
| Red | NUC `192.168.50.1/24` ↔ Tlaloc `192.168.50.20` (Ethernet dedicada) |
| Prefijo | `omrover/mm01` (único por robot). Los topics de abajo son relativos a él |
| MQTT | 3.1.1, `clean_session=1`, keepalive 2 s en el MCU |
| Client id | el prefijo (`omrover/mm01`) |
| LWT | `llc/status` = `{"v":1,"state":"offline"}`, QoS 1, **retain**. Al conectar, el MCU **borra** el LWT retenido publicando un payload vacío con retain en `llc/status`, y el bridge ignora los payloads vacíos |
| Payload | JSON UTF-8, objeto plano salvo `grip` en `llc/arm`, máximo **1024 bytes** |

## Reglas comunes

1. Todo mensaje lleva `"v":1`. Una versión distinta se rechaza.
2. La telemetría lleva `"boot_id"` (1–64 chars `[0-9a-zA-Z_-]`, nuevo en cada reset) y `"t_ms"`: el uptime del MCU en ms, entero ≥ 0 y monotónico dentro de un `boot_id`.
3. Los comandos llevan `"boot_id"` (el **último recibido del MCU**) y `"seq"` (uint32). Un comando con `boot_id` ajeno, con flag `retain` o con cualquier campo fuera de rango **se rechaza**. Un `cmd/vel` inválido además detiene la base: un comando inválido nunca deja el robot en movimiento.
4. La telemetría nunca se publica con retain. La única excepción es el LWT.
5. Unidades SI y frames ROS (REP-103/105), salvo los counts y rpm del brazo, que son unidades nativas del GIM6010-8. La conversión a rad se hace en la NUC (`config/arm.yaml`).
6. Números JSON finitos: el emisor nunca envía `NaN` ni `Inf`.

## Telemetría (MCU → NUC)

| Topic | QoS | Tasa | Campos además de `v`, `boot_id`, `t_ms` |
|---|---|---|---|
| `llc/odom` | 0 | 50 Hz | `x`, `y` [m], `yaw` [rad] en `odom`; `vx`, `vy` [m/s], `wz` [rad/s] en `base_footprint`; `cov`: [6] varianzas ≥ 0 de (x, y, yaw, vx, vy, wz) |
| `llc/imu` | 0 | 100 Hz | `q`: [w, x, y, z] Game Rotation Vector de la BNO080 (norma 0.9–1.1); `gyro`: [3] rad/s; `acc`: [3] m/s²; `acc_status`: 0–3 (precisión SH-2) |
| `llc/wheels` | 0 | 20 Hz | `pos`: [4] rad acumulados; `vel`: [4] rad/s; `rc_err`: [2] status de error de RoboClaw 128 y 129 (≥ 0). Orden de ruedas: **fl, rl, rr, fr** |
| `llc/arm` | 0 | 50 Hz | `enabled`: bool; `pos`: [4] int counts absolutos (0xA4); `vel`: [4] rpm; `cur`: [4] A; `temp`: [4] °C; `in_pos`: [4] bool; `fault`: [4] código 0xAE (0 = ok, 255 = la junta no responde por CAN); `grip`: {`pos` 0–1 (0 = abierto), `cur` A (ACS712), `grasped` bool} |
| `llc/battery` | 0 | 1 Hz | `volt`: tensión principal [V] (RoboClaw cmd 24); opcional `logic_volt` [V] |
| `llc/status` | 1 | 2 Hz | `state`: `init`/`ok`/`estop`/`fault`; `estop` bool; `faults` bitmask (abajo); `wd` bool (watchdog de velocidad disparado); `cpu` 0–1 |
| `llc/ack` | 1 | por comando | `seq`; `cmd`: `arm`/`gripper`/`estop`/`odom_reset`; `result`: `ok`/`rejected`/`busy`/`duplicate`; opcional `msg` (≤ 64 chars) |

### Bits de `faults`

| Bit | Significado |
|---|---|
| 0 | Sin respuesta de RoboClaw 128 |
| 1 | Sin respuesta de RoboClaw 129 |
| 2 | BNO080 sin datos |
| 3 | Bus CAN en error o bus-off |
| 4 | Algún GIM6010-8 reporta falla (ver `llc/arm.fault`) |
| 5 | Sobrecorriente del gripper |
| 6 | Batería bajo el umbral crítico del MCU |
| 7 | E-stop físico activo |

## Comandos (NUC → MCU)

Todos llevan `v`, `boot_id` y `seq`.

| Topic | QoS | Campos | Respuesta |
|---|---|---|---|
| `cmd/vel` | **0** | `vx`, `vy` [m/s] (\|·\| ≤ 2.0), `wz` [rad/s] (\|·\| ≤ 4.0), `valid_ms` 1–250 | Sin ack. El MCU aplica el comando hasta `valid_ms` y luego frena con rampa (watchdog). El bridge reenvía a 20 Hz |
| `cmd/arm` | 1 | `op`: `goal`/`enable`/`disable`/`clear`/`home`. Con `goal` además: `pos` [4] int32 counts, `acc` [4] y `dec` [4] rpm/s (0 < x ≤ 10000) | `llc/ack` |
| `cmd/gripper` | 1 | `pos` 0–1 (0 = abierto), `max_cur` A (0 < x ≤ 5) | `llc/ack` |
| `cmd/estop` | 1 | `active` bool | `llc/ack` |
| `cmd/odom_reset` | 1 | `x`, `y`, `yaw` | `llc/ack` |

**Deduplicación:** el MCU recuerda el último `seq` aceptado por topic. Si llega el mismo `seq` (retransmisión QoS 1), responde `duplicate` y no lo ejecuta de nuevo.

**Brazo:** `enable` energiza las 4 juntas en su posición actual (no mueve nada) y requiere haber leído la posición de todas. `goal` y `home` (todas a 0 counts) se rechazan con el brazo deshabilitado, con e-stop activo o fuera de los límites de seguridad del MCU (`TLALOC_ARM_LIMIT_COUNTS`, ±16384 counts por defecto). Con e-stop el brazo **frena y mantiene** la posición: no se libera (0xCF) porque caería por gravedad. `disable` sí lo libera.

**Gripper:** al cerrar, el MCU avanza el servo hasta `pos` o hasta que la corriente del ACS712 supera `max_cur`. En ese punto fija `grip.grasped = true` y mantiene la posición.

## Ejemplos

```json
{"v":1,"boot_id":"3f9c0a1e00a41b27","t_ms":120340,"x":1.2,"y":-0.4,"yaw":0.785,"vx":0.1,"vy":0.0,"wz":0.0,"cov":[0.001,0.001,0.002,0.0004,0.0004,0.001]}
{"v":1,"boot_id":"3f9c0a1e00a41b27","seq":42,"vx":0.2,"vy":-0.1,"wz":0.3,"valid_ms":250}
{"v":1,"boot_id":"3f9c0a1e00a41b27","seq":43,"op":"goal","pos":[0,4096,-2048,1024],"acc":[100,100,100,100],"dec":[100,100,100,100]}
```
