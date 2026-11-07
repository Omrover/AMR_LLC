/*
 * Contrato MQTT v1 Tlaloc <-> NUC (AMR_LLC/protocol/CONTRACT.md), lado MCU.
 *
 * C99 portable: sin Zephyr, RTD ni malloc. Se prueba en host
 * (tests/run_host_tests.sh) contra los mismos fixtures que el bridge Python y
 * se enlaza sin cambios en el firmware (Zephyr hoy, FreeRTOS+RTD si hiciera falta).
 */
#ifndef AMR_CONTRACT_H
#define AMR_CONTRACT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AMR_CONTRACT_VERSION 1
#define AMR_MAX_PAYLOAD      1024U
#define AMR_MAX_VALID_MS     250U
#define AMR_MAX_LINEAR       2.0f
#define AMR_MAX_ANGULAR      4.0f
#define AMR_MAX_ARM_ACCEL    10000.0f
#define AMR_MAX_GRIP_CURRENT 5.0f
#define AMR_N_WHEELS         4
#define AMR_N_JOINTS         4
#define AMR_BOOT_ID_MAX      64

/* Bits de faults (llc/status). */
enum {
	AMR_FAULT_ROBOCLAW_128 = 1u << 0,
	AMR_FAULT_ROBOCLAW_129 = 1u << 1,
	AMR_FAULT_IMU = 1u << 2,
	AMR_FAULT_CAN_BUS = 1u << 3,
	AMR_FAULT_ARM_MOTOR = 1u << 4,
	AMR_FAULT_GRIPPER_OVERCURRENT = 1u << 5,
	AMR_FAULT_BATTERY_CRITICAL = 1u << 6,
	AMR_FAULT_ESTOP_HW = 1u << 7,
};

/* ------------------------------------------------------------- telemetría */

typedef struct {
	const char *boot_id;
	int64_t t_ms;
} amr_hdr_t;

typedef struct {
	float x, y, yaw;      /* odom */
	float vx, vy, wz;     /* base_footprint */
	float cov[6];         /* var x, y, yaw, vx, vy, wz */
} amr_odom_t;

typedef struct {
	float q[4];           /* w, x, y, z (Game Rotation Vector) */
	float gyro[3];
	float acc[3];
	int acc_status;       /* 0..3 */
} amr_imu_t;

typedef struct {
	float pos[AMR_N_WHEELS];    /* rad, orden fl, rl, rr, fr */
	float vel[AMR_N_WHEELS];    /* rad/s */
	uint32_t rc_err[2];
} amr_wheels_t;

typedef struct {
	bool enabled;
	int32_t pos[AMR_N_JOINTS];  /* counts 0xA4 */
	float vel_rpm[AMR_N_JOINTS];
	float cur_a[AMR_N_JOINTS];
	float temp_c[AMR_N_JOINTS];
	bool in_pos[AMR_N_JOINTS];
	uint8_t fault[AMR_N_JOINTS];
	float grip_pos;             /* 0 abierto .. 1 cerrado */
	float grip_cur_a;
	bool grasped;
} amr_arm_t;

typedef struct {
	float volt;
	float logic_volt;           /* < 0: no se envía */
} amr_battery_t;

typedef enum { AMR_STATE_INIT, AMR_STATE_OK, AMR_STATE_ESTOP, AMR_STATE_FAULT } amr_state_t;

typedef struct {
	amr_state_t state;
	bool estop;
	uint32_t faults;
	bool wd_tripped;
	float cpu;                  /* 0..1 */
} amr_status_t;

typedef enum { AMR_CMD_ARM, AMR_CMD_GRIPPER, AMR_CMD_ESTOP, AMR_CMD_ODOM_RESET } amr_cmd_kind_t;
typedef enum { AMR_ACK_OK, AMR_ACK_REJECTED, AMR_ACK_BUSY, AMR_ACK_DUPLICATE } amr_ack_result_t;

/* Todas devuelven la longitud escrita (sin '\0') o -1 si no cabe o hay un
 * valor no finito/fuera de rango (nunca se publica JSON inválido). */
int amr_fmt_odom(char *buf, size_t size, const amr_hdr_t *h, const amr_odom_t *m);
int amr_fmt_imu(char *buf, size_t size, const amr_hdr_t *h, const amr_imu_t *m);
int amr_fmt_wheels(char *buf, size_t size, const amr_hdr_t *h, const amr_wheels_t *m);
int amr_fmt_arm(char *buf, size_t size, const amr_hdr_t *h, const amr_arm_t *m);
int amr_fmt_battery(char *buf, size_t size, const amr_hdr_t *h, const amr_battery_t *m);
int amr_fmt_status(char *buf, size_t size, const amr_hdr_t *h, const amr_status_t *m);
int amr_fmt_ack(char *buf, size_t size, const amr_hdr_t *h, uint32_t seq,
		amr_cmd_kind_t cmd, amr_ack_result_t result, const char *msg);
int amr_fmt_lwt(char *buf, size_t size);

/* --------------------------------------------------------------- comandos */

typedef struct {
	uint32_t seq;
	float vx, vy, wz;
	uint32_t valid_ms;
} amr_cmd_vel_t;

typedef enum { AMR_ARM_GOAL, AMR_ARM_ENABLE, AMR_ARM_DISABLE, AMR_ARM_CLEAR, AMR_ARM_HOME } amr_arm_op_t;

typedef struct {
	uint32_t seq;
	amr_arm_op_t op;
	int32_t pos[AMR_N_JOINTS];  /* solo GOAL */
	float acc[AMR_N_JOINTS];    /* rpm/s */
	float dec[AMR_N_JOINTS];
} amr_cmd_arm_t;

typedef struct {
	uint32_t seq;
	float pos;
	float max_cur;
} amr_cmd_gripper_t;

typedef struct {
	uint32_t seq;
	bool active;
} amr_cmd_estop_t;

typedef struct {
	uint32_t seq;
	float x, y, yaw;
} amr_cmd_odom_reset_t;

/* Validan el JSON completo contra el contrato y el boot_id vigente.
 * false ante cualquier desviación (campo faltante/extra/duplicado, tipo,
 * rango, versión, boot_id ajeno, basura al final). */
bool amr_parse_cmd_vel(const char *json, size_t len, const char *boot_id, amr_cmd_vel_t *out);
bool amr_parse_cmd_arm(const char *json, size_t len, const char *boot_id, amr_cmd_arm_t *out);
bool amr_parse_cmd_gripper(const char *json, size_t len, const char *boot_id, amr_cmd_gripper_t *out);
bool amr_parse_cmd_estop(const char *json, size_t len, const char *boot_id, amr_cmd_estop_t *out);
bool amr_parse_cmd_odom_reset(const char *json, size_t len, const char *boot_id,
			      amr_cmd_odom_reset_t *out);

/* Extrae solo "seq" (para responder "rejected" a un comando inválido). */
bool amr_peek_seq(const char *json, size_t len, uint32_t *seq);

/* --------------------------------------------- watchdog y deduplicación */

/* Último cmd/vel aceptado y su expiración local (tiempo de recepción). */
typedef struct {
	float vx, vy, wz;
	uint32_t cmd_ms;
	uint32_t valid_ms;
	bool active;
	bool tripped;       /* expiró estando activo (llc/status.wd) */
} amr_vel_wd_t;

void amr_vel_wd_init(amr_vel_wd_t *wd);
void amr_vel_wd_accept(amr_vel_wd_t *wd, const amr_cmd_vel_t *cmd, uint32_t now_ms);
void amr_vel_wd_reject(amr_vel_wd_t *wd);   /* comando inválido: detener */
/* Consigna a aplicar ahora: cero tras expirar, sin enlace o con e-stop. */
void amr_vel_wd_step(amr_vel_wd_t *wd, uint32_t now_ms, bool link_ok, bool estop,
		     float *vx, float *vy, float *wz);

typedef struct {
	uint32_t last;
	bool valid;
} amr_seq_t;

/* true si seq es nuevo (y lo registra); false si es la retransmisión del último. */
bool amr_seq_accept(amr_seq_t *s, uint32_t seq);

#ifdef __cplusplus
}
#endif

#endif /* AMR_CONTRACT_H */
