/*
 * Núcleo del controlador de bajo nivel de Tlaloc (C99 portable, sin Zephyr).
 *
 * Integra el contrato MQTT v1 (amr_contract), la tracción con 2 RoboClaw
 * (amr_drive), el EKF de odometría (amr_odom), la BNO080 (bno08x) y el
 * gripper. No hace I/O de red ni crea hilos: main.c llama a estas funciones
 * desde sus hilos con los locks adecuados, y tests/host_test.c las ejecuta en
 * host con los simuladores de RoboClaw y BNO080.
 *
 * Hilos en el firmware (ver main.c):
 *   control (20 ms): llc_control_step()  -> RoboClaw, EKF, gripper, batería
 *   imu    (INT/5 ms): llc_imu_service() -> I2C BNO080; llc_imu_sample() con el lock
 *   main   (MQTT):   llc_command(), llc_telemetry()
 */
#ifndef TLALOC_LLC_H
#define TLALOC_LLC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "amr_contract.h"
#include "amr_drive.h"
#include "bno08x.h"
#include "ekf.h"
#include "gripper.h"
#include "roboclaw.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	/* 0 en éxito. El pulso se aplica a 50 Hz (servo). */
	int (*servo_set_us)(void *ctx, uint32_t pulse_us);
	/* Tensión en el pin del ADC del ACS712, en mV. */
	int (*adc_read_mv)(void *ctx, float *mv);
	void *ctx;
} llc_io_t;

typedef struct {
	amr_drive_cfg_t drive;
	amr_ekf_cfg_t ekf;
	grip_cfg_t grip;
	uint32_t rc_timeout_ms;      /* por transacción RoboClaw */
	uint8_t rc_retries;
	uint32_t imu_timeout_ms;     /* sin datos -> falla IMU, EKF solo con ruedas */
	float imu_z_sign;            /* +1 si el z de la BNO080 apunta hacia arriba */
	float battery_critical_v;
	uint32_t slow_period_ms;     /* batería */
} llc_cfg_t;

typedef enum {
	LLC_CMD_VEL,
	LLC_CMD_ARM,
	LLC_CMD_GRIPPER,
	LLC_CMD_ESTOP,
	LLC_CMD_ODOM_RESET,
	LLC_CMD_COUNT
} llc_cmd_topic_t;

typedef enum {
	LLC_TEL_ODOM,
	LLC_TEL_IMU,
	LLC_TEL_WHEELS,
	LLC_TEL_ARM,
	LLC_TEL_BATTERY,
	LLC_TEL_STATUS,
	LLC_TEL_COUNT
} llc_tel_t;

/* Estado de la IMU, propiedad del hilo IMU (lock propio en main.c). */
typedef struct {
	bno08x_t bno;
	bool configured;
	bool attempted;
	uint32_t enable_ms;          /* último intento de habilitar reportes */
	uint32_t grv_interval_us;
	uint32_t gyro_interval_us;
	uint32_t accel_interval_us;
} llc_imu_t;

typedef struct {
	llc_cfg_t cfg;
	llc_io_t io;
	roboclaw_t rc;
	amr_drive_t drive;
	bool drive_ok;
	amr_drive_meas_t meas;
	amr_ekf_t ekf;
	grip_t grip;
	amr_vel_wd_t wd;
	amr_seq_t seq[LLC_CMD_COUNT];
	bool estop;
	bool estop_applied;
	bool started;                /* primera medición válida de encoders */
	uint32_t faults;
	uint32_t rc_err[2];
	uint8_t prev_fail[2];
	/* IMU (copia del último dato, la pone llc_imu_sample) */
	bno08x_data_t imu;
	bool imu_valid;
	bool imu_new_quat;
	bool imu_new_gyro;
	uint32_t imu_ms;
	/* batería */
	float batt_v;
	float logic_v;
	bool batt_valid;
	uint32_t slow_ms;
	/* tiempos */
	bool stepped;
	uint32_t last_ms;
	uint32_t servo_us;
	float cpu;                   /* ocupación del lazo de control (0..1) */
} llc_t;

void llc_default_cfg(llc_cfg_t *cfg);
/* false si la configuración es inválida (la base queda detenida). */
bool llc_init(llc_t *l, const llc_cfg_t *cfg, const rc_transport_t *rc_t, const llc_io_t *io);

/* Procesa un comando del contrato. Escribe en ack el JSON de llc/ack cuando el
 * comando lo requiere y devuelve su longitud (0 si no hay ack, p. ej. cmd/vel). */
int llc_command(llc_t *l, llc_cmd_topic_t topic, const char *json, size_t len, bool retain,
		const amr_hdr_t *hdr, uint32_t now_ms, char *ack, size_t ack_size);

/* Ciclo de control (20 ms). link_ok: sesión MQTT activa. */
void llc_control_step(llc_t *l, uint32_t now_ms, bool link_ok);

/* Telemetría: longitud del JSON o -1. */
int llc_telemetry(const llc_t *l, llc_tel_t kind, const amr_hdr_t *hdr, char *buf, size_t size);

/* ---------------------------------------------------------------- IMU */
void llc_imu_init(llc_imu_t *imu, const bno_transport_t *t);
/* Atiende la BNO080: re-habilita reportes tras un reset y drena hasta
 * max_packets. Devuelve la máscara BNO_UPD_* leída (0 si nada), <0 si error de bus. */
int llc_imu_service(llc_imu_t *imu, uint32_t now_ms, int max_packets);
/* Entrega el último dato al núcleo (llamar con el lock del núcleo). */
void llc_imu_sample(llc_t *l, const bno08x_data_t *d, uint32_t upd, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* TLALOC_LLC_H */
