/*
 * Control de tracción de la base omni X con 2 RoboClaw (4 motores + encoders
 * E6B2-CWZ leídos por las RoboClaw). Portable: depende solo de amr_odom/omni
 * y del driver roboclaw (transporte inyectado).
 *
 * Por ciclo (50 Hz recomendado a 115200 baud):
 *   1. rampa de la consigna del cuerpo (lin_acc / ang_acc)
 *   2. cinemática inversa -> rad/s por rueda -> saturación proporcional -> QPPS
 *   3. SpeedAccelM1M2 a cada RoboClaw (PID de velocidad interno del RoboClaw)
 *   4. lectura de encoders (cmd 78) -> posición/velocidad de rueda -> directa
 *
 * Orden de ruedas: fl, rl, rr, fr (contrato y URDF).
 */
#ifndef AMR_DRIVE_H
#define AMR_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

#include "omni.h"
#include "roboclaw.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bits de falla: coinciden con AMR_FAULT_ROBOCLAW_128/129 del contrato. */
#define AMR_DRIVE_FAULT_RC0 (1u << 0)
#define AMR_DRIVE_FAULT_RC1 (1u << 1)

typedef struct {
	uint8_t rc;      /* índice de RoboClaw (0 o 1) */
	uint8_t motor;   /* 0 = M1, 1 = M2 */
	int8_t sign;     /* +1/-1: sentido del motor/encoder vs. convención de la rueda */
} amr_wheel_map_t;

typedef struct {
	float wheel_radius;          /* m */
	float center_distance;       /* m */
	float angles_rad[AMR_OMNI_WHEELS];
	float counts_per_rev;        /* cuentas de encoder por vuelta de RUEDA (PPR*4*reducción) */
	float max_wheel_rad_s;       /* saturación por rueda */
	float lin_acc;               /* m/s^2 rampa del cuerpo */
	float ang_acc;               /* rad/s^2 */
	uint32_t rc_accel_qpps;      /* aceleración del PID del RoboClaw (QPPS/s) */
	uint8_t rc_addr[2];          /* 128, 129 */
	amr_wheel_map_t map[AMR_OMNI_WHEELS];
	uint8_t fault_after;         /* fallas consecutivas para marcar la RoboClaw */
} amr_drive_cfg_t;

typedef struct {
	bool valid;                          /* encoders de ambas RoboClaw leídos */
	float wheel_pos[AMR_OMNI_WHEELS];    /* rad acumulados */
	float wheel_vel[AMR_OMNI_WHEELS];    /* rad/s */
	float vx, vy, wz;                    /* velocidad del cuerpo medida (directa) */
	float residual;                      /* m/s, inconsistencia entre ruedas */
	float cmd_vx, cmd_vy, cmd_wz;        /* consigna aplicada tras la rampa */
} amr_drive_meas_t;

typedef struct {
	amr_drive_cfg_t cfg;
	amr_omni_t kin;
	roboclaw_t *rc;
	float vx, vy, wz;                    /* consigna con rampa */
	int32_t enc_prev[AMR_OMNI_WHEELS];
	bool enc_valid;
	double wheel_pos[AMR_OMNI_WHEELS];
	uint8_t fail_count[2];
	uint32_t faults;
	uint32_t last_status[2];
} amr_drive_t;

void amr_drive_default_cfg(amr_drive_cfg_t *cfg);
bool amr_drive_init(amr_drive_t *d, const amr_drive_cfg_t *cfg, roboclaw_t *rc);
/* Un ciclo de control con la consigna objetivo (ya validada por el watchdog). */
void amr_drive_step(amr_drive_t *d, float vx, float vy, float wz, float dt,
		    amr_drive_meas_t *meas);
/* Parada inmediata sin rampa (e-stop): duty 0 en ambas RoboClaw (rueda libre). */
void amr_drive_stop(amr_drive_t *d);
/* Convierte rad/s de rueda a QPPS con signo según el mapeo (expuesta para pruebas). */
int32_t amr_drive_qpps(const amr_drive_t *d, int wheel, float rad_s);

#ifdef __cplusplus
}
#endif

#endif /* AMR_DRIVE_H */
