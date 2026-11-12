/*
 * Brazo de 4 GIM6010-8 sobre gim6010.c: el MCU es un gateway con límites de
 * seguridad en counts (la conversión a rad y la cinemática viven en la NUC,
 * config/arm.yaml). C99 portable, sin RTOS; lo llama el lazo de control.
 *
 *  - gim_arm_command() valida y deja trabajo pendiente; no toca el bus.
 *  - gim_arm_step() (cada ciclo) envía lo pendiente y lee 0xA4 de cada junta;
 *    0xAE (fallas) cada ae_period_ms.
 *  - E-stop: gim_arm_hold() fija el objetivo en la posición medida. No se usa
 *    0xCF (salida libre) porque el brazo caería por gravedad.
 */
#ifndef GIM_ARM_H
#define GIM_ARM_H

#include <stdbool.h>
#include <stdint.h>

#include "gim6010.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GIM_ARM_JOINTS 4

typedef enum {
	GIM_ARM_OP_GOAL,
	GIM_ARM_OP_ENABLE,
	GIM_ARM_OP_DISABLE,
	GIM_ARM_OP_CLEAR,
	GIM_ARM_OP_HOME
} gim_arm_op_t;

typedef struct {
	uint8_t addr[GIM_ARM_JOINTS];
	int32_t pos_min[GIM_ARM_JOINTS];   /* counts, límite de seguridad del MCU */
	int32_t pos_max[GIM_ARM_JOINTS];
	int32_t in_pos_tol;                /* counts */
	float default_acc_rpm_s;           /* HOME y ENABLE */
	uint32_t ae_period_ms;
	uint8_t offline_after;             /* fallas seguidas para marcar la junta */
} gim_arm_cfg_t;

typedef struct {
	bool online;
	bool pos_valid;
	int32_t pos;                       /* counts desenvueltos */
	float vel_rpm;
	float cur_a;
	float temp_c;
	uint8_t fault;                     /* código 0xAE */
	int32_t target;
	bool has_target;
	uint8_t pending;                   /* bits PEND_* (gim_arm.c) */
	uint32_t acc_crpm_s, dec_crpm_s;
	uint8_t fails;
} gim_joint_t;

typedef struct {
	gim_arm_cfg_t cfg;
	gim_bus_t *bus;
	gim_joint_t j[GIM_ARM_JOINTS];
	bool enabled;
	bool hold;                         /* e-stop: no acepta movimientos */
	uint32_t ae_ms;
	bool stepped;
} gim_arm_t;

void gim_arm_default_cfg(gim_arm_cfg_t *cfg);
void gim_arm_init(gim_arm_t *a, gim_bus_t *bus, const gim_arm_cfg_t *cfg);

/* true si se acepta; si no, *msg explica el rechazo. acc/dec en rpm/s (solo GOAL). */
bool gim_arm_command(gim_arm_t *a, gim_arm_op_t op, const int32_t pos[GIM_ARM_JOINTS],
		     const float acc[GIM_ARM_JOINTS], const float dec[GIM_ARM_JOINTS],
		     const char **msg);

/* E-stop activo/inactivo: al activarse, cada junta se queda donde está. */
void gim_arm_set_hold(gim_arm_t *a, bool hold);

void gim_arm_step(gim_arm_t *a, uint32_t now_ms);

bool gim_arm_in_pos(const gim_arm_t *a, int i);
bool gim_arm_bus_fault(const gim_arm_t *a);     /* alguna junta sin respuesta */
bool gim_arm_motor_fault(const gim_arm_t *a);   /* alguna junta reporta falla */

#ifdef __cplusplus
}
#endif

#endif /* GIM_ARM_H */
