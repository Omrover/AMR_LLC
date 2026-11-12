/*
 * Simulador de un bus CAN con hasta 4 GIM6010-8 (pruebas en host y native_sim).
 * Responde 0xA4, 0xAE, 0xAF, 0xCF, 0xD0, 0xD1 y 0xDA como el motor real, con un
 * perfil trapezoidal en counts. No modela el eco del FlexCAN (se filtra en el HAL).
 */
#ifndef GIM6010_SIM_H
#define GIM6010_SIM_H

#include <stdbool.h>
#include <stdint.h>

#include "gim6010.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GIM_SIM_MOTORS 4
#define GIM_SIM_QUEUE 8

typedef struct {
	uint8_t addr;
	bool online;             /* false: no responde (cable suelto, sin alimentación) */
	bool enabled;            /* false tras 0xCF: no sigue el objetivo */
	double pos;              /* counts (multivuelta) */
	double vel;              /* counts/s */
	double target;
	double acc, dec;         /* counts/s^2 */
	double vmax;             /* counts/s */
	uint8_t fault;
	int8_t temp_c;
	uint32_t pos_cmds;       /* 0xDA recibidos */
} gim_sim_motor_t;

typedef struct {
	gim_sim_motor_t m[GIM_SIM_MOTORS];
	uint16_t q_id[GIM_SIM_QUEUE];
	uint8_t q_data[GIM_SIM_QUEUE][8];
	uint8_t q_len[GIM_SIM_QUEUE];
	int q_head, q_count;
	uint32_t frames_tx;      /* tramas recibidas del maestro */
} gim_sim_t;

/* Motores en addr 1..4, en posición 0, con 30 rpm de velocidad máxima. */
void gim_sim_init(gim_sim_t *s);
void gim_sim_step(gim_sim_t *s, double dt);
gim_transport_t gim_sim_transport(gim_sim_t *s);
double gim_counts_per_s(double rpm);

#ifdef __cplusplus
}
#endif

#endif /* GIM6010_SIM_H */
