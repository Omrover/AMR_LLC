/*
 * Simulador de la BNO080 a nivel SHTP: interpreta lo que escribe el driver
 * (Set Feature, soft reset) y entrega paquetes de reportes con el mismo formato
 * que el sensor real. Lo usan las pruebas en host y el HAL de native_sim.
 *
 * Semántica de lectura I2C (igual que el chip): cada lectura empieza con el
 * encabezado del paquete pendiente; leer solo 4 bytes no lo consume, leer el
 * paquete completo sí. Sin paquete pendiente se leen ceros (len = 0).
 */
#ifndef BNO08X_SIM_H
#define BNO08X_SIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bno08x.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BNO_SIM_QUEUE 8
#define BNO_SIM_PKT_MAX 64

typedef struct {
	uint8_t len;
	uint8_t data[BNO_SIM_PKT_MAX];
} bno_sim_pkt_t;

typedef struct {
	bool present;                 /* false: el bus no responde (error de I/O) */
	/* movimiento del robot (frame del sensor = base_link) */
	float yaw, wz, ax, ay;
	float gyro_bias;              /* rad/s, para probar el EKF */
	/* reportes habilitados con Set Feature */
	uint32_t interval_us[4];      /* accel, gyro, lin_accel, grv */
	double since_us[4];
	uint8_t seq[BNO_CHANNELS];
	uint8_t rpt_seq;
	bno_sim_pkt_t q[BNO_SIM_QUEUE];
	size_t q_head, q_len;
	uint32_t dropped;             /* paquetes perdidos por cola llena */
	uint32_t resets;
	uint32_t features_set;
} bno_sim_t;

/* Arranca como el chip tras encender: cola el anuncio de reset (canal 1). */
void bno_sim_init(bno_sim_t *s);
/* Reset espontáneo (brown-out, ESD): deshabilita reportes y anuncia el reset. */
void bno_sim_reset(bno_sim_t *s);
void bno_sim_set_motion(bno_sim_t *s, float yaw, float wz, float ax, float ay);
/* Avanza el tiempo y cola los reportes vencidos en un solo paquete. */
void bno_sim_step(bno_sim_t *s, double dt);
bno_transport_t bno_sim_transport(bno_sim_t *s);

#ifdef __cplusplus
}
#endif

#endif /* BNO08X_SIM_H */
