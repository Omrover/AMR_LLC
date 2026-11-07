/*
 * Simulador de RoboClaw en Packet Serial: interpreta las tramas que escribe el
 * driver y genera las respuestas byte a byte. Lo usan las pruebas en host y el
 * HAL de native_sim del firmware (sin hardware). Modela hasta 2 dispositivos en
 * el mismo bus, cada uno con M1/M2: rampa de velocidad, encoders, batería y
 * corriente. Permite inyectar fallas (sin respuesta, CRC corrupto).
 */
#ifndef ROBOCLAW_SIM_H
#define ROBOCLAW_SIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "roboclaw.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_SIM_DEVICES 2
#define RC_SIM_RX_MAX 64
#define RC_SIM_TX_MAX 64

typedef struct {
	uint8_t addr;
	bool present;
	double target[2];   /* QPPS */
	double speed[2];    /* QPPS */
	double accel;       /* QPPS/s (0 = inmediato) */
	double enc[2];
	float battery_v;
	float logic_v;
	uint32_t status;
	bool duty_mode;     /* último comando fue duty */
	uint32_t commands;  /* tramas de escritura aceptadas */
} rc_sim_dev_t;

typedef struct {
	rc_sim_dev_t dev[RC_SIM_DEVICES];
	uint8_t rx[RC_SIM_RX_MAX];
	size_t rx_len;
	uint8_t tx[RC_SIM_TX_MAX];
	size_t tx_len;
	size_t tx_pos;
	/* fallas */
	uint32_t drop_next;     /* número de respuestas a omitir */
	uint32_t corrupt_next;  /* número de respuestas con CRC corrupto */
	uint32_t bad_frames;
} rc_sim_t;

void rc_sim_init(rc_sim_t *s, uint8_t addr0, uint8_t addr1);
/* Avanza la física dt segundos (rampas e integración de encoders). */
void rc_sim_step(rc_sim_t *s, double dt);
/* Transporte listo para rc_init() con ctx = rc_sim_t*. */
rc_transport_t rc_sim_transport(rc_sim_t *s);
rc_sim_dev_t *rc_sim_device(rc_sim_t *s, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* ROBOCLAW_SIM_H */
