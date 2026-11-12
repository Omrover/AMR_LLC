/*
 * GIM6010-8 por CAN, protocolo "Custom CAN V3.10b2" (GDZ). C99 portable, sin RTOS.
 * Fuente: AMR_arm/examples/main.txt (prueba con RTD FlexCAN en la FRDM-A).
 *
 *  - Tramas estándar a 1 Mbps, little endian, DLC según comando.
 *  - El maestro transmite con ID = 0x100 | addr y el motor responde con ID = addr
 *    (filtrar la recepción en addr evita leer el eco propio del FlexCAN).
 *  - Cada comando se confirma con una trama cuyo data[0] es el mismo código.
 *  - Posición: 16384 counts por vuelta. 0xDA mueve con perfil trapezoidal a una
 *    posición absoluta (int32) y 0xA4 reporta la posición en 16 bits: esta
 *    librería la desenvuelve a int32 (ver gim_unwrap).
 */
#ifndef GIM6010_H
#define GIM6010_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GIM_COUNTS_PER_REV 16384
#define GIM_TX_BASE 0x100u

enum {
	GIM_CMD_READ_VER = 0xA0,
	GIM_CMD_READ_A4 = 0xA4,    /* temperatura, corriente, velocidad, posición */
	GIM_CMD_READ_AE = 0xAE,    /* Vbus, Ibus, temperatura, modo, falla */
	GIM_CMD_CLEAR_FAULT = 0xAF,
	GIM_CMD_SPEED = 0xC1,      /* int32, 0.01 rpm */
	GIM_CMD_DISABLE = 0xCF,    /* salida libre */
	GIM_CMD_TRAP_ACC = 0xD0,   /* uint32, 0.01 rpm/s */
	GIM_CMD_TRAP_DEC = 0xD1,
	GIM_CMD_POS_TRAP = 0xDA,   /* tipo (0 abs, 1 rel) + int32 counts */
};

typedef enum { GIM_OK = 0, GIM_ERR_TX = -1, GIM_ERR_TIMEOUT = -2, GIM_ERR_ARG = -3 } gim_status_t;

/* Transporte CAN (HAL): tramas estándar de hasta 8 bytes. */
typedef struct {
	/* 0 en éxito. */
	int (*send)(void *ctx, uint16_t id, const uint8_t *data, uint8_t len);
	/* Espera una trama hasta timeout_ms: 1 si llegó, 0 si no, <0 si error. */
	int (*recv)(void *ctx, uint16_t *id, uint8_t data[8], uint8_t *len, uint32_t timeout_ms);
	void *ctx;
} gim_transport_t;

typedef struct {
	gim_transport_t t;
	uint32_t timeout_ms;
	uint32_t tx_errors;
	uint32_t timeouts;
} gim_bus_t;

typedef struct {
	int8_t temp_c;
	int16_t cur_ma;
	int16_t vel_crpm;          /* 0.01 rpm */
	uint16_t pos16;            /* posición de 16 bits (envuelve) */
} gim_a4_t;

typedef struct {
	uint16_t vbus_cv;          /* 0.01 V */
	uint16_t ibus_ca;          /* 0.01 A */
	uint8_t temp_c;
	uint8_t mode;
	uint8_t fault;             /* 0 = sin falla */
} gim_ae_t;

void gim_init(gim_bus_t *b, const gim_transport_t *t, uint32_t timeout_ms);

/* Transacciones: envían a 0x100|addr y esperan la confirmación de addr. */
gim_status_t gim_read_a4(gim_bus_t *b, uint8_t addr, gim_a4_t *out);
gim_status_t gim_read_ae(gim_bus_t *b, uint8_t addr, gim_ae_t *out);
gim_status_t gim_clear_fault(gim_bus_t *b, uint8_t addr);
gim_status_t gim_disable(gim_bus_t *b, uint8_t addr);
gim_status_t gim_trap_acc_dec(gim_bus_t *b, uint8_t addr, uint32_t acc_crpm_s, uint32_t dec_crpm_s);
gim_status_t gim_pos_abs(gim_bus_t *b, uint8_t addr, int32_t counts);

/* Codificación pura (también la usa el simulador). Devuelven el DLC. */
uint8_t gim_encode_pos_abs(uint8_t out[8], int32_t counts);
uint8_t gim_encode_u32(uint8_t out[8], uint8_t cmd, uint32_t value);
void gim_decode_a4(const uint8_t d[8], gim_a4_t *out);
void gim_decode_ae(const uint8_t d[8], gim_ae_t *out);

/* Desenvuelve la posición de 16 bits a int32. La primera lectura (valid = false)
 * se toma con signo (-32768..32767): la referencia absoluta tras un encendido se
 * calibra en la NUC (arm.yaml zero_offset). Las siguientes suman la diferencia
 * modular, válida mientras la junta gire menos de media vuelta de 16 bits entre
 * lecturas. */
int32_t gim_unwrap(int32_t prev, bool valid, uint16_t pos16);

#ifdef __cplusplus
}
#endif

#endif /* GIM6010_H */
