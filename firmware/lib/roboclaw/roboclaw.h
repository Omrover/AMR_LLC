/*
 * Driver portable de RoboClaw (Basicmicro) en modo Packet Serial.
 *
 * Basado en read_packet/write_packet/crc_byte de examples/RoboClaw-NXP, pero
 * con transporte inyectado (UART de Zephyr, RTD o un simulador en las pruebas)
 * y comandos combinados M1/M2 para controlar 2 RoboClaw (dir 128 y 129) en el
 * mismo bus serie (modo "Multi-Unit" en Motion Studio).
 *
 * Trama de escritura: [addr][cmd][datos...][crc16 hi][crc16 lo] -> 0xFF
 * Trama de lectura:   [addr][cmd] -> [datos...][crc16 hi][crc16 lo]
 * CRC16-CCITT (poly 0x1021, init 0) sobre addr, cmd y datos. Big endian.
 */
#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Comandos usados (manual de usuario RoboClaw, sección Packet Serial). */
enum {
	RC_CMD_RESET_ENCODERS = 20,
	RC_CMD_READ_VERSION = 21,
	RC_CMD_READ_MAIN_BATTERY = 24,
	RC_CMD_READ_LOGIC_BATTERY = 25,
	RC_CMD_DUTY_M1M2 = 34,
	RC_CMD_SPEED_M1M2 = 37,
	RC_CMD_SPEED_ACCEL_M1M2 = 40,
	RC_CMD_READ_CURRENTS = 49,
	RC_CMD_READ_ENCODERS = 78,
	RC_CMD_READ_ISPEEDS = 79,
	RC_CMD_READ_STATUS = 90,
};

typedef enum {
	RC_OK = 0,
	RC_ERR_TIMEOUT,     /* no llegaron todos los bytes */
	RC_ERR_CRC,         /* CRC de la respuesta inválido */
	RC_ERR_NACK,        /* escritura sin 0xFF */
	RC_ERR_IO,          /* error del transporte (UART) */
	RC_ERR_ARG,
} rc_status_t;

/* Transporte serie. read() debe esperar hasta timeout_ms y devolver los bytes
 * leídos (0..n) o <0 ante error; write() devuelve n o <0. */
typedef struct {
	int (*write)(void *ctx, const uint8_t *buf, size_t n);
	int (*read)(void *ctx, uint8_t *buf, size_t n, uint32_t timeout_ms);
	void (*flush_rx)(void *ctx);
	void *ctx;
} rc_transport_t;

typedef struct {
	uint32_t ok;
	uint32_t timeout;
	uint32_t crc;
	uint32_t nack;
	uint32_t io;
} rc_stats_t;

typedef struct {
	rc_transport_t t;
	uint32_t timeout_ms;    /* por transacción; 10 ms holgado a 115200 */
	uint8_t retries;        /* reintentos ante timeout/CRC (0 = ninguno) */
	rc_stats_t stats;
} roboclaw_t;

void rc_init(roboclaw_t *rc, const rc_transport_t *t, uint32_t timeout_ms, uint8_t retries);
uint16_t rc_crc16(uint16_t crc, const uint8_t *data, size_t n);

/* Transacciones genéricas. */
rc_status_t rc_write(roboclaw_t *rc, uint8_t addr, uint8_t cmd, const uint8_t *data, size_t n);
rc_status_t rc_read(roboclaw_t *rc, uint8_t addr, uint8_t cmd, uint8_t *out, size_t n);

/* Comandos de alto nivel. Velocidades en cuentas de encoder por segundo (QPPS),
 * aceleración en QPPS/s, enteros con signo. */
rc_status_t rc_speed_accel_m1m2(roboclaw_t *rc, uint8_t addr, uint32_t accel,
				int32_t speed_m1, int32_t speed_m2);
rc_status_t rc_speed_m1m2(roboclaw_t *rc, uint8_t addr, int32_t speed_m1, int32_t speed_m2);
/* duty en [-32767, 32767]; 0,0 = rueda libre (parada sin retención). */
rc_status_t rc_duty_m1m2(roboclaw_t *rc, uint8_t addr, int16_t duty_m1, int16_t duty_m2);
rc_status_t rc_read_encoders(roboclaw_t *rc, uint8_t addr, int32_t *enc_m1, int32_t *enc_m2);
/* Velocidad instantánea (último 1/300 s) en QPPS. */
rc_status_t rc_read_ispeeds(roboclaw_t *rc, uint8_t addr, int32_t *spd_m1, int32_t *spd_m2);
rc_status_t rc_read_main_battery(roboclaw_t *rc, uint8_t addr, float *volt);
rc_status_t rc_read_logic_battery(roboclaw_t *rc, uint8_t addr, float *volt);
rc_status_t rc_read_currents(roboclaw_t *rc, uint8_t addr, float *amp_m1, float *amp_m2);
/* Estado de error de 32 bits (firmware 4.1.x+). */
rc_status_t rc_read_status(roboclaw_t *rc, uint8_t addr, uint32_t *status);
rc_status_t rc_reset_encoders(roboclaw_t *rc, uint8_t addr);
/* Cadena de versión terminada en '\n' '\0' + CRC (hasta 48 caracteres). */
rc_status_t rc_read_version(roboclaw_t *rc, uint8_t addr, char *out, size_t size);

const char *rc_status_str(rc_status_t st);

#ifdef __cplusplus
}
#endif

#endif /* ROBOCLAW_H */
