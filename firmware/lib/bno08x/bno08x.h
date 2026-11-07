/*
 * Driver mínimo de la IMU BNO080/BNO085 (CEVA SH-2 sobre SHTP) por I2C.
 *
 * La BNO080 ejecuta su propio filtro de fusión (Kalman) on-chip; aquí solo se
 * configuran y leen los reportes:
 *   - Game Rotation Vector (0x08): cuaternión sin magnetómetro (los motores
 *     distorsionan el campo; el yaw es relativo y lo ancla el EKF del MCU)
 *   - Gyroscope calibrated (0x02), Accelerometer (0x01), Linear accel (0x04)
 *
 * Transporte inyectado: en Zephyr i2c_read()/i2c_write() a la dirección 0x4A
 * (SA0 = GND) o 0x4B. La BNO080 hace clock stretching: usar <= 400 kHz.
 * Paquete SHTP: [len LSB][len MSB | bit15 continuación][canal][secuencia] + carga.
 */
#ifndef BNO08X_H
#define BNO08X_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BNO_I2C_ADDR_DEFAULT 0x4A
#define BNO_MAX_PKT 512

/* Canales SHTP */
enum { BNO_CH_COMMAND = 0, BNO_CH_EXECUTABLE = 1, BNO_CH_CONTROL = 2, BNO_CH_REPORTS = 3,
       BNO_CH_WAKE = 4, BNO_CH_GYRO_RV = 5, BNO_CHANNELS = 6 };

/* IDs de reporte SH-2 */
enum {
	BNO_RPT_ACCEL = 0x01,
	BNO_RPT_GYRO = 0x02,
	BNO_RPT_MAG = 0x03,
	BNO_RPT_LINEAR_ACCEL = 0x04,
	BNO_RPT_ROTATION_VECTOR = 0x05,
	BNO_RPT_GRAVITY = 0x06,
	BNO_RPT_GAME_ROTATION_VECTOR = 0x08,
	BNO_RPT_TIMESTAMP_REBASE = 0xFA,
	BNO_RPT_BASE_TIMESTAMP = 0xFB,
	BNO_RPT_SET_FEATURE = 0xFD,
	BNO_RPT_GET_FEATURE_RESP = 0xFC,
	BNO_RPT_PRODUCT_ID_REQ = 0xF9,
	BNO_RPT_PRODUCT_ID_RESP = 0xF8,
};

/* Bits de bno08x_data_t.updated y eventos de bno_service() */
#define BNO_UPD_QUAT (1u << 0)
#define BNO_UPD_GYRO (1u << 1)
#define BNO_UPD_ACCEL (1u << 2)
#define BNO_UPD_LIN_ACCEL (1u << 3)
#define BNO_EVT_RESET (1u << 8)     /* el sensor se reinició: re-habilitar reportes */
#define BNO_EVT_OTHER (1u << 9)     /* paquete de control/comando sin datos */

typedef struct {
	/* 0 en éxito, <0 en error. read() lee exactamente n bytes de un I2C read. */
	int (*write)(void *ctx, const uint8_t *buf, size_t n);
	int (*read)(void *ctx, uint8_t *buf, size_t n);
	void *ctx;
} bno_transport_t;

typedef struct {
	float quat[4];        /* w, x, y, z */
	float gyro[3];        /* rad/s */
	float accel[3];       /* m/s^2 (incluye gravedad) */
	float lin_accel[3];   /* m/s^2 */
	uint8_t quat_accuracy;    /* 0..3 */
	uint8_t accel_accuracy;   /* 0..3 */
	uint32_t updated;     /* BNO_UPD_* desde la última limpieza por el usuario */
} bno08x_data_t;

typedef struct {
	bno_transport_t t;
	uint8_t seq[BNO_CHANNELS];
	uint8_t buf[BNO_MAX_PKT];
	bno08x_data_t data;
	uint32_t resets;
	uint32_t errors;
	uint32_t packets;
} bno08x_t;

void bno_init(bno08x_t *b, const bno_transport_t *t);
int bno_soft_reset(bno08x_t *b);
/* Habilita un reporte periódico (interval_us = 10000 -> 100 Hz). */
int bno_enable_report(bno08x_t *b, uint8_t report_id, uint32_t interval_us);
int bno_request_product_id(bno08x_t *b);
/* Lee y procesa un paquete. >0: máscara BNO_UPD_* / BNO_EVT_*; 0: nada pendiente; <0: error. */
int bno_service(bno08x_t *b);
/* Procesa la carga de un paquete del canal de reportes (expuesta para pruebas). */
uint32_t bno_parse_reports(bno08x_t *b, const uint8_t *payload, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* BNO08X_H */
