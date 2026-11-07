#include "bno08x.h"

#include <string.h>

#define Q8 (1.0f / 256.0f)
#define Q9 (1.0f / 512.0f)
#define Q14 (1.0f / 16384.0f)

void bno_init(bno08x_t *b, const bno_transport_t *t)
{
	memset(b, 0, sizeof(*b));
	b->t = *t;
	b->data.quat[0] = 1.0f;
}

static int send_packet(bno08x_t *b, uint8_t channel, const uint8_t *payload, size_t n)
{
	uint8_t pkt[4 + 32];
	size_t len = n + 4U;

	if (n > 32U || channel >= BNO_CHANNELS) {
		return -1;
	}
	pkt[0] = (uint8_t)(len & 0xFFU);
	pkt[1] = (uint8_t)(len >> 8);
	pkt[2] = channel;
	pkt[3] = b->seq[channel]++;
	memcpy(&pkt[4], payload, n);
	return b->t.write(b->t.ctx, pkt, len) < 0 ? -1 : 0;
}

int bno_soft_reset(bno08x_t *b)
{
	const uint8_t reset = 1;

	return send_packet(b, BNO_CH_EXECUTABLE, &reset, 1);
}

int bno_enable_report(bno08x_t *b, uint8_t report_id, uint32_t interval_us)
{
	uint8_t c[17] = {0};

	c[0] = BNO_RPT_SET_FEATURE;
	c[1] = report_id;
	/* c[2] flags, c[3..4] change sensitivity = 0 */
	c[5] = (uint8_t)interval_us;
	c[6] = (uint8_t)(interval_us >> 8);
	c[7] = (uint8_t)(interval_us >> 16);
	c[8] = (uint8_t)(interval_us >> 24);
	/* c[9..12] batch interval = 0, c[13..16] sensor specific = 0 */
	return send_packet(b, BNO_CH_CONTROL, c, sizeof(c));
}

int bno_request_product_id(bno08x_t *b)
{
	const uint8_t req[2] = {BNO_RPT_PRODUCT_ID_REQ, 0};

	return send_packet(b, BNO_CH_CONTROL, req, sizeof(req));
}

static int16_t le16(const uint8_t *p)
{
	return (int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* Longitud total de cada reporte en el canal de reportes; 0 = desconocido. */
static size_t report_len(uint8_t id)
{
	switch (id) {
	case BNO_RPT_ACCEL:
	case BNO_RPT_GYRO:
	case BNO_RPT_MAG:
	case BNO_RPT_LINEAR_ACCEL:
	case BNO_RPT_GRAVITY:
		return 10;
	case BNO_RPT_GAME_ROTATION_VECTOR:
		return 12;
	case BNO_RPT_ROTATION_VECTOR:
		return 14;
	case 0x07: /* gyro sin calibrar */
		return 16;
	case BNO_RPT_TIMESTAMP_REBASE:
	case BNO_RPT_BASE_TIMESTAMP:
		return 5;
	default:
		return 0;
	}
}

static void vec3(float out[3], const uint8_t *p, float q)
{
	out[0] = (float)le16(&p[0]) * q;
	out[1] = (float)le16(&p[2]) * q;
	out[2] = (float)le16(&p[4]) * q;
}

uint32_t bno_parse_reports(bno08x_t *b, const uint8_t *payload, size_t n)
{
	uint32_t upd = 0;
	size_t i = 0;

	while (i < n) {
		uint8_t id = payload[i];
		size_t len = report_len(id);
		const uint8_t *r = &payload[i];

		if (len == 0U || i + len > n) {
			b->errors++; /* reporte desconocido o truncado: se descarta el resto */
			break;
		}
		switch (id) {
		case BNO_RPT_GAME_ROTATION_VECTOR:
		case BNO_RPT_ROTATION_VECTOR:
			/* datos: i, j, k, real en Q14 */
			b->data.quat[1] = (float)le16(&r[4]) * Q14;
			b->data.quat[2] = (float)le16(&r[6]) * Q14;
			b->data.quat[3] = (float)le16(&r[8]) * Q14;
			b->data.quat[0] = (float)le16(&r[10]) * Q14;
			b->data.quat_accuracy = r[2] & 0x03U;
			upd |= BNO_UPD_QUAT;
			break;
		case BNO_RPT_GYRO:
			vec3(b->data.gyro, &r[4], Q9);
			upd |= BNO_UPD_GYRO;
			break;
		case BNO_RPT_ACCEL:
			vec3(b->data.accel, &r[4], Q8);
			b->data.accel_accuracy = r[2] & 0x03U;
			upd |= BNO_UPD_ACCEL;
			break;
		case BNO_RPT_LINEAR_ACCEL:
			vec3(b->data.lin_accel, &r[4], Q8);
			upd |= BNO_UPD_LIN_ACCEL;
			break;
		default:
			break; /* timestamps y reportes no usados */
		}
		i += len;
	}
	b->data.updated |= upd;
	return upd;
}

int bno_service(bno08x_t *b)
{
	uint8_t hdr[4];
	size_t len;
	uint8_t channel;

	if (b->t.read(b->t.ctx, hdr, sizeof(hdr)) < 0) {
		b->errors++;
		return -1;
	}
	len = (size_t)(hdr[0] | ((hdr[1] & 0x7FU) << 8));
	if (len == 0U || (hdr[0] == 0xFF && hdr[1] == 0xFF)) {
		return 0; /* nada pendiente (o bus sin dispositivo con pull-ups) */
	}
	if (len < 4U || len > BNO_MAX_PKT) {
		b->errors++;
		return -1;
	}
	/* Cada lectura I2C empieza de nuevo con el encabezado. */
	if (b->t.read(b->t.ctx, b->buf, len) < 0) {
		b->errors++;
		return -1;
	}
	b->packets++;
	channel = b->buf[2];
	if (channel == BNO_CH_EXECUTABLE && len >= 5U && b->buf[4] == 1U) {
		b->resets++;
		memset(b->seq, 0, sizeof(b->seq));
		return (int)BNO_EVT_RESET;
	}
	if (channel == BNO_CH_REPORTS || channel == BNO_CH_WAKE) {
		uint32_t upd = bno_parse_reports(b, &b->buf[4], len - 4U);
		return upd ? (int)upd : (int)BNO_EVT_OTHER;
	}
	return (int)BNO_EVT_OTHER;
}
