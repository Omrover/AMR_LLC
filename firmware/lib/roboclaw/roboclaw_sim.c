#include "roboclaw_sim.h"

#include <math.h>
#include <string.h>

void rc_sim_init(rc_sim_t *s, uint8_t addr0, uint8_t addr1)
{
	memset(s, 0, sizeof(*s));
	s->dev[0].addr = addr0;
	s->dev[1].addr = addr1;
	for (int i = 0; i < RC_SIM_DEVICES; i++) {
		s->dev[i].present = true;
		s->dev[i].battery_v = 24.6f;
		s->dev[i].logic_v = 5.0f;
	}
}

rc_sim_dev_t *rc_sim_device(rc_sim_t *s, uint8_t addr)
{
	for (int i = 0; i < RC_SIM_DEVICES; i++) {
		if (s->dev[i].present && s->dev[i].addr == addr) {
			return &s->dev[i];
		}
	}
	return NULL;
}

void rc_sim_step(rc_sim_t *s, double dt)
{
	for (int i = 0; i < RC_SIM_DEVICES; i++) {
		rc_sim_dev_t *d = &s->dev[i];
		for (int m = 0; m < 2; m++) {
			double err = d->target[m] - d->speed[m];
			double step = d->accel > 0.0 ? d->accel * dt : fabs(err);
			d->speed[m] += fabs(err) <= step ? err : (err > 0 ? step : -step);
			d->enc[m] += d->speed[m] * dt;
		}
	}
}

static void put_be32(uint8_t *b, uint32_t v)
{
	b[0] = (uint8_t)(v >> 24);
	b[1] = (uint8_t)(v >> 16);
	b[2] = (uint8_t)(v >> 8);
	b[3] = (uint8_t)v;
}

static uint32_t get_be32(const uint8_t *b)
{
	return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
}

static int32_t as_signed(uint32_t raw)
{
	return (raw <= 0x7FFFFFFFUL) ? (int32_t)raw : -1 - (int32_t)(~raw);
}

static uint32_t enc_raw(double v)
{
	/* contador de 32 bits del RoboClaw, con desborde modular */
	int64_t n = (int64_t)llround(v);
	return (uint32_t)(uint64_t)n;
}

static void respond(rc_sim_t *s, const uint8_t *req2, const uint8_t *data, size_t n)
{
	uint16_t crc = rc_crc16(rc_crc16(0, req2, 2), data, n);

	if (s->drop_next > 0U) {
		s->drop_next--;
		return;
	}
	if (s->corrupt_next > 0U) {
		s->corrupt_next--;
		crc ^= 0x5555U;
	}
	memcpy(s->tx, data, n);
	s->tx[n] = (uint8_t)(crc >> 8);
	s->tx[n + 1U] = (uint8_t)crc;
	s->tx_len = n + 2U;
	s->tx_pos = 0;
}

static void ack(rc_sim_t *s)
{
	if (s->drop_next > 0U) {
		s->drop_next--;
		return;
	}
	s->tx[0] = 0xFF;
	s->tx_len = 1;
	s->tx_pos = 0;
}

/* Bytes de datos de cada comando de escritura soportado; -1 = lectura; -2 = desconocido. */
static int write_len(uint8_t cmd)
{
	switch (cmd) {
	case RC_CMD_RESET_ENCODERS: return 0;
	case RC_CMD_DUTY_M1M2: return 4;
	case RC_CMD_SPEED_M1M2: return 8;
	case RC_CMD_SPEED_ACCEL_M1M2: return 12;
	case RC_CMD_READ_VERSION:
	case RC_CMD_READ_MAIN_BATTERY:
	case RC_CMD_READ_LOGIC_BATTERY:
	case RC_CMD_READ_CURRENTS:
	case RC_CMD_READ_ENCODERS:
	case RC_CMD_READ_ISPEEDS:
	case RC_CMD_READ_STATUS:
		return -1;
	default:
		return -2;
	}
}

static void handle_read(rc_sim_t *s, rc_sim_dev_t *d, const uint8_t *req)
{
	uint8_t out[48];
	size_t n = 0;

	switch (req[1]) {
	case RC_CMD_READ_ENCODERS:
		put_be32(&out[0], enc_raw(d->enc[0]));
		put_be32(&out[4], enc_raw(d->enc[1]));
		n = 8;
		break;
	case RC_CMD_READ_ISPEEDS:
		put_be32(&out[0], (uint32_t)(int32_t)lround(d->speed[0]));
		put_be32(&out[4], (uint32_t)(int32_t)lround(d->speed[1]));
		n = 8;
		break;
	case RC_CMD_READ_MAIN_BATTERY:
	case RC_CMD_READ_LOGIC_BATTERY: {
		float v = req[1] == RC_CMD_READ_MAIN_BATTERY ? d->battery_v : d->logic_v;
		uint16_t tenth = (uint16_t)lroundf(v * 10.0f);
		out[0] = (uint8_t)(tenth >> 8);
		out[1] = (uint8_t)tenth;
		n = 2;
		break;
	}
	case RC_CMD_READ_CURRENTS:
		for (int m = 0; m < 2; m++) {
			uint16_t c = (uint16_t)(int16_t)lround(fabs(d->speed[m]) * 0.01); /* 10 mA */
			out[2 * m] = (uint8_t)(c >> 8);
			out[2 * m + 1] = (uint8_t)c;
		}
		n = 4;
		break;
	case RC_CMD_READ_STATUS:
		put_be32(out, d->status);
		n = 4;
		break;
	case RC_CMD_READ_VERSION: {
		static const char ver[] = "USB Roboclaw 2x15a v4.2.8 SIM\n";
		memcpy(out, ver, sizeof(ver)); /* incluye '\0' final */
		n = sizeof(ver);
		break;
	}
	default:
		return;
	}
	respond(s, req, out, n);
}

static void handle_write(rc_sim_t *s, rc_sim_dev_t *d, const uint8_t *frame, size_t n)
{
	const uint8_t *data = &frame[2];

	switch (frame[1]) {
	case RC_CMD_RESET_ENCODERS:
		d->enc[0] = d->enc[1] = 0.0;
		break;
	case RC_CMD_DUTY_M1M2:
		/* duty 0 = rueda libre; se modela como frenado inmediato */
		d->target[0] = d->target[1] = 0.0;
		d->speed[0] = d->speed[1] = 0.0;
		d->duty_mode = true;
		break;
	case RC_CMD_SPEED_M1M2:
		d->accel = 0.0;
		d->target[0] = as_signed(get_be32(&data[0]));
		d->target[1] = as_signed(get_be32(&data[4]));
		d->duty_mode = false;
		break;
	case RC_CMD_SPEED_ACCEL_M1M2:
		d->accel = (double)get_be32(&data[0]);
		d->target[0] = as_signed(get_be32(&data[4]));
		d->target[1] = as_signed(get_be32(&data[8]));
		d->duty_mode = false;
		break;
	default:
		return;
	}
	(void)n;
	d->commands++;
	ack(s);
}

static int sim_write(void *ctx, const uint8_t *buf, size_t n)
{
	rc_sim_t *s = ctx;

	for (size_t i = 0; i < n; i++) {
		rc_sim_dev_t *d;
		int wl;

		if (s->rx_len >= RC_SIM_RX_MAX) {
			s->rx_len = 0; /* desborde: descarta y resincroniza */
			s->bad_frames++;
		}
		s->rx[s->rx_len++] = buf[i];
		if (s->rx_len < 2U) {
			continue;
		}
		d = rc_sim_device(s, s->rx[0]);
		wl = write_len(s->rx[1]);
		if (d == NULL || wl == -2) {
			s->rx_len = 0; /* dirección ajena o comando desconocido: silencio */
			s->bad_frames += (d == NULL) ? 0U : 1U;
			continue;
		}
		if (wl == -1) {
			handle_read(s, d, s->rx);
			s->rx_len = 0;
		} else if (s->rx_len == (size_t)wl + 4U) {
			uint16_t crc = rc_crc16(0, s->rx, (size_t)wl + 2U);
			uint16_t got = (uint16_t)(((uint16_t)s->rx[wl + 2] << 8) | s->rx[wl + 3]);
			if (crc == got) {
				handle_write(s, d, s->rx, s->rx_len);
			} else {
				s->bad_frames++; /* el RoboClaw real no responde a CRC inválido */
			}
			s->rx_len = 0;
		}
	}
	return (int)n;
}

static int sim_read(void *ctx, uint8_t *buf, size_t n, uint32_t timeout_ms)
{
	rc_sim_t *s = ctx;
	size_t avail = s->tx_len - s->tx_pos;
	size_t k = n < avail ? n : avail;

	(void)timeout_ms;
	memcpy(buf, &s->tx[s->tx_pos], k);
	s->tx_pos += k;
	return (int)k;
}

static void sim_flush(void *ctx)
{
	rc_sim_t *s = ctx;

	s->tx_len = s->tx_pos = 0;
	s->rx_len = 0;
}

rc_transport_t rc_sim_transport(rc_sim_t *s)
{
	rc_transport_t t = {sim_write, sim_read, sim_flush, s};

	return t;
}
