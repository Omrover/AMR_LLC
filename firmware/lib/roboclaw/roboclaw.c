#include "roboclaw.h"

#include <string.h>

#define RC_MAX_DATA 16U

uint16_t rc_crc16(uint16_t crc, const uint8_t *data, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		crc ^= (uint16_t)((uint16_t)data[i] << 8);
		for (int bit = 0; bit < 8; bit++) {
			uint32_t shifted = (uint32_t)crc << 1;
			crc = (uint16_t)((crc & 0x8000U) ? (shifted ^ 0x1021U) : shifted);
		}
	}
	return crc;
}

void rc_init(roboclaw_t *rc, const rc_transport_t *t, uint32_t timeout_ms, uint8_t retries)
{
	memset(rc, 0, sizeof(*rc));
	rc->t = *t;
	rc->timeout_ms = timeout_ms;
	rc->retries = retries;
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

static uint16_t get_be16(const uint8_t *b)
{
	return (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
}

static int32_t as_signed(uint32_t raw)
{
	/* Conversión sin comportamiento definido por la implementación. */
	return (raw <= 0x7FFFFFFFUL) ? (int32_t)raw : -1 - (int32_t)(~raw);
}

static void count(roboclaw_t *rc, rc_status_t st)
{
	switch (st) {
	case RC_OK: rc->stats.ok++; break;
	case RC_ERR_TIMEOUT: rc->stats.timeout++; break;
	case RC_ERR_CRC: rc->stats.crc++; break;
	case RC_ERR_NACK: rc->stats.nack++; break;
	default: rc->stats.io++; break;
	}
}

static bool read_exact(roboclaw_t *rc, uint8_t *buf, size_t n, rc_status_t *st)
{
	size_t got = 0;

	while (got < n) {
		int r = rc->t.read(rc->t.ctx, buf + got, n - got, rc->timeout_ms);
		if (r < 0) {
			*st = RC_ERR_IO;
			return false;
		}
		if (r == 0) {
			*st = RC_ERR_TIMEOUT;
			return false;
		}
		got += (size_t)r;
	}
	return true;
}

static rc_status_t write_once(roboclaw_t *rc, uint8_t addr, uint8_t cmd, const uint8_t *data,
			      size_t n)
{
	uint8_t frame[2 + RC_MAX_DATA + 2];
	uint8_t ack;
	uint16_t crc;
	rc_status_t st = RC_OK;

	frame[0] = addr;
	frame[1] = cmd;
	if (n > 0U) {
		memcpy(&frame[2], data, n);
	}
	crc = rc_crc16(0, frame, n + 2U);
	frame[n + 2U] = (uint8_t)(crc >> 8);
	frame[n + 3U] = (uint8_t)crc;
	if (rc->t.flush_rx != NULL) {
		rc->t.flush_rx(rc->t.ctx);
	}
	if (rc->t.write(rc->t.ctx, frame, n + 4U) != (int)(n + 4U)) {
		return RC_ERR_IO;
	}
	if (!read_exact(rc, &ack, 1, &st)) {
		return st;
	}
	return ack == 0xFFU ? RC_OK : RC_ERR_NACK;
}

rc_status_t rc_write(roboclaw_t *rc, uint8_t addr, uint8_t cmd, const uint8_t *data, size_t n)
{
	rc_status_t st = RC_ERR_ARG;

	if (n > RC_MAX_DATA || (n > 0U && data == NULL)) {
		count(rc, st);
		return st;
	}
	for (int attempt = 0; attempt <= rc->retries; attempt++) {
		st = write_once(rc, addr, cmd, data, n);
		count(rc, st);
		if (st == RC_OK || st == RC_ERR_IO) {
			break;
		}
	}
	return st;
}

static rc_status_t read_once(roboclaw_t *rc, uint8_t addr, uint8_t cmd, uint8_t *out, size_t n)
{
	uint8_t req[2] = {addr, cmd};
	uint8_t crc_bytes[2];
	uint16_t crc;
	rc_status_t st = RC_OK;

	if (rc->t.flush_rx != NULL) {
		rc->t.flush_rx(rc->t.ctx);
	}
	if (rc->t.write(rc->t.ctx, req, 2) != 2) {
		return RC_ERR_IO;
	}
	if (!read_exact(rc, out, n, &st) || !read_exact(rc, crc_bytes, 2, &st)) {
		return st;
	}
	crc = rc_crc16(rc_crc16(0, req, 2), out, n);
	return get_be16(crc_bytes) == crc ? RC_OK : RC_ERR_CRC;
}

rc_status_t rc_read(roboclaw_t *rc, uint8_t addr, uint8_t cmd, uint8_t *out, size_t n)
{
	rc_status_t st = RC_ERR_ARG;

	if (out == NULL || n == 0U || n > RC_MAX_DATA) {
		count(rc, st);
		return st;
	}
	for (int attempt = 0; attempt <= rc->retries; attempt++) {
		st = read_once(rc, addr, cmd, out, n);
		count(rc, st);
		if (st == RC_OK || st == RC_ERR_IO) {
			break;
		}
	}
	return st;
}

rc_status_t rc_speed_accel_m1m2(roboclaw_t *rc, uint8_t addr, uint32_t accel,
				int32_t speed_m1, int32_t speed_m2)
{
	uint8_t d[12];

	put_be32(&d[0], accel);
	put_be32(&d[4], (uint32_t)speed_m1);
	put_be32(&d[8], (uint32_t)speed_m2);
	return rc_write(rc, addr, RC_CMD_SPEED_ACCEL_M1M2, d, sizeof(d));
}

rc_status_t rc_speed_m1m2(roboclaw_t *rc, uint8_t addr, int32_t speed_m1, int32_t speed_m2)
{
	uint8_t d[8];

	put_be32(&d[0], (uint32_t)speed_m1);
	put_be32(&d[4], (uint32_t)speed_m2);
	return rc_write(rc, addr, RC_CMD_SPEED_M1M2, d, sizeof(d));
}

rc_status_t rc_duty_m1m2(roboclaw_t *rc, uint8_t addr, int16_t duty_m1, int16_t duty_m2)
{
	uint8_t d[4];
	uint16_t a = (uint16_t)duty_m1, b = (uint16_t)duty_m2;

	d[0] = (uint8_t)(a >> 8);
	d[1] = (uint8_t)a;
	d[2] = (uint8_t)(b >> 8);
	d[3] = (uint8_t)b;
	return rc_write(rc, addr, RC_CMD_DUTY_M1M2, d, sizeof(d));
}

static rc_status_t read_pair32(roboclaw_t *rc, uint8_t addr, uint8_t cmd, int32_t *a, int32_t *b)
{
	uint8_t d[8];
	rc_status_t st = rc_read(rc, addr, cmd, d, sizeof(d));

	if (st == RC_OK) {
		*a = as_signed(get_be32(&d[0]));
		*b = as_signed(get_be32(&d[4]));
	}
	return st;
}

rc_status_t rc_read_encoders(roboclaw_t *rc, uint8_t addr, int32_t *enc_m1, int32_t *enc_m2)
{
	return read_pair32(rc, addr, RC_CMD_READ_ENCODERS, enc_m1, enc_m2);
}

rc_status_t rc_read_ispeeds(roboclaw_t *rc, uint8_t addr, int32_t *spd_m1, int32_t *spd_m2)
{
	return read_pair32(rc, addr, RC_CMD_READ_ISPEEDS, spd_m1, spd_m2);
}

static rc_status_t read_volt(roboclaw_t *rc, uint8_t addr, uint8_t cmd, float *volt)
{
	uint8_t d[2];
	rc_status_t st = rc_read(rc, addr, cmd, d, sizeof(d));

	if (st == RC_OK) {
		*volt = (float)get_be16(d) * 0.1f;
	}
	return st;
}

rc_status_t rc_read_main_battery(roboclaw_t *rc, uint8_t addr, float *volt)
{
	return read_volt(rc, addr, RC_CMD_READ_MAIN_BATTERY, volt);
}

rc_status_t rc_read_logic_battery(roboclaw_t *rc, uint8_t addr, float *volt)
{
	return read_volt(rc, addr, RC_CMD_READ_LOGIC_BATTERY, volt);
}

rc_status_t rc_read_currents(roboclaw_t *rc, uint8_t addr, float *amp_m1, float *amp_m2)
{
	uint8_t d[4];
	rc_status_t st = rc_read(rc, addr, RC_CMD_READ_CURRENTS, d, sizeof(d));

	if (st == RC_OK) {
		*amp_m1 = (float)(int16_t)get_be16(&d[0]) * 0.01f;
		*amp_m2 = (float)(int16_t)get_be16(&d[2]) * 0.01f;
	}
	return st;
}

rc_status_t rc_read_status(roboclaw_t *rc, uint8_t addr, uint32_t *status)
{
	uint8_t d[4];
	rc_status_t st = rc_read(rc, addr, RC_CMD_READ_STATUS, d, sizeof(d));

	if (st == RC_OK) {
		*status = get_be32(d);
	}
	return st;
}

rc_status_t rc_reset_encoders(roboclaw_t *rc, uint8_t addr)
{
	return rc_write(rc, addr, RC_CMD_RESET_ENCODERS, NULL, 0);
}

rc_status_t rc_read_version(roboclaw_t *rc, uint8_t addr, char *out, size_t size)
{
	uint8_t req[2] = {addr, RC_CMD_READ_VERSION};
	uint8_t c, crc_bytes[2];
	uint16_t crc;
	size_t len = 0;
	rc_status_t st = RC_OK;

	if (out == NULL || size == 0U) {
		return RC_ERR_ARG;
	}
	if (rc->t.flush_rx != NULL) {
		rc->t.flush_rx(rc->t.ctx);
	}
	if (rc->t.write(rc->t.ctx, req, 2) != 2) {
		count(rc, RC_ERR_IO);
		return RC_ERR_IO;
	}
	crc = rc_crc16(0, req, 2);
	for (size_t i = 0; i < 48U; i++) {
		if (!read_exact(rc, &c, 1, &st)) {
			count(rc, st);
			return st;
		}
		crc = rc_crc16(crc, &c, 1);
		if (c == 0U) {
			break;
		}
		if (c != '\n' && len + 1U < size) {
			out[len++] = (char)c;
		}
	}
	out[len] = '\0';
	if (!read_exact(rc, crc_bytes, 2, &st)) {
		count(rc, st);
		return st;
	}
	st = get_be16(crc_bytes) == crc ? RC_OK : RC_ERR_CRC;
	count(rc, st);
	return st;
}

const char *rc_status_str(rc_status_t st)
{
	static const char *const names[] = {"ok", "timeout", "crc", "nack", "io", "arg"};

	return ((unsigned int)st < sizeof(names) / sizeof(names[0])) ? names[st] : "?";
}
