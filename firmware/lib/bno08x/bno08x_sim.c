#include "bno08x_sim.h"

#include <math.h>
#include <string.h>

enum { SIM_ACCEL, SIM_GYRO, SIM_LIN, SIM_GRV, SIM_N };

static const uint8_t sim_ids[SIM_N] = {
	BNO_RPT_ACCEL, BNO_RPT_GYRO, BNO_RPT_LINEAR_ACCEL, BNO_RPT_GAME_ROTATION_VECTOR,
};

static int sim_index(uint8_t id)
{
	for (int i = 0; i < SIM_N; i++) {
		if (sim_ids[i] == id) {
			return i;
		}
	}
	return -1;
}

static void push(bno_sim_t *s, uint8_t channel, const uint8_t *payload, size_t n)
{
	bno_sim_pkt_t *p;
	size_t len = n + 4U;

	if (len > BNO_SIM_PKT_MAX) {
		return;
	}
	if (s->q_len == BNO_SIM_QUEUE) {
		s->dropped++; /* el chip también descarta si nadie lee */
		return;
	}
	p = &s->q[(s->q_head + s->q_len) % BNO_SIM_QUEUE];
	p->len = (uint8_t)len;
	p->data[0] = (uint8_t)len;
	p->data[1] = 0;
	p->data[2] = channel;
	p->data[3] = s->seq[channel]++;
	memcpy(&p->data[4], payload, n);
	s->q_len++;
}

void bno_sim_reset(bno_sim_t *s)
{
	const uint8_t reset_complete = 1;

	memset(s->interval_us, 0, sizeof(s->interval_us));
	memset(s->since_us, 0, sizeof(s->since_us));
	memset(s->seq, 0, sizeof(s->seq));
	s->q_head = 0;
	s->q_len = 0;
	s->resets++;
	push(s, BNO_CH_EXECUTABLE, &reset_complete, 1);
}

void bno_sim_init(bno_sim_t *s)
{
	memset(s, 0, sizeof(*s));
	s->present = true;
	bno_sim_reset(s);
	s->resets = 0;
}

void bno_sim_set_motion(bno_sim_t *s, float yaw, float wz, float ax, float ay)
{
	s->yaw = yaw;
	s->wz = wz;
	s->ax = ax;
	s->ay = ay;
}

static void put16(uint8_t *p, float value, float scale)
{
	long v = lroundf(value * scale);

	if (v > 32767) {
		v = 32767;
	} else if (v < -32768) {
		v = -32768;
	}
	p[0] = (uint8_t)((uint16_t)v & 0xFFU);
	p[1] = (uint8_t)((uint16_t)v >> 8);
}

static size_t put_report(bno_sim_t *s, int which, uint8_t *r)
{
	float half = 0.5f * s->yaw;

	memset(r, 0, 12);
	r[0] = sim_ids[which];
	r[1] = s->rpt_seq++;
	r[2] = 3; /* exactitud alta */
	switch (which) {
	case SIM_GRV:
		put16(&r[4], 0.0f, 16384.0f);
		put16(&r[6], 0.0f, 16384.0f);
		put16(&r[8], sinf(half), 16384.0f);
		put16(&r[10], cosf(half), 16384.0f);
		return 12;
	case SIM_GYRO:
		put16(&r[4], 0.0f, 512.0f);
		put16(&r[6], 0.0f, 512.0f);
		put16(&r[8], s->wz + s->gyro_bias, 512.0f);
		return 10;
	case SIM_ACCEL:
		put16(&r[4], s->ax, 256.0f);
		put16(&r[6], s->ay, 256.0f);
		put16(&r[8], 9.80665f, 256.0f);
		return 10;
	default:
		put16(&r[4], s->ax, 256.0f);
		put16(&r[6], s->ay, 256.0f);
		put16(&r[8], 0.0f, 256.0f);
		return 10;
	}
}

void bno_sim_step(bno_sim_t *s, double dt)
{
	uint8_t payload[BNO_SIM_PKT_MAX - 4];
	size_t n = 0;

	for (int i = 0; i < SIM_N; i++) {
		if (s->interval_us[i] == 0U) {
			continue;
		}
		s->since_us[i] += dt * 1e6;
		if (s->since_us[i] + 1e-3 < (double)s->interval_us[i]) {
			continue;
		}
		s->since_us[i] -= (double)s->interval_us[i];
		if (s->since_us[i] > (double)s->interval_us[i]) {
			s->since_us[i] = 0.0; /* no acumula atraso */
		}
		if (n == 0U) {
			/* base timestamp: 0xFB + 4 bytes (100 us) */
			memset(payload, 0, 5);
			payload[0] = BNO_RPT_BASE_TIMESTAMP;
			n = 5;
		}
		n += put_report(s, i, &payload[n]);
	}
	if (n > 0U) {
		push(s, BNO_CH_REPORTS, payload, n);
	}
}

static int sim_write(void *ctx, const uint8_t *buf, size_t n)
{
	bno_sim_t *s = ctx;
	size_t len;

	if (!s->present) {
		return -5;
	}
	if (n < 4U) {
		return -22;
	}
	len = (size_t)(buf[0] | ((buf[1] & 0x7FU) << 8));
	if (len != n) {
		return -22;
	}
	if (buf[2] == BNO_CH_EXECUTABLE && n >= 5U && buf[4] == 1U) {
		bno_sim_reset(s);
		return 0;
	}
	if (buf[2] == BNO_CH_CONTROL && n >= 4U + 17U && buf[4] == BNO_RPT_SET_FEATURE) {
		int i = sim_index(buf[5]);
		uint32_t us = (uint32_t)buf[9] | ((uint32_t)buf[10] << 8) |
			      ((uint32_t)buf[11] << 16) | ((uint32_t)buf[12] << 24);

		if (i >= 0) {
			s->interval_us[i] = us;
			s->since_us[i] = 0.0;
			s->features_set++;
		}
	}
	return 0;
}

static int sim_read(void *ctx, uint8_t *buf, size_t n)
{
	bno_sim_t *s = ctx;
	const bno_sim_pkt_t *p;

	if (!s->present) {
		return -5;
	}
	memset(buf, 0, n);
	if (s->q_len == 0U) {
		return 0;
	}
	p = &s->q[s->q_head];
	memcpy(buf, p->data, n < p->len ? n : p->len);
	if (n >= p->len) {
		s->q_head = (s->q_head + 1U) % BNO_SIM_QUEUE;
		s->q_len--;
	}
	return 0;
}

bno_transport_t bno_sim_transport(bno_sim_t *s)
{
	bno_transport_t t = {sim_write, sim_read, s};

	return t;
}
