#include "gim6010_sim.h"

#include <math.h>
#include <string.h>

double gim_counts_per_s(double rpm)
{
	return rpm * (double)GIM_COUNTS_PER_REV / 60.0;
}

void gim_sim_init(gim_sim_t *s)
{
	memset(s, 0, sizeof(*s));
	for (int i = 0; i < GIM_SIM_MOTORS; i++) {
		gim_sim_motor_t *m = &s->m[i];

		m->addr = (uint8_t)(i + 1);
		m->online = true;
		m->enabled = true;
		m->acc = m->dec = gim_counts_per_s(100.0); /* 100 rpm/s */
		m->vmax = gim_counts_per_s(30.0);
		m->temp_c = 30;
	}
}

static gim_sim_motor_t *find(gim_sim_t *s, uint16_t id)
{
	if ((id & 0xF00u) != GIM_TX_BASE) {
		return NULL;
	}
	for (int i = 0; i < GIM_SIM_MOTORS; i++) {
		if (s->m[i].addr == (uint8_t)(id & 0xFFu) && s->m[i].online) {
			return &s->m[i];
		}
	}
	return NULL;
}

static void reply(gim_sim_t *s, const gim_sim_motor_t *m, const uint8_t *d, uint8_t len)
{
	int slot;

	if (s->q_count >= GIM_SIM_QUEUE) {
		return; /* buzón lleno: la trama se pierde, como en un FIFO real */
	}
	slot = (s->q_head + s->q_count) % GIM_SIM_QUEUE;
	s->q_id[slot] = m->addr;
	memset(s->q_data[slot], 0, 8);
	memcpy(s->q_data[slot], d, len);
	s->q_len[slot] = len;
	s->q_count++;
}

static uint32_t get_u32(const uint8_t *b)
{
	return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static void put_u16(uint8_t *b, uint16_t v)
{
	b[0] = (uint8_t)v;
	b[1] = (uint8_t)(v >> 8);
}

static int sim_send(void *ctx, uint16_t id, const uint8_t *data, uint8_t len)
{
	gim_sim_t *s = ctx;
	gim_sim_motor_t *m = find(s, id);
	uint8_t r[8] = {0};

	s->frames_tx++;
	if (m == NULL || len == 0U) {
		return 0; /* nadie responde: el maestro verá timeout */
	}
	r[0] = data[0];
	switch (data[0]) {
	case GIM_CMD_READ_A4: {
		int32_t p = (int32_t)lround(m->pos);

		r[1] = (uint8_t)m->temp_c;
		put_u16(&r[2], (uint16_t)(int16_t)(fabs(m->vel) > 1.0 ? 400 : 50)); /* mA */
		put_u16(&r[4], (uint16_t)(int16_t)lround(m->vel * 60.0 / GIM_COUNTS_PER_REV * 100.0));
		put_u16(&r[6], (uint16_t)p);
		reply(s, m, r, 8);
		break;
	}
	case GIM_CMD_READ_AE:
		put_u16(&r[1], 2400);  /* 24.00 V */
		put_u16(&r[3], 50);
		r[5] = (uint8_t)m->temp_c;
		r[6] = m->enabled ? 1 : 0;
		r[7] = m->fault;
		reply(s, m, r, 8);
		break;
	case GIM_CMD_CLEAR_FAULT:
		m->fault = 0;
		reply(s, m, r, 2);
		break;
	case GIM_CMD_DISABLE:
		m->enabled = false;
		m->vel = 0.0;
		reply(s, m, r, 8);
		break;
	case GIM_CMD_TRAP_ACC:
	case GIM_CMD_TRAP_DEC:
		if (len >= 5U) {
			double a = gim_counts_per_s((double)get_u32(&data[1]) / 100.0);

			if (data[0] == GIM_CMD_TRAP_ACC) {
				m->acc = a;
			} else {
				m->dec = a;
			}
			memcpy(&r[1], &data[1], 4);
			reply(s, m, r, 5);
		}
		break;
	case GIM_CMD_POS_TRAP:
		if (len >= 6U && m->fault == 0U) {
			double t = (double)(int32_t)get_u32(&data[2]);

			m->target = data[1] == 0U ? t : m->pos + t;
			m->enabled = true; /* un comando de posición vuelve a energizar */
			m->pos_cmds++;
			memcpy(&r[1], &data[1], 5);
			reply(s, m, r, 7);
		}
		break;
	default:
		break;
	}
	return 0;
}

static int sim_recv(void *ctx, uint16_t *id, uint8_t data[8], uint8_t *len, uint32_t timeout_ms)
{
	gim_sim_t *s = ctx;

	(void)timeout_ms;
	if (s->q_count == 0) {
		return 0;
	}
	*id = s->q_id[s->q_head];
	memcpy(data, s->q_data[s->q_head], 8);
	*len = s->q_len[s->q_head];
	s->q_head = (s->q_head + 1) % GIM_SIM_QUEUE;
	s->q_count--;
	return 1;
}

gim_transport_t gim_sim_transport(gim_sim_t *s)
{
	gim_transport_t t = {sim_send, sim_recv, s};

	return t;
}

void gim_sim_step(gim_sim_t *s, double dt)
{
	for (int i = 0; i < GIM_SIM_MOTORS; i++) {
		gim_sim_motor_t *m = &s->m[i];
		double err, dir, stop, speed;

		if (!m->enabled || !(dt > 0.0)) {
			continue;
		}
		err = m->target - m->pos;
		if (fabs(err) < 0.5 && fabs(m->vel) < m->dec * dt) {
			m->pos = m->target;
			m->vel = 0.0;
			continue;
		}
		dir = err > 0.0 ? 1.0 : -1.0;
		speed = m->vel * dir; /* rapidez hacia el objetivo (negativa si se aleja) */
		stop = speed > 0.0 ? speed * speed / (2.0 * m->dec) : 0.0;
		if (stop >= fabs(err)) {
			speed -= m->dec * dt;
		} else {
			speed = fmin(speed + m->acc * dt, m->vmax);
		}
		if (speed * dt > fabs(err)) {
			speed = fabs(err) / dt; /* no pasarse del objetivo */
		}
		m->vel = speed * dir;
		m->pos += m->vel * dt;
	}
}
