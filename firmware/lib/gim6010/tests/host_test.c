/* Pruebas en host del protocolo GIM6010-8 y del gateway del brazo contra el simulador. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gim6010.h"
#include "gim6010_sim.h"
#include "gim_arm.h"

static int failures;
#define CHECK(c)                                                                   \
	do {                                                                       \
		if (!(c)) {                                                        \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);        \
			failures++;                                                \
		}                                                                  \
	} while (0)

/* Captura de la última trama enviada, para revisar bytes exactos. */
static uint16_t cap_id;
static uint8_t cap_data[8], cap_len;
static int cap_send(void *ctx, uint16_t id, const uint8_t *d, uint8_t len)
{
	(void)ctx;
	cap_id = id;
	memcpy(cap_data, d, len);
	cap_len = len;
	return 0;
}
static int none_recv(void *ctx, uint16_t *id, uint8_t d[8], uint8_t *len, uint32_t t)
{
	(void)ctx; (void)id; (void)d; (void)len; (void)t;
	return 0;
}

static void test_frames(void)
{
	gim_transport_t t = {cap_send, none_recv, NULL};
	gim_bus_t b;
	uint8_t d[8] = {0xA4, 31, 0x2C, 0x01, 0x9C, 0xFF, 0x00, 0x40};
	gim_a4_t a4;

	gim_init(&b, &t, 1);
	/* 0xDA absoluta a -4096 counts hacia el motor 2: ID 0x102, LE. */
	CHECK(gim_pos_abs(&b, 2, -4096) == GIM_ERR_TIMEOUT);
	CHECK(cap_id == 0x102 && cap_len == 6);
	CHECK(cap_data[0] == 0xDA && cap_data[1] == 0 && cap_data[2] == 0x00 &&
	      cap_data[3] == 0xF0 && cap_data[4] == 0xFF && cap_data[5] == 0xFF);
	CHECK(b.timeouts == 1);
	/* Aceleración 100 rpm/s = 10000 (0.01 rpm/s). */
	CHECK(gim_trap_acc_dec(&b, 1, 10000, 10000) == GIM_ERR_TIMEOUT);
	CHECK(cap_id == 0x101 && cap_len == 5 && cap_data[0] == 0xD0 && cap_data[1] == 0x10 &&
	      cap_data[2] == 0x27);
	CHECK(gim_read_a4(&b, 0, &a4) == GIM_ERR_ARG);
	/* Decodificación de 0xA4 (mismo formato que el ejemplo de RTD). */
	gim_decode_a4(d, &a4);
	CHECK(a4.temp_c == 31 && a4.cur_ma == 300 && a4.vel_crpm == -100 && a4.pos16 == 0x4000);
}

static void test_unwrap(void)
{
	int32_t p = gim_unwrap(0, false, 0xFFF0); /* primera lectura con signo */

	CHECK(p == -16);
	p = gim_unwrap(p, true, 0x0010);           /* cruza el cero hacia arriba */
	CHECK(p == 16);
	p = gim_unwrap(30000, true, 40000);        /* pasa de 32767 sin saltar */
	CHECK(p == 40000);
	p = gim_unwrap(p, true, (uint16_t)70000);  /* 70000 envuelto */
	CHECK(p == 70000);
	p = gim_unwrap(-70000, true, (uint16_t)(-70100));
	CHECK(p == -70100);
}

typedef struct {
	gim_sim_t sim;
	gim_bus_t bus;
	gim_arm_t arm;
	uint32_t now;
} rig_t;

static void rig_init(rig_t *r)
{
	gim_arm_cfg_t cfg;
	gim_transport_t t;

	memset(r, 0, sizeof(*r));
	gim_sim_init(&r->sim);
	t = gim_sim_transport(&r->sim);
	gim_init(&r->bus, &t, 2);
	gim_arm_default_cfg(&cfg);
	gim_arm_init(&r->arm, &r->bus, &cfg);
}

static void run(rig_t *r, double seconds)
{
	for (int k = 0; k < (int)(seconds / 0.02 + 0.5); k++) {
		gim_sim_step(&r->sim, 0.02);
		r->now += 20;
		gim_arm_step(&r->arm, r->now);
	}
}

static void test_arm_goal_and_limits(void)
{
	rig_t r;
	const char *msg = NULL;
	int32_t goal[4] = {4096, -2048, 1000, 0};
	float acc[4] = {200, 200, 200, 200};

	rig_init(&r);
	r.sim.m[1].pos = -300.0; /* juntas fuera de cero al encender */
	r.sim.m[1].target = -300.0;
	CHECK(!gim_arm_command(&r.arm, GIM_ARM_OP_GOAL, goal, acc, acc, &msg));  /* deshabilitado */
	CHECK(!gim_arm_command(&r.arm, GIM_ARM_OP_ENABLE, NULL, NULL, NULL, &msg)); /* sin posición */
	run(&r, 0.04);
	CHECK(r.arm.j[1].pos == -300);
	CHECK(gim_arm_command(&r.arm, GIM_ARM_OP_ENABLE, NULL, NULL, NULL, &msg));
	run(&r, 0.1);
	CHECK(r.arm.j[1].pos == -300 && gim_arm_in_pos(&r.arm, 1)); /* habilitar no mueve */

	CHECK(gim_arm_command(&r.arm, GIM_ARM_OP_GOAL, goal, acc, acc, &msg));
	run(&r, 0.1);
	CHECK(!gim_arm_in_pos(&r.arm, 0));
	run(&r, 3.0);
	for (int i = 0; i < 4; i++) {
		CHECK(gim_arm_in_pos(&r.arm, i));
		CHECK(abs(r.arm.j[i].pos - goal[i]) <= 1);
	}
	CHECK(fabs(r.sim.m[0].acc - gim_counts_per_s(200.0)) < 1e-6);

	{
		int32_t bad[4] = {0, 0, 20000, 0};

		CHECK(!gim_arm_command(&r.arm, GIM_ARM_OP_GOAL, bad, acc, acc, &msg));
		CHECK(strstr(msg, "limites") != NULL);
	}
	CHECK(gim_arm_command(&r.arm, GIM_ARM_OP_HOME, NULL, NULL, NULL, &msg));
	run(&r, 3.0);
	for (int i = 0; i < 4; i++) {
		CHECK(r.arm.j[i].pos == 0);
	}
	CHECK(!gim_arm_bus_fault(&r.arm) && !gim_arm_motor_fault(&r.arm));
}

static void test_hold_offline_and_faults(void)
{
	rig_t r;
	const char *msg = NULL;
	int32_t goal[4] = {8000, 8000, 8000, 8000};
	float acc[4] = {100, 100, 100, 100};

	rig_init(&r);
	run(&r, 0.04);
	CHECK(gim_arm_command(&r.arm, GIM_ARM_OP_ENABLE, NULL, NULL, NULL, &msg));
	CHECK(gim_arm_command(&r.arm, GIM_ARM_OP_GOAL, goal, acc, acc, &msg));
	run(&r, 0.6);
	/* E-stop a mitad del movimiento: se queda cerca de donde estaba, sin caer. */
	gim_arm_set_hold(&r.arm, true);
	{
		int32_t at = r.arm.j[0].pos;

		run(&r, 2.0);
		CHECK(r.sim.m[0].enabled);
		CHECK(abs(r.arm.j[0].pos - at) < 1500 && r.arm.j[0].pos < 7000);
	}
	CHECK(!gim_arm_command(&r.arm, GIM_ARM_OP_GOAL, goal, acc, acc, &msg));
	gim_arm_set_hold(&r.arm, false);

	/* Falla reportada por 0xAE y limpieza con CLEAR. */
	r.sim.m[2].fault = 0x08;
	run(&r, 0.6);
	CHECK(gim_arm_motor_fault(&r.arm) && r.arm.j[2].fault == 0x08);
	CHECK(gim_arm_command(&r.arm, GIM_ARM_OP_CLEAR, NULL, NULL, NULL, &msg));
	run(&r, 0.6);
	CHECK(!gim_arm_motor_fault(&r.arm));

	/* Junta sin respuesta: falla de bus tras offline_after ciclos. */
	r.sim.m[3].online = false;
	run(&r, 0.1);
	CHECK(gim_arm_bus_fault(&r.arm) && !r.arm.j[3].online);
	r.sim.m[3].online = true;
	run(&r, 0.1);
	CHECK(!gim_arm_bus_fault(&r.arm));

	/* DISABLE: salida libre y sin objetivo. */
	CHECK(gim_arm_command(&r.arm, GIM_ARM_OP_DISABLE, NULL, NULL, NULL, &msg));
	run(&r, 0.1);
	for (int i = 0; i < 4; i++) {
		CHECK(!r.sim.m[i].enabled);
	}
	CHECK(!r.arm.enabled);
}

int main(void)
{
	test_frames();
	test_unwrap();
	test_arm_goal_and_limits();
	test_hold_offline_and_faults();
	printf("gim6010: %s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
