/* Pruebas en host del driver RoboClaw contra el simulador y tramas grabadas. */
#include "roboclaw.h"
#include "roboclaw_sim.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond)                                                               \
	do {                                                                      \
		if (!(cond)) {                                                    \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
			failures++;                                               \
		}                                                                 \
	} while (0)

/* Transporte que graba lo escrito y responde 0xFF. */
typedef struct {
	uint8_t written[64];
	size_t n;
} rec_t;

static int rec_write(void *ctx, const uint8_t *buf, size_t n)
{
	rec_t *r = ctx;
	memcpy(r->written + r->n, buf, n);
	r->n += n;
	return (int)n;
}

static int rec_read(void *ctx, uint8_t *buf, size_t n, uint32_t timeout_ms)
{
	(void)ctx;
	(void)timeout_ms;
	if (n > 0U) {
		buf[0] = 0xFF;
		return 1;
	}
	return 0;
}

static void test_crc(void)
{
	/* CRC16-CCITT/XMODEM de "123456789" = 0x31C3 */
	CHECK(rc_crc16(0, (const uint8_t *)"123456789", 9) == 0x31C3);
}

static void test_frame_bytes(void)
{
	rec_t r = {{0}, 0};
	rc_transport_t t = {rec_write, rec_read, NULL, &r};
	roboclaw_t rc;
	const uint8_t expect_head[14] = {128, 40, 0x00, 0x00, 0x03, 0xE8,  /* accel 1000 */
					 0xFF, 0xFF, 0xFE, 0x0C,           /* -500 */
					 0x00, 0x01, 0x11, 0x70};          /* 70000 */
	uint16_t crc;

	rc_init(&rc, &t, 10, 0);
	CHECK(rc_speed_accel_m1m2(&rc, 128, 1000, -500, 70000) == RC_OK);
	CHECK(r.n == 16);
	CHECK(memcmp(r.written, expect_head, sizeof(expect_head)) == 0);
	crc = rc_crc16(0, r.written, 14);
	CHECK(r.written[14] == (uint8_t)(crc >> 8) && r.written[15] == (uint8_t)crc);

	r.n = 0;
	CHECK(rc_duty_m1m2(&rc, 129, -32767, 0) == RC_OK);
	CHECK(r.n == 8 && r.written[0] == 129 && r.written[1] == 34 && r.written[2] == 0x80 &&
	      r.written[3] == 0x01);

	r.n = 0; /* Set Serial Timeout: [addr][14][décimas][crc] */
	CHECK(rc_set_serial_timeout(&rc, 128, 5) == RC_OK);
	crc = rc_crc16(0, r.written, 3);
	CHECK(r.n == 5 && r.written[1] == 14 && r.written[2] == 5 &&
	      r.written[3] == (uint8_t)(crc >> 8) && r.written[4] == (uint8_t)crc);
}

static void test_reads_against_sim(void)
{
	rc_sim_t sim;
	rc_transport_t t;
	roboclaw_t rc;
	int32_t e1, e2, s1, s2;
	float v, a1, a2;
	char ver[64];

	rc_sim_init(&sim, 128, 129);
	t = rc_sim_transport(&sim);
	rc_init(&rc, &t, 10, 0);

	CHECK(rc_speed_m1m2(&rc, 128, 1000, -2000) == RC_OK);
	for (int i = 0; i < 100; i++) {
		rc_sim_step(&sim, 0.01);
	}
	CHECK(rc_read_encoders(&rc, 128, &e1, &e2) == RC_OK);
	CHECK(e1 == 1000 && e2 == -2000);
	CHECK(rc_read_ispeeds(&rc, 128, &s1, &s2) == RC_OK && s1 == 1000 && s2 == -2000);
	CHECK(rc_read_encoders(&rc, 129, &e1, &e2) == RC_OK && e1 == 0 && e2 == 0);
	CHECK(rc_read_main_battery(&rc, 128, &v) == RC_OK && v > 24.55f && v < 24.65f);
	CHECK(rc_read_currents(&rc, 128, &a1, &a2) == RC_OK);
	CHECK(rc_read_version(&rc, 129, ver, sizeof(ver)) == RC_OK);
	CHECK(strstr(ver, "Roboclaw") != NULL && strchr(ver, '\n') == NULL);
	CHECK(rc_reset_encoders(&rc, 128) == RC_OK);
	CHECK(rc_read_encoders(&rc, 128, &e1, &e2) == RC_OK && e1 == 0);
	{
		uint8_t to = 0;
		CHECK(rc_read_serial_timeout(&rc, 129, &to) == RC_OK && to == 30);
		CHECK(rc_set_serial_timeout(&rc, 129, 5) == RC_OK);
		CHECK(rc_read_serial_timeout(&rc, 129, &to) == RC_OK && to == 5);
	}

	/* Contador cerca del desborde: se reporta con signo (complemento a dos). */
	sim.dev[0].enc[0] = 2147483647.0 + 10.0;
	CHECK(rc_read_encoders(&rc, 128, &e1, &e2) == RC_OK && e1 == -2147483639); /* 2^31 - 1 + 10 -> -2^31 + 9 */
}

static void test_faults_and_retries(void)
{
	rc_sim_t sim;
	rc_transport_t t;
	roboclaw_t rc;
	int32_t e1, e2;

	rc_sim_init(&sim, 128, 129);
	t = rc_sim_transport(&sim);

	rc_init(&rc, &t, 10, 0);
	sim.drop_next = 1;
	CHECK(rc_read_encoders(&rc, 128, &e1, &e2) == RC_ERR_TIMEOUT);
	sim.corrupt_next = 1;
	CHECK(rc_read_encoders(&rc, 128, &e1, &e2) == RC_ERR_CRC);
	CHECK(rc_read_encoders(&rc, 130, &e1, &e2) == RC_ERR_TIMEOUT); /* dirección ausente */
	CHECK(rc.stats.timeout == 2 && rc.stats.crc == 1);

	rc_init(&rc, &t, 10, 2);
	sim.drop_next = 1;
	sim.corrupt_next = 1;
	CHECK(rc_read_encoders(&rc, 128, &e1, &e2) == RC_OK); /* 3er intento ok */
	CHECK(rc.stats.timeout == 1 && rc.stats.crc == 1 && rc.stats.ok == 1);
	sim.drop_next = 1;
	CHECK(rc_speed_m1m2(&rc, 128, 10, 10) == RC_OK);
	CHECK(rc_read(&rc, 128, RC_CMD_READ_ENCODERS, NULL, 8) == RC_ERR_ARG);
}

int main(void)
{
	test_crc();
	test_frame_bytes();
	test_reads_against_sim();
	test_faults_and_retries();
	if (failures == 0) {
		printf("roboclaw: PASSED (0 failures)\n");
		return 0;
	}
	printf("roboclaw: FAILED (%d failures)\n", failures);
	return 1;
}
