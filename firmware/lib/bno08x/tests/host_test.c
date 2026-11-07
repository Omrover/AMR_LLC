/* Pruebas en host del driver BNO08x con un dispositivo I2C falso. */
#include "bno08x.h"

#include <math.h>
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
#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))

/* Dispositivo falso: cola de paquetes SHTP y registro de escrituras. */
typedef struct {
	uint8_t pkts[8][128];
	size_t lens[8];
	int head, tail;
	uint8_t written[256];
	size_t wn;
	bool fail;
} fake_t;

static int fake_write(void *ctx, const uint8_t *buf, size_t n)
{
	fake_t *f = ctx;
	if (f->fail) {
		return -5;
	}
	memcpy(f->written + f->wn, buf, n);
	f->wn += n;
	return 0;
}

static int fake_read(void *ctx, uint8_t *buf, size_t n)
{
	fake_t *f = ctx;
	if (f->fail) {
		return -5;
	}
	memset(buf, 0, n);
	if (f->head == f->tail) {
		return 0; /* encabezado de longitud 0 */
	}
	memcpy(buf, f->pkts[f->head], n < f->lens[f->head] ? n : f->lens[f->head]);
	if (n > 4U) {
		f->head++; /* lectura completa consume el paquete */
	}
	return 0;
}

static void push(fake_t *f, uint8_t channel, const uint8_t *payload, size_t n)
{
	uint8_t *p = f->pkts[f->tail];
	size_t len = n + 4U;
	p[0] = (uint8_t)len;
	p[1] = (uint8_t)(len >> 8);
	p[2] = channel;
	p[3] = 0;
	memcpy(&p[4], payload, n);
	f->lens[f->tail++] = len;
}

static void put16(uint8_t *p, int v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void test_enable_report_bytes(void)
{
	fake_t f = {0};
	bno_transport_t t = {fake_write, fake_read, &f};
	bno08x_t b;
	const uint8_t expect[21] = {21, 0, 2, 0, 0xFD, 0x08, 0, 0, 0, 0x10, 0x27, 0, 0,
				    0, 0, 0, 0, 0, 0, 0, 0};

	bno_init(&b, &t);
	CHECK(bno_enable_report(&b, BNO_RPT_GAME_ROTATION_VECTOR, 10000) == 0);
	CHECK(f.wn == 21 && memcmp(f.written, expect, 21) == 0);
	CHECK(bno_enable_report(&b, BNO_RPT_GYRO, 10000) == 0);
	CHECK(f.written[21 + 3] == 1); /* secuencia del canal 2 incrementa */
	CHECK(bno_soft_reset(&b) == 0);
	CHECK(f.written[42] == 5 && f.written[44] == BNO_CH_EXECUTABLE && f.written[45] == 0 &&
	      f.written[46] == 1);
}

static void test_parse_reports(void)
{
	fake_t f = {0};
	bno_transport_t t = {fake_write, fake_read, &f};
	bno08x_t b;
	uint8_t pl[5 + 12 + 10 + 10];
	const float s = 0.70710678f;
	int ev;

	bno_init(&b, &t);
	memset(pl, 0, sizeof(pl));
	pl[0] = BNO_RPT_BASE_TIMESTAMP;
	/* Game RV: yaw 90 grados -> (w, z) = (0.7071, 0.7071) en Q14 */
	pl[5] = BNO_RPT_GAME_ROTATION_VECTOR;
	pl[7] = 0x03; /* precisión 3 */
	put16(&pl[5 + 8], (int)lroundf(s * 16384.0f));  /* k */
	put16(&pl[5 + 10], (int)lroundf(s * 16384.0f)); /* real */
	/* Gyro calibrado: z = 1.0 rad/s en Q9 */
	pl[17] = BNO_RPT_GYRO;
	put16(&pl[17 + 8], 512);
	/* Accel: z = 9.81 en Q8 */
	pl[27] = BNO_RPT_ACCEL;
	put16(&pl[27 + 8], (int)lroundf(9.81f * 256.0f));
	push(&f, BNO_CH_REPORTS, pl, sizeof(pl));

	ev = bno_service(&b);
	CHECK(ev == (int)(BNO_UPD_QUAT | BNO_UPD_GYRO | BNO_UPD_ACCEL));
	CHECK(NEAR(b.data.quat[0], s, 1e-4) && NEAR(b.data.quat[3], s, 1e-4));
	CHECK(NEAR(b.data.quat[1], 0, 1e-6) && b.data.quat_accuracy == 3);
	CHECK(NEAR(b.data.gyro[2], 1.0, 1e-6));
	CHECK(NEAR(b.data.accel[2], 9.81, 0.005));
	CHECK(bno_service(&b) == 0); /* nada pendiente */
}

static void test_reset_and_errors(void)
{
	fake_t f = {0};
	bno_transport_t t = {fake_write, fake_read, &f};
	bno08x_t b;
	const uint8_t reset_complete = 1;
	uint8_t unknown[6] = {0x77, 0, 0, 0, 0, 0};
	uint8_t truncated[8] = {BNO_RPT_GYRO, 0, 0, 0, 0, 0, 0, 0};
	uint8_t advert[20] = {0};

	bno_init(&b, &t);
	b.seq[2] = 9;
	push(&f, BNO_CH_EXECUTABLE, &reset_complete, 1);
	CHECK(bno_service(&b) == (int)BNO_EVT_RESET);
	CHECK(b.resets == 1 && b.seq[2] == 0);

	push(&f, BNO_CH_COMMAND, advert, sizeof(advert));
	CHECK(bno_service(&b) == (int)BNO_EVT_OTHER);

	push(&f, BNO_CH_REPORTS, unknown, sizeof(unknown));
	CHECK(bno_service(&b) == (int)BNO_EVT_OTHER && b.errors == 1);
	push(&f, BNO_CH_REPORTS, truncated, sizeof(truncated));
	CHECK(bno_service(&b) == (int)BNO_EVT_OTHER && b.errors == 2);

	f.fail = true;
	CHECK(bno_service(&b) < 0);
	CHECK(bno_enable_report(&b, BNO_RPT_GYRO, 10000) < 0);
}

int main(void)
{
	test_enable_report_bytes();
	test_parse_reports();
	test_reset_and_errors();
	if (failures == 0) {
		printf("bno08x: PASSED (0 failures)\n");
		return 0;
	}
	printf("bno08x: FAILED (%d failures)\n", failures);
	return 1;
}
