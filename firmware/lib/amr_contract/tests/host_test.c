/*
 * Pruebas en host de amr_contract (gcc, sin Zephyr):
 *   host_test <fixtures/commands.txt> <salida_telemetria.txt>
 * 1) Clasifica cada fixture de comando como válido/inválido igual que Python.
 * 2) Pruebas unitarias de formateo, watchdog y deduplicación.
 * 3) Escribe telemetría formateada para que el decodificador Python la valide.
 */
#include "amr_contract.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;
static const char *BOOT = "b00710c0ffee0001";

#define CHECK(cond)                                                                   \
	do {                                                                          \
		if (!(cond)) {                                                        \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
			failures++;                                                   \
		}                                                                     \
	} while (0)

static bool parse_any(const char *topic, const char *payload)
{
	size_t n = strlen(payload);

	if (strcmp(topic, "cmd/vel") == 0) {
		amr_cmd_vel_t c;
		return amr_parse_cmd_vel(payload, n, BOOT, &c);
	}
	if (strcmp(topic, "cmd/arm") == 0) {
		amr_cmd_arm_t c;
		return amr_parse_cmd_arm(payload, n, BOOT, &c);
	}
	if (strcmp(topic, "cmd/gripper") == 0) {
		amr_cmd_gripper_t c;
		return amr_parse_cmd_gripper(payload, n, BOOT, &c);
	}
	if (strcmp(topic, "cmd/estop") == 0) {
		amr_cmd_estop_t c;
		return amr_parse_cmd_estop(payload, n, BOOT, &c);
	}
	if (strcmp(topic, "cmd/odom_reset") == 0) {
		amr_cmd_odom_reset_t c;
		return amr_parse_cmd_odom_reset(payload, n, BOOT, &c);
	}
	printf("topic desconocido en fixtures: %s\n", topic);
	failures++;
	return false;
}

static void test_fixtures(const char *path)
{
	char line[1400];
	int count = 0;
	FILE *f = fopen(path, "r");

	if (f == NULL) {
		printf("FAIL: no se pudo abrir %s\n", path);
		failures++;
		return;
	}
	while (fgets(line, sizeof(line), f) != NULL) {
		char *topic, *verdict, *payload;
		size_t n = strlen(line);

		while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) {
			line[--n] = '\0';
		}
		if (n == 0 || line[0] == '#') {
			continue;
		}
		topic = line;
		verdict = strchr(topic, '\t');
		if (verdict == NULL) {
			continue;
		}
		*verdict++ = '\0';
		payload = strchr(verdict, '\t');
		if (payload == NULL) {
			continue;
		}
		*payload++ = '\0';
		bool expect = strcmp(verdict, "valid") == 0;
		bool got = parse_any(topic, payload);
		if (got != expect) {
			printf("FAIL fixture %s esperado %s: %s\n", topic, verdict, payload);
			failures++;
		}
		count++;
	}
	fclose(f);
	CHECK(count >= 40);
	printf("fixtures de comandos: %d\n", count);
}

static void test_parsed_values(void)
{
	amr_cmd_vel_t v;
	amr_cmd_arm_t a;
	const char *vel = "{\"v\":1,\"boot_id\":\"b00710c0ffee0001\",\"seq\":42,\"vx\":0.2,"
			  "\"vy\":-0.1,\"wz\":0.3,\"valid_ms\":250}";
	const char *arm = "{\"v\":1,\"boot_id\":\"b00710c0ffee0001\",\"seq\":7,\"op\":\"goal\","
			  "\"pos\":[0,4096,-2048,1024],\"acc\":[100,100,100,100],"
			  "\"dec\":[50,50,50,50.5]}";
	uint32_t seq = 0;

	CHECK(amr_parse_cmd_vel(vel, strlen(vel), BOOT, &v));
	CHECK(v.seq == 42 && v.valid_ms == 250);
	CHECK(fabsf(v.vx - 0.2f) < 1e-6f && fabsf(v.vy + 0.1f) < 1e-6f && fabsf(v.wz - 0.3f) < 1e-6f);
	CHECK(!amr_parse_cmd_vel(vel, strlen(vel), "otroboot", &v));
	CHECK(!amr_parse_cmd_vel(vel, strlen(vel) - 1, BOOT, &v)); /* truncado */

	CHECK(amr_parse_cmd_arm(arm, strlen(arm), BOOT, &a));
	CHECK(a.op == AMR_ARM_GOAL && a.pos[1] == 4096 && a.pos[2] == -2048);
	CHECK(fabsf(a.dec[3] - 50.5f) < 1e-6f);

	CHECK(amr_peek_seq(arm, strlen(arm), &seq) && seq == 7);
}

static void test_watchdog(void)
{
	amr_vel_wd_t wd;
	amr_cmd_vel_t c = {.seq = 1, .vx = 0.5f, .vy = 0.0f, .wz = 0.1f, .valid_ms = 250};
	float vx, vy, wz;

	amr_vel_wd_init(&wd);
	amr_vel_wd_step(&wd, 0, true, false, &vx, &vy, &wz);
	CHECK(vx == 0.0f);

	/* Cerca del desborde de 32 bits. */
	amr_vel_wd_accept(&wd, &c, 0xFFFFFF00u);
	amr_vel_wd_step(&wd, 0xFFFFFF00u + 249u, true, false, &vx, &vy, &wz);
	CHECK(vx == 0.5f && wz == 0.1f && !wd.tripped);
	amr_vel_wd_step(&wd, 0xFFFFFF00u + 250u, true, false, &vx, &vy, &wz);
	CHECK(vx == 0.0f && wd.tripped);

	amr_vel_wd_accept(&wd, &c, 1000);
	amr_vel_wd_step(&wd, 1010, false, false, &vx, &vy, &wz); /* sin enlace */
	CHECK(vx == 0.0f && !wd.tripped);

	amr_vel_wd_accept(&wd, &c, 2000);
	amr_vel_wd_step(&wd, 2010, true, true, &vx, &vy, &wz); /* e-stop */
	CHECK(vx == 0.0f);

	amr_vel_wd_accept(&wd, &c, 3000);
	amr_vel_wd_reject(&wd);
	amr_vel_wd_step(&wd, 3001, true, false, &vx, &vy, &wz);
	CHECK(vx == 0.0f);
}

static void test_seq(void)
{
	amr_seq_t s = {0};

	CHECK(amr_seq_accept(&s, 0));
	CHECK(!amr_seq_accept(&s, 0));
	CHECK(amr_seq_accept(&s, 1));
	CHECK(!amr_seq_accept(&s, 1));
	CHECK(amr_seq_accept(&s, 0));
}

static void test_format_guards(void)
{
	char buf[512];
	char tiny[16];
	amr_hdr_t h = {.boot_id = BOOT, .t_ms = 5};
	amr_hdr_t bad = {.boot_id = "con espacio", .t_ms = 5};
	amr_odom_t o = {0};
	amr_battery_t b = {.volt = 24.5f, .logic_volt = -1.0f};

	CHECK(amr_fmt_battery(buf, sizeof(buf), &h, &b) > 0);
	CHECK(strcmp(buf, "{\"v\":1,\"boot_id\":\"b00710c0ffee0001\",\"t_ms\":5,\"volt\":24.5}") == 0);
	CHECK(amr_fmt_battery(tiny, sizeof(tiny), &h, &b) == -1);
	CHECK(amr_fmt_battery(buf, sizeof(buf), &bad, &b) == -1);
	o.x = NAN;
	CHECK(amr_fmt_odom(buf, sizeof(buf), &h, &o) == -1);
	CHECK(amr_fmt_lwt(buf, sizeof(buf)) > 0 &&
	      strcmp(buf, "{\"v\":1,\"state\":\"offline\"}") == 0);
}

static void write_telemetry(const char *path)
{
	char buf[AMR_MAX_PAYLOAD + 1];
	amr_hdr_t h = {.boot_id = BOOT, .t_ms = 123456};
	FILE *f = fopen(path, "w");
	amr_odom_t o = {.x = 1.25f, .y = -0.5f, .yaw = 3.1f, .vx = 0.2f, .vy = -0.1f, .wz = 0.05f,
			.cov = {1e-3f, 1e-3f, 2e-3f, 4e-4f, 4e-4f, 1e-6f}};
	amr_imu_t i = {.q = {0.7071068f, 0.0f, 0.0f, 0.7071068f}, .gyro = {0.01f, -0.02f, 0.3f},
		       .acc = {0.1f, 0.0f, 9.81f}, .acc_status = 3};
	amr_wheels_t w = {.pos = {1.0f, -2.0f, 3.5f, 1e5f}, .vel = {0.1f, 0.2f, -0.3f, 0.0f},
			  .rc_err = {0, 4096}};
	amr_arm_t a = {.enabled = true, .pos = {0, 4096, -2048, 2147483647},
		       .vel_rpm = {0.0f, 12.5f, -3.0f, 0.0f}, .cur_a = {0.1f, 1.2f, 0.4f, 0.0f},
		       .temp_c = {30.0f, 41.5f, 35.0f, 29.0f}, .in_pos = {true, false, true, true},
		       .fault = {0, 0, 0x08, 0}, .grip_pos = 0.6f, .grip_cur_a = 0.85f, .grasped = true};
	amr_battery_t b = {.volt = 23.7f, .logic_volt = 5.1f};
	amr_status_t s = {.state = AMR_STATE_OK, .estop = false,
			  .faults = AMR_FAULT_IMU | AMR_FAULT_ESTOP_HW, .wd_tripped = true, .cpu = 0.37f};

	if (f == NULL) {
		printf("FAIL: no se pudo escribir %s\n", path);
		failures++;
		return;
	}
#define EMIT(topic, call)                                    \
	do {                                                 \
		int n = (call);                              \
		CHECK(n > 0);                                \
		if (n > 0) {                                 \
			fprintf(f, "%s\t%s\n", topic, buf);  \
		}                                            \
	} while (0)
	EMIT("llc/odom", amr_fmt_odom(buf, sizeof(buf), &h, &o));
	EMIT("llc/imu", amr_fmt_imu(buf, sizeof(buf), &h, &i));
	EMIT("llc/wheels", amr_fmt_wheels(buf, sizeof(buf), &h, &w));
	EMIT("llc/arm", amr_fmt_arm(buf, sizeof(buf), &h, &a));
	EMIT("llc/battery", amr_fmt_battery(buf, sizeof(buf), &h, &b));
	EMIT("llc/status", amr_fmt_status(buf, sizeof(buf), &h, &s));
	EMIT("llc/ack", amr_fmt_ack(buf, sizeof(buf), &h, 77, AMR_CMD_ARM, AMR_ACK_REJECTED,
				     "brazo \"deshabilitado\""));
	EMIT("llc/status", amr_fmt_lwt(buf, sizeof(buf)));
#undef EMIT
	fclose(f);
}

int main(int argc, char **argv)
{
	if (argc < 3) {
		printf("uso: %s <commands.txt> <salida_telemetria.txt>\n", argv[0]);
		return 2;
	}
	test_fixtures(argv[1]);
	test_parsed_values();
	test_watchdog();
	test_seq();
	test_format_guards();
	write_telemetry(argv[2]);
	if (failures == 0) {
		printf("amr_contract: PASSED (0 failures)\n");
		return 0;
	}
	printf("amr_contract: FAILED (%d failures)\n", failures);
	return 1;
}
