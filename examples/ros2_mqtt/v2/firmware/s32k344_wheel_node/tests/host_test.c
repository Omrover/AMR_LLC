/* Host unit tests: watchdog, JSON contract and the student exercise. */
#include "../src/lab_logic.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(cond)                                                     \
	do {                                                            \
		if (!(cond)) {                                          \
			printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
			failures++;                                     \
		}                                                       \
	} while (0)

static bool parse(const char *json, float *l, float *r, uint32_t *ms)
{
	return lab_parse_pwm(json, strlen(json), "ab12", l, r, ms);
}

static void test_watchdog(void)
{
	lab_cmd_t c;
	float l, r;

	lab_init(&c);
	lab_step(&c, 0, true, &l, &r);
	CHECK(l == 0.0f && r == 0.0f);
	CHECK(lab_command(&c, 0.1f, -0.1f, 250, 100));
	lab_step(&c, 349, true, &l, &r);
	CHECK(l == 0.1f && r == -0.1f);
	lab_step(&c, 350, true, &l, &r);          /* expired */
	CHECK(l == 0.0f && r == 0.0f);
	CHECK(lab_command(&c, 0.1f, 0.1f, 250, 400));
	lab_step(&c, 401, false, &l, &r);         /* link lost */
	CHECK(l == 0.0f);
	lab_step(&c, 402, true, &l, &r);          /* does not resume */
	CHECK(l == 0.0f);
	CHECK(!lab_command(&c, NAN, 0, 250, 500));
	CHECK(!lab_command(&c, 0.21f, 0, 250, 500));
	CHECK(!lab_command(&c, 0.1f, 0, 251, 500));
	CHECK(lab_command(&c, 0.1f, 0, 250, UINT32_MAX - 100)); /* timer wrap */
	lab_step(&c, 100, true, &l, &r);
	CHECK(l == 0.1f);
	lab_step(&c, 150, true, &l, &r);
	CHECK(l == 0.0f);
}

static void test_json(void)
{
	char buf[128];
	float l, r;
	uint32_t ms;

	CHECK(lab_format_encoder(buf, sizeof(buf), -1234567890123LL, 5678, "ab12") > 0);
	CHECK(strcmp(buf, "{\"ticks\":-1234567890123,\"uptime_ms\":5678,\"boot_id\":\"ab12\"}") == 0);
	CHECK(lab_format_encoder(buf, 10, 1, 1, "ab12") == -1);

	/* Exactly what Python json.dumps in the ROS bridge produces. */
	CHECK(parse("{\"left\": 0.1, \"right\": -0.2, \"valid_ms\": 250, \"boot_id\": \"ab12\"}",
		    &l, &r, &ms));
	CHECK(fabsf(l - 0.1f) < 1e-6f && fabsf(r + 0.2f) < 1e-6f && ms == 250);
	CHECK(parse("{\"left\":0.0,\"right\":0.0,\"valid_ms\":1,\"boot_id\":\"ab12\"}", &l, &r, &ms));
	CHECK(!parse("{\"left\":0.1,\"right\":0.1,\"valid_ms\":250,\"boot_id\":\"ab123\"}", &l, &r, &ms));
	CHECK(!parse("{\"left\":0.1,\"right\":0.1,\"valid_ms\":250,\"boot_id\":\"zz\"}", &l, &r, &ms));
	CHECK(!parse("{\"left\":0.3,\"right\":0.1,\"valid_ms\":250,\"boot_id\":\"ab12\"}", &l, &r, &ms));
	CHECK(!parse("{\"left\":NaN,\"right\":0.1,\"valid_ms\":250,\"boot_id\":\"ab12\"}", &l, &r, &ms));
	CHECK(!parse("{\"left\":\"0.1\",\"right\":0.1,\"valid_ms\":250,\"boot_id\":\"ab12\"}", &l, &r, &ms));
	CHECK(!parse("{\"left\":0.1,\"right\":0.1,\"valid_ms\":0,\"boot_id\":\"ab12\"}", &l, &r, &ms));
	CHECK(!parse("{\"left\":0.1,\"right\":0.1,\"valid_ms\":2.5,\"boot_id\":\"ab12\"}", &l, &r, &ms));
	CHECK(!parse("{\"left\":0.1,\"valid_ms\":250,\"boot_id\":\"ab12\"}", &l, &r, &ms));
	CHECK(!parse("", &l, &r, &ms));
}

static void test_student(void)
{
	uint32_t in1 = 7, in2 = 7;

	CHECK(student_ticks_from_rotation(0.0, 600.0) == 0);
	CHECK(student_ticks_from_rotation(LAB_TWO_PI, 600.0) == 600);
	CHECK(student_ticks_from_rotation(-2.5 * LAB_TWO_PI, 600.0) == -1500);
	CHECK(student_ticks_from_rotation(LAB_TWO_PI * 1.4 / 685.0, 685.0) == 1);
	CHECK(student_ticks_from_rotation(1000.0 * LAB_TWO_PI, 2400.0) == 2400000);

	student_hbridge_pulses(0.5f, 1000000U, &in1, &in2);
	CHECK(in1 == 500000U && in2 == 0U);
	student_hbridge_pulses(-0.2f, 1000000U, &in1, &in2);
	CHECK(in1 == 0U && in2 == 200000U);
	student_hbridge_pulses(1.0f, 1000000U, &in1, &in2);
	CHECK(in1 == 1000000U && in2 == 0U);
	in1 = in2 = 7;
	student_hbridge_pulses(0.0f, 1000000U, &in1, &in2);
	CHECK(in1 == 0U && in2 == 0U);
	in1 = in2 = 7;
	student_hbridge_pulses(NAN, 1000000U, &in1, &in2);
	CHECK(in1 == 0U && in2 == 0U);
	in1 = in2 = 7;
	student_hbridge_pulses(1.5f, 1000000U, &in1, &in2);
	CHECK(in1 == 0U && in2 == 0U);
	CHECK(lab_student_selftest());
}

int main(void)
{
	test_watchdog();
	test_json();
	test_student();
	printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
	       failures == 1 ? "" : "s");
	return failures != 0;
}
