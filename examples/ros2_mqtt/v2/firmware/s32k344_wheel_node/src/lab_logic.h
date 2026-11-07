/*
 * Portable lab logic: no Zephyr or NXP headers, so it is unit-tested on the
 * host (tests/host_test.c) and linked unchanged into the S32K344 firmware.
 */
#ifndef LAB_LOGIC_H
#define LAB_LOGIC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LAB_MAX_DUTY 0.2f
#define LAB_MAX_VALID_MS 250U
#define LAB_TWO_PI 6.283185307179586

/* Last accepted wheel command and its local expiry (receive-time watchdog). */
typedef struct {
	float left;
	float right;
	uint32_t command_ms;
	uint32_t valid_ms;
	bool active;
} lab_cmd_t;

void lab_init(lab_cmd_t *cmd);

/* Accept a validated command; anything invalid deactivates outputs. */
bool lab_command(lab_cmd_t *cmd, float left, float right,
		 uint32_t valid_ms, uint32_t now_ms);

/* Duty that must be applied now. Zero after expiry or link loss. */
void lab_step(lab_cmd_t *cmd, uint32_t now_ms, bool link_ok,
	      float *left, float *right);

/* {"ticks":..,"uptime_ms":..,"boot_id":".."}; returns length or -1. */
int lab_format_encoder(char *buf, size_t size, int64_t ticks,
		       int64_t uptime_ms, const char *boot_id);

/*
 * Parse {"left":f,"right":f,"valid_ms":n,"boot_id":"s"} as sent by the ROS
 * bridge. Rejects malformed JSON fields, wrong boot_id and out-of-range data.
 */
bool lab_parse_pwm(const char *json, size_t len, const char *boot_id,
		   float *left, float *right, uint32_t *valid_ms);

/* ---- Student exercise (src/student.c, reference in student_solution.c) ---- */

/* Cumulative quadrature angle (rad) -> nearest integer decoded tick count. */
int64_t student_ticks_from_rotation(double rotation_rad, double ticks_per_rev);

/*
 * Signed duty in [-1, 1] -> pulse widths for a 2-input H-bridge
 * (DRV8833/DRV8871 style): forward drives IN1, reverse drives IN2,
 * zero (or invalid) leaves both inputs low.
 */
void student_hbridge_pulses(float duty, uint32_t period_ns,
			    uint32_t *in1_ns, uint32_t *in2_ns);

/* Boot-time self-check of the two student functions. */
bool lab_student_selftest(void);

#endif
