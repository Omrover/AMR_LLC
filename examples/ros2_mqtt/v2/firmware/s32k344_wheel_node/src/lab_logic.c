#include "lab_logic.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void lab_init(lab_cmd_t *cmd)
{
	*cmd = (lab_cmd_t){0};
}

bool lab_command(lab_cmd_t *cmd, float left, float right,
		 uint32_t valid_ms, uint32_t now_ms)
{
	if (!isfinite(left) || !isfinite(right) ||
	    fabsf(left) > LAB_MAX_DUTY || fabsf(right) > LAB_MAX_DUTY ||
	    valid_ms == 0U || valid_ms > LAB_MAX_VALID_MS) {
		cmd->active = false;
		return false;
	}
	cmd->left = left;
	cmd->right = right;
	cmd->command_ms = now_ms;
	cmd->valid_ms = valid_ms;
	cmd->active = true;
	return true;
}

void lab_step(lab_cmd_t *cmd, uint32_t now_ms, bool link_ok,
	      float *left, float *right)
{
	/* Unsigned subtraction keeps the timeout correct across 32-bit wrap. */
	if (!link_ok || (uint32_t)(now_ms - cmd->command_ms) >= cmd->valid_ms) {
		cmd->active = false;
	}
	*left = cmd->active ? cmd->left : 0.0f;
	*right = cmd->active ? cmd->right : 0.0f;
}

int lab_format_encoder(char *buf, size_t size, int64_t ticks,
		       int64_t uptime_ms, const char *boot_id)
{
	int n = snprintf(buf, size,
			 "{\"ticks\":%lld,\"uptime_ms\":%lld,\"boot_id\":\"%s\"}",
			 (long long)ticks, (long long)uptime_ms, boot_id);

	return (n < 0 || (size_t)n >= size) ? -1 : n;
}

/* Minimal parser for the fixed bridge contract, not a general JSON parser. */
static const char *json_value(const char *json, const char *key)
{
	char pattern[24];
	const char *p;

	if (snprintf(pattern, sizeof(pattern), "\"%s\"", key) >= (int)sizeof(pattern)) {
		return NULL;
	}
	p = strstr(json, pattern);
	if (p == NULL) {
		return NULL;
	}
	p += strlen(pattern);
	while (*p == ' ' || *p == '\t') {
		p++;
	}
	if (*p++ != ':') {
		return NULL;
	}
	while (*p == ' ' || *p == '\t') {
		p++;
	}
	return p;
}

static bool json_end(const char *p)
{
	while (*p == ' ' || *p == '\t') {
		p++;
	}
	return *p == ',' || *p == '}';
}

static bool json_float(const char *json, const char *key, float *out)
{
	const char *p = json_value(json, key);
	char *end;

	if (p == NULL) {
		return false;
	}
	*out = strtof(p, &end);
	return end != p && json_end(end) && isfinite(*out);
}

bool lab_parse_pwm(const char *json, size_t len, const char *boot_id,
		   float *left, float *right, uint32_t *valid_ms)
{
	char buf[256];
	const char *p;
	char *end;
	size_t id_len = strlen(boot_id);
	unsigned long ms;

	if (len == 0U || len >= sizeof(buf) || memchr(json, '\0', len) != NULL) {
		return false;
	}
	memcpy(buf, json, len);
	buf[len] = '\0';

	if (!json_float(buf, "left", left) || !json_float(buf, "right", right)) {
		return false;
	}
	p = json_value(buf, "valid_ms");
	if (p == NULL || *p < '0' || *p > '9') {
		return false;
	}
	ms = strtoul(p, &end, 10);
	if (!json_end(end) || ms == 0UL || ms > LAB_MAX_VALID_MS) {
		return false;
	}
	*valid_ms = (uint32_t)ms;

	p = json_value(buf, "boot_id");
	if (p == NULL || *p != '"' || strncmp(p + 1, boot_id, id_len) != 0 ||
	    p[1 + id_len] != '"' || !json_end(p + 2 + id_len)) {
		return false;
	}
	return fabsf(*left) <= LAB_MAX_DUTY && fabsf(*right) <= LAB_MAX_DUTY;
}

bool lab_student_selftest(void)
{
	uint32_t in1, in2;
	bool ok = true;

	ok &= student_ticks_from_rotation(LAB_TWO_PI, 600.0) == 600;
	ok &= student_ticks_from_rotation(-0.5 * LAB_TWO_PI, 600.0) == -300;
	student_hbridge_pulses(0.25f, 1000000U, &in1, &in2);
	ok &= in1 == 250000U && in2 == 0U;
	student_hbridge_pulses(-0.1f, 1000000U, &in1, &in2);
	ok &= in1 == 0U && in2 == 100000U;
	return ok;
}
