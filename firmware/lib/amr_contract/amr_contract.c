/*
 * Contrato MQTT v1, lado MCU: formateo de telemetría y validación estricta de
 * comandos. Ver amr_contract.h y AMR_LLC/protocol/CONTRACT.md.
 */
#include "amr_contract.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================ escritura */

typedef struct {
	char *buf;
	size_t size;
	size_t len;
	bool ok;
} writer_t;

static void w_init(writer_t *w, char *buf, size_t size)
{
	w->buf = buf;
	w->size = size;
	w->len = 0U;
	w->ok = (buf != NULL && size > 0U);
	if (w->ok) {
		buf[0] = '\0';
	}
}

static void w_printf(writer_t *w, const char *fmt, ...)
{
	va_list ap;
	int n;

	if (!w->ok) {
		return;
	}
	va_start(ap, fmt);
	n = vsnprintf(w->buf + w->len, w->size - w->len, fmt, ap);
	va_end(ap);
	if (n < 0 || (size_t)n >= w->size - w->len) {
		w->ok = false;
		return;
	}
	w->len += (size_t)n;
}

static void w_float(writer_t *w, float v)
{
	if (!isfinite(v)) {
		w->ok = false; /* nunca se publica NaN/Inf */
		return;
	}
	w_printf(w, "%.7g", (double)v);
}

static void w_key_float(writer_t *w, const char *key, float v)
{
	w_printf(w, ",\"%s\":", key);
	w_float(w, v);
}

static void w_key_floats(writer_t *w, const char *key, const float *v, int n)
{
	w_printf(w, ",\"%s\":[", key);
	for (int i = 0; i < n; i++) {
		if (i > 0) {
			w_printf(w, ",");
		}
		w_float(w, v[i]);
	}
	w_printf(w, "]");
}

static void w_key_bools(writer_t *w, const char *key, const bool *v, int n)
{
	w_printf(w, ",\"%s\":[", key);
	for (int i = 0; i < n; i++) {
		w_printf(w, "%s%s", i > 0 ? "," : "", v[i] ? "true" : "false");
	}
	w_printf(w, "]");
}

static bool boot_id_valid(const char *id)
{
	size_t n;

	if (id == NULL) {
		return false;
	}
	n = strlen(id);
	if (n == 0U || n > AMR_BOOT_ID_MAX) {
		return false;
	}
	for (size_t i = 0; i < n; i++) {
		char c = id[i];
		bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
			  (c >= 'A' && c <= 'Z') || c == '_' || c == '-';
		if (!ok) {
			return false;
		}
	}
	return true;
}

static void w_header(writer_t *w, const amr_hdr_t *h)
{
	if (h == NULL || !boot_id_valid(h->boot_id) || h->t_ms < 0) {
		w->ok = false;
		return;
	}
	w_printf(w, "{\"v\":%d,\"boot_id\":\"%s\",\"t_ms\":%lld", AMR_CONTRACT_VERSION,
		 h->boot_id, (long long)h->t_ms);
}

static int w_finish(writer_t *w)
{
	w_printf(w, "}");
	if (!w->ok || w->len > AMR_MAX_PAYLOAD) {
		if (w->buf != NULL && w->size > 0U) {
			w->buf[0] = '\0';
		}
		return -1;
	}
	return (int)w->len;
}

int amr_fmt_odom(char *buf, size_t size, const amr_hdr_t *h, const amr_odom_t *m)
{
	writer_t w;

	w_init(&w, buf, size);
	w_header(&w, h);
	w_key_float(&w, "x", m->x);
	w_key_float(&w, "y", m->y);
	w_key_float(&w, "yaw", m->yaw);
	w_key_float(&w, "vx", m->vx);
	w_key_float(&w, "vy", m->vy);
	w_key_float(&w, "wz", m->wz);
	w_key_floats(&w, "cov", m->cov, 6);
	return w_finish(&w);
}

int amr_fmt_imu(char *buf, size_t size, const amr_hdr_t *h, const amr_imu_t *m)
{
	writer_t w;

	w_init(&w, buf, size);
	if (m->acc_status < 0 || m->acc_status > 3) {
		w.ok = false;
	}
	w_header(&w, h);
	w_key_floats(&w, "q", m->q, 4);
	w_key_floats(&w, "gyro", m->gyro, 3);
	w_key_floats(&w, "acc", m->acc, 3);
	w_printf(&w, ",\"acc_status\":%d", m->acc_status);
	return w_finish(&w);
}

int amr_fmt_wheels(char *buf, size_t size, const amr_hdr_t *h, const amr_wheels_t *m)
{
	writer_t w;

	w_init(&w, buf, size);
	w_header(&w, h);
	w_key_floats(&w, "pos", m->pos, AMR_N_WHEELS);
	w_key_floats(&w, "vel", m->vel, AMR_N_WHEELS);
	w_printf(&w, ",\"rc_err\":[%lu,%lu]", (unsigned long)m->rc_err[0],
		 (unsigned long)m->rc_err[1]);
	return w_finish(&w);
}

int amr_fmt_arm(char *buf, size_t size, const amr_hdr_t *h, const amr_arm_t *m)
{
	writer_t w;

	w_init(&w, buf, size);
	if (!(m->grip_pos >= 0.0f && m->grip_pos <= 1.0f)) {
		w.ok = false;
	}
	w_header(&w, h);
	w_printf(&w, ",\"enabled\":%s,\"pos\":[", m->enabled ? "true" : "false");
	for (int i = 0; i < AMR_N_JOINTS; i++) {
		w_printf(&w, "%s%ld", i > 0 ? "," : "", (long)m->pos[i]);
	}
	w_printf(&w, "]");
	w_key_floats(&w, "vel", m->vel_rpm, AMR_N_JOINTS);
	w_key_floats(&w, "cur", m->cur_a, AMR_N_JOINTS);
	w_key_floats(&w, "temp", m->temp_c, AMR_N_JOINTS);
	w_key_bools(&w, "in_pos", m->in_pos, AMR_N_JOINTS);
	w_printf(&w, ",\"fault\":[");
	for (int i = 0; i < AMR_N_JOINTS; i++) {
		w_printf(&w, "%s%u", i > 0 ? "," : "", (unsigned int)m->fault[i]);
	}
	w_printf(&w, "],\"grip\":{\"pos\":");
	w_float(&w, m->grip_pos);
	w_printf(&w, ",\"cur\":");
	w_float(&w, m->grip_cur_a);
	w_printf(&w, ",\"grasped\":%s}", m->grasped ? "true" : "false");
	return w_finish(&w);
}

int amr_fmt_battery(char *buf, size_t size, const amr_hdr_t *h, const amr_battery_t *m)
{
	writer_t w;

	w_init(&w, buf, size);
	w_header(&w, h);
	w_key_float(&w, "volt", m->volt);
	if (m->logic_volt >= 0.0f) {
		w_key_float(&w, "logic_volt", m->logic_volt);
	}
	return w_finish(&w);
}

static const char *const STATE_NAMES[] = {"init", "ok", "estop", "fault"};
static const char *const CMD_NAMES[] = {"arm", "gripper", "estop", "odom_reset"};
static const char *const RESULT_NAMES[] = {"ok", "rejected", "busy", "duplicate"};
static const char *const ARM_OPS[] = {"goal", "enable", "disable", "clear", "home"};

int amr_fmt_status(char *buf, size_t size, const amr_hdr_t *h, const amr_status_t *m)
{
	writer_t w;

	w_init(&w, buf, size);
	if ((unsigned int)m->state > (unsigned int)AMR_STATE_FAULT ||
	    !(m->cpu >= 0.0f && m->cpu <= 1.0f)) {
		w.ok = false;
	}
	w_header(&w, h);
	if (w.ok) {
		w_printf(&w, ",\"state\":\"%s\",\"estop\":%s,\"faults\":%lu,\"wd\":%s",
			 STATE_NAMES[m->state], m->estop ? "true" : "false",
			 (unsigned long)m->faults, m->wd_tripped ? "true" : "false");
	}
	w_key_float(&w, "cpu", m->cpu);
	return w_finish(&w);
}

int amr_fmt_ack(char *buf, size_t size, const amr_hdr_t *h, uint32_t seq,
		amr_cmd_kind_t cmd, amr_ack_result_t result, const char *msg)
{
	writer_t w;

	w_init(&w, buf, size);
	if ((unsigned int)cmd > (unsigned int)AMR_CMD_ODOM_RESET ||
	    (unsigned int)result > (unsigned int)AMR_ACK_DUPLICATE) {
		w.ok = false;
	}
	w_header(&w, h);
	if (w.ok) {
		w_printf(&w, ",\"seq\":%lu,\"cmd\":\"%s\",\"result\":\"%s\"", (unsigned long)seq,
			 CMD_NAMES[cmd], RESULT_NAMES[result]);
	}
	if (msg != NULL && msg[0] != '\0') {
		/* Solo ASCII imprimible sin comillas ni barras: no requiere escapes. */
		w_printf(&w, ",\"msg\":\"");
		for (size_t i = 0; msg[i] != '\0' && i < 64U; i++) {
			char c = msg[i];
			w_printf(&w, "%c", (c >= 0x20 && c < 0x7f && c != '"' && c != '\\') ? c : '_');
		}
		w_printf(&w, "\"");
	}
	return w_finish(&w);
}

int amr_fmt_lwt(char *buf, size_t size)
{
	writer_t w;

	w_init(&w, buf, size);
	w_printf(&w, "{\"v\":%d,\"state\":\"offline\"", AMR_CONTRACT_VERSION);
	return w_finish(&w);
}

/* ================================================================== lectura */

#define MAX_MEMBERS 16
#define MAX_DEPTH 4

typedef struct {
	const char *key;
	size_t key_len;
	const char *val;
	size_t val_len;
} member_t;

typedef struct {
	member_t m[MAX_MEMBERS];
	int n;
} members_t;

static const char *skip_ws(const char *p, const char *end)
{
	while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) {
		p++;
	}
	return p;
}

/* Cadenas sin escapes ni caracteres de control (el contrato no los necesita). */
static const char *scan_string(const char *p, const char *end)
{
	if (p >= end || *p != '"') {
		return NULL;
	}
	for (p++; p < end; p++) {
		unsigned char c = (unsigned char)*p;
		if (c == '"') {
			return p + 1;
		}
		if (c == '\\' || c < 0x20) {
			return NULL;
		}
	}
	return NULL;
}

static const char *scan_value(const char *p, const char *end, int depth);

static const char *scan_container(const char *p, const char *end, int depth, char open,
				  char close)
{
	if (depth >= MAX_DEPTH || p >= end || *p != open) {
		return NULL;
	}
	p = skip_ws(p + 1, end);
	if (p < end && *p == close) {
		return p + 1;
	}
	while (p < end) {
		if (open == '{') {
			p = scan_string(p, end);
			if (p == NULL) {
				return NULL;
			}
			p = skip_ws(p, end);
			if (p >= end || *p != ':') {
				return NULL;
			}
			p = skip_ws(p + 1, end);
		}
		p = scan_value(p, end, depth + 1);
		if (p == NULL) {
			return NULL;
		}
		p = skip_ws(p, end);
		if (p < end && *p == ',') {
			p = skip_ws(p + 1, end);
			continue;
		}
		if (p < end && *p == close) {
			return p + 1;
		}
		return NULL;
	}
	return NULL;
}

static const char *scan_value(const char *p, const char *end, int depth)
{
	if (p >= end) {
		return NULL;
	}
	switch (*p) {
	case '"':
		return scan_string(p, end);
	case '[':
		return scan_container(p, end, depth, '[', ']');
	case '{':
		return scan_container(p, end, depth, '{', '}');
	case 't':
		return (end - p >= 4 && memcmp(p, "true", 4) == 0) ? p + 4 : NULL;
	case 'f':
		return (end - p >= 5 && memcmp(p, "false", 5) == 0) ? p + 5 : NULL;
	case 'n':
		return (end - p >= 4 && memcmp(p, "null", 4) == 0) ? p + 4 : NULL;
	default:
		if (*p != '-' && !(*p >= '0' && *p <= '9')) {
			return NULL;
		}
		while (p < end && ((*p >= '0' && *p <= '9') || *p == '-' || *p == '+' ||
				   *p == '.' || *p == 'e' || *p == 'E')) {
			p++;
		}
		return p;
	}
}

/* Objeto JSON plano de nivel superior: miembros únicos, nada después del '}'. */
static bool parse_members(const char *json, size_t len, members_t *ms)
{
	const char *end = json + len;
	const char *p;

	ms->n = 0;
	if (json == NULL || len == 0U || len > AMR_MAX_PAYLOAD || memchr(json, '\0', len) != NULL) {
		return false;
	}
	p = skip_ws(json, end);
	if (p >= end || *p != '{') {
		return false;
	}
	p = skip_ws(p + 1, end);
	if (p < end && *p == '}') {
		return skip_ws(p + 1, end) == end;
	}
	while (p < end) {
		member_t *m;
		const char *k = p;
		const char *v;

		p = scan_string(p, end);
		if (p == NULL || ms->n >= MAX_MEMBERS) {
			return false;
		}
		m = &ms->m[ms->n];
		m->key = k + 1;
		m->key_len = (size_t)(p - k) - 2U;
		for (int i = 0; i < ms->n; i++) {
			if (ms->m[i].key_len == m->key_len &&
			    memcmp(ms->m[i].key, m->key, m->key_len) == 0) {
				return false; /* clave duplicada */
			}
		}
		p = skip_ws(p, end);
		if (p >= end || *p != ':') {
			return false;
		}
		v = skip_ws(p + 1, end);
		p = scan_value(v, end, 1);
		if (p == NULL) {
			return false;
		}
		m->val = v;
		m->val_len = (size_t)(p - v);
		ms->n++;
		p = skip_ws(p, end);
		if (p < end && *p == ',') {
			p = skip_ws(p + 1, end);
			continue;
		}
		if (p < end && *p == '}') {
			return skip_ws(p + 1, end) == end;
		}
		return false;
	}
	return false;
}

static const member_t *find(const members_t *ms, const char *key)
{
	size_t n = strlen(key);

	for (int i = 0; i < ms->n; i++) {
		if (ms->m[i].key_len == n && memcmp(ms->m[i].key, key, n) == 0) {
			return &ms->m[i];
		}
	}
	return NULL;
}

static bool only_keys(const members_t *ms, const char *const *keys, int n_keys)
{
	for (int i = 0; i < ms->n; i++) {
		bool known = false;
		for (int k = 0; k < n_keys && !known; k++) {
			known = strlen(keys[k]) == ms->m[i].key_len &&
				memcmp(keys[k], ms->m[i].key, ms->m[i].key_len) == 0;
		}
		if (!known) {
			return false;
		}
	}
	return true;
}

static bool chars_only(const char *s, size_t n, const char *allowed)
{
	for (size_t i = 0; i < n; i++) {
		if (strchr(allowed, s[i]) == NULL) {
			return false;
		}
	}
	return n > 0U;
}

static bool num_from(const char *s, size_t n, double *out)
{
	char tmp[40];
	char *endp;

	if (n == 0U || n >= sizeof(tmp) || !chars_only(s, n, "0123456789+-.eE")) {
		return false;
	}
	memcpy(tmp, s, n);
	tmp[n] = '\0';
	*out = strtod(tmp, &endp);
	return endp == tmp + n && isfinite(*out);
}

static bool int_from(const char *s, size_t n, long long *out)
{
	char tmp[24];
	char *endp;

	if (n == 0U || n >= sizeof(tmp) || !chars_only(s, n, "0123456789-")) {
		return false;
	}
	memcpy(tmp, s, n);
	tmp[n] = '\0';
	errno = 0;
	*out = strtoll(tmp, &endp, 10);
	return endp == tmp + n && errno == 0;
}

static bool get_num(const members_t *ms, const char *key, double lo, double hi, float *out)
{
	const member_t *m = find(ms, key);
	double v;

	if (m == NULL || !num_from(m->val, m->val_len, &v) || v < lo || v > hi) {
		return false;
	}
	*out = (float)v;
	return true;
}

static bool get_int(const members_t *ms, const char *key, long long lo, long long hi,
		    long long *out)
{
	const member_t *m = find(ms, key);

	return m != NULL && int_from(m->val, m->val_len, out) && *out >= lo && *out <= hi;
}

static bool get_bool(const members_t *ms, const char *key, bool *out)
{
	const member_t *m = find(ms, key);

	if (m == NULL) {
		return false;
	}
	if (m->val_len == 4U && memcmp(m->val, "true", 4) == 0) {
		*out = true;
		return true;
	}
	if (m->val_len == 5U && memcmp(m->val, "false", 5) == 0) {
		*out = false;
		return true;
	}
	return false;
}

static bool str_eq(const member_t *m, const char *s)
{
	size_t n = strlen(s);

	return m != NULL && m->val_len == n + 2U && m->val[0] == '"' &&
	       memcmp(m->val + 1, s, n) == 0;
}

static int str_choice(const members_t *ms, const char *key, const char *const *opts, int n)
{
	const member_t *m = find(ms, key);

	for (int i = 0; i < n; i++) {
		if (str_eq(m, opts[i])) {
			return i;
		}
	}
	return -1;
}

/* Arreglo de exactamente n números; ints=true exige enteros. */
static bool get_array(const members_t *ms, const char *key, int n, bool ints,
		      double lo, double hi, double *out)
{
	const member_t *m = find(ms, key);
	const char *p, *end;
	int count = 0;

	if (m == NULL || m->val_len < 2U || m->val[0] != '[') {
		return false;
	}
	p = m->val + 1;
	end = m->val + m->val_len - 1; /* ']' */
	while (true) {
		const char *s = skip_ws(p, end);
		const char *e = s;
		long long iv;
		double v;
		bool ok;

		while (e < end && *e != ',' && *e != ' ' && *e != '\t' && *e != '\r' && *e != '\n') {
			e++;
		}
		if (count >= n) {
			return false;
		}
		if (ints) {
			ok = int_from(s, (size_t)(e - s), &iv);
			v = (double)iv;
		} else {
			ok = num_from(s, (size_t)(e - s), &v);
		}
		if (!ok || v < lo || v > hi) {
			return false;
		}
		out[count++] = v;
		p = skip_ws(e, end);
		if (p == end) {
			break;
		}
		if (*p != ',') {
			return false;
		}
		p++;
	}
	return count == n;
}

static bool cmd_header(const members_t *ms, const char *boot_id, uint32_t *seq)
{
	long long v, s;

	if (!get_int(ms, "v", AMR_CONTRACT_VERSION, AMR_CONTRACT_VERSION, &v) ||
	    !get_int(ms, "seq", 0, 0xFFFFFFFFLL, &s) || !boot_id_valid(boot_id) ||
	    !str_eq(find(ms, "boot_id"), boot_id)) {
		return false;
	}
	*seq = (uint32_t)s;
	return true;
}

bool amr_parse_cmd_vel(const char *json, size_t len, const char *boot_id, amr_cmd_vel_t *out)
{
	static const char *const keys[] = {"v", "boot_id", "seq", "vx", "vy", "wz", "valid_ms"};
	members_t ms;
	amr_cmd_vel_t c;
	long long valid;

	if (!parse_members(json, len, &ms) || !only_keys(&ms, keys, 7) ||
	    !cmd_header(&ms, boot_id, &c.seq) ||
	    !get_num(&ms, "vx", -AMR_MAX_LINEAR, AMR_MAX_LINEAR, &c.vx) ||
	    !get_num(&ms, "vy", -AMR_MAX_LINEAR, AMR_MAX_LINEAR, &c.vy) ||
	    !get_num(&ms, "wz", -AMR_MAX_ANGULAR, AMR_MAX_ANGULAR, &c.wz) ||
	    !get_int(&ms, "valid_ms", 1, AMR_MAX_VALID_MS, &valid)) {
		return false;
	}
	c.valid_ms = (uint32_t)valid;
	*out = c;
	return true;
}

bool amr_parse_cmd_arm(const char *json, size_t len, const char *boot_id, amr_cmd_arm_t *out)
{
	static const char *const goal_keys[] = {"v", "boot_id", "seq", "op", "pos", "acc", "dec"};
	members_t ms;
	amr_cmd_arm_t c;
	double pos[AMR_N_JOINTS], acc[AMR_N_JOINTS], dec[AMR_N_JOINTS];
	int op;

	memset(&c, 0, sizeof(c));
	if (!parse_members(json, len, &ms) || !cmd_header(&ms, boot_id, &c.seq)) {
		return false;
	}
	op = str_choice(&ms, "op", ARM_OPS, 5);
	if (op < 0) {
		return false;
	}
	c.op = (amr_arm_op_t)op;
	if (c.op == AMR_ARM_GOAL) {
		if (!only_keys(&ms, goal_keys, 7) ||
		    !get_array(&ms, "pos", AMR_N_JOINTS, true, -2147483648.0, 2147483647.0, pos) ||
		    !get_array(&ms, "acc", AMR_N_JOINTS, false, 1e-9, AMR_MAX_ARM_ACCEL, acc) ||
		    !get_array(&ms, "dec", AMR_N_JOINTS, false, 1e-9, AMR_MAX_ARM_ACCEL, dec)) {
			return false;
		}
		for (int i = 0; i < AMR_N_JOINTS; i++) {
			c.pos[i] = (int32_t)pos[i];
			c.acc[i] = (float)acc[i];
			c.dec[i] = (float)dec[i];
		}
	} else if (!only_keys(&ms, goal_keys, 4)) {
		return false; /* pos/acc/dec solo con op=goal */
	}
	*out = c;
	return true;
}

bool amr_parse_cmd_gripper(const char *json, size_t len, const char *boot_id, amr_cmd_gripper_t *out)
{
	static const char *const keys[] = {"v", "boot_id", "seq", "pos", "max_cur"};
	members_t ms;
	amr_cmd_gripper_t c;

	if (!parse_members(json, len, &ms) || !only_keys(&ms, keys, 5) ||
	    !cmd_header(&ms, boot_id, &c.seq) || !get_num(&ms, "pos", 0.0, 1.0, &c.pos) ||
	    !get_num(&ms, "max_cur", 1e-9, AMR_MAX_GRIP_CURRENT, &c.max_cur)) {
		return false;
	}
	*out = c;
	return true;
}

bool amr_parse_cmd_estop(const char *json, size_t len, const char *boot_id, amr_cmd_estop_t *out)
{
	static const char *const keys[] = {"v", "boot_id", "seq", "active"};
	members_t ms;
	amr_cmd_estop_t c;

	if (!parse_members(json, len, &ms) || !only_keys(&ms, keys, 4) ||
	    !cmd_header(&ms, boot_id, &c.seq) || !get_bool(&ms, "active", &c.active)) {
		return false;
	}
	*out = c;
	return true;
}

bool amr_parse_cmd_odom_reset(const char *json, size_t len, const char *boot_id,
			      amr_cmd_odom_reset_t *out)
{
	static const char *const keys[] = {"v", "boot_id", "seq", "x", "y", "yaw"};
	members_t ms;
	amr_cmd_odom_reset_t c;

	if (!parse_members(json, len, &ms) || !only_keys(&ms, keys, 6) ||
	    !cmd_header(&ms, boot_id, &c.seq) || !get_num(&ms, "x", -1e4, 1e4, &c.x) ||
	    !get_num(&ms, "y", -1e4, 1e4, &c.y) ||
	    !get_num(&ms, "yaw", -3.14159265358979, 3.14159265358979, &c.yaw)) {
		return false;
	}
	*out = c;
	return true;
}

bool amr_peek_seq(const char *json, size_t len, uint32_t *seq)
{
	members_t ms;
	long long s;

	if (!parse_members(json, len, &ms) || !get_int(&ms, "seq", 0, 0xFFFFFFFFLL, &s)) {
		return false;
	}
	*seq = (uint32_t)s;
	return true;
}

/* ================================================= watchdog y deduplicación */

void amr_vel_wd_init(amr_vel_wd_t *wd)
{
	memset(wd, 0, sizeof(*wd));
}

void amr_vel_wd_accept(amr_vel_wd_t *wd, const amr_cmd_vel_t *cmd, uint32_t now_ms)
{
	wd->vx = cmd->vx;
	wd->vy = cmd->vy;
	wd->wz = cmd->wz;
	wd->cmd_ms = now_ms;
	wd->valid_ms = cmd->valid_ms;
	wd->active = true;
	wd->tripped = false;
}

void amr_vel_wd_reject(amr_vel_wd_t *wd)
{
	wd->active = false;
}

void amr_vel_wd_step(amr_vel_wd_t *wd, uint32_t now_ms, bool link_ok, bool estop,
		     float *vx, float *vy, float *wz)
{
	if (wd->active) {
		/* La resta sin signo mantiene el timeout correcto al desbordar 32 bits. */
		if ((uint32_t)(now_ms - wd->cmd_ms) >= wd->valid_ms) {
			wd->active = false;
			wd->tripped = true;
		} else if (!link_ok || estop) {
			wd->active = false;
		}
	}
	*vx = wd->active ? wd->vx : 0.0f;
	*vy = wd->active ? wd->vy : 0.0f;
	*wz = wd->active ? wd->wz : 0.0f;
}

bool amr_seq_accept(amr_seq_t *s, uint32_t seq)
{
	if (s->valid && s->last == seq) {
		return false;
	}
	s->last = seq;
	s->valid = true;
	return true;
}
