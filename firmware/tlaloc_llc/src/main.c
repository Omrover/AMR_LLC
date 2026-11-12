/*
 * tlaloc_llc: controlador de bajo nivel del Mobile Manipulator en la Tlaloc
 * (S32K344). Contrato MQTT v1 con la NUC (AMR_LLC/protocol/CONTRACT.md).
 *
 * Hilos (prioridad, periodo):
 *   control (2, 20 ms): watchdog de cmd/vel, RoboClaw, EKF, gripper, batería,
 *                       FS26. Funciona aunque se caiga la red: sin enlace frena.
 *   imu     (3, INT):   BNO080 por I2C (SH-2), entrega datos al núcleo.
 *   main    (MQTT):     conexión, comandos (QoS 1 con PUBACK y ack del
 *                       contrato) y telemetría a las tasas del contrato.
 * Toda la lógica vive en llc.c (probada en host); aquí solo hay hilos y red.
 */
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/atomic.h>

#include "app_config.h"
#include "hal.h"
#include "llc.h"

LOG_MODULE_REGISTER(tlaloc, LOG_LEVEL_INF);

#define CONTROL_PERIOD_MS 20
#define TOPIC_MAX 64

static llc_t llc;
static llc_imu_t imu;
static K_MUTEX_DEFINE(core_lock);
static atomic_t mqtt_up;
static char boot_id[17];
static char client_id[TOPIC_MAX];

/* --------------------------------------------------------------------- hilos */

static void control_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	int64_t next = k_uptime_get();
	bool zero_logged = false;
	uint32_t faults_logged = 0;

	while (true) {
		uint32_t t0 = k_cycle_get_32();
		uint32_t now = k_uptime_get_32();
		uint32_t faults;
		float zero_mv = -1.0f;

		k_mutex_lock(&core_lock, K_FOREVER);
		llc_control_step(&llc, now, atomic_get(&mqtt_up) != 0);
		llc.cpu = (float)k_cyc_to_us_floor32(k_cycle_get_32() - t0) /
			  (CONTROL_PERIOD_MS * 1000.0f);
		faults = llc.faults;
		if (!zero_logged && llc.grip.zero_ready) {
			zero_mv = llc.grip.zero_mv;
		}
		k_mutex_unlock(&core_lock);
		hal_wdt_kick(now);

		/* diagnóstico de bring-up en la consola (fuera del lock) */
		if (zero_mv >= 0.0f) {
			zero_logged = true;
			LOG_INF("ACS712: cero en %.1f mV", (double)zero_mv);
		}
		if (faults != faults_logged) {
			LOG_INF("faults 0x%02x (bits: 0 RC128, 1 RC129, 2 IMU, 5 gripper, 6 batería)",
				faults);
			faults_logged = faults;
		}

		next += CONTROL_PERIOD_MS;
		if (next < k_uptime_get()) {
			next = k_uptime_get() + CONTROL_PERIOD_MS; /* atrasado: no acumular */
		}
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
}

K_THREAD_DEFINE(control_tid, 3072, control_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(2), 0,
		K_TICKS_FOREVER);

static void imu_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	uint32_t errors = 0;

	while (true) {
		hal_imu_wait(5);
		uint32_t now = k_uptime_get_32();
		int upd = llc_imu_service(&imu, now, 8);

		if (upd > 0) {
			k_mutex_lock(&core_lock, K_FOREVER);
			llc_imu_sample(&llc, &imu.bno.data, (uint32_t)upd, now);
			k_mutex_unlock(&core_lock);
		} else if (upd < 0) {
			if ((errors++ % 200U) == 0U) {
				LOG_WRN("BNO080 sin respuesta en 0x%02x", CONFIG_TLALOC_BNO080_ADDR);
			}
			k_msleep(5); /* INT pegado o bus caído: no girar en vacío */
		}
	}
}

K_THREAD_DEFINE(imu_tid, 2048, imu_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(3), 0,
		K_TICKS_FOREVER);

/* ---------------------------------------------------------------------- MQTT */

static const char *const cmd_names[LLC_CMD_COUNT] = {
	[LLC_CMD_VEL] = "cmd/vel",
	[LLC_CMD_ARM] = "cmd/arm",
	[LLC_CMD_GRIPPER] = "cmd/gripper",
	[LLC_CMD_ESTOP] = "cmd/estop",
	[LLC_CMD_ODOM_RESET] = "cmd/odom_reset",
};

static const struct {
	const char *name;
	uint32_t period_ms;
	enum mqtt_qos qos;
} tel_def[LLC_TEL_COUNT] = {
	[LLC_TEL_ODOM] = {"llc/odom", 20, MQTT_QOS_0_AT_MOST_ONCE},
	[LLC_TEL_IMU] = {"llc/imu", 10, MQTT_QOS_0_AT_MOST_ONCE},
	[LLC_TEL_WHEELS] = {"llc/wheels", 50, MQTT_QOS_0_AT_MOST_ONCE},
	[LLC_TEL_ARM] = {"llc/arm", 20, MQTT_QOS_0_AT_MOST_ONCE},
	[LLC_TEL_BATTERY] = {"llc/battery", 1000, MQTT_QOS_0_AT_MOST_ONCE},
	[LLC_TEL_STATUS] = {"llc/status", 500, MQTT_QOS_1_AT_LEAST_ONCE},
};

static char cmd_topics[LLC_CMD_COUNT][TOPIC_MAX];
static char tel_topics[LLC_TEL_COUNT][TOPIC_MAX];
static char topic_ack[TOPIC_MAX];

static uint8_t rx_buf[AMR_MAX_PAYLOAD + 128];
static uint8_t tx_buf[AMR_MAX_PAYLOAD + 128];
static char payload[AMR_MAX_PAYLOAD + 1];
static char out[AMR_MAX_PAYLOAD];
static char lwt_payload[48];
static struct mqtt_client client;
static struct sockaddr_storage broker;
static struct mqtt_topic will_topic;
static struct mqtt_utf8 will_msg;
static uint16_t msg_id = 1;

static uint16_t next_msg_id(void)
{
	if (++msg_id == 0U) {
		msg_id = 1;
	}
	return msg_id;
}

static int publish(const char *topic, const char *data, size_t len, enum mqtt_qos qos,
		   bool retain)
{
	struct mqtt_publish_param p = {0};

	p.message.topic.topic.utf8 = (const uint8_t *)topic;
	p.message.topic.topic.size = strlen(topic);
	p.message.topic.qos = qos;
	p.message.payload.data = (uint8_t *)data;
	p.message.payload.len = len;
	p.message_id = qos == MQTT_QOS_0_AT_MOST_ONCE ? 0U : next_msg_id();
	p.retain_flag = retain ? 1U : 0U;
	return mqtt_publish(&client, &p);
}

static amr_hdr_t make_hdr(void)
{
	amr_hdr_t h = {boot_id, k_uptime_get()};

	return h;
}

static void handle_command(const struct mqtt_publish_param *pub)
{
	const struct mqtt_utf8 *t = &pub->message.topic.topic;
	uint32_t len = pub->message.payload.len;
	int topic = -1;
	bool ok = false;
	int n;

	for (int i = 0; i < LLC_CMD_COUNT; i++) {
		if (t->size == strlen(cmd_topics[i]) && memcmp(t->utf8, cmd_topics[i], t->size) == 0) {
			topic = i;
			break;
		}
	}
	if (len <= AMR_MAX_PAYLOAD) {
		ok = mqtt_readall_publish_payload(&client, (uint8_t *)payload, len) == 0;
	} else {
		/* demasiado largo para el contrato: se drena y se trata como inválido */
		while (len > 0U) {
			uint32_t chunk = MIN(len, (uint32_t)AMR_MAX_PAYLOAD);

			if (mqtt_readall_publish_payload(&client, (uint8_t *)payload, chunk) != 0) {
				break;
			}
			len -= chunk;
		}
		len = 0;
	}
	if (pub->message.topic.qos == MQTT_QOS_1_AT_LEAST_ONCE) {
		struct mqtt_puback_param ack = {.message_id = pub->message_id};

		(void)mqtt_publish_qos1_ack(&client, &ack);
	}
	if (topic < 0) {
		return;
	}
	if (!ok) {
		len = 0; /* el parser lo rechaza: cmd/vel inválido detiene la base */
	}
	payload[len] = '\0';
	if (topic == LLC_CMD_VEL && !IS_ENABLED(CONFIG_TLALOC_ALLOW_MOTION)) {
		len = 0;
	}

	amr_hdr_t h = make_hdr();

	k_mutex_lock(&core_lock, K_FOREVER);
	n = llc_command(&llc, (llc_cmd_topic_t)topic, payload, len, pub->retain_flag != 0U, &h,
			k_uptime_get_32(), out, sizeof(out));
	k_mutex_unlock(&core_lock);
	if (n > 0) {
		(void)publish(topic_ack, out, (size_t)n, MQTT_QOS_1_AT_LEAST_ONCE, false);
	}
}

static void mqtt_evt(struct mqtt_client *const c, const struct mqtt_evt *evt)
{
	ARG_UNUSED(c);

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result == 0) {
			if (boot_id[0] == '\0') {
				/* nuevo por cada reset del MCU; el bridge reinicia sus
				 * referencias al verlo cambiar */
				snprintf(boot_id, sizeof(boot_id), "%08x%08x", sys_rand32_get(),
					 k_cycle_get_32());
			}
			atomic_set(&mqtt_up, 1);
			LOG_INF("MQTT conectado, boot_id %s", boot_id);
		}
		break;
	case MQTT_EVT_DISCONNECT:
		atomic_set(&mqtt_up, 0);
		LOG_WRN("MQTT desconectado");
		break;
	case MQTT_EVT_PUBLISH:
		handle_command(&evt->param.publish);
		break;
	default:
		break;
	}
}

static int subscribe_commands(void)
{
	struct mqtt_topic list[LLC_CMD_COUNT];
	struct mqtt_subscription_list subs = {
		.list = list,
		.list_count = LLC_CMD_COUNT,
		.message_id = next_msg_id(),
	};

	for (int i = 0; i < LLC_CMD_COUNT; i++) {
		list[i].topic.utf8 = (const uint8_t *)cmd_topics[i];
		list[i].topic.size = strlen(cmd_topics[i]);
		/* cmd/vel en QoS 0: una retransmisión entregaría una velocidad vieja */
		list[i].qos = i == LLC_CMD_VEL ? MQTT_QOS_0_AT_MOST_ONCE : MQTT_QOS_1_AT_LEAST_ONCE;
	}
	return mqtt_subscribe(&client, &subs);
}

static int connect_broker(void)
{
	struct sockaddr_in *b4 = (struct sockaddr_in *)&broker;
	struct zsock_pollfd pfd;
	int rc;

	b4->sin_family = AF_INET;
	b4->sin_port = htons(hal_broker_port());
	zsock_inet_pton(AF_INET, CONFIG_TLALOC_BROKER_ADDR, &b4->sin_addr);

	rc = amr_fmt_lwt(lwt_payload, sizeof(lwt_payload));
	if (rc <= 0) {
		return -EINVAL;
	}
	will_topic.topic.utf8 = (const uint8_t *)tel_topics[LLC_TEL_STATUS];
	will_topic.topic.size = strlen(tel_topics[LLC_TEL_STATUS]);
	will_topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
	will_msg.utf8 = (const uint8_t *)lwt_payload;
	will_msg.size = (uint32_t)rc;

	mqtt_client_init(&client);
	client.broker = &broker;
	client.evt_cb = mqtt_evt;
	client.client_id.utf8 = (const uint8_t *)client_id;
	client.client_id.size = strlen(client_id);
	client.protocol_version = MQTT_VERSION_3_1_1;
	client.clean_session = 1U;
	client.keepalive = 5U;
	client.will_topic = &will_topic;
	client.will_message = &will_msg;
	client.will_retain = 1U;
	client.rx_buf = rx_buf;
	client.rx_buf_size = sizeof(rx_buf);
	client.tx_buf = tx_buf;
	client.tx_buf_size = sizeof(tx_buf);
	client.transport.type = MQTT_TRANSPORT_NON_SECURE;

	rc = mqtt_connect(&client);
	if (rc != 0) {
		return rc;
	}
	pfd.fd = client.transport.tcp.sock;
	pfd.events = ZSOCK_POLLIN;
	for (int i = 0; i < 20 && !atomic_get(&mqtt_up); i++) {
		if (zsock_poll(&pfd, 1, 100) > 0 && mqtt_input(&client) != 0) {
			break;
		}
	}
	if (!atomic_get(&mqtt_up)) {
		mqtt_abort(&client);
		return -ETIMEDOUT;
	}
	/* Borra el LWT retenido de una sesión anterior (payload vacío + retain). */
	rc = publish(tel_topics[LLC_TEL_STATUS], "", 0, MQTT_QOS_1_AT_LEAST_ONCE, true);
	return rc != 0 ? rc : subscribe_commands();
}

static void publish_due(int64_t now, int64_t next[LLC_TEL_COUNT])
{
	for (int k = 0; k < LLC_TEL_COUNT; k++) {
		int n;

		if (now < next[k]) {
			continue;
		}
		next[k] += tel_def[k].period_ms;
		if (next[k] <= now) {
			next[k] = now + tel_def[k].period_ms;
		}
		amr_hdr_t h = make_hdr();

		k_mutex_lock(&core_lock, K_FOREVER);
		n = llc_telemetry(&llc, (llc_tel_t)k, &h, out, sizeof(out));
		k_mutex_unlock(&core_lock);
		if (n > 0) {
			(void)publish(tel_topics[k], out, (size_t)n, tel_def[k].qos, false);
		}
	}
}

static void serve_connection(void)
{
	struct zsock_pollfd pfd = {
		.fd = client.transport.tcp.sock,
		.events = ZSOCK_POLLIN,
	};
	int64_t next[LLC_TEL_COUNT];
	int64_t now = k_uptime_get();

	for (int k = 0; k < LLC_TEL_COUNT; k++) {
		next[k] = now + k; /* escalonados para no salir todos en el mismo ms */
	}
	while (atomic_get(&mqtt_up)) {
		int64_t wait = INT64_MAX;
		int rc;

		now = k_uptime_get();
		for (int k = 0; k < LLC_TEL_COUNT; k++) {
			wait = MIN(wait, next[k] - now);
		}
		rc = zsock_poll(&pfd, 1, (int)CLAMP(wait, 0, 10));
		if (rc < 0 ||
		    (rc > 0 && ((pfd.revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP | ZSOCK_POLLNVAL)) ||
				mqtt_input(&client) != 0))) {
			break;
		}
		rc = mqtt_live(&client);
		if (rc != 0 && rc != -EAGAIN) {
			break;
		}
		publish_due(k_uptime_get(), next);
	}
	atomic_set(&mqtt_up, 0); /* el hilo de control frena en el siguiente ciclo */
	mqtt_abort(&client);
}

int main(void)
{
	llc_cfg_t cfg;
	rc_transport_t rct;
	bno_transport_t bnot;
	gim_transport_t cant;
	llc_io_t io;

	const char *prefix = hal_mqtt_prefix();

	for (int i = 0; i < LLC_CMD_COUNT; i++) {
		snprintf(cmd_topics[i], TOPIC_MAX, "%s/%s", prefix, cmd_names[i]);
	}
	for (int k = 0; k < LLC_TEL_COUNT; k++) {
		snprintf(tel_topics[k], TOPIC_MAX, "%s/%s", prefix, tel_def[k].name);
	}
	snprintf(topic_ack, sizeof(topic_ack), "%s/llc/ack", prefix);
	snprintf(client_id, sizeof(client_id), "%s/llc", prefix);

	if (hal_init() != 0) {
		LOG_ERR("falló la inicialización del hardware: la base queda detenida");
		return 0;
	}
	if (IS_ENABLED(CONFIG_TLALOC_FS26_WATCHDOG) && hal_wdt_start() != 0) {
		LOG_WRN("sin watchdog externo");
	}
	app_config(&cfg);
	rct = hal_roboclaw_transport();
	bnot = hal_imu_transport();
	cant = hal_can_transport();
	io = hal_io();
	if (!llc_init(&llc, &cfg, &rct, &cant, &io)) {
		LOG_ERR("configuración de tracción inválida: la base no se moverá");
	}
	llc_imu_init(&imu, &bnot);
	k_thread_start(control_tid);
	k_thread_start(imu_tid);

	LOG_INF("Tlaloc LLC: broker %s:%d prefijo %s, movimiento %s", CONFIG_TLALOC_BROKER_ADDR,
		hal_broker_port(), prefix,
		IS_ENABLED(CONFIG_TLALOC_ALLOW_MOTION) ? "HABILITADO" : "deshabilitado");
	LOG_INF("ruedas mecanum: r=%.4f m lx=%.4f m ly=%.4f m, %d cuentas/vuelta, max %.2f rad/s",
		(double)cfg.drive.wheel_radius, (double)cfg.drive.half_length,
		(double)cfg.drive.half_width,
		(int)cfg.drive.counts_per_rev, (double)cfg.drive.max_wheel_rad_s);

	while (true) {
		int rc = connect_broker();

		if (rc == 0) {
			serve_connection();
		} else {
			LOG_WRN("no se pudo conectar al broker (%d), reintentando", rc);
		}
		k_msleep(1000);
	}
	return 0;
}
