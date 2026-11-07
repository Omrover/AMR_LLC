/*
 * Class 10 v2 -- S32K344 wheel node: encoder -> MQTT -> NUC bridge -> ROS 2.
 *
 * Two threads, deliberately separated:
 *  - control thread (every 5 ms): local watchdog, always drives the motors,
 *    even when the network, broker or NUC is gone;
 *  - main thread: MQTT client, 10 Hz encoder telemetry, receives wheel PWM.
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

#include "hal.h"
#include "lab_logic.h"

LOG_MODULE_REGISTER(wheel_node, LOG_LEVEL_INF);

#define PREFIX CONFIG_LAB_MQTT_PREFIX
#define CONTROL_PERIOD_MS 5
#define PUBLISH_PERIOD_MS 100

static char boot_id[17];
static char topic_encoder[64];
static char topic_applied[64];
static char topic_pwm[64];

static K_MUTEX_DEFINE(cmd_lock);
static lab_cmd_t cmd;
static atomic_t mqtt_up;
static float applied_left;
static float applied_right;

static uint8_t rx_buf[512];
static uint8_t tx_buf[512];
static struct mqtt_client client;
static struct sockaddr_storage broker;

/* ------------------------------------------------------------------ control */

static void control_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	float left = 0.0f, right = 0.0f;
	float last_left = 1.0f, last_right = 1.0f; /* force first write */

	while (true) {
		k_mutex_lock(&cmd_lock, K_FOREVER);
		lab_step(&cmd, k_uptime_get_32(), atomic_get(&mqtt_up) != 0,
			 &left, &right);
		applied_left = left;
		applied_right = right;
		k_mutex_unlock(&cmd_lock);

		if (left != last_left || right != last_right) {
			if (hal_set_wheels(left, right) != 0) {
				LOG_ERR("PWM write failed");
			}
			last_left = left;
			last_right = right;
		}
		k_msleep(CONTROL_PERIOD_MS);
	}
}

K_THREAD_DEFINE(control_tid, 2048, control_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(2), 0, K_TICKS_FOREVER);

/* --------------------------------------------------------------------- mqtt */

static void handle_pwm(const struct mqtt_publish_param *pub)
{
	char payload[256];
	uint32_t len = pub->message.payload.len;
	float left, right;
	uint32_t valid_ms;
	bool ok = false;

	if (len < sizeof(payload)) {
		ok = mqtt_readall_publish_payload(&client, payload, len) == 0;
	} else {
		/* Too long for the contract: drain it and treat as invalid. */
		while (len > 0U) {
			uint32_t chunk = MIN(len, sizeof(payload));

			if (mqtt_readall_publish_payload(&client, payload, chunk) != 0) {
				break;
			}
			len -= chunk;
		}
	}

	ok = ok && !pub->retain_flag &&
	     lab_parse_pwm(payload, pub->message.payload.len, boot_id,
			   &left, &right, &valid_ms);

	k_mutex_lock(&cmd_lock, K_FOREVER);
	if (ok) {
		ok = lab_command(&cmd, left, right, valid_ms, k_uptime_get_32());
	} else {
		cmd.active = false; /* invalid input stops, never keeps moving */
	}
	k_mutex_unlock(&cmd_lock);
	if (!ok) {
		LOG_WRN("rejected wheel_pwm command");
	}
}

static void mqtt_evt(struct mqtt_client *const c, const struct mqtt_evt *evt)
{
	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result == 0) {
			/*
			 * New boot_id per reset; the bridge uses it to reset its
			 * baselines. Created after the first CONNACK because the
			 * link-up/TCP timing then differs on every boot.
			 */
			if (boot_id[0] == '\0') {
				snprintf(boot_id, sizeof(boot_id), "%08x%08x",
					 sys_rand32_get(), k_cycle_get_32());
			}
			atomic_set(&mqtt_up, 1);
			LOG_INF("MQTT connected, boot_id %s", boot_id);
		}
		break;
	case MQTT_EVT_DISCONNECT:
		atomic_set(&mqtt_up, 0);
		LOG_WRN("MQTT disconnected");
		break;
	case MQTT_EVT_PUBLISH: {
		const struct mqtt_publish_param *pub = &evt->param.publish;

		if (pub->message.topic.topic.size == strlen(topic_pwm) &&
		    memcmp(pub->message.topic.topic.utf8, topic_pwm,
			   pub->message.topic.topic.size) == 0) {
			handle_pwm(pub);
		}
		break;
	}
	default:
		break;
	}
	ARG_UNUSED(c);
}

static int publish(const char *topic, const char *payload)
{
	struct mqtt_publish_param p = {0};

	p.message.topic.topic.utf8 = (const uint8_t *)topic;
	p.message.topic.topic.size = strlen(topic);
	p.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
	p.message.payload.data = (uint8_t *)payload;
	p.message.payload.len = strlen(payload);
	p.retain_flag = 0U; /* live telemetry is never retained */
	return mqtt_publish(&client, &p);
}

static void publish_telemetry(void)
{
	char buf[160];
	double rotation;
	float left, right;

	if (hal_read_rotation(&rotation) != 0) {
		LOG_WRN("encoder read failed");
		return;
	}
	int64_t ticks = student_ticks_from_rotation(rotation, hal_ticks_per_rev());

	if (lab_format_encoder(buf, sizeof(buf), ticks, k_uptime_get(), boot_id) > 0) {
		publish(topic_encoder, buf);
	}

	k_mutex_lock(&cmd_lock, K_FOREVER);
	left = applied_left;
	right = applied_right;
	k_mutex_unlock(&cmd_lock);
	snprintf(buf, sizeof(buf), "{\"left\":%.3f,\"right\":%.3f}",
		 (double)left, (double)right);
	publish(topic_applied, buf);
}

static int subscribe_pwm(void)
{
	struct mqtt_topic t = {
		.topic = {.utf8 = (const uint8_t *)topic_pwm, .size = strlen(topic_pwm)},
		.qos = MQTT_QOS_0_AT_MOST_ONCE,
	};
	struct mqtt_subscription_list list = {
		.list = &t, .list_count = 1U, .message_id = 1U,
	};

	return mqtt_subscribe(&client, &list);
}

static int connect_broker(void)
{
	struct sockaddr_in *b4 = (struct sockaddr_in *)&broker;
	struct zsock_pollfd pfd;
	int rc;

	b4->sin_family = AF_INET;
	b4->sin_port = htons(CONFIG_LAB_BROKER_PORT);
	zsock_inet_pton(AF_INET, CONFIG_LAB_BROKER_ADDR, &b4->sin_addr);

	mqtt_client_init(&client);
	client.broker = &broker;
	client.evt_cb = mqtt_evt;
	/* One client per robot: a reconnect replaces its own stale session. */
	client.client_id.utf8 = (const uint8_t *)PREFIX;
	client.client_id.size = strlen(PREFIX);
	client.protocol_version = MQTT_VERSION_3_1_1;
	client.clean_session = 1U;
	client.keepalive = 10U;
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
	if (IS_ENABLED(CONFIG_LAB_ALLOW_PWM)) {
		return subscribe_pwm();
	}
	return 0;
}

static void serve_connection(void)
{
	struct zsock_pollfd pfd = {
		.fd = client.transport.tcp.sock, .events = ZSOCK_POLLIN,
	};
	int64_t next_pub = k_uptime_get();

	while (atomic_get(&mqtt_up)) {
		int64_t wait = MAX(0, next_pub - k_uptime_get());
		int rc = zsock_poll(&pfd, 1, (int)MIN(wait, 50));

		if (rc > 0 && ((pfd.revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP)) ||
			       mqtt_input(&client) != 0)) {
			break;
		}
		if (rc < 0) {
			break;
		}
		rc = mqtt_live(&client);
		if (rc != 0 && rc != -EAGAIN) { /* -EAGAIN: no ping due yet */
			break;
		}
		if (k_uptime_get() >= next_pub) {
			publish_telemetry();
			next_pub += PUBLISH_PERIOD_MS;
			if (next_pub < k_uptime_get()) {
				next_pub = k_uptime_get() + PUBLISH_PERIOD_MS;
			}
		}
	}
	atomic_set(&mqtt_up, 0); /* control thread zeros outputs immediately */
	mqtt_abort(&client);
}

int main(void)
{
	lab_init(&cmd);
	if (hal_init() != 0) {
		LOG_ERR("hardware init failed; motors stay off");
		return 0;
	}
	k_thread_start(control_tid);

	LOG_INF("student self-test: %s", lab_student_selftest() ? "PASS" :
		"FAIL (complete ## TODO in src/student.c)");
	LOG_INF("broker %s:%d prefix %s, wheel PWM %s", CONFIG_LAB_BROKER_ADDR,
		CONFIG_LAB_BROKER_PORT, PREFIX,
		IS_ENABLED(CONFIG_LAB_ALLOW_PWM) ? "ENABLED" : "disabled (read-only)");

	snprintf(topic_encoder, sizeof(topic_encoder), "%s/encoder", PREFIX);
	snprintf(topic_applied, sizeof(topic_applied), "%s/applied_pwm", PREFIX);
	snprintf(topic_pwm, sizeof(topic_pwm), "%s/wheel_pwm", PREFIX);

	while (true) {
		int rc = connect_broker();

		if (rc == 0) {
			serve_connection();
		} else {
			LOG_WRN("broker connect failed (%d), retrying", rc);
		}
		k_msleep(1000);
	}
	return 0;
}
