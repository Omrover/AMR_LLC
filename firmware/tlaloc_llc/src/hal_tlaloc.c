/*
 * HAL de la Tlaloc (FRDM-A-S32K344) sobre drivers de Zephyr 4.5.
 *
 *   RoboClaw: LPUART3 a 115200 por interrupciones (sin espera activa)
 *   BNO080:   LPI2C0 a 400 kHz, INT en PTA21 (EIRQ3)
 *   Servo:    eMIOS1 ch10 (PTA27), 50 Hz
 *   ACS712:   ADC0 precisión ch0 (PTD1), 14 bits, promedio de 4
 *   FS26:     watchdog por LPSPI0 (driver nxp,fs26-wdog)
 *   Brazo:    FlexCAN0 (zephyr,canbus) a 1 Mbps, 4 GIM6010-8
 */
#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "hal.h"

LOG_MODULE_REGISTER(hal_tlaloc, LOG_LEVEL_INF);

#define USER_NODE DT_PATH(zephyr_user)

/* ------------------------------------------------------------------ RoboClaw */

/*
 * RX: FIFO de un productor (ISR) y un consumidor (hilo de control); índices
 * libres que solo escribe su dueño, tamaño potencia de 2.
 * TX: una trama a la vez (el driver espera la respuesta antes de la siguiente).
 */
#define RC_RX_SIZE 256U
static const struct device *const rc_uart = DEVICE_DT_GET(DT_ALIAS(roboclaw_uart));
static uint8_t rc_rx[RC_RX_SIZE];
static volatile uint32_t rc_rx_head; /* escribe la ISR */
static volatile uint32_t rc_rx_tail; /* escribe el hilo */
static uint8_t rc_tx[64];
static volatile uint32_t rc_tx_len;
static volatile uint32_t rc_tx_pos;
static K_SEM_DEFINE(rc_rx_sem, 0, 1);
static K_SEM_DEFINE(rc_tx_idle, 1, 1);

static void rc_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	if (uart_irq_rx_ready(dev)) {
		uint8_t buf[16];
		int n;

		while ((n = uart_fifo_read(dev, buf, sizeof(buf))) > 0) {
			for (int i = 0; i < n; i++) {
				uint32_t head = rc_rx_head;

				if (head - rc_rx_tail < RC_RX_SIZE) { /* lleno: se descarta */
					rc_rx[head % RC_RX_SIZE] = buf[i];
					rc_rx_head = head + 1U;
				}
			}
		}
		k_sem_give(&rc_rx_sem);
	}
	if (uart_irq_tx_ready(dev)) {
		if (rc_tx_pos < rc_tx_len) {
			int w = uart_fifo_fill(dev, &rc_tx[rc_tx_pos], (int)(rc_tx_len - rc_tx_pos));

			if (w > 0) {
				rc_tx_pos += (uint32_t)w;
			}
		} else {
			uart_irq_tx_disable(dev);
			k_sem_give(&rc_tx_idle);
		}
	}
}

static int uart_rc_write(void *ctx, const uint8_t *buf, size_t n)
{
	ARG_UNUSED(ctx);

	if (n > sizeof(rc_tx)) {
		return -EMSGSIZE;
	}
	/* 64 bytes a 115200 baud tardan ~6 ms: 20 ms es una trama atascada */
	if (k_sem_take(&rc_tx_idle, K_MSEC(20)) != 0) {
		uart_irq_tx_disable(rc_uart);
	}
	memcpy(rc_tx, buf, n);
	rc_tx_pos = 0;
	rc_tx_len = (uint32_t)n;
	uart_irq_tx_enable(rc_uart);
	return 0;
}

static int uart_rc_read(void *ctx, uint8_t *buf, size_t n, uint32_t timeout_ms)
{
	ARG_UNUSED(ctx);
	int64_t deadline = k_uptime_get() + timeout_ms;
	size_t got = 0;

	while (got < n) {
		uint32_t tail = rc_rx_tail;

		while (got < n && tail != rc_rx_head) {
			buf[got++] = rc_rx[tail % RC_RX_SIZE];
			tail++;
		}
		rc_rx_tail = tail;
		if (got >= n) {
			break;
		}
		int64_t left = deadline - k_uptime_get();

		if (left <= 0 || k_sem_take(&rc_rx_sem, K_MSEC(left)) != 0) {
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static void rc_flush_rx(void *ctx)
{
	ARG_UNUSED(ctx);
	rc_rx_tail = rc_rx_head;
	k_sem_reset(&rc_rx_sem);
}

rc_transport_t hal_roboclaw_transport(void)
{
	rc_transport_t t = {uart_rc_write, uart_rc_read, rc_flush_rx, NULL};

	return t;
}

/* -------------------------------------------------------------------- BNO080 */

static const struct device *const imu_bus = DEVICE_DT_GET(DT_ALIAS(imu_i2c));
static K_SEM_DEFINE(imu_int_sem, 0, 1);

#if DT_NODE_HAS_PROP(USER_NODE, imu_int_gpios)
static const struct gpio_dt_spec imu_int = GPIO_DT_SPEC_GET(USER_NODE, imu_int_gpios);
static struct gpio_callback imu_int_cb;
static bool imu_int_ok;

static void imu_int_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_sem_give(&imu_int_sem);
}
#endif

static int imu_write(void *ctx, const uint8_t *buf, size_t n)
{
	ARG_UNUSED(ctx);
	return i2c_write(imu_bus, buf, (uint32_t)n, CONFIG_TLALOC_BNO080_ADDR);
}

static int imu_read(void *ctx, uint8_t *buf, size_t n)
{
	ARG_UNUSED(ctx);
	return i2c_read(imu_bus, buf, (uint32_t)n, CONFIG_TLALOC_BNO080_ADDR);
}

bno_transport_t hal_imu_transport(void)
{
	bno_transport_t t = {imu_write, imu_read, NULL};

	return t;
}

void hal_imu_wait(int32_t timeout_ms)
{
#if DT_NODE_HAS_PROP(USER_NODE, imu_int_gpios)
	if (imu_int_ok && gpio_pin_get_dt(&imu_int) > 0) {
		return; /* INT ya activo: hay datos pendientes */
	}
#endif
	(void)k_sem_take(&imu_int_sem, K_MSEC(timeout_ms));
}

/* ---------------------------------------------------------- servo y ACS712 */

static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET_BY_NAME(USER_NODE, gripper);
static const struct adc_dt_spec acs712 = ADC_DT_SPEC_GET_BY_IDX(USER_NODE, 0);
static bool adc_ok;

static int servo_set_us(void *ctx, uint32_t pulse_us)
{
	ARG_UNUSED(ctx);
	return pwm_set_pulse_dt(&servo, PWM_USEC(pulse_us));
}

static int adc_read_mv(void *ctx, float *mv)
{
	ARG_UNUSED(ctx);
	int16_t raw;
	int32_t val;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};

	if (!adc_ok || adc_sequence_init_dt(&acs712, &seq) != 0 || adc_read_dt(&acs712, &seq) != 0) {
		return -EIO;
	}
	val = raw;
	if (adc_raw_to_millivolts_dt(&acs712, &val) != 0) {
		return -EIO;
	}
	*mv = (float)val;
	return 0;
}

llc_io_t hal_io(void)
{
	llc_io_t io = {servo_set_us, adc_read_mv, NULL};

	return io;
}

/* ------------------------------------------------------------------- FS26 */

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(watchdog0)) && defined(CONFIG_TLALOC_FS26_WATCHDOG)
static const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int wdt_channel = -1;
static uint32_t wdt_last_ms;

int hal_wdt_start(void)
{
	/* Ventana de 128 ms con la parte cerrada mínima (~40 ms): se alimenta cada
	 * ~80 ms, siempre dentro de la parte abierta. Un MCU colgado -> RSTB. */
	struct wdt_timeout_cfg cfg = {
		.window = {.min = 0, .max = 128},
		.flags = WDT_FLAG_RESET_SOC,
	};

	if (!device_is_ready(wdt)) {
		LOG_ERR("FS26 no listo: revisar LPSPI0 o modo debug del SBC");
		return -ENODEV;
	}
	wdt_channel = wdt_install_timeout(wdt, &cfg);
	if (wdt_channel < 0 || wdt_setup(wdt, 0) != 0) {
		LOG_ERR("FS26: no se pudo activar el watchdog (%d)", wdt_channel);
		wdt_channel = -1;
		return -EIO;
	}
	wdt_last_ms = k_uptime_get_32();
	LOG_INF("FS26 watchdog activo (128 ms)");
	return 0;
}

void hal_wdt_kick(uint32_t now_ms)
{
	if (wdt_channel >= 0 && (uint32_t)(now_ms - wdt_last_ms) >= 75U) {
		if (wdt_feed(wdt, wdt_channel) == 0) {
			wdt_last_ms = now_ms;
		}
	}
}
#else
int hal_wdt_start(void)
{
	return -ENOTSUP;
}

void hal_wdt_kick(uint32_t now_ms)
{
	ARG_UNUSED(now_ms);
}
#endif

/* ------------------------------------------------------------------- init */

uint16_t hal_broker_port(void)
{
	return CONFIG_TLALOC_BROKER_PORT;
}

const char *hal_mqtt_prefix(void)
{
	return CONFIG_TLALOC_MQTT_PREFIX;
}

/* ---------------------------------------------------------------- CAN brazo */

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
/* Solo IDs estándar 0x000-0x0FF: respuestas de los motores (ID = addr). El eco
 * propio de las tramas 0x100|addr no pasa el filtro. */
CAN_MSGQ_DEFINE(can_rx_q, 16);
static bool can_ok;

static int can_tx(void *ctx, uint16_t id, const uint8_t *data, uint8_t len)
{
	struct can_frame f = {0};

	ARG_UNUSED(ctx);
	if (len > 8U) {
		return -EINVAL;
	}
	f.id = id;
	f.dlc = can_bytes_to_dlc(len);
	memcpy(f.data, data, len);
	k_msgq_purge(&can_rx_q); /* una respuesta vieja no debe confirmar este comando */
	return can_send(can_dev, &f, K_MSEC(5), NULL, NULL);
}

static int can_rx(void *ctx, uint16_t *id, uint8_t data[8], uint8_t *len, uint32_t timeout_ms)
{
	struct can_frame f;

	ARG_UNUSED(ctx);
	if (k_msgq_get(&can_rx_q, &f, K_MSEC(timeout_ms)) != 0) {
		return 0;
	}
	*id = (uint16_t)f.id;
	*len = can_dlc_to_bytes(f.dlc);
	if (*len > 8U) {
		*len = 8U;
	}
	memcpy(data, f.data, *len);
	return 1;
}

gim_transport_t hal_can_transport(void)
{
	gim_transport_t t = {NULL, NULL, NULL};

	if (can_ok) {
		t.send = can_tx;
		t.recv = can_rx;
	}
	return t;
}

static int can_setup(void)
{
	const struct can_filter filter = {.id = 0x000, .mask = 0x700, .flags = 0};
	int rc;

	if (!device_is_ready(can_dev)) {
		return -ENODEV;
	}
	rc = can_set_bitrate(can_dev, 1000000);
	if (rc == 0) {
		rc = can_set_mode(can_dev, CAN_MODE_NORMAL);
	}
	if (rc == 0) {
		rc = can_add_rx_filter_msgq(can_dev, &can_rx_q, &filter);
		rc = rc < 0 ? rc : 0;
	}
	if (rc == 0) {
		rc = can_start(can_dev);
	}
	return rc;
}

int hal_init(void)
{
	int rc;

	if (!device_is_ready(rc_uart)) {
		LOG_ERR("UART de las RoboClaw no lista");
		return -ENODEV;
	}
	uart_irq_rx_disable(rc_uart);
	uart_irq_tx_disable(rc_uart);
	rc = uart_irq_callback_user_data_set(rc_uart, rc_uart_isr, NULL);
	if (rc != 0) {
		LOG_ERR("UART RoboClaw sin modo por interrupciones (%d)", rc);
		return rc;
	}
	uart_irq_rx_enable(rc_uart);

	if (!device_is_ready(imu_bus)) {
		LOG_ERR("I2C de la BNO080 no listo");
		return -ENODEV;
	}
#if DT_NODE_HAS_PROP(USER_NODE, imu_int_gpios)
	if (gpio_is_ready_dt(&imu_int) && gpio_pin_configure_dt(&imu_int, GPIO_INPUT) == 0 &&
	    gpio_pin_interrupt_configure_dt(&imu_int, GPIO_INT_EDGE_TO_ACTIVE) == 0) {
		gpio_init_callback(&imu_int_cb, imu_int_handler, BIT(imu_int.pin));
		imu_int_ok = gpio_add_callback(imu_int.port, &imu_int_cb) == 0;
	}
	if (!imu_int_ok) {
		LOG_WRN("INT de la BNO080 no disponible: se sondea cada 5 ms");
	}
#endif

	if (!pwm_is_ready_dt(&servo)) {
		LOG_ERR("PWM del servo no listo");
		return -ENODEV;
	}
	adc_ok = adc_is_ready_dt(&acs712) && adc_channel_setup_dt(&acs712) == 0;
	if (!adc_ok) {
		LOG_WRN("ADC del ACS712 no disponible: el gripper no detecta agarre");
	}
	rc = can_setup();
	can_ok = rc == 0;
	if (!can_ok) {
		/* Sin CAN la base funciona igual; cmd/arm se rechaza. */
		LOG_WRN("FlexCAN0 no disponible (%d): brazo deshabilitado", rc);
	}
	return 0;
}
