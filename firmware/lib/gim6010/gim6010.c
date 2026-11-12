#include "gim6010.h"

#include <string.h>

static void put_u32(uint8_t *b, uint32_t v)
{
	b[0] = (uint8_t)v;
	b[1] = (uint8_t)(v >> 8);
	b[2] = (uint8_t)(v >> 16);
	b[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *b)
{
	return (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

void gim_init(gim_bus_t *b, const gim_transport_t *t, uint32_t timeout_ms)
{
	memset(b, 0, sizeof(*b));
	b->t = *t;
	b->timeout_ms = timeout_ms;
}

uint8_t gim_encode_pos_abs(uint8_t out[8], int32_t counts)
{
	memset(out, 0, 8);
	out[0] = GIM_CMD_POS_TRAP;
	out[1] = 0; /* absoluta */
	put_u32(&out[2], (uint32_t)counts);
	return 6;
}

uint8_t gim_encode_u32(uint8_t out[8], uint8_t cmd, uint32_t value)
{
	memset(out, 0, 8);
	out[0] = cmd;
	put_u32(&out[1], value);
	return 5;
}

void gim_decode_a4(const uint8_t d[8], gim_a4_t *out)
{
	out->temp_c = (int8_t)d[1];
	out->cur_ma = (int16_t)get_u16(&d[2]);
	out->vel_crpm = (int16_t)get_u16(&d[4]);
	out->pos16 = get_u16(&d[6]);
}

void gim_decode_ae(const uint8_t d[8], gim_ae_t *out)
{
	out->vbus_cv = get_u16(&d[1]);
	out->ibus_ca = get_u16(&d[3]);
	out->temp_c = d[5];
	out->mode = d[6];
	out->fault = d[7];
}

int32_t gim_unwrap(int32_t prev, bool valid, uint16_t pos16)
{
	if (!valid) {
		return (int32_t)(int16_t)pos16;
	}
	/* Diferencia modular de 16 bits con signo, sumada en 32 bits sin desbordar. */
	return (int32_t)((uint32_t)prev + (uint32_t)(int32_t)(int16_t)(uint16_t)(pos16 - (uint16_t)prev));
}

/* Envía y espera la confirmación: trama de addr con data[0] == cmd y DLC >= min_len.
 * Ignora el resto (p. ej. reportes 0xAE espontáneos u otros motores). */
static gim_status_t xfer(gim_bus_t *b, uint8_t addr, const uint8_t *tx, uint8_t tx_len,
			 uint8_t min_len, uint8_t rx[8])
{
	uint16_t id;
	uint8_t len;
	/* Un par de tramas ajenas por transacción como máximo antes de rendirse. */
	int budget = 8;

	if (addr == 0U || addr == 0xFFU) {
		return GIM_ERR_ARG;
	}
	if (b->t.send(b->t.ctx, (uint16_t)(GIM_TX_BASE | addr), tx, tx_len) != 0) {
		b->tx_errors++;
		return GIM_ERR_TX;
	}
	while (budget-- > 0) {
		int r = b->t.recv(b->t.ctx, &id, rx, &len, b->timeout_ms);

		if (r <= 0) {
			break;
		}
		if (id == addr && len >= min_len && rx[0] == tx[0]) {
			return GIM_OK;
		}
	}
	b->timeouts++;
	return GIM_ERR_TIMEOUT;
}

static gim_status_t simple(gim_bus_t *b, uint8_t addr, uint8_t cmd, uint8_t min_len, uint8_t rx[8])
{
	uint8_t tx[1] = {cmd};

	return xfer(b, addr, tx, 1, min_len, rx);
}

gim_status_t gim_read_a4(gim_bus_t *b, uint8_t addr, gim_a4_t *out)
{
	uint8_t rx[8];
	gim_status_t st = simple(b, addr, GIM_CMD_READ_A4, 8, rx);

	if (st == GIM_OK) {
		gim_decode_a4(rx, out);
	}
	return st;
}

gim_status_t gim_read_ae(gim_bus_t *b, uint8_t addr, gim_ae_t *out)
{
	uint8_t rx[8];
	gim_status_t st = simple(b, addr, GIM_CMD_READ_AE, 8, rx);

	if (st == GIM_OK) {
		gim_decode_ae(rx, out);
	}
	return st;
}

gim_status_t gim_clear_fault(gim_bus_t *b, uint8_t addr)
{
	uint8_t rx[8];

	return simple(b, addr, GIM_CMD_CLEAR_FAULT, 2, rx);
}

gim_status_t gim_disable(gim_bus_t *b, uint8_t addr)
{
	uint8_t rx[8];

	return simple(b, addr, GIM_CMD_DISABLE, 8, rx);
}

gim_status_t gim_trap_acc_dec(gim_bus_t *b, uint8_t addr, uint32_t acc_crpm_s, uint32_t dec_crpm_s)
{
	uint8_t tx[8], rx[8];
	uint8_t n = gim_encode_u32(tx, GIM_CMD_TRAP_ACC, acc_crpm_s);
	gim_status_t st = xfer(b, addr, tx, n, 5, rx);

	if (st != GIM_OK) {
		return st;
	}
	n = gim_encode_u32(tx, GIM_CMD_TRAP_DEC, dec_crpm_s);
	return xfer(b, addr, tx, n, 5, rx);
}

gim_status_t gim_pos_abs(gim_bus_t *b, uint8_t addr, int32_t counts)
{
	uint8_t tx[8], rx[8];
	uint8_t n = gim_encode_pos_abs(tx, counts);

	return xfer(b, addr, tx, n, 7, rx);
}
