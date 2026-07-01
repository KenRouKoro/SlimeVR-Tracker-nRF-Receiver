/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "ecan.h"
#include "globals.h"
#include "usb.h"
#include "util.h"

#include <math.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(ecan, LOG_LEVEL_INF);

struct ecan_tracker_state {
	uint8_t battery;
	uint8_t slime_temp_raw;
	bool has_info;
};

static struct ecan_tracker_state tracker_state[MAX_TRACKERS];

static const struct device *cdc_dev;
static bool cdc_ready;

#define ECAN_BUF_SIZE 16384
static uint8_t ecan_buf[ECAN_BUF_SIZE];
static volatile uint32_t ecan_buf_head;
static volatile uint32_t ecan_buf_tail;

static uint32_t ecan_frames_enqueued;
static uint32_t ecan_frames_dropped;

static inline uint32_t buf_used(void)
{
	int32_t diff = (int32_t)ecan_buf_head - (int32_t)ecan_buf_tail;
	if (diff < 0) {
		diff += ECAN_BUF_SIZE;
	}
	return (uint32_t)diff;
}

static inline uint32_t buf_free(void)
{
	return ECAN_BUF_SIZE - 1 - buf_used();
}

static void buf_put(uint8_t byte)
{
	ecan_buf[ecan_buf_head] = byte;
	ecan_buf_head = (ecan_buf_head + 1) % ECAN_BUF_SIZE;
}

static void ecan_discard_buffer(void)
{
	ecan_buf_tail = ecan_buf_head;
}

static uint32_t ecan_contiguous_len(void)
{
	uint32_t head = ecan_buf_head;
	uint32_t tail = ecan_buf_tail;

	if (tail == head) {
		return 0;
	}

	if (tail < head) {
		return head - tail;
	}

	return ECAN_BUF_SIZE - tail;
}

static void ecan_uart_irq_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (dev != cdc_dev) {
		return;
	}

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_tx_ready(dev)) {
			continue;
		}

		uint32_t len = ecan_contiguous_len();
		if (len == 0) {
			uart_irq_tx_disable(dev);
			continue;
		}

		int sent = uart_fifo_fill(dev, &ecan_buf[ecan_buf_tail], len);
		if (sent <= 0) {
			uart_irq_tx_disable(dev);
			continue;
		}

		ecan_buf_tail = (ecan_buf_tail + (uint32_t)sent) % ECAN_BUF_SIZE;
		if (ecan_buf_tail == ecan_buf_head) {
			uart_irq_tx_disable(dev);
		}
	}
}

static void ecan_tx_kick_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!cdc_dev) {
		return;
	}

	if (!receiver_usb_is_configured()) {
		ecan_discard_buffer();
		return;
	}

	if (ecan_buf_tail != ecan_buf_head) {
		uart_irq_tx_enable(cdc_dev);
	}
}
static K_WORK_DEFINE(ecan_tx_kick_work, ecan_tx_kick_work_handler);

static void ecan_timer_handler(struct k_timer *timer);
static K_TIMER_DEFINE(ecan_timer, ecan_timer_handler, NULL);

static void ecan_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&ecan_tx_kick_work);
}

static int16_t float_to_ecan_q15(float q)
{
	if (q > 1.0f) {
		q = 1.0f;
	} else if (q < -1.0f) {
		q = -1.0f;
	}

	return (int16_t)SATURATE_INT16(q * (float)ECAN_QUAT_SCALE_FACTOR);
}

static int16_t slime_int16_le_to_ecan(const uint8_t *data, int off)
{
	float q = (float)(int16_t)sys_get_le16(&data[off]) / (float)SLIME_QUAT_SOURCE_SCALE;

	return float_to_ecan_q15(q);
}

/* Slime type 2/7: uint32 LE packed quat (slimehid from_compressed_packed). */
static void slime_compressed_to_ecan_q15(const uint8_t *data, int off,
					 int16_t *qx, int16_t *qy, int16_t *qz, int16_t *qw)
{
	uint32_t packed = sys_get_le32(&data[off]);
	uint32_t u0 = packed & 0x3FFU;
	uint32_t u1 = (packed >> 10) & 0x7FFU;
	uint32_t u2 = (packed >> 21) & 0x7FFU;

	float v0 = ((float)u0 / 1024.0f) * 2.0f - 1.0f;
	float v1 = ((float)u1 / 2048.0f) * 2.0f - 1.0f;
	float v2 = ((float)u2 / 2048.0f) * 2.0f - 1.0f;

	float d = v0 * v0 + v1 * v1 + v2 * v2;
	float inv_sqrt_d = 1.0f / sqrtf(d + 1e-6f);
	float alpha = ((float)M_PI / 2.0f) * d * inv_sqrt_d;
	float s = sinf(alpha);
	float k = s * inv_sqrt_d;

	float fw = cosf(alpha);
	float fx = k * v0;
	float fy = k * v1;
	float fz = k * v2;

	*qw = float_to_ecan_q15(fw);
	*qx = float_to_ecan_q15(fx);
	*qy = float_to_ecan_q15(fy);
	*qz = float_to_ecan_q15(fz);
}

static bool is_zero_quaternion(const uint8_t *data)
{
	const uint16_t *q = (const uint16_t *)&data[2];

	return q[0] == 0 && q[1] == 0 && q[2] == 0 && q[3] == 0;
}

/* Slime type 0/2 B2 (batt) → ECAN-ECBT-16B §3.3 0..100 percent. */
static uint8_t slime_batt_to_ecan_percent(uint8_t slime_batt)
{
	if (slime_batt == 0xFF) {
		return 0xFF;
	}
	if (slime_batt == 128) {
		return 100;
	}
	if (slime_batt <= 100) {
		return slime_batt;
	}
	uint8_t v = slime_batt & 0x7F;

	return (uint8_t)(((uint16_t)v * 100U + 63U) / 127U);
}

static void cache_info(uint8_t tracker_id, const uint8_t *data)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	struct ecan_tracker_state *st = &tracker_state[tracker_id];

	st->battery = slime_batt_to_ecan_percent(data[2]);
	st->slime_temp_raw = data[4];
	st->has_info = true;
}

/* Slime type 0/2 B4: uint8 temp; °C = raw/2 - 39 (same as slimehid2ykwxyz).
 * MagDist=1 when decoded temperature < 0, i.e. raw != 0 && raw < 78. */
static uint8_t slime_temp_raw_to_mag_disturbed(uint8_t temp_raw)
{
	if (temp_raw == 0) {
		return 0;
	}

	return (temp_raw < 78) ? 1u : 0u;
}

static void enqueue_frame(const uint8_t frame[ECAN_ECBT_16B_FRAME_LEN])
{
	if (!cdc_ready) {
		return;
	}

	if (!receiver_usb_is_configured()) {
		return;
	}

	if (buf_free() < ECAN_ECBT_16B_FRAME_LEN) {
		ecan_frames_dropped++;
		return;
	}

	for (int i = 0; i < ECAN_ECBT_16B_FRAME_LEN; i++) {
		buf_put(frame[i]);
	}

	ecan_frames_enqueued++;
	k_work_submit(&ecan_tx_kick_work);
}

static void emit_ecan_quat(uint8_t tracker_id, int16_t qw, int16_t qx, int16_t qy, int16_t qz)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	struct ecan_tracker_state *st = &tracker_state[tracker_id];
	uint8_t frame[ECAN_ECBT_16B_FRAME_LEN];
	uint8_t batt = 0;

	if (st->has_info) {
		batt = st->battery;
		if (batt != 0xFF && batt > 100) {
			batt = 100;
		}
	}

	uint8_t mag = slime_temp_raw_to_mag_disturbed(
		st->has_info ? st->slime_temp_raw : 0);

	frame[0] = ECAN_ECBT_16B_FRAME_HEAD;
	frame[1] = ECAN_ECBT_16B_FRAME_TYPE;
	frame[2] = batt;
	frame[3] = tracker_id;
	/* ECAN-ECBT-16B.md §2.3: W,X,Y,Z @ 4,6,8,10 int16 BE */
	sys_put_be16((uint16_t)qw, &frame[4]);
	sys_put_be16((uint16_t)qx, &frame[6]);
	sys_put_be16((uint16_t)qy, &frame[8]);
	sys_put_be16((uint16_t)qz, &frame[10]);
	frame[12] = mag;
	frame[13] = 0;
	frame[14] = 0;
	frame[15] = ECAN_ECBT_16B_FRAME_TAIL;

	enqueue_frame(frame);
}

static void emit_quat_frame_type14(uint8_t tracker_id, const uint8_t *data)
{
	if (is_zero_quaternion(data)) {
		return;
	}

	int16_t qx = slime_int16_le_to_ecan(data, 2);
	int16_t qy = slime_int16_le_to_ecan(data, 4);
	int16_t qz = slime_int16_le_to_ecan(data, 6);
	int16_t qw = slime_int16_le_to_ecan(data, 8);

	emit_ecan_quat(tracker_id, qw, qx, qy, qz);
}

static void emit_quat_frame_compressed(uint8_t tracker_id, const uint8_t *data)
{
	int16_t qx, qy, qz, qw;

	slime_compressed_to_ecan_q15(data, 5, &qx, &qy, &qz, &qw);
	emit_ecan_quat(tracker_id, qw, qx, qy, qz);
}

void ecan_handle_packet(const uint8_t *data, uint8_t rssi)
{
	ARG_UNUSED(rssi);

	uint8_t type = data[0];
	uint8_t tracker_id = data[1];

	switch (type) {
	case 0:
		cache_info(tracker_id, data);
		break;
	case 1:
	case 4:
		emit_quat_frame_type14(tracker_id, data);
		break;
	case 2:
		cache_info(tracker_id, data);
		emit_quat_frame_compressed(tracker_id, data);
		break;
	case 7:
		emit_quat_frame_compressed(tracker_id, data);
		break;
	default:
		break;
	}
}

void ecan_reset_tracker(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	memset(&tracker_state[tracker_id], 0, sizeof(tracker_state[tracker_id]));
}

void ecan_reset_all(void)
{
	memset(tracker_state, 0, sizeof(tracker_state));
}

static int ecan_init_fn(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cdc_acm_uart1), okay)
	cdc_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));
#else
	cdc_dev = NULL;
	LOG_WRN("cdc_acm_uart1 not in devicetree; ECAN serial output disabled");
	return 0;
#endif

	if (!device_is_ready(cdc_dev)) {
		LOG_ERR("CDC ACM (ECAN) device not ready");
		return -ENODEV;
	}

	int ret = uart_irq_callback_user_data_set(cdc_dev, ecan_uart_irq_callback, NULL);

	if (ret != 0) {
		LOG_ERR("Failed to set ECAN CDC IRQ callback: %d", ret);
		return ret;
	}

	cdc_ready = true;
	ecan_buf_head = 0;
	ecan_buf_tail = 0;
	ecan_frames_enqueued = 0;
	ecan_frames_dropped = 0;
	ecan_reset_all();

	k_timer_start(&ecan_timer, K_MSEC(1), K_MSEC(1));

	LOG_INF("ECAN-ECBT-16B subsystem initialized (CDC uart1)");
	return 0;
}

int ecan_init(void)
{
	return ecan_init_fn();
}

SYS_INIT(ecan_init_fn, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);