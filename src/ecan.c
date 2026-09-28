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
#include "thread_priority.h"
#include "usb.h"
#include "util.h"

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(ecan, LOG_LEVEL_INF);

/* ---------------------------------------------------------------------------
 * Intake: the ESB event IRQ queues raw 16-byte Slime packets here. Framing
 * runs in workqueue context because the quaternion decoding uses the FPU,
 * which CONFIG_FPU_SHARING forbids in an ISR.
 * -------------------------------------------------------------------------*/
#define ECAN_PACKET_LEN 16
#define ECAN_PACKET_QUEUE_DEPTH 64
#define ECAN_MAP_BATCH 32

K_MSGQ_DEFINE(ecan_pkt_msgq, ECAN_PACKET_LEN, ECAN_PACKET_QUEUE_DEPTH, 4);

/* Own queue: framing runs libm on the pose path and must not wait behind
 * HID report building or console work on the system workqueue. */
static K_THREAD_STACK_DEFINE(ecan_wq_stack, 1024);
static struct k_work_q ecan_wq;

static atomic_t ecan_current_tps;
static atomic_t ecan_frames_in_interval;
static atomic_t ecan_last_tps_time_ms;
static atomic_t ecan_intake_dropped;
static atomic_t ecan_ring_dropped;
static atomic_t ecan_tracker_drops[MAX_TRACKERS];

/* Per-tracker battery/temperature cache (workqueue context only). */
struct ecan_tracker_state {
	uint8_t battery;
	uint8_t slime_temp_raw;
	bool has_info;
};

static struct ecan_tracker_state tracker_state[MAX_TRACKERS];

/* ---------------------------------------------------------------------------
 * CDC ACM output: same ring + IRQ + retry pattern as the raw data collector.
 * -------------------------------------------------------------------------*/
static const struct device *cdc_dev;
static bool cdc_ready;

/* 16 KB holds 1024 frames. 20 trackers at 100 Hz produce 32 KB/s, so the
 * ring absorbs ~0.5 s of USB stall before frames start dropping. */
#define ECAN_BUF_SIZE 16384
static uint8_t ecan_buf[ECAN_BUF_SIZE];
static volatile uint32_t ecan_buf_head;
static volatile uint32_t ecan_buf_tail;

static inline uint32_t ecan_buf_used(void)
{
	int32_t diff = (int32_t)ecan_buf_head - (int32_t)ecan_buf_tail;

	if (diff < 0) {
		diff += ECAN_BUF_SIZE;
	}
	return (uint32_t)diff;
}

static inline uint32_t ecan_buf_free(void)
{
	return ECAN_BUF_SIZE - 1 - ecan_buf_used();
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

static void ecan_tx_kick_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ecan_tx_kick_work, ecan_tx_kick_work_handler);

/* IRQ callback and producer kicks may run on different workqueues. Disable
 * first, then recheck: a producer before the disable must not lose its enable.
 * A producer after the recheck schedules its own kick. */
static void ecan_tx_pause(const struct device *dev)
{
	uart_irq_tx_disable(dev);
	if (ecan_buf_tail != ecan_buf_head) {
		k_work_schedule_for_queue(&ecan_wq, &ecan_tx_kick_work, K_MSEC(1));
	}
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

		if (!receiver_usb_is_configured()) {
			ecan_discard_buffer();
			ecan_tx_pause(dev);
			continue;
		}

		uint32_t len = ecan_contiguous_len();

		if (len == 0) {
			ecan_tx_pause(dev);
			continue;
		}

		int sent = uart_fifo_fill(dev, &ecan_buf[ecan_buf_tail], len);

		if (sent <= 0) {
			ecan_tx_pause(dev);
			continue;
		}

		ecan_buf_tail = (ecan_buf_tail + (uint32_t)sent) % ECAN_BUF_SIZE;
		if (ecan_buf_tail == ecan_buf_head) {
			ecan_tx_pause(dev);
		}
	}
}

static void ecan_tx_kick_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!cdc_ready) {
		return;
	}

	if (!receiver_usb_is_configured()) {
		ecan_discard_buffer();
		return;
	}

	if (ecan_buf_tail != ecan_buf_head) {
		uart_irq_tx_enable(cdc_dev);
		/* The CDC driver normally advances TX from completion/enable/resume.
		 * Keep a bounded fallback only while app bytes remain: FIFO-full can
		 * suppress callbacks until the host drains the port again. */
		if (ecan_buf_tail != ecan_buf_head) {
			k_work_schedule_for_queue(&ecan_wq, &ecan_tx_kick_work, K_MSEC(1));
		}
	}
}

static void ecan_timer_handler(struct k_timer *timer);
static K_TIMER_DEFINE(ecan_timer, ecan_timer_handler, NULL);

/* Integer-only, runs in timer ISR context. */
static void ecan_update_tps(void)
{
	uint32_t now = (uint32_t)k_uptime_get_32();
	uint32_t frames = (uint32_t)atomic_set(&ecan_frames_in_interval, 0);
	uint32_t last = (uint32_t)atomic_get(&ecan_last_tps_time_ms);

	if (last != 0 && now > last) {
		atomic_set(&ecan_current_tps, (atomic_val_t)(((uint64_t)frames * 1000U) / (now - last)));
	}
	atomic_set(&ecan_last_tps_time_ms, (atomic_val_t)now);
}

static void ecan_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	ecan_update_tps();
	/* Retry a stalled transfer even if no new frame arrives. */
	if (ecan_buf_tail != ecan_buf_head) {
		k_work_schedule_for_queue(&ecan_wq, &ecan_tx_kick_work, K_NO_WAIT);
	}
}

/* ---------------------------------------------------------------------------
 * Slime packet -> ECAN-ECBT-16B frame mapping (workqueue context).
 * -------------------------------------------------------------------------*/

/* Slime B2 (type 0/2): 0 = no battery, 0x80|percent = battery available,
 * 0xFF = fully charged (tracker clamps the level at 100). */
static uint8_t ecan_batt_to_percent(uint8_t slime_batt)
{
	if (slime_batt == 0) {
		return 0;
	}
	if (slime_batt & 0x80) {
		uint8_t percent = slime_batt & 0x7F;

		return percent > 100 ? 100 : percent;
	}
	if (slime_batt <= 100) {
		return slime_batt;
	}
	/* Legacy 7-bit level without the availability flag. */
	return (uint8_t)(((uint16_t)slime_batt * 100U + 63U) / 127U);
}

/* Slime B4 (type 0/2): uint8 temperature, degC = raw/2 - 39. The tracker
 * inverts the temperature while the fusion reports magnetic disturbance, so
 * raw in 1..77 means a negative temperature and therefore a disturbed mag. */
static uint8_t ecan_temp_raw_to_mag_disturbed(uint8_t temp_raw)
{
	if (temp_raw == 0) {
		return 0;
	}
	return (temp_raw < 78) ? 1u : 0u;
}

static int16_t ecan_float_to_q15(float q)
{
	if (q > 1.0f) {
		q = 1.0f;
	} else if (q < -1.0f) {
		q = -1.0f;
	}

	return (int16_t)SATURATE_INT16(q * (float)ECAN_QUAT_SCALE_FACTOR);
}

static int16_t ecan_slime_q15_to_ecan(const uint8_t *data, int offset)
{
	float q = (float)(int16_t)sys_get_le16(&data[offset]) / (float)SLIME_QUAT_SOURCE_SCALE;

	return ecan_float_to_q15(q);
}

/* Slime type 2 B3..B6: packed half-angle quat (10/11/11 bits, uint32 LE),
 * the inverse of the tracker's q_fem(). */
static void ecan_decode_packed_quat(const uint8_t *data, int offset, float *qw, float *qx, float *qy, float *qz)
{
	uint32_t packed = sys_get_le32(&data[offset]);
	uint32_t u0 = packed & 0x3FFU;
	uint32_t u1 = (packed >> 10) & 0x7FFU;
	uint32_t u2 = (packed >> 21) & 0x7FFU;

	float v0 = ((float)u0 / 1024.0f) * 2.0f - 1.0f;
	float v1 = ((float)u1 / 2048.0f) * 2.0f - 1.0f;
	float v2 = ((float)u2 / 2048.0f) * 2.0f - 1.0f;

	float d = v0 * v0 + v1 * v1 + v2 * v2;
	float inv_sqrt_d = 1.0f / sqrtf(d + (float)EPS);
	float alpha = (float)M_PI * 0.5f * d * inv_sqrt_d;
	float k = sinf(alpha) * inv_sqrt_d;

	*qw = cosf(alpha);
	*qx = k * v0;
	*qy = k * v1;
	*qz = k * v2;
}

static void ecan_cache_info(uint8_t tracker_id, const uint8_t *data)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	struct ecan_tracker_state *state = &tracker_state[tracker_id];

	state->battery = ecan_batt_to_percent(data[2]);
	state->slime_temp_raw = data[4];
	state->has_info = true;
}

static void ecan_enqueue_frame(uint8_t tracker_id, const uint8_t frame[ECAN_ECBT_16B_FRAME_LEN])
{
	if (!cdc_ready) {
		return;
	}

	if (ecan_buf_free() < ECAN_ECBT_16B_FRAME_LEN) {
		atomic_inc(&ecan_ring_dropped);
		if (tracker_id < MAX_TRACKERS) {
			atomic_inc(&ecan_tracker_drops[tracker_id]);
		}
		return;
	}

	for (int i = 0; i < ECAN_ECBT_16B_FRAME_LEN; i++) {
		ecan_buf[ecan_buf_head] = frame[i];
		ecan_buf_head = (ecan_buf_head + 1) % ECAN_BUF_SIZE;
	}

	atomic_inc(&ecan_frames_in_interval);
	k_work_schedule_for_queue(&ecan_wq, &ecan_tx_kick_work, K_NO_WAIT);
}

static void ecan_emit_quat(uint8_t tracker_id, int16_t qw, int16_t qx, int16_t qy, int16_t qz)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	const struct ecan_tracker_state *state = &tracker_state[tracker_id];
	uint8_t frame[ECAN_ECBT_16B_FRAME_LEN];

	frame[0] = ECAN_ECBT_16B_FRAME_HEAD;
	frame[1] = ECAN_ECBT_16B_FRAME_TYPE;
	/* No info packet seen yet: report 0 rather than a stale level. */
	frame[2] = state->has_info ? state->battery : 0;
	frame[3] = tracker_id;
	/* ECAN-ECBT-16B.md 2.3: W,X,Y,Z at 4,6,8,10 as int16 BE. */
	sys_put_be16((uint16_t)qw, &frame[4]);
	sys_put_be16((uint16_t)qx, &frame[6]);
	sys_put_be16((uint16_t)qy, &frame[8]);
	sys_put_be16((uint16_t)qz, &frame[10]);
	frame[12] = ecan_temp_raw_to_mag_disturbed(state->has_info ? state->slime_temp_raw : 0);
	frame[13] = 0;
	frame[14] = 0;
	frame[15] = ECAN_ECBT_16B_FRAME_TAIL;

	ecan_enqueue_frame(tracker_id, frame);
}

static void ecan_emit_full_quat(uint8_t tracker_id, const uint8_t *data)
{
	/* Type 1/4: qx,qy,qz,qw int16 LE Q15 at B2..B9. */
	int16_t qx = (int16_t)sys_get_le16(&data[2]);
	int16_t qy = (int16_t)sys_get_le16(&data[4]);
	int16_t qz = (int16_t)sys_get_le16(&data[6]);
	int16_t qw = (int16_t)sys_get_le16(&data[8]);

	/* Drop the all-zero placeholder the tracker sends before its first fix. */
	if ((qx | qy | qz | qw) == 0) {
		return;
	}

	ecan_emit_quat(
		tracker_id,
		ecan_slime_q15_to_ecan(data, 8),
		ecan_slime_q15_to_ecan(data, 2),
		ecan_slime_q15_to_ecan(data, 4),
		ecan_slime_q15_to_ecan(data, 6)
	);
}

static void ecan_emit_packed_quat(uint8_t tracker_id, const uint8_t *data)
{
	float qw, qx, qy, qz;

	ecan_decode_packed_quat(data, 5, &qw, &qx, &qy, &qz);
	ecan_emit_quat(
		tracker_id,
		ecan_float_to_q15(qw),
		ecan_float_to_q15(qx),
		ecan_float_to_q15(qy),
		ecan_float_to_q15(qz)
	);
}

/* data[0] = Slime packet type, data[1] = tracker id (see src/hid.c table).
 * Types 6/7 are legacy and carry no quaternion; the current tracker sends
 * 0/1/2/3/4/5 plus composite packets only. */
static void ecan_map_packet(const uint8_t *data)
{
	uint8_t tracker_id = data[1];

	switch (data[0]) {
	case 0:
		ecan_cache_info(tracker_id, data);
		break;
	case 1:
	case 4:
		ecan_emit_full_quat(tracker_id, data);
		break;
	case 2:
		ecan_cache_info(tracker_id, data);
		ecan_emit_packed_quat(tracker_id, data);
		break;
	default:
		break;
	}
}

static void ecan_map_work_handler(struct k_work *work);
static K_WORK_DEFINE(ecan_map_work, ecan_map_work_handler);

static void ecan_map_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	uint8_t packet[ECAN_PACKET_LEN];

	for (int i = 0; i < ECAN_MAP_BATCH; i++) {
		if (k_msgq_get(&ecan_pkt_msgq, packet, K_NO_WAIT) != 0) {
			break;
		}
		ecan_map_packet(packet);
	}

	if (k_msgq_num_used_get(&ecan_pkt_msgq) > 0) {
		k_work_submit_to_queue(&ecan_wq, &ecan_map_work);
	}
}

void ecan_handle_packet(const uint8_t *data, uint8_t rssi)
{
	/* RSSI is not part of the ECAN frame; kept for the shared pose sink. */
	ARG_UNUSED(rssi);

	if (k_msgq_put(&ecan_pkt_msgq, data, K_NO_WAIT) != 0) {
		uint8_t tracker_id = data[1];

		atomic_inc(&ecan_intake_dropped);
		if (tracker_id < MAX_TRACKERS) {
			atomic_inc(&ecan_tracker_drops[tracker_id]);
		}
		return;
	}

	k_work_submit_to_queue(&ecan_wq, &ecan_map_work);
}

void ecan_reset_tracker(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	memset(&tracker_state[tracker_id], 0, sizeof(tracker_state[tracker_id]));
	atomic_set(&ecan_tracker_drops[tracker_id], 0);
}

void ecan_reset_all(void)
{
	memset(tracker_state, 0, sizeof(tracker_state));
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		atomic_set(&ecan_tracker_drops[i], 0);
	}
}

uint32_t ecan_get_current_tps(void)
{
	return (uint32_t)atomic_get(&ecan_current_tps);
}

uint32_t ecan_get_total_drop_count(void)
{
	return (uint32_t)atomic_get(&ecan_ring_dropped) + (uint32_t)atomic_get(&ecan_intake_dropped);
}

uint32_t ecan_get_total_tracker_drop_count(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return 0;
	}
	return (uint32_t)atomic_get(&ecan_tracker_drops[tracker_id]);
}

static int ecan_init(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cdc_acm_uart1), okay)
	cdc_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));

	if (!device_is_ready(cdc_dev)) {
		LOG_ERR("ECAN CDC device not ready");
		return -ENODEV;
	}

	int ret = uart_irq_callback_user_data_set(cdc_dev, ecan_uart_irq_callback, NULL);

	if (ret != 0) {
		LOG_ERR("Failed to set ECAN CDC IRQ callback: %d", ret);
		return ret;
	}
#else
	if (IS_ENABLED(CONFIG_SLIMEVR_ECAN_STREAM)) {
		LOG_ERR(
			"ECAN stream enabled but no cdc_acm_uart1 node: pass "
			"-DEXTRA_DTC_OVERLAY_FILE=boards/ecan_cdc.overlay"
		);
	}
	cdc_dev = NULL;
	return -ENODEV;
#endif

	cdc_ready = true;
	ecan_buf_head = 0;
	ecan_buf_tail = 0;
	k_work_queue_init(&ecan_wq);
	k_work_queue_start(&ecan_wq, ecan_wq_stack, K_THREAD_STACK_SIZEOF(ecan_wq_stack), ECAN_WQ_PRIORITY, NULL);
	k_thread_name_set(&ecan_wq.thread, "ecan_wq");
	ecan_reset_all();
	k_timer_start(&ecan_timer, K_SECONDS(1), K_SECONDS(1));

	LOG_INF("ECAN-ECBT-16B stream initialized on cdc_acm_uart1");
	return 0;
}

SYS_INIT(ecan_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
