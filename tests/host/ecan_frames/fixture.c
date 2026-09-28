/* Host fixture for the ECAN-ECBT-16B framing module: stubs the UART/USB edges
 * and drives the production code in src/ecan.c. */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "ecan.h"

/* math.h only exposes M_PI to POSIX callers; the firmware takes it from
 * util.h, which the fixture does not need otherwise. */
#define TEST_PI 3.14159265358979323846f

/* --------------------------------------------------------------------------
 * Transport stubs
 * -----------------------------------------------------------------------*/
uint32_t host_now;

const struct device host_cdc_device = {0};

static bool host_usb_configured = true;
static bool host_fifo_drains = true;
static size_t host_fifo_free = 0;

bool receiver_usb_is_configured(void)
{
	return host_usb_configured;
}

static uint8_t captured[64 * 1024];
static size_t captured_len;
static int fill_calls;

/* FIFO that accepts everything, or exactly host_fifo_free bytes when the
 * harness simulates a host that stopped reading. */
int uart_fifo_fill(const struct device *dev, const uint8_t *data, int size)
{
	(void)dev;
	fill_calls++;
	if (!host_fifo_drains) {
		if (host_fifo_free == 0) {
			return 0;
		}
		size_t take = host_fifo_free < (size_t)size ? host_fifo_free : (size_t)size;

		memcpy(captured + captured_len, data, take);
		captured_len += take;
		host_fifo_free -= take;
		return (int)take;
	}
	assert(captured_len + (size_t)size <= sizeof(captured));
	memcpy(captured + captured_len, data, (size_t)size);
	captured_len += (size_t)size;
	return size;
}

static uart_irq_callback_user_data_t host_irq_cb;
static void *host_irq_user_data;
static bool host_tx_irq_enabled;

int uart_irq_callback_user_data_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *user_data)
{
	(void)dev;
	host_irq_cb = cb;
	host_irq_user_data = user_data;
	return 0;
}

void uart_irq_tx_enable(const struct device *dev)
{
	host_tx_irq_enabled = true;
	if (host_irq_cb) {
		host_irq_cb(dev, host_irq_user_data);
	}
}

void uart_irq_tx_disable(const struct device *dev)
{
	(void)dev;
	host_tx_irq_enabled = false;
}

bool uart_irq_tx_ready(const struct device *dev)
{
	(void)dev;
	return true;
}

bool uart_irq_update(const struct device *dev)
{
	(void)dev;
	return true;
}

/* The driver reports a pending TX interrupt only while the app has TX enabled. */
bool uart_irq_is_pending(const struct device *dev)
{
	(void)dev;
	return host_tx_irq_enabled;
}

/* Work queue: queue handlers, run them explicitly (production retries carry a
 * 1 ms delay, so running them synchronously would spin). */
#define HOST_PENDING_MAX 8
static struct k_work *pending[HOST_PENDING_MAX];
static size_t pending_count;

void host_queue_work(struct k_work *work)
{
	for (size_t i = 0; i < pending_count; i++) {
		if (pending[i] == work) {
			return; /* already scheduled, like k_work_schedule */
		}
	}
	assert(pending_count < HOST_PENDING_MAX);
	pending[pending_count++] = work;
}

/* Runs the queued handlers once; handlers that requeue wait for the next call. */
static void host_run_pending(void)
{
	size_t count = pending_count;

	pending_count = 0;
	for (size_t i = 0; i < count; i++) {
		pending[i]->handler(pending[i]);
	}
}

/* One ring push schedules the TX kick on top of the mapping work, so a feed
 * needs several rounds before the bytes reach the port. */
static void host_run_pending_bounded(int rounds)
{
	for (int i = 0; i < rounds && pending_count > 0; i++) {
		host_run_pending();
	}
}

/* --------------------------------------------------------------------------
 * Test helpers
 * -----------------------------------------------------------------------*/
static void reset_stream(void)
{
	captured_len = 0;
	fill_calls = 0;
	host_usb_configured = true;
	host_fifo_drains = true;
	pending_count = 0;
	ecan_reset_all();
}

static void feed_packet(const uint8_t packet[16])
{
	ecan_handle_packet(packet, 0);
	host_run_pending_bounded(4);
}

static void feed_info(uint8_t tracker_id, uint8_t batt, uint8_t temp)
{
	uint8_t packet[16] = {0};

	packet[0] = 0;
	packet[1] = tracker_id;
	packet[2] = batt;
	packet[3] = 0x80; /* batt_v */
	packet[4] = temp;
	feed_packet(packet);
}

static void feed_quat(uint8_t tracker_id, int16_t qx, int16_t qy, int16_t qz, int16_t qw)
{
	uint8_t packet[16] = {0};
	uint16_t raw[4] = {(uint16_t)qx, (uint16_t)qy, (uint16_t)qz, (uint16_t)qw};

	packet[0] = 1;
	packet[1] = tracker_id;
	memcpy(&packet[2], raw, sizeof(raw)); /* int16 LE, x,y,z,w */
	feed_packet(packet);
}

static void feed_compact(uint8_t tracker_id, uint8_t batt, uint8_t temp, uint32_t packed)
{
	uint8_t packet[16] = {0};

	packet[0] = 2;
	packet[1] = tracker_id;
	packet[2] = batt;
	packet[4] = temp;
	memcpy(&packet[5], &packed, sizeof(packed));
	feed_packet(packet);
}

static int16_t frame_q(const uint8_t *frame, int offset)
{
	return (int16_t)((uint16_t)frame[offset] << 8 | frame[offset + 1]);
}

/* Wire values may differ by 1 LSB from the ideal rescale (32767/32768). */
static bool q_near(int16_t got, int16_t want)
{
	int diff = (int)got - (int)want;

	return diff >= -1 && diff <= 1;
}

static size_t frame_count(void)
{
	return captured_len / ECAN_ECBT_16B_FRAME_LEN;
}

static const uint8_t *frame_at(size_t index)
{
	assert(index < frame_count());
	return &captured[index * ECAN_ECBT_16B_FRAME_LEN];
}

/* Slime side of the compact quaternion: q_fem() then 10/11/11 bit packing,
 * copied from the tracker firmware so the decode is checked against a real
 * encoder rather than a restatement of the decoder. */
static uint32_t pack_quat(float w, float x, float y, float z)
{
	float a = 1.0f - fabsf(w) * fabsf(w);
	float inv_sqrt_a = 1.0f / sqrtf(a + 1e-6f);
	float k = a * inv_sqrt_a;
	float atan_term = (2.0f / TEST_PI) * atanf(k / w);
	float s = atan_term * inv_sqrt_a * (w == 0.0f ? 1.0f : copysignf(1.0f, w));
	float v[3] = {s * x, s * y, s * z};
	uint32_t u[3];

	for (int i = 0; i < 3; i++) {
		float n = (v[i] + 1.0f) / 2.0f;
		float scaled = n * (i == 0 ? 1024.0f : 2048.0f);
		int truncated = (int)scaled; /* the tracker casts, it does not round */
		int limit = (i == 0 ? 1023 : 2047);

		if (truncated > limit) {
			truncated = limit;
		} else if (truncated < 0) {
			truncated = 0;
		}
		u[i] = (uint32_t)truncated;
	}
	return u[0] | (u[1] << 10) | (u[2] << 21);
}

/* --------------------------------------------------------------------------
 * Cases
 * -----------------------------------------------------------------------*/
static void layout(void)
{
	reset_stream();
	feed_info(3, 0x80 | 73, 60);
	assert(frame_count() == 0); /* info updates the cache only */
	feed_quat(3, 32767, -32768, 0, 0);
	assert(frame_count() == 1);

	const uint8_t *frame = frame_at(0);

	assert(frame[0] == ECAN_ECBT_16B_FRAME_HEAD);
	assert(frame[1] == ECAN_ECBT_16B_FRAME_TYPE);
	assert(frame[2] == 73);
	assert(frame[3] == 3);
	assert(frame_q(frame, 4) == 0);            /* W */
	assert(q_near(frame_q(frame, 6), 32766));  /* X: 32767 -> +1.0 */
	assert(q_near(frame_q(frame, 8), -32767)); /* Y: -32768 -> -1.0 */
	assert(frame_q(frame, 10) == 0);           /* Z */
	assert(frame[12] == 1);                    /* temp 60 -> disturbed */
	assert(frame[13] == 0 && frame[14] == 0);
	assert(frame[15] == ECAN_ECBT_16B_FRAME_TAIL);
	printf("PASS layout\n");
}

static void quaternion(void)
{
	reset_stream();
	/* All-zero placeholder packets are not poses. */
	feed_quat(1, 0, 0, 0, 0);
	assert(frame_count() == 0);

	/* Identity and axis rotations keep unit length and sign. */
	feed_quat(1, 0, 0, 0, 32767);
	assert(frame_count() == 1 && q_near(frame_q(frame_at(0), 4), 32766));
	assert(frame_q(frame_at(0), 6) == 0 && frame_q(frame_at(0), 10) == 0);

	feed_quat(2, 0, 0, -32768, 0);
	assert(frame_count() == 2);
	assert(q_near(frame_q(frame_at(1), 10), -32767) && frame_at(1)[3] == 2);

	/* A tracker not in the store still maps: the id is the pairing slot. */
	feed_quat(15, 16384, 0, 0, 0);
	assert(frame_count() == 3 && frame_at(2)[3] == 15);
	printf("PASS quaternion\n");
}

static void battery(void)
{
	static const struct {
		uint8_t raw;
		uint8_t percent;
	} cases[] = {
		{0x00, 0}, /* no battery reported */
		{0x80, 0}, /* available, 0 % */
		{0x80 | 1, 1},
		{0x80 | 73, 73},
		{0x80 | 100, 100},
		{0xFF, 100}, /* fully charged */
		{0x64, 100}, /* percent without the availability flag */
		{64, 64},
		{101, 80}, /* legacy 7-bit level, never sent by current trackers */
		{127, 100},
	};

	reset_stream();
	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		feed_info(0, cases[i].raw, 200);
		feed_quat(0, 0, 0, 0, 32767);
		assert(frame_at(frame_count() - 1)[2] == cases[i].percent);
	}
	printf("PASS battery\n");
}

static void mag(void)
{
	static const struct {
		uint8_t temp_raw;
		uint8_t disturbed;
	} cases[] = {
		{0, 0},  /* no temperature data */
		{1, 1},  /* -38.5 degC */
		{60, 1}, /* -9 degC: tracker inverts temperature on mag disturbance */
		{77, 1},
		{78, 0}, /* 0 degC */
		{150, 0},
		{255, 0},
	};

	reset_stream();
	/* No info packet yet: no stale level, no disturbance flag. */
	feed_quat(0, 0, 0, 0, 32767);
	assert(frame_at(0)[2] == 0 && frame_at(0)[12] == 0);

	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		feed_info(0, 0x80 | 50, cases[i].temp_raw);
		feed_quat(0, 0, 0, 0, 32767);
		assert(frame_at(frame_count() - 1)[12] == cases[i].disturbed);
	}
	printf("PASS mag\n");
}

static void packed(void)
{
	/* Light rotations only: the 10/11/11 bit half-angle format is coarse. */
	static const float quats[][4] = {
		{1.0f, 0.0f, 0.0f, 0.0f},
		{0.70710678f, 0.0f, 0.0f, 0.70710678f},
		{0.70710678f, 0.70710678f, 0.0f, 0.0f},
		{0.96592583f, 0.0f, 0.25881905f, 0.0f},
		{0.99144486f, 0.13052619f, 0.0f, 0.0f},
	};

	reset_stream();
	for (size_t i = 0; i < sizeof(quats) / sizeof(quats[0]); i++) {
		const float *q = quats[i]; /* w,x,y,z */
		uint32_t packed = pack_quat(q[0], q[1], q[2], q[3]);

		feed_compact(4, 0x80 | 55, 200, packed);
		assert(frame_count() == i + 1);
		const uint8_t *frame = frame_at(i);

		assert(frame[3] == 4 && frame[2] == 55 && frame[12] == 0);
		float w = frame_q(frame, 4) / 32767.0f;
		float x = frame_q(frame, 6) / 32767.0f;
		float y = frame_q(frame, 8) / 32767.0f;
		float z = frame_q(frame, 10) / 32767.0f;

		assert(fabsf(w - q[0]) < 0.005f);
		assert(fabsf(x - q[1]) < 0.005f);
		assert(fabsf(y - q[2]) < 0.005f);
		assert(fabsf(z - q[3]) < 0.005f);
	}
	/* Type 7 is button/sleeptime in the current protocol, not a pose. */
	uint8_t packet[16] = {7, 4};

	feed_packet(packet);
	assert(frame_count() == sizeof(quats) / sizeof(quats[0]));
	printf("PASS packed\n");
}

static void drop(void)
{
	/* Ring drop: a host that stops reading fills the 16 KB ring; frames that
	 * no longer fit are counted per tracker instead of corrupting the stream. */
	reset_stream();
	host_fifo_drains = false;
	host_fifo_free = 0;
	feed_info(5, 0x80 | 90, 200);

	const int frames = 1040;
	const int queued = (16384 - 1) / ECAN_ECBT_16B_FRAME_LEN;

	for (int i = 0; i < frames; i++) {
		feed_quat(5, 0, 0, 0, 32767);
	}
	assert(captured_len == 0); /* nothing reaches the port while it is stalled */
	assert(ecan_get_total_tracker_drop_count(5) == (uint32_t)(frames - queued));
	assert(ecan_get_total_drop_count() == (uint32_t)(frames - queued));

	/* Draining resumes delivery of exactly the buffered frames, oldest first. */
	host_fifo_drains = true;
	host_run_pending_bounded(4);
	assert(captured_len == (size_t)queued * ECAN_ECBT_16B_FRAME_LEN);
	assert(frame_count() == (size_t)queued);
	assert(frame_at(0)[3] == 5 && frame_at((size_t)queued - 1)[15] == ECAN_ECBT_16B_FRAME_TAIL);
	printf("PASS drop\n");
}

static void buffer(void)
{
	/* Intake queue: packets that cannot be queued before the mapping work runs
	 * are dropped and attributed, not silently lost. */
	reset_stream();
	const int packets = 100;

	for (int i = 0; i < packets; i++) {
		uint8_t packet[16] = {1, 9, 0, 0, 0, 0, 0, 0, 0, 0x7F, 0xFF};

		ecan_handle_packet(packet, 0); /* no work run: fill the intake queue */
	}
	assert(ecan_get_total_tracker_drop_count(9) == (uint32_t)(packets - 64));
	assert(ecan_get_total_drop_count() == (uint32_t)(packets - 64));

	host_run_pending_bounded(8);
	assert(frame_count() == 64);
	printf("PASS buffer\n");
}

static void usb_closed(void)
{
	/* Frames produced while the device is not configured are discarded. */
	reset_stream();
	feed_info(0, 0x80 | 30, 200);
	host_usb_configured = false;
	feed_quat(0, 0, 0, 0, 32767);
	assert(captured_len == 0);

	/* Re-enumerating resumes the stream with the cached level. */
	host_usb_configured = true;
	feed_quat(0, 0, 0, 0, 32767);
	assert(frame_count() == 1 && frame_at(0)[2] == 30);
	printf("PASS usb-closed\n");
}

int main(int argc, char **argv)
{
	assert(argc == 2);
	if (!strcmp(argv[1], "layout")) {
		layout();
	} else if (!strcmp(argv[1], "quaternion")) {
		quaternion();
	} else if (!strcmp(argv[1], "battery")) {
		battery();
	} else if (!strcmp(argv[1], "mag")) {
		mag();
	} else if (!strcmp(argv[1], "packed")) {
		packed();
	} else if (!strcmp(argv[1], "drop")) {
		drop();
	} else if (!strcmp(argv[1], "buffer")) {
		buffer();
	} else if (!strcmp(argv[1], "usb-closed")) {
		usb_closed();
	} else {
		abort();
	}
	return 0;
}
