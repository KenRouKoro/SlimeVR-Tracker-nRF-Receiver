/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

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
#ifndef SLIMENRF_ESB
#define SLIMENRF_ESB

#include <esb.h>
#include <stdint.h>

// Ping/Pong constants (shared protocol)
#define ESB_PING_TYPE 0xF0
#define ESB_PONG_TYPE 0xF1
#define ESB_PING_LEN 13
#define ESB_PONG_LEN 13

// Remote command flags for PONG data[7]
#define ESB_PONG_FLAG_NORMAL 0x00
#define ESB_PONG_FLAG_SHUTDOWN 0x01
#define ESB_PONG_FLAG_CALIBRATE 0x02        // Trigger gyro/accel ZRO calibration
#define ESB_PONG_FLAG_SIX_SIDE_CAL 0x03     // Trigger 6-point accelerometer calibration
#define ESB_PONG_FLAG_MEOW 0x04             // Trigger meow output
#define ESB_PONG_FLAG_SCAN 0x05             // Trigger sensor scan
#define ESB_PONG_FLAG_MAG_CLEAR 0x06        // Clear magnetometer calibration
// Reserved for future use: 0x07-0xFF

void event_handler(struct esb_evt const* event);
int clocks_start(void);
int esb_initialize(bool);

void esb_set_addr_discovery(void);
void esb_set_addr_paired(void);

int esb_add_pair(uint64_t addr, bool checksum);
void esb_pop_pair(void);

void esb_pair(void);
void esb_start_pairing(void);
void esb_reset_pair(void);
void esb_finish_pair(void);
void esb_clear(void);
void esb_reset_tracker_sequence(uint8_t tracker_id);
void esb_print_all_stats(void);
void esb_reset_all_stats(void);
void esb_write_sync(uint16_t led_clock);
void esb_receive(void);

// Remote command API
void esb_send_remote_command(uint8_t tracker_id, uint8_t command_flag);
void esb_send_remote_command_all(uint8_t command_flag);

// Convenience wrappers for specific commands
static inline void esb_request_tracker_shutdown(uint8_t tracker_id) {
	esb_send_remote_command(tracker_id, ESB_PONG_FLAG_SHUTDOWN);
}

static inline void esb_request_all_shutdown(void) {
	esb_send_remote_command_all(ESB_PONG_FLAG_SHUTDOWN);
}

#endif
