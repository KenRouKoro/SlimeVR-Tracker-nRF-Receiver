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
#ifndef SLIMENRF_ECAN
#define SLIMENRF_ECAN

#include <stdint.h>

/* ECAN-ECBT-16B serial posture stream (see ECAN-ECBT-16B.md).
 *
 * Emits fixed 16-byte frames on the dedicated CDC ACM port
 * (DT_NODELABEL(cdc_acm_uart1), wired up by boards/ecan_cdc.overlay):
 *
 *   0x5A, 0x10, Batt, DevID, W/X/Y/Z @ 4..11 (int16 BE), MagDist @ 12,
 *   reserved, 0xA5.
 *
 * Tracker -> frame mapping:
 *   - DevID   = pairing slot (tracker_id).
 *   - Batt    = Slime type 0/2 B2: 0 = no battery, 0x80|percent = available,
 *               0xFF = charged; see ecan_batt_to_percent().
 *   - MagDist = 1 while Slime type 0/2 B4 (temperature) decodes below 0 degC,
 *               which the tracker firmware uses to flag magnetic disturbance
 *               (temperature is inverted when the fusion detects it).
 *   - Quat    = type 1/4 (int16 LE Q15 x,y,z,w) or type 2/7 (packed 10/11/11
 *               bit half-angle), rescaled to W,X,Y,Z int16 BE with
 *               ECAN_QUAT_SCALE_FACTOR.
 *
 * Enabled by CONFIG_SLIMEVR_ECAN_STREAM; the pose sink in esb.c then feeds
 * this module instead of the HID report path.
 */
#define ECAN_ECBT_16B_FRAME_HEAD 0x5A
#define ECAN_ECBT_16B_FRAME_TAIL 0xA5
#define ECAN_ECBT_16B_FRAME_TYPE 0x10
#define ECAN_ECBT_16B_FRAME_LEN 16

/* Wire scale of the ECAN quaternions; Slime full-precision quats are Q15. */
#define ECAN_QUAT_SCALE_FACTOR 32767
#define SLIME_QUAT_SOURCE_SCALE 32768

/* Queue one Slime 16-byte ESB packet (data[0] = type, data[1] = tracker id)
 * for framing. Safe to call from the ESB event IRQ; framing itself runs in
 * workqueue context (the quaternion decoding needs the FPU). */
void ecan_handle_packet(const uint8_t *data, uint8_t rssi);

/* Drop one tracker's cached battery/temperature (e.g. on unpair). */
void ecan_reset_tracker(uint8_t tracker_id);

/* Drop the cache for every tracker (replaces hid_reset_all_rssi_smooth). */
void ecan_reset_all(void);

/* Frame rate and drop counters for the console health snapshot. */
uint32_t ecan_get_current_tps(void);
uint32_t ecan_get_total_drop_count(void);
uint32_t ecan_get_total_tracker_drop_count(uint8_t tracker_id);

#endif /* SLIMENRF_ECAN */
