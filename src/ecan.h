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

/* ECAN-ECBT-16B serial posture stream.
 *
 * Emits fixed 16-byte frames on the dedicated CDC ACM (cdc_acm_uart1):
 *
 *   Layout per ECAN-ECBT-16B.md: 0x5A, 0x10, Batt, DevID,
 *   W/X/Y/Z @ bytes 4..11 (int16 BE), MagDist @12, Rsvd, 0xA5.
 *
 * Receiver dongle maps Slime ESB packets to this wire format:
 *   - DevID = pairing order (tracker_id).
 *   - Batt/temp from Slime type 0/2; MagDist=1 if Slime temp byte decodes to °C<0
 *     (°C = raw/2 - 39, same as slimehid2ykwxyz; raw in 1..77).
 *   - Quat from type 1/4 (int16 LE Q15 x,y,z,w) or type 2/7 compressed;
 *     converted to (w,x,y,z) float then int16 BE with ECAN_QUAT_SCALE_FACTOR.
 */
#define ECAN_ECBT_16B_FRAME_HEAD  0x5A
#define ECAN_ECBT_16B_FRAME_TAIL  0xA5
#define ECAN_ECBT_16B_FRAME_TYPE  0x10
#define ECAN_ECBT_16B_FRAME_LEN   16

#define ECAN_QUAT_SCALE_FACTOR 32767
#define SLIME_QUAT_SOURCE_SCALE 32768

int ecan_init(void);

/* Process a Slime packet from the ESB forward path.
 * data: Slime 16-byte packet (data[0] = type, data[1] = tracker_id).
 * rssi: kept for API compatibility with the former hid_write_packet_n;
 *       not encoded in ECAN frames. */
void ecan_handle_packet(const uint8_t *data, uint8_t rssi);

/* Clear one tracker's battery/temperature cache (e.g. on unpair). */
void ecan_reset_tracker(uint8_t tracker_id);

/* Clear the cache for all trackers (replaces hid_reset_all_rssi_smooth). */
void ecan_reset_all(void);

#endif /* SLIMENRF_ECAN */