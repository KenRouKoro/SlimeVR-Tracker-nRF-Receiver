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
#ifndef SLIMENRF_HID
#define SLIMENRF_HID

#include <stdint.h>

/**
 * @brief Anomaly detection modes for motion tracking
 */
typedef enum {
	HID_ANOMALY_ORIGINAL = 0,    /**< Exact replica of original simple algorithm before modifications */
	HID_ANOMALY_BALANCED = 1,  /**< Recommended balance with fixed algorithm (default) */
	HID_ANOMALY_DISABLED = 2   /**< No anomaly detection */
} hid_anomaly_mode_t;

/**
 * @brief Process and queue HID packet data
 * @param data Packet data (16 bytes)
 * @param rssi Radio signal strength indicator
 */
void hid_write_packet_n(uint8_t* data, uint8_t rssi);

#ifndef CONFIG_SOC_NRF52820
/**
 * @brief Set the anomaly detection mode
 * @param mode Detection mode (0-2, see hid_anomaly_mode_t)
 */
void hid_set_anomaly_detection_mode(int mode);

/**
 * @brief Get the current anomaly detection mode
 * @return Current mode (0-2)
 */
int hid_get_anomaly_detection_mode(void);

#endif

#endif
