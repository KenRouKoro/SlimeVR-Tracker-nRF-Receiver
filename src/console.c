/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "system/system.h"
#include "build_defines.h"
#include "console_cmd.h"

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay)

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void console_thread(void);
K_THREAD_DEFINE(console_thread_id, 1024, console_thread, NULL, NULL, NULL, 6, 0, 0);

static void console_thread(void)
{
#if DFU_EXISTS
	if (button_read()) {
#if ADAFRUIT_BOOTLOADER
		console_cmd_request_local_dfu(false);
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
		k_msleep(100);
		sys_reboot(SYS_REBOOT_COLD);
#endif
	}
#endif

	console_getline_init();

	while (log_data_pending()) {
		k_usleep(1);
	}

	const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (device_is_ready(uart_dev)) {
		uint32_t dtr = 0;
		for (int i = 0; i < 50; i++) {
			if (uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr) {
				break;
			}
			k_msleep(100);
		}
		k_msleep(100);
	}

	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s | Author: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH, FW_GIT_AUTHOR);

	console_cmd_print_help();

	while (1) {
		uint8_t *line = console_getline();

		console_cmd_execute_line((char *)line, NULL, NULL);
	}
}

#endif
