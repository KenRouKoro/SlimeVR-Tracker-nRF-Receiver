/*
 * SlimeVR Code is placed under the MIT license
 * Copyright (c) 2025 SlimeVR Contributors
 *
 * Shared console command execution (USB CDC and HID_1 command channel).
 */
#ifndef CONSOLE_CMD_H
#define CONSOLE_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Max message length including terminating NUL (matches HID status payload + margin). */
#define CONSOLE_CMD_RESULT_MSG_LEN 64

enum console_cmd_status {
	CONSOLE_CMD_STATUS_OK = 0,
	CONSOLE_CMD_STATUS_UNKNOWN = 1,
	CONSOLE_CMD_STATUS_BAD_ARGS = 2,
	CONSOLE_CMD_STATUS_TOO_LONG = 3,
	CONSOLE_CMD_STATUS_BUSY = 4,
	CONSOLE_CMD_STATUS_INTERNAL = 5,
};

struct console_cmd_result {
	int status_code;
	bool accepted;
	char message[CONSOLE_CMD_RESULT_MSG_LEN];
};

/**
 * Optional buffer filled during command execution with the same text as printk (CDC + long HID replies).
 * @a buf must remain valid for the whole execute_line call; @a cap includes space for a trailing NUL.
 */
struct console_cmd_capture {
	char *buf;
	size_t cap;
	size_t len;
	bool truncated;
};

/**
 * Print formatted line to printk and, when a capture context is active (HID path), append to its buffer.
 * Also used by helpers such as RSSI scan so HID can collect full multi-line output.
 */
void console_cmd_print(const char *fmt, ...);

/**
 * Execute one line of console input (same semantics as USB CDC console).
 * @param line Mutable buffer; will be lowercased and tokenized in place.
 * @param result Optional; if non-NULL, last status line is written for HID / tooling.
 * @param capture Optional; if non-NULL, full printk-equivalent output is accumulated (RAM-limited, see truncated).
 */
void console_cmd_execute_line(char *line, struct console_cmd_result *result, struct console_cmd_capture *capture);

/** Short English name for @a status_code (HID JSON `status.name` / tooling). */
const char *console_cmd_status_name(int status_code);

/** Print full help text to printk (used by CDC startup and "help" command). */
void console_cmd_print_help(void);

/** Enter bootloader DFU (same as `dfu` console command). */
void console_cmd_request_local_dfu(bool ota);

#endif /* CONSOLE_CMD_H */
