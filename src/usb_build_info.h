/*
 * SlimeVR Code is placed under the MIT license
 * Copyright (c) 2025 SlimeVR Contributors
 *
 * USB HID Feature report: firmware build metadata (see docs/usb-build-info-hid.md).
 * HID_1 Report ID 2/3: JSON over chunked interrupt OUT/IN (see docs/usb-hid-host-integration.zh.md).
 */
#ifndef USB_BUILD_INFO_H
#define USB_BUILD_INFO_H

#include <stdint.h>

/** Report ID for GET_REPORT (Feature) build-info payload. */
#define SLIMENRF_USB_BUILD_INFO_REPORT_ID 1U

/** HID interrupt OUT: host sends JSON request fragments (see struct slime_hid_cmd_out_report). */
#define SLIMENRF_HID_CMD_OUT_REPORT_ID 2U

/** HID interrupt IN: device returns JSON response fragments (see struct slime_hid_cmd_status_report). */
#define SLIMENRF_HID_CMD_STATUS_REPORT_ID 3U

/** Bytes after Report ID in one interrupt MPS (CONFIG_HID_INTERRUPT_EP_MPS, typically 64). */
#define SLIMENRF_HID_CMD_MPS_PAYLOAD 63U

/** UTF-8 payload bytes per OUT/IN chunk (after 8-byte transport header under Report ID). */
#define SLIMENRF_HID_CHUNK_PAYLOAD_MAX 56U

/** Max assembled JSON request size (bytes, excluding terminating NUL used internally). */
#define SLIMENRF_HID_JSON_REQ_MAX_LEN 2048U

/** Buffer for one encoded JSON response (see usb_build_info.c). */
#define SLIMENRF_HID_JSON_RESP_MAX_LEN 8192U

/** Max bytes collected for one console capture during HID JSON `console.exec` (including trailing NUL). */
#define SLIMENRF_HID_CMD_CAPTURE_MAX 4096U

/** Max UTF-8 bytes encodable in one HID JSON response (limited by uint8_t chunk_total × chunk payload). */
#define SLIMENRF_HID_JSON_RESP_WIRE_MAX ((size_t)255U * (size_t)SLIMENRF_HID_CHUNK_PAYLOAD_MAX)

/** Legacy alias: same as SLIMENRF_HID_CHUNK_PAYLOAD_MAX. */
#define SLIMENRF_HID_R3_PAYLOAD_MAX SLIMENRF_HID_CHUNK_PAYLOAD_MAX

/** Report ID 3 @a flags bits (device → host). */
#define SLIMENRF_HID_R3_F_FIRST (1U << 0)
#define SLIMENRF_HID_R3_F_LAST (1U << 1)
/** Deprecated for JSON protocol: success responses do not truncate; kept for ABI compatibility. */
#define SLIMENRF_HID_R3_F_TRUNCATED (1U << 2)

/** IN @a status_code: JSON payload in frames (protocol v2). */
#define SLIMENRF_HID_WIRE_STATUS_JSON 0U

/** JSON envelope: request. */
#define SLIMENRF_HID_JSON_TYPE_REQUEST "request"
/** JSON envelope: response. */
#define SLIMENRF_HID_JSON_TYPE_RESPONSE "response"

/** JSON @a op: execute one CDC console line (see @a data.line). */
#define SLIMENRF_HID_JSON_OP_CONSOLE_EXEC "console.exec"

/** Protocol-level @a error.code strings (device → host). */
#define SLIMENRF_HID_JSON_ERR_BAD_JSON "bad_json"
#define SLIMENRF_HID_JSON_ERR_UNSUPPORTED_TYPE "unsupported_type"
#define SLIMENRF_HID_JSON_ERR_UNSUPPORTED_OP "unsupported_op"
#define SLIMENRF_HID_JSON_ERR_BAD_REQUEST "bad_request"
#define SLIMENRF_HID_JSON_ERR_MESSAGE_TOO_LARGE "message_too_large"
#define SLIMENRF_HID_JSON_ERR_CAPTURE_OVERFLOW "capture_overflow"
#define SLIMENRF_HID_JSON_ERR_RESPONSE_TOO_LARGE "response_too_large"
#define SLIMENRF_HID_JSON_ERR_BUSY "busy"
#define SLIMENRF_HID_JSON_ERR_QUEUE_FULL "queue_full"
#define SLIMENRF_HID_JSON_ERR_INTERNAL "internal_error"

/** Size of the binary payload after Report ID (struct usb_build_info_v1). */
#define SLIMENRF_USB_BUILD_INFO_PAYLOAD_SIZE 24U

/**
 * Wire format for HID_1 interrupt OUT (host -> device).
 * Total USB buffer = 1 byte Report ID + 63 bytes below = 64 bytes (typical MPS).
 * JSON request is split across frames using the same chunking rules as IN.
 */
struct slime_hid_cmd_out_report {
	uint8_t report_id;
	uint8_t seq;
	uint8_t reserved0;
	uint8_t flags;
	uint8_t chunk_idx;
	uint8_t chunk_total;
	uint8_t payload_len;
	uint8_t reserved1;
	char payload[SLIMENRF_HID_CHUNK_PAYLOAD_MAX];
} __attribute__((packed));

/**
 * Wire format for HID_1 interrupt IN (device -> host).
 * One logical JSON response may span multiple 64-byte reports with the same @a seq.
 */
struct slime_hid_cmd_status_report {
	uint8_t report_id;
	uint8_t seq;
	uint8_t status_code;
	uint8_t flags;
	uint8_t chunk_idx;
	uint8_t chunk_total;
	uint8_t payload_len;
	uint8_t reserved;
	char payload[SLIMENRF_HID_CHUNK_PAYLOAD_MAX];
} __attribute__((packed));

/** bit0: working tree had uncommitted changes at build time (git describe --dirty). */
#define SLIMENRF_BUILD_INFO_F_DIRTY 0x0001U

/**
 * Binary layout for HID Feature payload (little-endian multi-byte fields).
 * Host must match schema_version before interpreting remaining fields.
 */
struct usb_build_info_v1 {
	uint8_t schema_version;
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t version_patch;
	uint16_t version_tweak;
	uint16_t flags;
	uint32_t build_unix_time;
	char git_hash[12];
} __attribute__((packed));

#endif /* USB_BUILD_INFO_H */
