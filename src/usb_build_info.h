/*
 * SlimeVR Code is placed under the MIT license
 * Copyright (c) 2025 SlimeVR Contributors
 *
 * USB HID Feature report: firmware build metadata (see docs/usb-build-info-hid.md).
 */
#ifndef USB_BUILD_INFO_H
#define USB_BUILD_INFO_H

#include <stdint.h>

/** Report ID for GET_REPORT (Feature) build-info payload. */
#define SLIMENRF_USB_BUILD_INFO_REPORT_ID 1U

/** Size of the binary payload after Report ID (struct usb_build_info_v1). */
#define SLIMENRF_USB_BUILD_INFO_PAYLOAD_SIZE 24U

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
