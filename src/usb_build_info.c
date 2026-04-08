/*
 * SlimeVR Code is placed under the MIT license
 * Copyright (c) 2025 SlimeVR Contributors
 */
#include "usb_build_info.h"

#include <app_version.h>
#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_ch9.h>

#if IS_ENABLED(CONFIG_SLIMENRF_USB_BUILD_INFO_HID) && IS_ENABLED(CONFIG_USB_DEVICE_STACK)

LOG_MODULE_REGISTER(usb_build_info, LOG_LEVEL_INF);

#ifndef FW_GIT_HASH_SHORT
#define FW_GIT_HASH_SHORT "000000000000"
#endif
#ifndef FW_GIT_DIRTY
#define FW_GIT_DIRTY 0
#endif
#ifndef FW_BUILD_UNIX_TIME
#define FW_BUILD_UNIX_TIME 0U
#endif
#ifndef FW_VERSION_TWEAK
#define FW_VERSION_TWEAK 0
#endif

/** HID 1.11 GET_REPORT wValue high byte: Feature */
#define HID_REPORT_TYPE_FEATURE_LEGACY 0x03U

static struct usb_build_info_v1 build_info_v1 = {
	.schema_version = 1,
	.version_major = (uint8_t)APP_VERSION_MAJOR,
	.version_minor = (uint8_t)APP_VERSION_MINOR,
	.version_patch = (uint8_t)APP_PATCHLEVEL,
	.version_tweak = (uint16_t)FW_VERSION_TWEAK,
	.flags = (FW_GIT_DIRTY) ? SLIMENRF_BUILD_INFO_F_DIRTY : 0U,
	.build_unix_time = (uint32_t)FW_BUILD_UNIX_TIME,
};

static const uint8_t slime_usb_build_info_report_desc[] = {
	HID_USAGE_PAGE(0xFF),
	HID_USAGE(0x01),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
	HID_REPORT_ID(SLIMENRF_USB_BUILD_INFO_REPORT_ID),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(SLIMENRF_USB_BUILD_INFO_PAYLOAD_SIZE),
	HID_FEATURE(0x02),
	HID_END_COLLECTION,
};

static int slime_usb_build_info_get_report(const struct device *dev, struct usb_setup_packet *setup,
					   int32_t *len, uint8_t **data)
{
	ARG_UNUSED(dev);

	const uint8_t report_type = (uint8_t)(setup->wValue >> 8);
	const uint8_t report_id = (uint8_t)(setup->wValue & 0xFF);

	if (report_type != HID_REPORT_TYPE_FEATURE_LEGACY) {
		return -ENOTSUP;
	}
	if (report_id != SLIMENRF_USB_BUILD_INFO_REPORT_ID) {
		return -ENOTSUP;
	}
	if (setup->wLength == 0U) {
		return -EINVAL;
	}

	static uint8_t feature_report[1U + sizeof(struct usb_build_info_v1)];
	static bool logged_request;

	feature_report[0] = SLIMENRF_USB_BUILD_INFO_REPORT_ID;
	memcpy(&feature_report[1], &build_info_v1, sizeof(build_info_v1));

	const size_t total = sizeof(feature_report);
	const uint16_t req = setup->wLength;

	*len = (int32_t)MIN((size_t)req, total);
	*data = feature_report;

	if (!logged_request) {
		logged_request = true;
		LOG_INF("Build info GET_REPORT: wLength=%u len=%d", setup->wLength, *len);
	}

	return 0;
}

static const struct hid_ops build_info_hid_ops = {
	.get_report = slime_usb_build_info_get_report,
};

static int slime_usb_build_info_init(void)
{
	const struct device *hid_dev = device_get_binding("HID_1");

	if (hid_dev == NULL) {
		LOG_ERR("Cannot get USB HID_1 (check CONFIG_USB_HID_DEVICE_COUNT>=2)");
		return -ENODEV;
	}

	usb_hid_register_device(hid_dev, slime_usb_build_info_report_desc,
				sizeof(slime_usb_build_info_report_desc), &build_info_hid_ops);

	if (usb_hid_set_proto_code(hid_dev, HID_BOOT_IFACE_CODE_NONE)) {
		LOG_WRN("Failed to set Protocol Code on build-info HID");
	}

	return usb_hid_init(hid_dev);
}

static void slime_usb_build_info_fill_git_hash(void)
{
	static const char hash_src[] = FW_GIT_HASH_SHORT;
	const size_t n = MIN(sizeof(build_info_v1.git_hash), sizeof(hash_src) - 1U);

	memcpy(build_info_v1.git_hash, hash_src, n);
}

SYS_INIT(slime_usb_build_info_fill_git_hash, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
SYS_INIT(slime_usb_build_info_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

#endif /* CONFIG_SLIMENRF_USB_BUILD_INFO_HID && CONFIG_USB_DEVICE_STACK */
