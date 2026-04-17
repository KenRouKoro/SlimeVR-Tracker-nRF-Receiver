/*
 * SlimeVR Code is placed under the MIT license
 * Copyright (c) 2025 SlimeVR Contributors
 */
#include "usb_build_info.h"

#include "console_cmd.h"

#include <app_version.h>
#include <errno.h>
#include <string.h>

#include <zephyr/data/json.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_ch9.h>

#if IS_ENABLED(CONFIG_SLIMENRF_USB_BUILD_INFO_HID) && IS_ENABLED(CONFIG_USB_DEVICE_STACK) &&                          \
	IS_ENABLED(CONFIG_ENABLE_HID_INT_OUT_EP) && IS_ENABLED(CONFIG_JSON_LIBRARY)

BUILD_ASSERT(sizeof(struct slime_hid_cmd_out_report) == 64U);
BUILD_ASSERT(sizeof(struct slime_hid_cmd_status_report) == 64U);

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

#define HID_REPORT_TYPE_FEATURE_LEGACY 0x03U

enum hid_json_work_kind {
	HID_JSON_WORK_REQ = 0,
	HID_JSON_WORK_ERR = 1,
};

enum hid_json_transport_detail {
	HID_JSON_TERR_PAYLOAD_LEN = 0,
	HID_JSON_TERR_CHUNK,
	HID_JSON_TERR_MISSING_FIRST,
	HID_JSON_TERR_CHUNK_IDX,
	HID_JSON_TERR_REASSEMBLY_BUSY,
	HID_JSON_TERR_CHUNK_TOTAL,
	HID_JSON_TERR_CHUNK_ORDER,
	HID_JSON_TERR_REQUEST_SIZE,
	HID_JSON_TERR_LAST_MISMATCH,
	HID_JSON_TERR_QUEUE_FULL,
	HID_JSON_TERR_REQ_SLOT_BUSY,
};

/** +1 for terminating NUL after @a len bytes of JSON body (len <= SLIMENRF_HID_JSON_REQ_MAX_LEN). */
#define SLIMENRF_HID_JSON_REQ_BUF_MAX (SLIMENRF_HID_JSON_REQ_MAX_LEN + 1U)

struct hid_json_work {
	uint8_t kind;
	uint8_t seq;
	uint16_t len;
	uint8_t detail;
};

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
	HID_LOGICAL_MIN8(0),
	HID_LOGICAL_MAX8(255),
	HID_USAGE_MIN8(1),
	HID_USAGE_MAX8(SLIMENRF_USB_BUILD_INFO_PAYLOAD_SIZE),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(SLIMENRF_USB_BUILD_INFO_PAYLOAD_SIZE),
	HID_FEATURE(0x02),
	HID_REPORT_ID(SLIMENRF_HID_CMD_OUT_REPORT_ID),
	HID_LOGICAL_MIN8(0),
	HID_LOGICAL_MAX8(255),
	HID_USAGE_MIN8(1),
	HID_USAGE_MAX8(SLIMENRF_HID_CMD_MPS_PAYLOAD),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(SLIMENRF_HID_CMD_MPS_PAYLOAD),
	HID_OUTPUT(0x02),
	HID_REPORT_ID(SLIMENRF_HID_CMD_STATUS_REPORT_ID),
	HID_LOGICAL_MIN8(0),
	HID_LOGICAL_MAX8(255),
	HID_USAGE_MIN8(1),
	HID_USAGE_MAX8(SLIMENRF_HID_CMD_MPS_PAYLOAD),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(SLIMENRF_HID_CMD_MPS_PAYLOAD),
	HID_INPUT(0x02),
	HID_END_COLLECTION,
};

K_MSGQ_DEFINE(hid_json_workq, sizeof(struct hid_json_work), 3, 4);

static const struct device *hid1_dev;

static char hid_cmd_capture_buf[SLIMENRF_HID_CMD_CAPTURE_MAX];
static struct slime_hid_cmd_status_report hid_status_tx_report;
static K_SEM_DEFINE(hid_status_tx_idle, 1, 1);

/*
 * OUT completion runs from USB endpoint callback (ISR-level context).
 * Never use k_mutex here — it is not allowed in ISR and will fault/reset the device.
 */
static struct k_spinlock rx_asm_lock;
static struct {
	bool active;
	uint8_t seq;
	uint8_t expect_chunk_idx;
	uint8_t chunk_total;
	uint16_t total_len;
	char buf[SLIMENRF_HID_JSON_REQ_BUF_MAX];
} rx_asm;

static struct {
	bool pending;
	uint8_t seq;
	uint16_t len;
	char json[SLIMENRF_HID_JSON_REQ_BUF_MAX];
} req_slot;

struct hid_json_req_data {
	const char *line;
};

struct hid_json_req {
	const char *type;
	const char *op;
	int32_t id;
	struct hid_json_req_data data;
};

static const struct json_obj_descr hid_json_descr_req_data[] = {
	JSON_OBJ_DESCR_PRIM(struct hid_json_req_data, line, JSON_TOK_STRING),
};

static const struct json_obj_descr hid_json_descr_req[] = {
	JSON_OBJ_DESCR_PRIM(struct hid_json_req, type, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hid_json_req, op, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hid_json_req, id, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_OBJECT(struct hid_json_req, data, hid_json_descr_req_data),
};

struct hid_json_resp_data {
	const char *output;
};

struct hid_json_resp_status {
	int32_t code;
	const char *name;
};

struct hid_json_resp_ok {
	const char *type;
	const char *op;
	int32_t id;
	bool ok;
	struct hid_json_resp_status status;
	struct hid_json_resp_data data;
};

static const struct json_obj_descr hid_json_descr_data_out[] = {
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_data, output, JSON_TOK_STRING),
};

static const struct json_obj_descr hid_json_descr_status[] = {
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_status, code, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_status, name, JSON_TOK_STRING),
};

static const struct json_obj_descr hid_json_descr_ok[] = {
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_ok, type, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_ok, op, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_ok, id, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_ok, ok, JSON_TOK_TRUE),
	JSON_OBJ_DESCR_OBJECT(struct hid_json_resp_ok, status, hid_json_descr_status),
	JSON_OBJ_DESCR_OBJECT(struct hid_json_resp_ok, data, hid_json_descr_data_out),
};

struct hid_json_err_obj {
	const char *code;
	const char *message;
};

struct hid_json_resp_err {
	const char *type;
	int32_t id;
	bool ok;
	struct hid_json_err_obj error;
};

static const struct json_obj_descr hid_json_descr_err_inner[] = {
	JSON_OBJ_DESCR_PRIM(struct hid_json_err_obj, code, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hid_json_err_obj, message, JSON_TOK_STRING),
};

static const struct json_obj_descr hid_json_descr_resp_err[] = {
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_err, type, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_err, id, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct hid_json_resp_err, ok, JSON_TOK_TRUE),
	JSON_OBJ_DESCR_OBJECT(struct hid_json_resp_err, error, hid_json_descr_err_inner),
};

static const char *hid_transport_error_code(enum hid_json_transport_detail detail)
{
	switch (detail) {
	case HID_JSON_TERR_REASSEMBLY_BUSY:
	case HID_JSON_TERR_REQ_SLOT_BUSY:
		return SLIMENRF_HID_JSON_ERR_BUSY;
	case HID_JSON_TERR_REQUEST_SIZE:
		return SLIMENRF_HID_JSON_ERR_MESSAGE_TOO_LARGE;
	case HID_JSON_TERR_QUEUE_FULL:
		return SLIMENRF_HID_JSON_ERR_QUEUE_FULL;
	default:
		return SLIMENRF_HID_JSON_ERR_BAD_REQUEST;
	}
}

static const char *hid_transport_error_message(enum hid_json_transport_detail detail)
{
	switch (detail) {
	case HID_JSON_TERR_PAYLOAD_LEN:
		return "payload_len";
	case HID_JSON_TERR_CHUNK:
		return "chunk";
	case HID_JSON_TERR_MISSING_FIRST:
		return "missing first";
	case HID_JSON_TERR_CHUNK_IDX:
		return "chunk_idx";
	case HID_JSON_TERR_REASSEMBLY_BUSY:
		return "reassembly busy";
	case HID_JSON_TERR_CHUNK_TOTAL:
		return "chunk_total";
	case HID_JSON_TERR_CHUNK_ORDER:
		return "chunk order";
	case HID_JSON_TERR_REQUEST_SIZE:
		return "request size";
	case HID_JSON_TERR_LAST_MISMATCH:
		return "last mismatch";
	case HID_JSON_TERR_QUEUE_FULL:
		return "queue full";
	case HID_JSON_TERR_REQ_SLOT_BUSY:
		return "request slot busy";
	default:
		return "bad request";
	}
}

static int hid_send_status_report(const struct slime_hid_cmd_status_report *st)
{
	for (int retry = 0; retry < 100; retry++) {
		if (k_sem_take(&hid_status_tx_idle, K_MSEC(10)) != 0) {
			continue;
		}

		memcpy(&hid_status_tx_report, st, sizeof(*st));

		uint32_t wr = 0U;
		int err = hid_int_ep_write(hid1_dev, (const uint8_t *)&hid_status_tx_report, sizeof(hid_status_tx_report),
					   &wr);

		if (err == 0) {
			return 0;
		}

		k_sem_give(&hid_status_tx_idle);
		k_msleep(1);
	}
	return -EIO;
}

static void hid_send_json_in(uint8_t seq, const char *json, size_t json_len)
{
	const size_t max_chunk = SLIMENRF_HID_CHUNK_PAYLOAD_MAX;

	if (json_len == 0U) {
		json = "";
	}

	size_t nchunks = (json_len + max_chunk - 1U) / max_chunk;

	if (nchunks == 0U) {
		nchunks = 1U;
	}
	if (nchunks > 255U) {
		return;
	}

	uint8_t chunk_total = (uint8_t)nchunks;

	for (uint8_t ci = 0; ci < chunk_total; ci++) {
		struct slime_hid_cmd_status_report st = {0};

		st.report_id = SLIMENRF_HID_CMD_STATUS_REPORT_ID;
		st.seq = seq;
		st.status_code = SLIMENRF_HID_WIRE_STATUS_JSON;
		st.chunk_idx = ci;
		st.chunk_total = chunk_total;

		size_t off = (size_t)ci * max_chunk;
		size_t remain = json_len > off ? (json_len - off) : 0U;
		size_t plen = remain > max_chunk ? max_chunk : remain;

		st.payload_len = (uint8_t)plen;
		if (plen > 0U) {
			memcpy(st.payload, json + off, plen);
		}
		if (plen < max_chunk) {
			(void)memset(st.payload + plen, 0, max_chunk - plen);
		}

		uint8_t flags = 0U;

		if (ci == 0U) {
			flags |= SLIMENRF_HID_R3_F_FIRST;
		}
		if (ci == chunk_total - 1U) {
			flags |= SLIMENRF_HID_R3_F_LAST;
		}
		st.flags = flags;

		(void)hid_send_status_report(&st);
	}
}

static void hid_queue_transport_error(uint8_t seq, enum hid_json_transport_detail detail)
{
	struct hid_json_work w = {
		.kind = HID_JSON_WORK_ERR,
		.seq = seq,
		.len = 0,
		.detail = (uint8_t)detail,
	};

	if (k_msgq_put(&hid_json_workq, &w, K_NO_WAIT) != 0) {
		LOG_WRN("HID_1 work queue full (transport err)");
	}
}

static void hid_queue_request(uint8_t seq, const char *json, uint16_t len)
{
	struct hid_json_work w = {
		.kind = HID_JSON_WORK_REQ,
		.seq = seq,
		.len = len,
	};

	k_spinlock_key_t key = k_spin_lock(&rx_asm_lock);

	if (req_slot.pending) {
		k_spin_unlock(&rx_asm_lock, key);
		hid_queue_transport_error(seq, HID_JSON_TERR_REQ_SLOT_BUSY);
		return;
	}

	if (len >= sizeof(req_slot.json)) {
		len = (uint16_t)(sizeof(req_slot.json) - 1U);
	}

	req_slot.pending = true;
	req_slot.seq = seq;
	req_slot.len = len;
	memcpy(req_slot.json, json, len);
	req_slot.json[len] = '\0';

	k_spin_unlock(&rx_asm_lock, key);

	if (k_msgq_put(&hid_json_workq, &w, K_NO_WAIT) != 0) {
		LOG_WRN("HID_1 work queue full");
		key = k_spin_lock(&rx_asm_lock);
		if (req_slot.pending && req_slot.seq == seq) {
			req_slot.pending = false;
			req_slot.len = 0U;
		}
		k_spin_unlock(&rx_asm_lock, key);
		hid_queue_transport_error(seq, HID_JSON_TERR_QUEUE_FULL);
	}
}

static void hid_send_error_json(uint8_t seq, const char *code, const char *message)
{
	static char err_buf[512];
	struct hid_json_resp_err err = {
		.type = SLIMENRF_HID_JSON_TYPE_RESPONSE,
		.id = (int32_t)seq,
		.ok = false,
		.error = {
			.code = code,
			.message = message,
		},
	};

	int enc = json_obj_encode_buf(hid_json_descr_resp_err, ARRAY_SIZE(hid_json_descr_resp_err), &err, err_buf,
				      sizeof(err_buf));

	if (enc != 0) {
		LOG_ERR("HID JSON error encode failed: %d", enc);
		return;
	}

	/* Prefer calc length over strlen: buffer may retain stale bytes past JSON if not fully overwritten. */
	ssize_t elen = json_calc_encoded_len(hid_json_descr_resp_err, ARRAY_SIZE(hid_json_descr_resp_err), &err);

	if (elen < 0) {
		return;
	}
	hid_send_json_in(seq, err_buf, (size_t)elen);
}

/** Caller must hold @ref rx_asm_lock. */
static void rx_asm_reset(void)
{
	rx_asm.active = false;
	rx_asm.total_len = 0U;
}

static void hid_cmd_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	static char json_resp_buf[SLIMENRF_HID_JSON_RESP_MAX_LEN];
	static char req_work_json[SLIMENRF_HID_JSON_REQ_BUF_MAX];
	static char line_work[SLIMENRF_HID_JSON_REQ_MAX_LEN];

	struct hid_json_work w;

	for (;;) {
		k_msgq_get(&hid_json_workq, &w, K_FOREVER);

		if (w.kind == HID_JSON_WORK_ERR) {
			hid_send_error_json(w.seq,
					    hid_transport_error_code((enum hid_json_transport_detail)w.detail),
					    hid_transport_error_message((enum hid_json_transport_detail)w.detail));
			continue;
		}

		k_spinlock_key_t key = k_spin_lock(&rx_asm_lock);

		if (!req_slot.pending || req_slot.seq != w.seq || req_slot.len != w.len) {
			k_spin_unlock(&rx_asm_lock, key);
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_INTERNAL, "request slot mismatch");
			continue;
		}

		memcpy(req_work_json, req_slot.json, req_slot.len + 1U);
		req_slot.pending = false;
		req_slot.len = 0U;

		k_spin_unlock(&rx_asm_lock, key);

		struct hid_json_req req = {0};
		int64_t pr = json_obj_parse(req_work_json, w.len, hid_json_descr_req, ARRAY_SIZE(hid_json_descr_req), &req);

		if (pr < 0) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_BAD_JSON, "parse failed");
			continue;
		}

		if (req.type == NULL || req.op == NULL || req.data.line == NULL) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_BAD_JSON, "missing field");
			continue;
		}

		if (strcmp(req.type, SLIMENRF_HID_JSON_TYPE_REQUEST) != 0) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_UNSUPPORTED_TYPE, req.type);
			continue;
		}

		if (req.id != (int32_t)w.seq) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_BAD_REQUEST, "id mismatch");
			continue;
		}

		if (strcmp(req.op, SLIMENRF_HID_JSON_OP_CONSOLE_EXEC) != 0) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_UNSUPPORTED_OP, req.op);
			continue;
		}

		size_t line_len = strlen(req.data.line);

		if (line_len >= sizeof(line_work)) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_BAD_REQUEST, "line too long");
			continue;
		}

		memcpy(line_work, req.data.line, line_len);
		line_work[line_len] = '\0';

		struct console_cmd_result res = {0};
		struct console_cmd_capture cap = {
			.buf = hid_cmd_capture_buf,
			.cap = sizeof(hid_cmd_capture_buf),
		};

		console_cmd_execute_line(line_work, &res, &cap);

		if (cap.truncated) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_CAPTURE_OVERFLOW, "capture overflow");
			continue;
		}

		struct hid_json_resp_ok ok = {
			.type = SLIMENRF_HID_JSON_TYPE_RESPONSE,
			.op = SLIMENRF_HID_JSON_OP_CONSOLE_EXEC,
			.id = (int32_t)w.seq,
			.ok = true,
			.status = {
				.code = (int32_t)res.status_code,
				.name = console_cmd_status_name(res.status_code),
			},
			.data = {
				.output = cap.buf ? cap.buf : "",
			},
		};

		(void)memset(json_resp_buf, 0, sizeof(json_resp_buf));

		ssize_t need = json_calc_encoded_len(hid_json_descr_ok, ARRAY_SIZE(hid_json_descr_ok), &ok);

		if (need < 0 || (size_t)need >= sizeof(json_resp_buf)) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_RESPONSE_TOO_LARGE, "encoded response too large");
			continue;
		}

		int enc = json_obj_encode_buf(hid_json_descr_ok, ARRAY_SIZE(hid_json_descr_ok), &ok, json_resp_buf,
					      sizeof(json_resp_buf));

		if (enc != 0) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_INTERNAL, "encode failed");
			continue;
		}

		/*
		 * Wire length must match what encode actually wrote. json_calc_encoded_len() can disagree
		 * with json_obj_encode_buf() in some cases; using need as out_len is unsafe: if need is
		 * larger than the encoded string, json_resp_buf[need] may still be '\0' from memset,
		 * the mismatch branch never runs, and HID sends trailing garbage (often a 't' from
		 * stale buffer or adjacent content). Buffer is zeroed before encode, so strlen is the
		 * authoritative wire length (JSON body contains no embedded NUL from this encoder).
		 */
		size_t out_len = strlen(json_resp_buf);

		if (out_len != (size_t)need) {
			LOG_WRN("HID JSON len calc=%zd wire=%zu (using wire)", need, out_len);
		}

		if (out_len > SLIMENRF_HID_JSON_RESP_WIRE_MAX) {
			hid_send_error_json(w.seq, SLIMENRF_HID_JSON_ERR_RESPONSE_TOO_LARGE, "wire limit");
			continue;
		}

		hid_send_json_in(w.seq, json_resp_buf, out_len);
	}
}

K_THREAD_DEFINE(hid_cmd_thread_id, 4096, hid_cmd_thread, NULL, NULL, NULL, 7, 0, 0);

static void build_info_int_out_ready(const struct device *dev)
{
	uint8_t buf[CONFIG_HID_INTERRUPT_EP_MPS];
	uint32_t br = 0U;
	int err = hid_int_ep_read(dev, buf, sizeof(buf), &br);

	if (err != 0) {
		return;
	}
	if (br < sizeof(struct slime_hid_cmd_out_report)) {
		return;
	}

	struct slime_hid_cmd_out_report in;

	memcpy(&in, buf, sizeof(in));

	if (in.report_id != SLIMENRF_HID_CMD_OUT_REPORT_ID) {
		return;
	}

	/*
	 * Staging for one assembled request: copy out under spinlock, then queue after unlock
	 * (do not call k_msgq_put with spinlock held; do not use k_mutex in this callback — ISR context).
	 */
	static char staged_req_json[SLIMENRF_HID_JSON_REQ_BUF_MAX];
	uint8_t staged_seq = 0U;
	uint16_t staged_len = 0U;
	bool fire_queue = false;

	k_spinlock_key_t key = k_spin_lock(&rx_asm_lock);

	if (in.payload_len > SLIMENRF_HID_CHUNK_PAYLOAD_MAX) {
		rx_asm_reset();
		k_spin_unlock(&rx_asm_lock, key);
		hid_queue_transport_error(in.seq, HID_JSON_TERR_PAYLOAD_LEN);
		return;
	}
	if (in.chunk_total == 0U || in.chunk_idx >= in.chunk_total) {
		rx_asm_reset();
		k_spin_unlock(&rx_asm_lock, key);
		hid_queue_transport_error(in.seq, HID_JSON_TERR_CHUNK);
		return;
	}

	if (!rx_asm.active) {
		if ((in.flags & SLIMENRF_HID_R3_F_FIRST) == 0U) {
			k_spin_unlock(&rx_asm_lock, key);
			hid_queue_transport_error(in.seq, HID_JSON_TERR_MISSING_FIRST);
			return;
		}
		if (in.chunk_idx != 0U) {
			k_spin_unlock(&rx_asm_lock, key);
			hid_queue_transport_error(in.seq, HID_JSON_TERR_CHUNK_IDX);
			return;
		}

		rx_asm.active = true;
		rx_asm.seq = in.seq;
		rx_asm.chunk_total = in.chunk_total;
		rx_asm.expect_chunk_idx = 0U;
		rx_asm.total_len = 0U;
	} else if (in.seq != rx_asm.seq) {
		k_spin_unlock(&rx_asm_lock, key);
		hid_queue_transport_error(in.seq, HID_JSON_TERR_REASSEMBLY_BUSY);
		return;
	} else if (in.chunk_total != rx_asm.chunk_total) {
		rx_asm_reset();
		k_spin_unlock(&rx_asm_lock, key);
		hid_queue_transport_error(in.seq, HID_JSON_TERR_CHUNK_TOTAL);
		return;
	}

	if (in.chunk_idx != rx_asm.expect_chunk_idx) {
		rx_asm_reset();
		k_spin_unlock(&rx_asm_lock, key);
		hid_queue_transport_error(in.seq, HID_JSON_TERR_CHUNK_ORDER);
		return;
	}

	if (rx_asm.total_len + in.payload_len > SLIMENRF_HID_JSON_REQ_MAX_LEN) {
		/* body max SLIMENRF_HID_JSON_REQ_MAX_LEN; buf has +1 for NUL */
		rx_asm_reset();
		k_spin_unlock(&rx_asm_lock, key);
		hid_queue_transport_error(in.seq, HID_JSON_TERR_REQUEST_SIZE);
		return;
	}

	if (in.payload_len > 0U) {
		memcpy(rx_asm.buf + rx_asm.total_len, in.payload, in.payload_len);
		rx_asm.total_len += in.payload_len;
	}

	rx_asm.expect_chunk_idx++;

	bool is_last = ((in.flags & SLIMENRF_HID_R3_F_LAST) != 0U);

	if (is_last) {
		if (in.chunk_idx != in.chunk_total - 1U) {
			rx_asm_reset();
			k_spin_unlock(&rx_asm_lock, key);
			hid_queue_transport_error(in.seq, HID_JSON_TERR_LAST_MISMATCH);
			return;
		}

		rx_asm.buf[rx_asm.total_len] = '\0';
		memcpy(staged_req_json, rx_asm.buf, rx_asm.total_len + 1U);
		staged_seq = rx_asm.seq;
		staged_len = rx_asm.total_len;
		fire_queue = true;
		rx_asm_reset();
	}

	k_spin_unlock(&rx_asm_lock, key);

	if (fire_queue) {
		hid_queue_request(staged_seq, staged_req_json, staged_len);
	}
}

static void build_info_int_in_ready(const struct device *dev)
{
	ARG_UNUSED(dev);

	k_sem_give(&hid_status_tx_idle);
}

static int slime_usb_build_info_get_report(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
					   uint8_t **data)
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
	.int_in_ready = build_info_int_in_ready,
	.int_out_ready = build_info_int_out_ready,
};

void slime_usb_hid_cmd_usb_configured(void)
{
	if (hid1_dev == NULL) {
		return;
	}

	static uint8_t prime_buf[CONFIG_HID_INTERRUPT_EP_MPS];

	uint32_t br = 0U;

	(void)hid_int_ep_read(hid1_dev, prime_buf, sizeof(prime_buf), &br);
}

static int slime_usb_build_info_init(void)
{
	hid1_dev = device_get_binding("HID_1");

	if (hid1_dev == NULL) {
		LOG_ERR("Cannot get USB HID_1 (check CONFIG_USB_HID_DEVICE_COUNT>=2)");
		return -ENODEV;
	}

	usb_hid_register_device(hid1_dev, slime_usb_build_info_report_desc, sizeof(slime_usb_build_info_report_desc),
				&build_info_hid_ops);

	if (usb_hid_set_proto_code(hid1_dev, HID_BOOT_IFACE_CODE_NONE)) {
		LOG_WRN("Failed to set Protocol Code on build-info HID");
	}

	return usb_hid_init(hid1_dev);
}

static void slime_usb_build_info_fill_git_hash(void)
{
	static const char hash_src[] = FW_GIT_HASH_SHORT;
	const size_t n = MIN(sizeof(build_info_v1.git_hash), sizeof(hash_src) - 1U);

	memcpy(build_info_v1.git_hash, hash_src, n);
}

SYS_INIT(slime_usb_build_info_fill_git_hash, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
SYS_INIT(slime_usb_build_info_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

#elif IS_ENABLED(CONFIG_SLIMENRF_USB_BUILD_INFO_HID) && IS_ENABLED(CONFIG_USB_DEVICE_STACK)

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

static int slime_usb_build_info_get_report(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
					   uint8_t **data)
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

	usb_hid_register_device(hid_dev, slime_usb_build_info_report_desc, sizeof(slime_usb_build_info_report_desc),
				&build_info_hid_ops);

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

#endif
