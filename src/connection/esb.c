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
#include "globals.h"
#include "system/system.h"
#include "hid.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

#include "esb.h"

#define RADIO_RETRANSMIT_DELAY CONFIG_RADIO_RETRANSMIT_DELAY
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL

static struct esb_payload rx_payload;
//static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);
//static struct esb_payload tx_payload_timer = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_sync = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0);

struct pairing_event {
	uint8_t packet[8];
};

// Queue pairing packets from the ISR context to the pairing worker thread.
K_MSGQ_DEFINE(esb_pairing_msgq, sizeof(struct pairing_event), 8, 4);

static K_MUTEX_DEFINE(tracker_store_lock);

static uint8_t discovered_trackers[MAX_TRACKERS] = {0};
static uint8_t last_packet_sequence[MAX_TRACKERS]; // 追踪每个追踪器的最后一个包序号
static uint8_t packet_count[MAX_TRACKERS] = {0}; // 每个追踪器接收到的包计数

// 丢包统计结构
struct packet_stats {
	uint32_t total_received;      // 实际接收到的包数（不包括重复包）
	uint32_t normal_packets;      // 正常按序的包数
	uint32_t gap_events;          // 跳跃事件数（可能是丢包）
	uint32_t out_of_order;        // 乱序包数
	uint32_t duplicate_packets;   // 重复包数
	uint32_t restart_events;      // 重启事件数
	uint32_t total_gaps;          // 总跳跃数（估计丢包数）
	uint32_t last_sequence;       // 最后一个正常序列号
	uint64_t last_packet_time;    // 最后一个包的时间戳
	bool first_packet;            // 是否是第一个包
	// TPS 计算相关
	uint32_t packets_in_last_second;  // 上一秒的包数
	uint64_t last_tps_time;           // 上次TPS计算时间
	uint32_t current_tps;             // 当前TPS
};

static struct packet_stats tracker_stats[MAX_TRACKERS] = {0};
#define STATS_PRINT_INTERVAL_MS 5000  // 每5秒打印一次统计
#define TPS_CALCULATION_INTERVAL_MS 1000  // 每秒计算一次TPS
#define TPS_MONITOR_INTERVAL_MS 250

// 统计线程相关
static void esb_stats_thread(void);
K_THREAD_DEFINE(esb_stats_thread_id, 512, esb_stats_thread, NULL, NULL, NULL, 7, 0, 0);

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_packet_filter_thread(void);
K_THREAD_DEFINE(esb_packet_filter_thread_id, 256, esb_packet_filter_thread, NULL, NULL, NULL, 6, 0, 0);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 1024, esb_thread, NULL, NULL, NULL, 5, 0, 0);

static bool esb_parse_pair(const uint8_t packet[8]);

static int check_packet_sequence(uint8_t tracker_id, uint8_t received_seq)
{
	if (tracker_id >= MAX_TRACKERS) return 2;

	struct packet_stats *stats = &tracker_stats[tracker_id];
	uint64_t current_time = k_uptime_get();

	// 更新TPS计算
	if (stats->last_tps_time == 0) {
		stats->last_tps_time = current_time;
		stats->packets_in_last_second = 0;
	} else if (current_time - stats->last_tps_time >= TPS_CALCULATION_INTERVAL_MS) {
		// 计算TPS并重置计数器
		stats->current_tps = stats->packets_in_last_second;
		stats->packets_in_last_second = 0;
		stats->last_tps_time = current_time;
	}

	// 每个包都计入TPS计算
	stats->packets_in_last_second++;
	stats->last_packet_time = current_time;

	// 第一个包，直接接受
	if (packet_count[tracker_id] == 0) {
		// 更新接收包计数
		stats->total_received++;
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id] = 1;
		stats->normal_packets++;
		stats->last_sequence = received_seq;
		stats->first_packet = false;
		LOG_DBG("First packet: tracker=%d, seq=%d", tracker_id, received_seq);
		return 0;
	}

	uint8_t last_seq = last_packet_sequence[tracker_id];
	uint8_t expected_seq = (last_seq + 1) & 0xFF;

	LOG_DBG("Packet check: tracker=%d, received=%d, last=%d, expected=%d",
			tracker_id, received_seq, last_seq, expected_seq);

	// 正常的下一个序号
	if (received_seq == expected_seq) {
		// 更新接收包计数
		stats->total_received++;
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id]++;
		stats->normal_packets++;
		stats->last_sequence = received_seq;
		LOG_DBG("Normal packet: tracker=%d, seq=%d", tracker_id, received_seq);
		return 0;
	}

	// 计算序号差值（考虑循环）
	uint8_t diff_forward = (received_seq - last_seq) & 0xFF;  // 向前差值
	uint8_t diff_backward = (last_seq - received_seq) & 0xFF; // 向后差值

	if (diff_forward == 0) {
		// 相同序号 - 这是真正的重复包
		stats->duplicate_packets++;
		return 4; // 重复包
	}

	// 如果是向前的跳跃（1-128），可能是丢包
	if (diff_forward > 0 && diff_forward <= 128) {
		// 更新接收包计数
		stats->total_received++;
		stats->gap_events++;
		stats->total_gaps += (diff_forward - 1); // 估计丢失的包数
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id]++;
		stats->last_sequence = received_seq;
		LOG_DBG("Gap detected: tracker=%d, seq=%d, gap=%d", tracker_id, received_seq, diff_forward - 1);
		return 1;
	}

	// 如果是向后的小差值（1-32），可能是乱序
	if (diff_backward > 0 && diff_backward <= 32) {
		// 更新接收包计数
		stats->total_received++;
		stats->out_of_order++;
		// 不更新last_packet_sequence，因为这可能是旧包
		LOG_DBG("Out-of-order: tracker=%d, seq=%d, backward=%d", tracker_id, received_seq, diff_backward);
		return 2;
	}

	// 大跳跃，可能是追踪器重启
	// 更新接收包计数
	stats->total_received++;
	stats->restart_events++;

	// 重置该追踪器的状态
	last_packet_sequence[tracker_id] = received_seq;
	packet_count[tracker_id] = 1;
	stats->last_sequence = received_seq;
	LOG_INF("Large jump: tracker=%d, seq=%d, jump=%d", tracker_id, received_seq, diff_forward);
	return 3;
}

// 打印特定追踪器的统计信息
static void print_tracker_stats(uint8_t tracker_id)
{
	struct packet_stats *stats = &tracker_stats[tracker_id];

	if (stats->total_received == 0 && stats->duplicate_packets == 0) return; // 没有数据则不打印

	// 总的接收次数（包括重复包）
	uint32_t total_receives = stats->total_received + stats->duplicate_packets;

	// 计算各种比率
	float duplicate_rate = 0.0f;
	float out_of_order_rate = 0.0f;
	float gap_rate = 0.0f;

	if (total_receives > 0) {
		duplicate_rate = ((float)stats->duplicate_packets / total_receives) * 100.0f;
	}

	if (stats->total_received > 0) {
		out_of_order_rate = ((float)stats->out_of_order / stats->total_received) * 100.0f;
		gap_rate = ((float)stats->gap_events / stats->total_received) * 100.0f;
	}

	// 估算丢包率：基于跳跃的总数和接收的包数
	uint32_t estimated_sent = stats->total_received + stats->total_gaps;
	float estimated_loss_rate = 0.0f;
	if (estimated_sent > 0) {
		estimated_loss_rate = ((float)stats->total_gaps / estimated_sent) * 100.0f;
	}

	LOG_INF("Tracker %d: Recv=%u(+%u dup), Normal=%u, EstLoss=%.1f%% (%u gaps), Dup=%.1f%%, OOO=%.1f%%, Restart=%u, TPS=%u",
			tracker_id,
			stats->total_received, stats->duplicate_packets,
			stats->normal_packets,
			(double)estimated_loss_rate, stats->total_gaps,
			(double)duplicate_rate,
			(double)out_of_order_rate,
			stats->restart_events,
			stats->current_tps);
}

// 统计线程 - 定期打印统计信息
static void esb_stats_thread(void)
{
	int64_t last_log_time = k_uptime_get();

	while (1) {
		k_msleep(TPS_MONITOR_INTERVAL_MS);

		uint64_t now = (uint64_t)k_uptime_get();
		for (int i = 0; i < MAX_TRACKERS; i++) {
			struct packet_stats *stats = &tracker_stats[i];
			if (stats->last_tps_time == 0) {
				continue;
			}
			if (now - stats->last_tps_time >= TPS_CALCULATION_INTERVAL_MS) {
				if (stats->packets_in_last_second > 0) {
					stats->current_tps = stats->packets_in_last_second;
				} else if (stats->last_packet_time &&
					   now - stats->last_packet_time >= TPS_CALCULATION_INTERVAL_MS) {
					stats->current_tps = 0;
				}
				stats->packets_in_last_second = 0;
				stats->last_tps_time = now;
			}
		}

		if (now - last_log_time >= STATS_PRINT_INTERVAL_MS) {
			bool has_data = false;
			for (int i = 0; i < MAX_TRACKERS; i++) {
				if (tracker_stats[i].total_received > 0) {
					if (!has_data) {
						LOG_INF("=== Packet Statistics ===");
						has_data = true;
					}
					print_tracker_stats(i);
				}
			}
			if (has_data) {
				LOG_INF("========================");
			}
			last_log_time = now;
		}
	}
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED");
		break;
	case ESB_EVENT_RX_RECEIVED:
	// TODO: make tx payload for ack here
		int err = esb_read_rx_payload(&rx_payload);
		if (!err) // zero, rx success
		{
			switch (rx_payload.length)
			{
			case 1:
			  // Heartbeat packet
				break;
			case 8:
			{
				struct pairing_event evt = {0};
				memcpy(evt.packet, rx_payload.data, sizeof(evt.packet));
				LOG_DBG("rx: %16llX", *(uint64_t *)evt.packet);
				int q_err = k_msgq_put(&esb_pairing_msgq, &evt, K_NO_WAIT);
				if (q_err)
				{
					struct pairing_event discarded;
					if (k_msgq_get(&esb_pairing_msgq, &discarded, K_NO_WAIT) == 0)
					{
						q_err = k_msgq_put(&esb_pairing_msgq, &evt, K_NO_WAIT);
					}
					if (q_err)
					{
						LOG_WRN("Pairing queue full, dropping packet type %u", evt.packet[1]);
					}
				}
				switch (evt.packet[1])
				{
				case 1: // receives ack generated from last packet
					LOG_DBG("RX Pairing Sent ACK");
					break;
				case 2: // should "acknowledge" pairing data sent from receiver
					LOG_DBG("RX Pairing ACK Receiver");
					break;
				case 0:
					LOG_INF("RX Pairing Request");
					break;
				default:
					LOG_WRN("Unexpected pairing packet type %u", evt.packet[1]);
					break;
				}
				break;
			}
			case 21: // 16 bytes data + 4 bytes CRC32 + 1 byte sequence number
				{
					uint32_t crc_check = crc32_k_4_2_update(0x93a409eb, rx_payload.data, 16);
					uint32_t *crc_ptr = (uint32_t *)&rx_payload.data[16];
					if (*crc_ptr != crc_check)
					{
						LOG_ERR("Incorrect checksum, computed %08X, received %08X", crc_check, *crc_ptr);
						printk("%08llx%016llX%016llX\n", *(uint64_t *)&rx_payload.data[16] & 0XFFFFFFFF, *(uint64_t *)&rx_payload.data[8], *(uint64_t *)rx_payload.data);
						break;
					}

					uint8_t imu_id = rx_payload.data[1];
					if (imu_id >= stored_trackers) // not a stored tracker
						return;
					if (discovered_trackers[imu_id] < DETECTION_THRESHOLD) // garbage filtering of nonexistent tracker
					{
						discovered_trackers[imu_id]++;
						return;
					}
					if (rx_payload.data[0] > 223) // reserved for receiver only
						break;

					// 智能验证包序号
					uint8_t received_sequence = rx_payload.data[20];
					int seq_result = check_packet_sequence(imu_id, received_sequence);

					// 根据序号检查结果决定是否转发数据包
					// seq_result: 0=正常, 1=可能丢包, 2=乱序, 3=重启, 4=重复
					if (seq_result == 4) break; // 丢弃重复包
					// if (seq_result == 2) {
					// 	// 丢弃不转发
					// 	break;
					// }

					// 其他情况（正常、丢包、重启）都转发数据包
					hid_write_packet_n(rx_payload.data, rx_payload.rssi); // write to hid endpoint
				}
				break;
			case 20: // has crc32 (legacy format without sequence number)
				{
					uint32_t crc_check = crc32_k_4_2_update(0x93a409eb, rx_payload.data, 16);
					uint32_t *crc_ptr = (uint32_t *)&rx_payload.data[16];
					if (*crc_ptr != crc_check)
					{
						LOG_ERR("Incorrect checksum, computed %08X, received %08X", crc_check, *crc_ptr);
						printk("%08llx%016llX%016llX\n", *(uint64_t *)&rx_payload.data[16] & 0XFFFFFFFF, *(uint64_t *)&rx_payload.data[8], *(uint64_t *)rx_payload.data);
						break;
					}

					uint8_t imu_id = rx_payload.data[1];
					if (imu_id >= stored_trackers) // not a stored tracker
						return;
					if (discovered_trackers[imu_id] < DETECTION_THRESHOLD) // garbage filtering of nonexistent tracker
					{
						discovered_trackers[imu_id]++;
						return;
					}
					if (rx_payload.data[0] > 223) // reserved for receiver only
						break;
					hid_write_packet_n(rx_payload.data, rx_payload.rssi); // write to hid endpoint
				}
				break;
			case 17: // 16 bytes data + 1 byte sequence number
				{
					uint8_t imu_id = rx_payload.data[1];
					if (imu_id >= stored_trackers) // not a stored tracker
						return;
					if (discovered_trackers[imu_id] < DETECTION_THRESHOLD) // garbage filtering of nonexistent tracker
					{
						discovered_trackers[imu_id]++;
						return;
					}
					if (rx_payload.data[0] > 223) // reserved for receiver only
						break;

					uint8_t received_sequence = rx_payload.data[16];
					int seq_result = check_packet_sequence(imu_id, received_sequence);

					// 根据序号检查结果决定是否转发数据包
					// seq_result: 0=正常, 1=可能丢包, 2=乱序, 3=重启, 4=重复
					if (seq_result == 4) break; // 丢弃重复包
					// if (seq_result == 2) {
					// 	// 丢弃不转发
					// 	// LOG_WRN("Discarding out-of-order packet for tracker %d", imu_id);
					// 	break;
					// }

					// 其他情况（正常、丢包、重启）都转发数据包
					hid_write_packet_n(rx_payload.data, rx_payload.rssi); // write to hid endpoint
				}
				break;
			case 16: // legacy format without CRC
				{
					uint8_t imu_id = rx_payload.data[1];
					if (imu_id >= stored_trackers) // not a stored tracker
						return;
					if (discovered_trackers[imu_id] < DETECTION_THRESHOLD) // garbage filtering of nonexistent tracker
					{
						discovered_trackers[imu_id]++;
						return;
					}
					if (rx_payload.data[0] > 223) // reserved for receiver only
						break;
					hid_write_packet_n(rx_payload.data, rx_payload.rssi); // write to hid endpoint
				}
				break;
			default:
				LOG_ERR("Wrong packet length: %d", rx_payload.length);
				break;
			}
		}
		else
		{
			LOG_ERR("Error while reading rx packet: %d", err);
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10000) {
			LOG_WRN("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

// this was randomly generated
// TODO: I have no idea?
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;

int esb_initialize(bool tx)
{
	if (esb_initialized)
		LOG_WRN("ESB already initialized");
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	uint16_t jitter = (rand() % 200) - 100;  // ±100 µs
	uint16_t retransmit_delay_with_jitter = RADIO_RETRANSMIT_DELAY + jitter;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 30;
		config.retransmit_delay = retransmit_delay_with_jitter;
		// config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = true;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 30;
		config.retransmit_delay = retransmit_delay_with_jitter;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = true;
	}

	LOG_INF("Initializing ESB, %sX mode", tx ? "T" : "R");
	err = esb_init(&config);

	if (!err)
		esb_set_rf_channel(RADIO_RF_CHANNEL);

	if (!err)
		esb_set_base_address_0(base_addr_0);

	if (!err)
		esb_set_base_address_1(base_addr_1);

	if (!err)
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	if (err)
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}

	esb_initialized = true;
	return 0;
}

static void esb_deinitialize(void)
{
	LOG_INF("ESB deinitialize requested");
	if (esb_initialized)
	{
		esb_initialized = false;
		LOG_INF("Deinitializing ESB");
		k_msleep(10); // wait for pending transmissions
		if (esb_initialized)
		{
			LOG_INF("ESB denitialize cancelled");
			return;
		}
		esb_disable();
	}
	esb_initialized = false;
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Generate addresses from device address
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint8_t buf[6] = {0};
	memcpy(buf, addr, 6);
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = buf[i];
		addr_buffer[i + 4] = buf[i] + buf[4];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = buf[5] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

static bool esb_pairing = false;
static bool esb_paired = false;

int esb_add_pair(uint64_t addr, bool checksum)
{
	if (addr == 0)
		return -EINVAL;

	bool new_entry = false;
	int assigned_id = -1;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	for (int i = 0; i < stored_trackers; i++)
	{
		if (stored_tracker_addr[i] == addr)
		{
			assigned_id = i;
			break;
		}
	}

	if (assigned_id < 0)
	{
		if (stored_trackers >= MAX_TRACKERS)
		{
			k_mutex_unlock(&tracker_store_lock);
			LOG_WRN("Tracker storage full, cannot add %012llX", addr);
			return -ENOSPC;
		}
		assigned_id = stored_trackers;
		stored_tracker_addr[assigned_id] = addr;
		stored_trackers++;
		new_entry = true;
	}

	k_mutex_unlock(&tracker_store_lock);

	if (new_entry)
	{
		LOG_INF("Added device on id %d with address %012llX", assigned_id, addr);
		sys_write(STORED_ADDR_0 + assigned_id, NULL, &stored_tracker_addr[assigned_id], sizeof(stored_tracker_addr[0]));
		sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	}
	else
	{
		LOG_INF("Device already stored with id %d", assigned_id);
	}

	if (checksum)
	{
		uint8_t buf[6] = {0};
		memcpy(buf, &addr, 6);
		uint8_t checksum_byte = crc8_ccitt(0x07, buf, 6);
		if (checksum_byte == 0)
			checksum_byte = 8;
		uint64_t *receiver_addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet
		uint64_t pair_addr = (*receiver_addr & 0xFFFFFFFFFFFF) << 16;
		pair_addr |= checksum_byte; // Add checksum to the address
		pair_addr |= (uint64_t)assigned_id << 8; // Add tracker id to the address
		LOG_INF("Pair the device with %016llX", pair_addr);
	}

	return assigned_id;
}

void esb_pop_pair(void)
{
	uint64_t removed_addr = 0;
	int removed_id = -1;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	if (stored_trackers > 0)
	{
		stored_trackers--;
		removed_id = stored_trackers;
		removed_addr = stored_tracker_addr[removed_id];
		stored_tracker_addr[removed_id] = 0;
	}
	k_mutex_unlock(&tracker_store_lock);

	if (removed_id >= 0)
	{
		sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
		sys_write(STORED_ADDR_0 + removed_id, NULL, &stored_tracker_addr[removed_id], sizeof(stored_tracker_addr[0]));
		LOG_INF("Removed device on id %d with address %012llX", removed_id, removed_addr);
	}
	else
	{
		LOG_WRN("No devices to remove");
	}
}

static bool esb_parse_pair(const uint8_t packet[8])
{
	uint64_t raw_addr = 0;
	memcpy(&raw_addr, packet, sizeof(raw_addr));
	uint64_t found_addr = (raw_addr >> 16) & 0xFFFFFFFFFFFF;
	uint8_t checksum = crc8_ccitt(0x07, &packet[2], 6);
	if (checksum == 0)
		checksum = 8;

	uint16_t send_tracker_id = 0;
	uint8_t tracker_count_snapshot = 0;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	tracker_count_snapshot = stored_trackers;
	send_tracker_id = tracker_count_snapshot; // default to next available ID
	for (uint8_t i = 0; i < tracker_count_snapshot; i++)
	{
		if (found_addr != 0 && stored_tracker_addr[i] == found_addr)
		{
			send_tracker_id = i;
			break;
		}
	}
	k_mutex_unlock(&tracker_store_lock);

	bool checksum_valid = (checksum == packet[0]);
	bool has_capacity = tracker_count_snapshot < MAX_TRACKERS;
	bool is_new_device = checksum_valid && found_addr != 0 && send_tracker_id == tracker_count_snapshot && has_capacity;
	bool ack_valid = false;

	if (is_new_device)
	{
		int assigned_id = esb_add_pair(found_addr, false);
		if (assigned_id >= 0)
		{
			send_tracker_id = (uint16_t)assigned_id;
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
		}
		else if (assigned_id == -ENOSPC)
		{
			LOG_WRN("Maximum tracker slots reached, cannot pair %012llX", found_addr);
		}
		else
		{
			LOG_ERR("Failed to store tracker address %012llX: %d", found_addr, assigned_id);
		}
	}

	ack_valid = checksum_valid && send_tracker_id < MAX_TRACKERS;

	tx_payload_pair.data[0] = ack_valid ? packet[0] : 0;
	tx_payload_pair.data[1] = (send_tracker_id < MAX_TRACKERS) ? (uint8_t)send_tracker_id : 0xFF;

	return ack_valid;
}

void esb_start_pairing(void)
{
	LOG_INF("Starting pairing mode (non-blocking)");
	esb_set_addr_discovery();
	esb_initialize(false);
	esb_start_rx();
	tx_payload_pair.noack = false;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	memcpy(&tx_payload_pair.data[2], addr, 6);
	LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	esb_pairing = true;
	k_msgq_purge(&esb_pairing_msgq);
}

void esb_pair(void)
{
	LOG_INF("Pairing");
	esb_set_addr_discovery();
	esb_initialize(false);
	esb_start_rx();
	tx_payload_pair.noack = false;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	memcpy(&tx_payload_pair.data[2], addr, 6);
	LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	esb_pairing = true;
	k_msgq_purge(&esb_pairing_msgq);
	while (esb_pairing)
	{
		if (!esb_initialized)
		{
			esb_initialize(false);
			esb_start_rx();
		}

		struct pairing_event evt;
		int q_err = k_msgq_get(&esb_pairing_msgq, &evt, K_MSEC(10));
		if (q_err != 0)
		{
			continue;
		}

		switch (evt.packet[1])
		{
		case 0:
		{
			bool ack_ready = esb_parse_pair(evt.packet);
			if (!ack_ready)
			{
				LOG_DBG("Pairing request invalid, not queueing response");
				break;
			}
			esb_flush_tx();
			int tx_err = esb_write_payload(&tx_payload_pair);
			if (tx_err == -ENOSPC)
			{
				esb_flush_tx();
				tx_err = esb_write_payload(&tx_payload_pair);
			}
			if (tx_err)
			{
				LOG_ERR("Failed to queue pairing response: %d", tx_err);
			}
			else
			{
				LOG_DBG("tx: %16llX", *(uint64_t *)tx_payload_pair.data);
			}
			break;
		}
		case 2:
			esb_flush_tx();
			break;
		case 1:
			// Tracker acknowledged previous response, nothing to do
			break;
		default:
			LOG_WRN("Unhandled pairing packet type %u", evt.packet[1]);
			break;
		}
	}
	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CONNECTION);
	esb_deinitialize();
}

void esb_reset_pair(void)
{
	esb_deinitialize(); // make sure esb is off
	esb_paired = false;
}

void esb_finish_pair(void)
{
	esb_pairing = false;
	k_msgq_purge(&esb_pairing_msgq);
}

void esb_clear(void)
{
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	uint8_t previous_count = stored_trackers;
	stored_trackers = 0;
	memset(stored_tracker_addr, 0, sizeof(stored_tracker_addr));
	k_mutex_unlock(&tracker_store_lock);

	sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	for (uint8_t i = 0; i < previous_count && i < MAX_TRACKERS; i++)
	{
		sys_write(STORED_ADDR_0 + i, NULL, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	}
	LOG_INF("NVS Reset");
	esb_reset_pair();

	// 重置所有追踪器的包序号状态
	for (int i = 0; i < MAX_TRACKERS; i++)
	{
		last_packet_sequence[i] = 0;
		packet_count[i] = 0;
		discovered_trackers[i] = 0;
		// 重置统计信息
		memset(&tracker_stats[i], 0, sizeof(struct packet_stats));
	}
	LOG_INF("Packet sequence state and statistics reset for all trackers");
}

// 重置特定追踪器的包序号状态
void esb_reset_tracker_sequence(uint8_t tracker_id)
{
	if (tracker_id < MAX_TRACKERS)
	{
		last_packet_sequence[tracker_id] = 0;
		packet_count[tracker_id] = 0;
		discovered_trackers[tracker_id] = 0;
		// 重置统计信息
		memset(&tracker_stats[tracker_id], 0, sizeof(struct packet_stats));
		LOG_INF("Packet sequence state and statistics reset for tracker %d", tracker_id);
	}
}

// 手动打印所有活跃追踪器的统计信息
void esb_print_all_stats(void)
{
	LOG_INF("=== Packet Statistics Summary ===");
	for (int i = 0; i < MAX_TRACKERS; i++)
	{
		if (tracker_stats[i].total_received > 0)
		{
			print_tracker_stats(i);
		}
	}
	LOG_INF("================================");
}

// 重置所有追踪器的统计信息
void esb_reset_all_stats(void)
{
	for (int i = 0; i < MAX_TRACKERS; i++)
	{
		memset(&tracker_stats[i], 0, sizeof(struct packet_stats));
	}
	LOG_INF("All packet statistics have been reset");
}

// TODO:
void esb_write_sync(uint16_t led_clock)
{
	if (!esb_initialized || !esb_paired)
		return;
	tx_payload_sync.noack = false;
	tx_payload_sync.data[0] = (led_clock >> 8) & 255;
	tx_payload_sync.data[1] = led_clock & 255;
	esb_write_payload(&tx_payload_sync);
}

// TODO:
void esb_receive(void)
{
	esb_set_addr_paired();
	esb_paired = true;
}

static void esb_packet_filter_thread(void)
{
	memset(discovered_trackers, 0, sizeof(discovered_trackers));
	while (1) // reset count if its not above threshold
	{
		k_msleep(1000);
		for (int i = 0; i < MAX_TRACKERS; i++)
			if (discovered_trackers[i] < DETECTION_THRESHOLD)
				discovered_trackers[i] = 0;
	}
}

static void esb_thread(void)
{
	clocks_start();

	sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	uint8_t tracker_count = stored_trackers;
	for (uint8_t i = 0; i < tracker_count && i < MAX_TRACKERS; i++)
	{
		sys_read(STORED_ADDR_0 + i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	}
	k_mutex_unlock(&tracker_store_lock);

	if (tracker_count)
		esb_paired = true;
	LOG_INF("%d/%d devices stored", tracker_count, MAX_TRACKERS);

	if (esb_paired)
	{
		esb_receive();
		esb_initialize(false);
		esb_start_rx();
	}

	while (1)
	{
		if (!esb_paired)
		{
			esb_pair();
			esb_receive();
			esb_initialize(false);
			esb_start_rx();
		}
		k_msleep(100);
	}
}
