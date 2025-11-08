#ifndef SLIMENRF_SYSTEM
#define SLIMENRF_SYSTEM

#include "led.h"
#include "status.h"

#define STORED_TRACKERS 1
#define STORED_TRACKER_ADDR 2

#define RBT_CNT_ID 2
#define STORED_ADDR_0 3
// 0-15 -> id 3-18
// 0-255 -> id 3-258

#define RF_CHANNEL 259  // NVS key for receiver RF channel

uint8_t reboot_counter_read(void);
void reboot_counter_write(uint8_t reboot_counter);

void sys_write(uint16_t id, void* ptr, const void* data, size_t len);
void sys_read(uint16_t id, void* data, size_t len);

bool button_read(void);
void sys_request_system_off(void);
void sys_request_system_reboot(void);

#endif
