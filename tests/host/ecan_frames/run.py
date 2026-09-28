#!/usr/bin/env python3
"""Compile the real ECAN-ECBT-16B framing module against host stubs.

Only the transport edges are stubbed: the UART driver, the USB state and the
kernel work/msgq/timer primitives execute synchronously so a test can feed
Slime packets through ecan_handle_packet() and inspect the bytes that would
reach cdc_acm_uart1. Framing, battery/temperature mapping and quaternion
decoding are the production code.
"""
import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path

CASES = ('layout', 'quaternion', 'battery', 'mag', 'packed', 'drop',
         'buffer', 'usb-closed')

KERNEL = r'''
#ifndef HOST_KERNEL_H
#define HOST_KERNEL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#define ARG_UNUSED(x) ((void)(x))
#define IS_ENABLED(x) (x)
#define BIT(n) (1U << (n))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define BUILD_ASSERT(c, ...) _Static_assert(c, #c)

#define K_NO_WAIT 0
#define K_MSEC(ms) (ms)
#define K_SECONDS(s) ((s) * 1000)
#define CONFIG_KERNEL_INIT_PRIORITY_DEVICE 50
#ifndef CONFIG_SLIMEVR_ECAN_STREAM
#define CONFIG_SLIMEVR_ECAN_STREAM 0
#endif

extern uint32_t host_now;
static inline uint32_t k_uptime_get_32(void) { return host_now; }

typedef int atomic_t;
typedef int atomic_val_t;
static inline atomic_val_t atomic_get(const atomic_t *t) { return *t; }
static inline atomic_val_t atomic_set(atomic_t *t, atomic_val_t v) {
    atomic_val_t old = *t; *t = v; return old;
}
static inline atomic_val_t atomic_inc(atomic_t *t) { return (*t)++; }

struct k_msgq { uint8_t *buffer; size_t item_size, capacity, read, count; };
#define K_MSGQ_DEFINE(name, size, num, align) \
    static uint8_t name##_storage[(size) * (num)]; \
    struct k_msgq name = { name##_storage, (size), (num), 0, 0 }
static inline int k_msgq_put(struct k_msgq *q, const void *item, int timeout) {
    (void)timeout;
    if (q->count == q->capacity) return -ENOMSG;
    memcpy(q->buffer + ((q->read + q->count) % q->capacity) * q->item_size,
           item, q->item_size);
    q->count++;
    return 0;
}
static inline int k_msgq_get(struct k_msgq *q, void *item, int timeout) {
    (void)timeout;
    if (!q->count) return -ENOMSG;
    memcpy(item, q->buffer + q->read * q->item_size, q->item_size);
    q->read = (q->read + 1) % q->capacity;
    q->count--;
    return 0;
}
static inline uint32_t k_msgq_num_used_get(const struct k_msgq *q) { return (uint32_t)q->count; }

struct k_work { void (*handler)(struct k_work *); };
struct k_work_delayable { struct k_work work; };
struct k_work_q { void *thread; };

/* Work items are queued, then run explicitly by the fixture: production
 * retries have a 1 ms delay, so a synchronous re-entry would spin. */
void host_queue_work(struct k_work *work);
static inline int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work) {
    (void)queue; host_queue_work(work); return 1;
}
static inline int k_work_schedule_for_queue(struct k_work_q *queue,
                                            struct k_work_delayable *dwork, int delay) {
    (void)queue; (void)delay; host_queue_work(&dwork->work); return 1;
}
#define K_WORK_DEFINE(name, fn) struct k_work name = { .handler = fn }
#define K_WORK_DELAYABLE_DEFINE(name, fn) \
    struct k_work_delayable name = { .work = { .handler = fn } }
static inline void k_work_queue_init(struct k_work_q *queue) { (void)queue; }
static inline void k_work_queue_start(struct k_work_q *queue, void *stack, size_t size,
                                      int prio, const void *config) {
    (void)queue; (void)stack; (void)size; (void)prio; (void)config;
}
static inline int k_thread_name_set(void *thread, const char *name) {
    (void)thread; (void)name; return 0;
}
#define K_THREAD_STACK_DEFINE(name, size) uint8_t name[size]
#define K_THREAD_STACK_SIZEOF(name) sizeof(name)

struct k_timer { void (*handler)(struct k_timer *); };
#define K_TIMER_DEFINE(name, fn, stop_fn) struct k_timer name = { .handler = fn }
static inline void k_timer_start(struct k_timer *timer, int duration, int period) {
    (void)timer; (void)duration; (void)period;
}
#endif
'''

LOGGING = r'''#ifndef HOST_LOG_H
#define HOST_LOG_H
#include <stdarg.h>
#define LOG_MODULE_REGISTER(...)
static inline void host_log(const char *format, ...) { (void)format; }
#define LOG_INF(...) host_log(__VA_ARGS__)
#define LOG_WRN(...) host_log(__VA_ARGS__)
#define LOG_ERR(...) host_log(__VA_ARGS__)
#define LOG_DBG(...) host_log(__VA_ARGS__)
#endif
'''

DEVICE = r'''
#ifndef HOST_DEVICE_H
#define HOST_DEVICE_H
#include <stdbool.h>
#include <stddef.h>
struct device { int unused; };
extern const struct device host_cdc_device;
static inline bool device_is_ready(const struct device *dev) { return dev != NULL; }
/* The framing path only needs the node to exist; the DTS is out of scope. */
#define DT_NODELABEL(label) 0
#define DT_NODE_HAS_STATUS(node, status) 1
#define DEVICE_DT_GET(node) (&host_cdc_device)
#endif
'''

UART = r'''
#ifndef HOST_UART_H
#define HOST_UART_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>

typedef void (*uart_irq_callback_user_data_t)(const struct device *dev, void *user_data);

int uart_irq_callback_user_data_set(const struct device *dev,
                                    uart_irq_callback_user_data_t cb, void *user_data);
void uart_irq_tx_enable(const struct device *dev);
void uart_irq_tx_disable(const struct device *dev);
bool uart_irq_tx_ready(const struct device *dev);
bool uart_irq_update(const struct device *dev);
bool uart_irq_is_pending(const struct device *dev);
int uart_fifo_fill(const struct device *dev, const uint8_t *data, int size);
#endif
'''

INIT = r'''
#ifndef HOST_INIT_H
#define HOST_INIT_H
/* Run the module's SYS_INIT before main, like the boot sequence does. */
#define SYS_INIT(fn, level, prio) \
    static int fn##_host_sys_init(void) { return fn(); } \
    __attribute__((constructor)) static void fn##_host_run(void) { (void)fn##_host_sys_init(); }
#endif
'''

BYTEORDER = r'''
#ifndef HOST_BYTEORDER_H
#define HOST_BYTEORDER_H
#include <stdint.h>
static inline uint16_t sys_get_le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t sys_get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void sys_put_be16(uint16_t value, uint8_t *p) {
    p[0] = (uint8_t)(value >> 8); p[1] = (uint8_t)value;
}
#endif
'''

HEADERS = {
    'zephyr/kernel.h': KERNEL,
    'zephyr/logging/log.h': LOGGING,
    'zephyr/sys/util.h': '#include <zephyr/kernel.h>\n',
    'zephyr/sys/atomic.h': '#include <zephyr/kernel.h>\n',
    'zephyr/sys/byteorder.h': BYTEORDER,
    'zephyr/device.h': DEVICE,
    'zephyr/drivers/uart.h': UART,
    'zephyr/init.h': INIT,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path,
                        default=Path(os.environ.get('SOURCE_ROOT', Path(__file__).resolve().parents[3])))
    parser.add_argument('--case', action='append', choices=CASES)
    args = parser.parse_args()
    root = args.source_root.resolve()
    with subprocess_run(root, args.case or CASES) as failed:
        sys.exit(1 if failed else 0)


class subprocess_run:
    def __init__(self, root, cases):
        self.root = root
        self.cases = cases
        self.failed = False

    def __enter__(self):
        import tempfile
        with tempfile.TemporaryDirectory(prefix='receiver-ecan-') as directory:
            path = Path(directory)
            for name, text in HEADERS.items():
                target = path / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(text)
            binary = path / 'test'
            command = shlex.split(os.environ.get('CC', 'cc')) + [
                '-std=c11', '-Wall', '-Wextra', '-Werror',
                '-I' + str(path), '-I' + str(self.root / 'src'),
                str(self.root / 'src/ecan.c'), str(Path(__file__).with_name('fixture.c')),
                '-o', str(binary), '-lm',
            ]
            subprocess.run(command, check=True)
            for case in self.cases:
                completed = subprocess.run([str(binary), case], text=True,
                                           capture_output=True)
                if completed.returncode:
                    sys.stderr.write(completed.stderr)
                    self.failed = True
                else:
                    sys.stdout.write(completed.stdout)
        return self.failed

    def __exit__(self, *exc):
        return False


if __name__ == '__main__':
    main()
