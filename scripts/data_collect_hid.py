#!/usr/bin/env python
# /// script
# dependencies = [
#   "hidapi",
# ]
# ///

"""
SlimeVR Raw Sensor Data Collector (HID version)

Reads raw sensor data from the SlimeVR receiver's dedicated HID endpoint
and saves it to CSV for offline VQF parameter optimization.

HID report format (64 bytes):
  [0..N-1]    ESB payload (same format as over-the-air)
  [N]         RSSI
  [N+1..N+4]  rx_ticks (BE32, Zephyr uptime ticks)
  [N+5..63]   zero padding

Packet types in ESB payload (byte 0):
  0x10: Raw IMU (48 bytes) - float gyro+accel (+optional mag)
  0x11: Raw Mag (17 bytes) - float magnetometer
  0x12: Metadata (48 bytes) - ODR, range, sensor IDs

Output: CSV file with columns: seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp
"""

import argparse
import struct
import sys
import time
from pathlib import Path

# SlimeNRF Receiver USB IDs
SLIME_VID = 0x1209
SLIME_PID = 0x7690

# HID interface: vendor usage page 0xFF00 (data collection endpoint)
DC_USAGE_PAGE = 0xFF00


class SensorMetadata:
    """Stores metadata from type 0x12 packet."""

    def __init__(self):
        self.gyro_range = 0.0
        self.accel_range = 0.0
        self.gyro_odr = 0.0
        self.accel_odr = 0.0
        self.mag_odr = 0.0
        self.imu_id = 0
        self.mag_id = 0
        self.received = False

    def parse(self, payload: bytes):
        if len(payload) < 24:
            return
        self.gyro_range = struct.unpack_from("<f", payload, 2)[0]
        self.accel_range = struct.unpack_from("<f", payload, 6)[0]
        self.gyro_odr = struct.unpack_from("<f", payload, 10)[0]
        self.accel_odr = struct.unpack_from("<f", payload, 14)[0]
        self.mag_odr = struct.unpack_from("<f", payload, 18)[0]
        self.imu_id = payload[22]
        self.mag_id = payload[23]
        self.received = True

    def __str__(self):
        return (
            f"Gyro: {self.gyro_range:.0f} dps @ {self.gyro_odr:.0f} Hz, "
            f"Accel: {self.accel_range:.0f} g @ {self.accel_odr:.0f} Hz, "
            f"Mag: {self.mag_odr:.0f} Hz, "
            f"IMU ID: {self.imu_id}, Mag ID: {self.mag_id}"
        )


def parse_raw_imu(payload: bytes):
    """Parse type 0x10 raw IMU packet. Returns sample dict."""
    if len(payload) < 42:
        return None

    seq = struct.unpack_from(">H", payload, 2)[0]
    gx, gy, gz = struct.unpack_from("<fff", payload, 4)
    ax, ay, az = struct.unpack_from("<fff", payload, 16)

    flags = payload[40]
    temperature = payload[41]
    temp_c = ((temperature - 128.5) / 2 + 25) if temperature > 0 else None

    mag = None
    if flags & 0x01:
        mx, my, mz = struct.unpack_from("<fff", payload, 28)
        mag = (mx, my, mz)

    return {
        "seq": seq,
        "gyro": (gx, gy, gz),
        "accel": (ax, ay, az),
        "mag": mag,
        "temp_c": temp_c,
    }


def find_data_hid(device_index=None):
    """Find the SlimeNRF data collection HID interface.

    The receiver has two HID interfaces:
      - Interface with standard usage page: tracker data
      - Interface with vendor usage page 0xFF00: data collection

    If multiple receivers are connected, lists them and requires
    device_index to select one.

    Returns (path, info_dict) or (None, None).
    """
    import hid

    devices = hid.enumerate(SLIME_VID, SLIME_PID)
    if not devices:
        return None, None

    # Filter for vendor-defined data collection interfaces
    dc_devices = [d for d in devices if d.get("usage_page") == DC_USAGE_PAGE]

    if not dc_devices:
        # Debug: show what was enumerated
        print("No device with vendor usage page 0xFF00 found.")
        print("Enumerated HID interfaces:")
        for d in devices:
            print(f"  interface={d.get('interface_number', '?')}, "
                  f"usage_page=0x{d.get('usage_page', 0):04X}, "
                  f"usage=0x{d.get('usage', 0):04X}")
        return None, None

    if len(dc_devices) == 1 and device_index is None:
        return dc_devices[0]["path"], dc_devices[0]

    if len(dc_devices) > 1:
        print(f"Found {len(dc_devices)} SlimeNRF data collection devices:")
        for i, d in enumerate(dc_devices):
            sn = d.get("serial_number", "?")
            mfr = d.get("manufacturer_string", "?")
            iface = d.get("interface_number", "?")
            print(f"  [{i}] SN={sn}  MFR={mfr}  interface={iface}")

        if device_index is None:
            print("\nUse --device N to select a device.")
            return None, None

    idx = device_index if device_index is not None else 0
    if idx >= len(dc_devices):
        print(f"Error: device index {idx} out of range (0-{len(dc_devices)-1})")
        return None, None

    return dc_devices[idx]["path"], dc_devices[idx]


def collect_hid(output_path, duration=None, device_index=None):
    """Main HID-based data collection loop."""
    import hid

    dev_path, dev_info = find_data_hid(device_index)
    if dev_path is None:
        print("Error: No SlimeNRF data collection HID endpoint found.")
        print("Make sure the receiver firmware was built with CONFIG_DATA_COLLECT=y")
        sys.exit(1)

    iface = dev_info.get("interface_number", "?")
    print(f"Found data HID: interface={iface}, "
          f"usage_page=0x{dev_info.get('usage_page', 0):04X}")

    h = hid.device()
    h.open_path(dev_path)

    meta = SensorMetadata()
    start_time = time.time()
    frame_count = 0
    sample_count = 0
    last_status = start_time
    last_status_samples = 0
    last_rssi = 0
    # Sequence tracking for loss rate
    last_seq = None
    total_expected = 0
    total_lost = 0
    period_expected = 0
    period_lost = 0

    base = Path(output_path)
    meta_path = base.with_suffix(".meta.txt")
    csv_path = base.with_suffix(".csv")

    print(f"Collecting data via HID...")
    print(f"Output: {csv_path}")
    if duration:
        print(f"Duration: {duration}s")
    print("Press Ctrl+C to stop\n")

    csv_file = open(csv_path, "w", encoding="utf-8")
    csv_file.write("seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp\n")

    try:
        while True:
            # Non-blocking read with 500ms timeout so Ctrl+C works
            # when data collection is stopped on the receiver
            report = h.read(64, timeout_ms=500)
            if not report:
                continue

            report = bytes(report)
            if len(report) < 2:
                continue

            pkt_type = report[0]
            frame_count += 1

            # Extract RSSI and rx_ticks from footer
            # Footer position depends on payload type
            if pkt_type == 0x10:  # Raw IMU: 48 bytes payload
                esb_len = 48
            elif pkt_type == 0x11:  # Raw Mag: variable
                esb_len = 16
            elif pkt_type == 0x12:  # Metadata: 48 bytes
                esb_len = 48
            else:
                continue

            esb_payload = report[:esb_len]
            if len(report) > esb_len:
                last_rssi = report[esb_len]

            if pkt_type == 0x12:  # Metadata
                meta.parse(esb_payload)
                print(f"\nMetadata received: {meta}")
                with open(meta_path, "w", encoding="utf-8") as f:
                    f.write(f"gyro_range_dps={meta.gyro_range}\n")
                    f.write(f"accel_range_g={meta.accel_range}\n")
                    f.write(f"gyro_odr_hz={meta.gyro_odr}\n")
                    f.write(f"accel_odr_hz={meta.accel_odr}\n")
                    f.write(f"mag_odr_hz={meta.mag_odr}\n")
                    f.write(f"imu_id={meta.imu_id}\n")
                    f.write(f"mag_id={meta.mag_id}\n")

            elif pkt_type == 0x10:  # Raw IMU
                s = parse_raw_imu(esb_payload)
                if s:
                    # Track sequence gaps for loss rate
                    seq = s["seq"]
                    if last_seq is not None:
                        gap = (seq - last_seq) & 0xFFFF  # uint16 wrap
                        if 0 < gap <= 1000:  # sane range
                            total_expected += gap
                            period_expected += gap
                            lost = gap - 1
                            if lost > 0:
                                total_lost += lost
                                period_lost += lost
                    last_seq = seq

                    mag = s.get("mag") or (0.0, 0.0, 0.0)
                    temp = s.get("temp_c") or 0.0
                    csv_file.write(
                        f"{s['seq']},"
                        f"{s['gyro'][0]:.6f},{s['gyro'][1]:.6f},{s['gyro'][2]:.6f},"
                        f"{s['accel'][0]:.6f},{s['accel'][1]:.6f},{s['accel'][2]:.6f},"
                        f"{mag[0]:.6f},{mag[1]:.6f},{mag[2]:.6f},"
                        f"{temp:.2f}\n"
                    )
                    sample_count += 1

            now = time.time()
            if now - last_status >= 2.0:
                csv_file.flush()
                elapsed = now - start_time
                period = now - last_status
                period_samples = sample_count - last_status_samples
                rate = period_samples / period if period > 0 else 0
                loss_pct = (period_lost / period_expected * 100) if period_expected > 0 else 0
                total_loss_pct = (total_lost / total_expected * 100) if total_expected > 0 else 0
                print(
                    f"\r[{elapsed:.1f}s] Samples: {sample_count} ({rate:.0f}/s), "
                    f"Loss: {loss_pct:.1f}% (total {total_loss_pct:.1f}%), "
                    f"RSSI: {last_rssi}   ",
                    end="",
                    flush=True,
                )
                last_status = now
                last_status_samples = sample_count
                period_expected = 0
                period_lost = 0

            if duration and (now - start_time >= duration):
                break

    except KeyboardInterrupt:
        print("\n\nStopping collection...")
    finally:
        csv_file.close()
        h.close()

    elapsed = time.time() - start_time
    total_loss_pct = (total_lost / total_expected * 100) if total_expected > 0 else 0
    print(f"\n\nCollection complete:")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Samples: {sample_count} -> {csv_path}")
    print(f"  Packet loss: {total_loss_pct:.2f}% ({total_lost}/{total_expected})")
    if meta.received:
        with open(meta_path, "a", encoding="utf-8") as f:
            f.write(f"duration_s={elapsed:.3f}\n")
            f.write(f"sample_count={sample_count}\n")
            if elapsed > 0:
                f.write(f"actual_odr_hz={sample_count / elapsed:.6f}\n")
        print(f"  Metadata: {meta_path}")


def main():
    parser = argparse.ArgumentParser(
        description="SlimeVR Raw Sensor Data Collector (HID)"
    )
    parser.add_argument(
        "-o", "--output", default="sensor_data",
        help="Output file base name (default: sensor_data)",
    )
    parser.add_argument(
        "-d", "--duration", type=float, default=None,
        help="Collection duration in seconds (default: until Ctrl+C)",
    )
    parser.add_argument(
        "--device", type=int, default=None,
        help="Device index when multiple receivers are connected (0-based)",
    )
    args = parser.parse_args()

    try:
        import hid  # noqa: F401
    except ImportError:
        print("Error: hidapi is required. Install with: pip install hidapi")
        sys.exit(1)

    collect_hid(args.output, args.duration, args.device)


if __name__ == "__main__":
    main()
