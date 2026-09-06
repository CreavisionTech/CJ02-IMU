#!/usr/bin/env python3
"""
CJ02-IMU Python example — read and print IMU data.

Usage:
    python example.py COM11           # Windows
    python example.py /dev/ttyUSB0    # Linux

Requirements: pip install pyserial
"""

import sys
import time
from cj02_imu import CJ02IMU, AttitudeFrame, RawFrame

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port> [baud]")
        print(f"  Linux: {sys.argv[0]} /dev/ttyUSB0")
        print(f"  Win:   {sys.argv[0]} COM11")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 460800

    imu = CJ02IMU()

    # Attitude callback (default 800 Hz; configurable to 1600/800/400/200 Hz)
    frame_count = [0]
    start_time = [time.time()]

    def on_attitude(f: AttitudeFrame):
        frame_count[0] += 1
        elapsed = time.time() - start_time[0]
        if elapsed > 1.0:
            rate = frame_count[0] / elapsed
            print(f"\rR={f.roll:7.2f}°  P={f.pitch:7.2f}°  Y={f.yaw:7.2f}°  "
                  f"mode={f.mode_name:6s}  zaru={f.zaru_used}  static={f.is_static}  "
                  f"[{rate:.0f} Hz]  ", end="", flush=True)
            frame_count[0] = 0
            start_time[0] = time.time()

    def on_raw(f: RawFrame):
        # Print raw every 100th frame (~8 Hz)
        pass  # Enable if needed:
        # if frame_count[0] % 100 == 0:
        #     acc = f.acc_mg
        #     gyr = f.gyr_dps
        #     print(f"\nacc=({acc[0]:.1f}, {acc[1]:.1f}, {acc[2]:.1f})mg  "
        #           f"gyr=({gyr[0]:.2f}, {gyr[1]:.2f}, {gyr[2]:.2f})°/s", flush=True)

    imu.on_attitude = on_attitude
    imu.on_raw = on_raw

    if not imu.open(port, baud):
        sys.exit(1)

    print(f"Connected to CJ02-IMU on {port} @ {baud} baud")
    print("Press Ctrl+C to quit\n")

    # Request config
    imu.get_config()

    try:
        imu.run(blocking=True)
    except KeyboardInterrupt:
        pass
    finally:
        imu.close()
        print(f"\n\nGood frames: {imu.good_frames}  Bad frames: {imu.bad_frames}")

if __name__ == "__main__":
    main()
