#!/usr/bin/env python3
"""
CJ02-IMU Python SDK — read IMU data over serial.

Usage:
    from cj02_imu import CJ02IMU
    imu = CJ02IMU()
    imu.on_attitude = lambda f: print(f"R={f['roll']:.1f} P={f['pitch']:.1f} Y={f['yaw']:.1f}")
    imu.open("COM11")       # or "/dev/ttyUSB0" on Linux
    imu.run()               # blocking loop

Requirements: pip install pyserial
"""

import struct
import threading
import time
from typing import Callable, Optional
from dataclasses import dataclass

try:
    import serial
except ImportError:
    raise ImportError("pyserial is required: pip install pyserial")


# Frame sync bytes
SYNC_RAW = 0xAA
SYNC_ATTITUDE = 0xAB
SYNC_EVENT = 0xAD
SYNC_CONFIG = 0xAC

# Frame sizes
SIZE_RAW = 16
SIZE_ATTITUDE = 20
SIZE_EVENT = 22
ACCEL_LSB_PER_G = 2048.0  # Current firmware: BMI160 +/-16 g.


@dataclass
class RawFrame:
    """Raw IMU data frame (0xAA, 16 bytes)."""
    acc: tuple          # (x, y, z) in LSB, ±16g, 2048 LSB/g
    gyr: tuple          # (x, y, z) in LSB, ±2000°/s, 16.4 LSB/(°/s)
    seq: int            # frame sequence

    @property
    def acc_mg(self) -> tuple:
        """Acceleration in milligravity."""
        return tuple(v * 1000.0 / ACCEL_LSB_PER_G for v in self.acc)

    @property
    def gyr_dps(self) -> tuple:
        """Angular velocity in degrees per second."""
        return tuple(v / 16.4 for v in self.gyr)


@dataclass
class AttitudeFrame:
    """Attitude frame (0xAB, 20 bytes)."""
    roll: float         # degrees, -180..180
    pitch: float        # degrees, -90..90
    yaw: float          # degrees, -180..180
    seq: int            # attitude sequence
    mode: int           # 0=INIT, 1=RUN, 2=NO_ACC, 3=REJECT
    flags: int          # bit0=accel_used, bit1=zaru_used, bit2=is_static

    @property
    def mode_name(self) -> str:
        return {0: "INIT", 1: "RUN", 2: "NO_ACC", 3: "REJECT"}.get(self.mode, "UNKNOWN")

    @property
    def accel_used(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def zaru_used(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def is_static(self) -> bool:
        return bool(self.flags & 0x04)


@dataclass
class SyncEventFrame:
    """External trigger sync event frame (0xAD, 22 bytes)."""
    trigger_seq: int    # 1600Hz sample counter at trigger
    roll: float
    pitch: float
    yaw: float
    att_seq: int        # attitude frame seq at trigger


@dataclass
class Config:
    """Device configuration (80 bytes)."""
    gyro_tilt_std_1s_deg: float = 0.3
    sigma_gyro_bias: float = 3e-4
    gyro_scale_factor_error: float = 0.01
    accel_variance_base: float = 0.05
    accel_g_3sigma: float = 0.1
    accel_release_tau: float = 0.3
    nis_reject: float = 50.0
    nis_inflate_gamma: float = 6.0
    zaru_variance: float = 5e-4
    static_gyro_threshold: float = 0.0524
    static_accel_rel_std_thresh: float = 0.026
    motion_gyro_full: float = 0.349
    motion_accel_full: float = 0.30
    initialization_tilt_seconds: float = 0.5
    initial_attitude_variance: float = 0.00122
    initial_bias_variance: float = 2.5e-5
    maximum_delta_seconds: float = 0.05
    zaru_static_frames: int = 300
    motion_window_length: int = 200
    trigger_divider: int = 0
    trigger_duty: int = 20

@dataclass
class FilterConfig:
    """ESKF-input filtering and transport configuration (68 bytes)."""
    flags: int = 0
    output_rate_hz: int = 800
    baud_rate: int = 460800
    accel_lpf_hz: float = 200.0
    gyro_lpf_hz: float = 200.0
    accel_notch_hz: tuple = (100.0, 150.0, 200.0)
    accel_notch_q: tuple = (10.0, 10.0, 10.0)
    gyro_notch_hz: tuple = (100.0, 150.0, 200.0)
    gyro_notch_q: tuple = (10.0, 10.0, 10.0)


def _xor_checksum(data: bytes) -> int:
    """XOR checksum of all bytes."""
    result = 0
    for b in data:
        result ^= b
    return result


class CJ02IMU:
    """
    CJ02-IMU serial reader.

    Callbacks:
        on_raw:       Callable[[RawFrame], None]
        on_attitude:  Callable[[AttitudeFrame], None]
        on_sync:      Callable[[SyncEventFrame], None]
        on_config:    Callable[[Config], None]
        on_filter_config: Callable[[FilterConfig], None]
        on_config_reply: Callable[[int, bool], None] (command, device success)
    """

    def __init__(self):
        self.serial: Optional[serial.Serial] = None
        self._buffer = bytearray()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Callbacks
        self.on_raw: Callable[[RawFrame], None] = None
        self.on_attitude: Callable[[AttitudeFrame], None] = None
        self.on_sync: Callable[[SyncEventFrame], None] = None
        self.on_config: Callable[[Config], None] = None
        self.on_filter_config: Callable[[FilterConfig], None] = None
        # Command/status acknowledgement; set_config returns write success only.
        self.on_config_reply: Callable[[int, bool], None] = None

        # Statistics
        self.good_frames = 0
        self.bad_frames = 0

    def open(self, port: str, baud: int = 460800) -> bool:
        """Open serial port. Returns True on success."""
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            return True
        except serial.SerialException as e:
            print(f"Error opening {port}: {e}")
            return False

    def close(self):
        """Close serial port and stop reading."""
        self.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()

    def run(self, blocking: bool = True):
        """Start reading. If blocking=True, runs forever. Otherwise starts a thread."""
        self._running = True
        if blocking:
            self._read_loop()
        else:
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()

    def stop(self):
        """Stop the read loop."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            self._thread = None

    def get_config(self) -> bool:
        """Send GET_CONFIG command."""
        if not self.serial or not self.serial.is_open:
            return False
        pkt = bytes([SYNC_CONFIG, 0x01, 0x00, 0x01 ^ 0x00])
        self.serial.write(pkt)
        return True

    def set_config(self, cfg: Config) -> bool:
        """Send SET_CONFIG. True means sent; check on_config_reply for device status.

        Wait for the reply before sending another command. Keep the reader running.
        """
        if not self.serial or not self.serial.is_open:
            return False
        payload = struct.pack('<17fIIHH',
            cfg.gyro_tilt_std_1s_deg, cfg.sigma_gyro_bias,
            cfg.gyro_scale_factor_error, cfg.accel_variance_base,
            cfg.accel_g_3sigma, cfg.accel_release_tau,
            cfg.nis_reject, cfg.nis_inflate_gamma,
            cfg.zaru_variance, cfg.static_gyro_threshold,
            cfg.static_accel_rel_std_thresh, cfg.motion_gyro_full,
            cfg.motion_accel_full, cfg.initialization_tilt_seconds,
            cfg.initial_attitude_variance, cfg.initial_bias_variance,
            cfg.maximum_delta_seconds,
            cfg.zaru_static_frames, cfg.motion_window_length,
            cfg.trigger_divider, cfg.trigger_duty)
        assert len(payload) == 80, f"Config payload must be 80 bytes, got {len(payload)}"
        hdr = bytes([SYNC_CONFIG, 0x02, 80])
        xor_v = 0x02 ^ 80
        for b in payload:
            xor_v ^= b
        pkt = hdr + payload + bytes([xor_v])
        self.serial.write(pkt)
        return True

    def reset_config(self) -> bool:
        """Send RESET_CONFIG command."""
        if not self.serial or not self.serial.is_open:
            return False
        pkt = bytes([SYNC_CONFIG, 0x03, 0x00, 0x03 ^ 0x00])
        self.serial.write(pkt)
        return True

    def _simple_command(self, cmd: int, payload: bytes = b'') -> bool:
        if not self.serial or not self.serial.is_open or len(payload) > 255:
            return False
        checksum = cmd ^ len(payload)
        for b in payload: checksum ^= b
        self.serial.write(bytes([SYNC_CONFIG, cmd, len(payload)]) + payload + bytes([checksum]))
        return True

    def get_filter_config(self) -> bool:
        return self._simple_command(0x04)

    def set_filter_temporary(self, cfg: FilterConfig) -> bool:
        vals = (cfg.accel_lpf_hz, cfg.gyro_lpf_hz, *cfg.accel_notch_hz,
                *cfg.accel_notch_q, *cfg.gyro_notch_hz, *cfg.gyro_notch_q)
        payload = struct.pack('<III14f', cfg.flags, cfg.output_rate_hz, cfg.baud_rate, *vals)
        return self._simple_command(0x05, payload)

    def save_filter_config(self) -> bool:
        return self._simple_command(0x06)

    def disable_filters(self) -> bool:
        return self._simple_command(0x07)

    def enter_vibration_mode(self) -> bool:
        return self._simple_command(0x08)

    def exit_vibration_mode(self) -> bool:
        return self._simple_command(0x09)

    def _read_loop(self):
        """Internal: read serial data and feed to parser."""
        while self._running and self.serial and self.serial.is_open:
            try:
                data = self.serial.read(4096)
                if data:
                    self._feed(data)
            except serial.SerialException:
                break
            except Exception:
                break
        self._running = False

    def _feed(self, data: bytes):
        """Feed raw bytes to the frame parser."""
        self._buffer.extend(data)
        self._parse()

    def _parse(self):
        """Parse buffered data into frames."""
        buf = self._buffer

        while len(buf) >= 1:
            sync = buf[0]
            frame_len = 0

            if sync == SYNC_RAW:
                frame_len = SIZE_RAW
            elif sync == SYNC_ATTITUDE:
                frame_len = SIZE_ATTITUDE
            elif sync == SYNC_EVENT:
                frame_len = SIZE_EVENT
            elif sync == SYNC_CONFIG:
                if len(buf) < 3:
                    break
                frame_len = 3 + buf[2] + 1
            else:
                # Unknown byte — skip 1
                del buf[0]
                continue

            if len(buf) < frame_len:
                break  # wait for more data

            frame = bytes(buf[:frame_len])

            ok = False
            if sync == SYNC_RAW:
                ok = self._parse_raw(frame)
            elif sync == SYNC_ATTITUDE:
                ok = self._parse_attitude(frame)
            elif sync == SYNC_EVENT:
                ok = self._parse_event(frame)
            elif sync == SYNC_CONFIG:
                ok = self._parse_config(frame)

            if ok:
                self.good_frames += 1
                del buf[:frame_len]
            else:
                self.bad_frames += 1
                del buf[0]  # advance only 1 byte to re-sync

    def _parse_raw(self, frame: bytes) -> bool:
        if _xor_checksum(frame[1:15]) != frame[15]:
            return False
        acc_x, acc_y, acc_z, gx, gy, gz, seq = struct.unpack('<3h3hH', frame[1:15])
        f = RawFrame(acc=(acc_x, acc_y, acc_z), gyr=(gx, gy, gz), seq=seq)
        if self.on_raw:
            self.on_raw(f)
        return True

    def _parse_attitude(self, frame: bytes) -> bool:
        if _xor_checksum(frame[1:19]) != frame[19]:
            return False
        roll, pitch, yaw, seq, mode, flags = struct.unpack('<fffIBB', frame[1:19])
        f = AttitudeFrame(roll=roll, pitch=pitch, yaw=yaw,
                          seq=seq, mode=mode, flags=flags)
        if self.on_attitude:
            self.on_attitude(f)
        return True

    def _parse_event(self, frame: bytes) -> bool:
        if _xor_checksum(frame[1:21]) != frame[21]:
            return False
        trig_seq, roll, pitch, yaw, att_seq = struct.unpack('<IfffI', frame[1:21])
        f = SyncEventFrame(trigger_seq=trig_seq, roll=roll, pitch=pitch,
                           yaw=yaw, att_seq=att_seq)
        if self.on_sync:
            self.on_sync(f)
        return True

    def _parse_config(self, frame: bytes) -> bool:
        if len(frame) < 5:
            return False
        cmd = frame[1]
        payload_len = frame[2]

        xor_v = _xor_checksum(frame[1:len(frame)-1])
        if xor_v != frame[-1]:
            return False

        if payload_len < 1:
            return False
        if self.on_config_reply:
            self.on_config_reply(cmd, frame[3] == 0x01)

        if cmd == 0x01 and payload_len == 81 and frame[3] == 0x01:
            # GET_CONFIG reply: [status=1] + 80 bytes
            vals = struct.unpack('<17fIIHH', frame[4:84])
            cfg = Config(
                gyro_tilt_std_1s_deg=vals[0], sigma_gyro_bias=vals[1],
                gyro_scale_factor_error=vals[2], accel_variance_base=vals[3],
                accel_g_3sigma=vals[4], accel_release_tau=vals[5],
                nis_reject=vals[6], nis_inflate_gamma=vals[7],
                zaru_variance=vals[8], static_gyro_threshold=vals[9],
                static_accel_rel_std_thresh=vals[10], motion_gyro_full=vals[11],
                motion_accel_full=vals[12], initialization_tilt_seconds=vals[13],
                initial_attitude_variance=vals[14], initial_bias_variance=vals[15],
                maximum_delta_seconds=vals[16],
                zaru_static_frames=vals[17], motion_window_length=vals[18],
                trigger_divider=vals[19], trigger_duty=vals[20])
            if self.on_config:
                self.on_config(cfg)
        elif cmd == 0x04 and payload_len == 69 and frame[3] == 0x01:
            vals = struct.unpack('<III14f', frame[4:72])
            cfg = FilterConfig(vals[0], vals[1], vals[2], vals[3], vals[4],
                tuple(vals[5:8]), tuple(vals[8:11]), tuple(vals[11:14]), tuple(vals[14:17]))
            if self.on_filter_config:
                self.on_filter_config(cfg)
        return True
