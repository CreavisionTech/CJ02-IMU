/*!
 * \file  cj02_imu.h
 * \brief CJ02-IMU SDK — header-only C++ library for reading IMU data.
 *
 * Parses the CJ02-IMU binary serial protocol (0xAA raw, 0xAB attitude,
 * 0xAD sync event, 0xAC config) and provides callbacks for each frame type.
 *
 * Usage:
 *   ```cpp
 *   #include "cj02_imu.h"
 *   CJ02IMU imu;
 *   imu.onAttitude([](const AttitudeFrame& f) {
 *       printf("roll=%.2f pitch=%.2f yaw=%.2f\n", f.roll, f.pitch, f.yaw);
 *   });
 *   imu.open("/dev/ttyUSB0", 460800);   // or "COM11" on Windows
 *   imu.run();                           // blocking loop
 *   ```
 *
 * License: MIT
 */

#ifndef CJ02_IMU_H
#define CJ02_IMU_H

#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>
#include <string>
#include <atomic>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstdlib>
#endif

// usleep fallback for Windows
#ifdef _WIN32
#define usleep(us) Sleep((us) / 1000)
#endif

namespace cj02 {

// ============================================================
// Frame structures (mirrors the on-wire format)
// ============================================================

#pragma pack(push, 1)

struct RawFrame {
    static constexpr uint8_t SYNC = 0xAA;
    static constexpr int SIZE = 16;

    uint8_t  sync;        // 0xAA
    int16_t  acc[3];      // ±2g, 16384 LSB/g
    int16_t  gyr[3];      // ±2000°/s, 16.4 LSB/(°/s)
    uint16_t seq;         // frame sequence
    uint8_t  checksum;    // XOR of bytes 1..14

    float accX_mg() const { return acc[0] * 1000.0f / 16384.0f; }
    float accY_mg() const { return acc[1] * 1000.0f / 16384.0f; }
    float accZ_mg() const { return acc[2] * 1000.0f / 16384.0f; }
    float gyrX_dps() const { return gyr[0] / 16.4f; }
    float gyrY_dps() const { return gyr[1] / 16.4f; }
    float gyrZ_dps() const { return gyr[2] / 16.4f; }
};

struct AttitudeFrame {
    static constexpr uint8_t SYNC = 0xAB;
    static constexpr int SIZE = 20;

    uint8_t  sync;        // 0xAB
    float    roll;        // degrees, -180..180
    float    pitch;       // degrees, -90..90
    float    yaw;         // degrees, -180..180
    uint32_t seq;         // attitude sequence
    uint8_t  mode;        // 0=INIT, 1=RUN, 2=NO_ACC, 3=REJECT
    uint8_t  flags;       // bit0=accel_used, bit1=zaru_used, bit2=is_static
    uint8_t  checksum;    // XOR of bytes 1..18

    bool accelUsed() const { return flags & 0x01; }
    bool zaruUsed()  const { return flags & 0x02; }
    bool isStatic()  const { return flags & 0x04; }
    const char* modeName() const {
        switch (mode) {
            case 0: return "INIT";
            case 1: return "RUN";
            case 2: return "NO_ACC";
            case 3: return "REJECT";
            default: return "UNKNOWN";
        }
    }
};

struct SyncEventFrame {
    static constexpr uint8_t SYNC = 0xAD;
    static constexpr int SIZE = 22;

    uint8_t  sync;        // 0xAD
    uint32_t triggerSeq;  // 1600Hz sample counter at trigger
    float    roll;        // degrees at trigger
    float    pitch;
    float    yaw;
    uint32_t attSeq;      // attitude frame seq at trigger
    uint8_t  checksum;    // XOR of bytes 1..20
};

#pragma pack(pop)

// ============================================================
// Config structure (80 bytes payload)
// ============================================================

struct Config {
    float gyro_tilt_std_1s_deg;
    float sigma_gyro_bias;
    float gyro_scale_factor_error;
    float accel_variance_base;
    float accel_g_3sigma;
    float accel_release_tau;
    float nis_reject;
    float nis_inflate_gamma;
    float zaru_variance;
    float static_gyro_threshold;
    float static_accel_rel_std_thresh;
    float motion_gyro_full;
    float motion_accel_full;
    float initialization_tilt_seconds;
    float initial_attitude_variance;
    float initial_bias_variance;
    float maximum_delta_seconds;
    uint32_t zaru_static_frames;
    uint32_t motion_window_length;
    uint16_t trigger_divider;
    uint16_t trigger_duty;
};
static_assert(sizeof(Config) == 80, "Config must be 80 bytes");

// ============================================================
// Serial port abstraction
// ============================================================

class SerialPort {
public:
    SerialPort() {
#ifdef _WIN32
        handle_ = INVALID_HANDLE_VALUE;
#else
        fd_ = -1;
#endif
    }
    ~SerialPort() { close(); }

    bool open(const std::string& port, int baud) {
#ifdef _WIN32
        handle_ = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE,
                              0, NULL, OPEN_EXISTING, 0, NULL);
        if (handle_ == INVALID_HANDLE_VALUE) return false;

        DCB dcb = {};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(handle_, &dcb)) { close(); return false; }
        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity   = NOPARITY;
        if (!SetCommState(handle_, &dcb)) { close(); return false; }

        COMMTIMEOUTS to = {};
        to.ReadIntervalTimeout         = 50;
        to.ReadTotalTimeoutConstant    = 50;
        to.ReadTotalTimeoutMultiplier  = 10;
        to.WriteTotalTimeoutConstant   = 50;
        to.WriteTotalTimeoutMultiplier = 10;
        SetCommTimeouts(handle_, &to);
        return true;
#else
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) return false;

        termios tty = {};
        if (tcgetattr(fd_, &tty) != 0) { close(); return false; }

        cfsetospeed(&tty, B460800);
        cfsetispeed(&tty, B460800);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;  // 100ms timeout

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }

        // Set to blocking mode
        int flags = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
        return true;
#endif
    }

    void close() {
#ifdef _WIN32
        if (handle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(handle_);
            handle_ = INVALID_HANDLE_VALUE;
        }
#else
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
#endif
    }

    bool isOpen() const {
#ifdef _WIN32
        return handle_ != INVALID_HANDLE_VALUE;
#else
        return fd_ >= 0;
#endif
    }

    int read(uint8_t* buf, int len) {
#ifdef _WIN32
        DWORD bytesRead = 0;
        if (!ReadFile(handle_, buf, len, &bytesRead, NULL)) return -1;
        return (int)bytesRead;
#else
        return (int)::read(fd_, buf, len);
#endif
    }

    int write(const uint8_t* buf, int len) {
#ifdef _WIN32
        DWORD bytesWritten = 0;
        if (!WriteFile(handle_, buf, len, &bytesWritten, NULL)) return -1;
        return (int)bytesWritten;
#else
        return (int)::write(fd_, buf, len);
#endif
    }

private:
#ifdef _WIN32
    HANDLE handle_;
#else
    int fd_;
#endif
};

// ============================================================
// Main IMU class
// ============================================================

class CJ02IMU {
public:
    CJ02IMU() : running_(false), framesGood_(0), framesBad_(0) {}
    ~CJ02IMU() { stop(); }

    // Callbacks
    std::function<void(const RawFrame&)>       onRaw;
    std::function<void(const AttitudeFrame&)>  onAttitude;
    std::function<void(const SyncEventFrame&)> onSyncEvent;
    std::function<void(const Config&)>         onConfig;

    bool open(const std::string& port, int baud = 460800) {
        return serial_.open(port, baud);
    }

    SerialPort& getSerialPort() { return serial_; }

    void close() { stop(); serial_.close(); }

    // Blocking read loop. Call from your main thread or a worker thread.
    void run() {
        running_ = true;
        uint8_t buf[4096];
        while (running_) {
            int n = serial_.read(buf, sizeof(buf));
            if (n > 0) {
                feed(buf, n);
            } else if (n < 0) {
                break;  // error
            }
        }
        running_ = false;
    }

    void stop() { running_ = false; }
    bool isRunning() const { return running_; }

    // Feed raw bytes (for integration with your own I/O loop)
    void feed(const uint8_t* data, int len) {
        buffer_.insert(buffer_.end(), data, data + len);
        parseBuffer();
    }

    // Statistics
    uint32_t goodFrames() const { return framesGood_; }
    uint32_t badFrames()  const { return framesBad_; }

    // Config commands
    bool getConfig() {
        uint8_t cmd[3] = {0xAC, 0x01, 0x00};
        uint8_t xor_v = cmd[1] ^ cmd[2];
        uint8_t frame[5] = {0xAC, 0x01, 0x00, 0x00, xor_v};
        // frame = sync, cmd, len, ...no payload..., checksum
        // Actually: 0xAC | 0x01 | 0x00 | checksum (4 bytes)
        uint8_t pkt[4] = {0xAC, 0x01, 0x00, 0x01};
        pkt[3] = 0x01 ^ 0x00;  // cmd ^ len
        return serial_.write(pkt, 4) == 4;
    }

    bool setConfig(const Config& cfg) {
        uint8_t pkt[3 + 80 + 1];
        pkt[0] = 0xAC; pkt[1] = 0x02; pkt[2] = 80;
        memcpy(pkt + 3, &cfg, 80);
        uint8_t xor_v = pkt[1] ^ pkt[2];
        for (int i = 0; i < 80; i++) xor_v ^= pkt[3 + i];
        pkt[83] = xor_v;
        return serial_.write(pkt, 84) == 84;
    }

    bool resetConfig() {
        uint8_t pkt[4] = {0xAC, 0x03, 0x00, 0x00};
        pkt[3] = 0x03 ^ 0x00;
        return serial_.write(pkt, 4) == 4;
    }

private:
    SerialPort serial_;
    std::vector<uint8_t> buffer_;
    std::atomic<bool> running_;
    uint32_t framesGood_;
    uint32_t framesBad_;

    void parseBuffer() {
        while (!buffer_.empty()) {
            uint8_t sync = buffer_[0];
            int frameLen = 0;

            switch (sync) {
                case RawFrame::SYNC:       frameLen = RawFrame::SIZE;       break;
                case AttitudeFrame::SYNC:  frameLen = AttitudeFrame::SIZE;  break;
                case SyncEventFrame::SYNC: frameLen = SyncEventFrame::SIZE; break;
                case 0xAC: {
                    if (buffer_.size() < 3) return;
                    frameLen = 3 + buffer_[2] + 1;
                    break;
                }
                default:
                    buffer_.erase(buffer_.begin());
                    continue;
            }

            if ((int)buffer_.size() < frameLen) return;

            bool ok = false;
            if (sync == RawFrame::SYNC) {
                RawFrame f;
                memcpy(&f, buffer_.data(), RawFrame::SIZE);
                ok = checkXor(buffer_.data(), RawFrame::SIZE);
                if (ok && onRaw) onRaw(f);
            } else if (sync == AttitudeFrame::SYNC) {
                AttitudeFrame f;
                memcpy(&f, buffer_.data(), AttitudeFrame::SIZE);
                ok = checkXor(buffer_.data(), AttitudeFrame::SIZE);
                if (ok && onAttitude) onAttitude(f);
            } else if (sync == SyncEventFrame::SYNC) {
                SyncEventFrame f;
                memcpy(&f, buffer_.data(), SyncEventFrame::SIZE);
                ok = checkXor(buffer_.data(), SyncEventFrame::SIZE);
                if (ok && onSyncEvent) onSyncEvent(f);
            } else if (sync == 0xAC) {
                ok = handleConfigReply(buffer_.data(), frameLen);
            }

            if (ok) {
                framesGood_++;
                buffer_.erase(buffer_.begin(), buffer_.begin() + frameLen);
            } else {
                framesBad_++;
                // Only advance 1 byte to re-sync quickly
                buffer_.erase(buffer_.begin());
            }
        }
    }

    static bool checkXor(const uint8_t* data, int len) {
        uint8_t xor_v = 0;
        for (int i = 1; i < len - 1; i++) xor_v ^= data[i];
        return xor_v == data[len - 1];
    }

    bool handleConfigReply(const uint8_t* data, int len) {
        if (len < 5) return false;
        uint8_t cmd = data[1];
        uint8_t payloadLen = data[2];

        // verify checksum
        uint8_t xor_v = 0;
        for (int i = 1; i < len - 1; i++) xor_v ^= data[i];
        if (xor_v != data[len - 1]) return false;

        if (cmd == 0x01 && payloadLen == 81 && data[3] == 0x01) {
            // GET_CONFIG reply: [status=1] + 80 bytes config
            Config cfg;
            memcpy(&cfg, data + 4, 80);
            if (onConfig) onConfig(cfg);
        }
        return true;
    }
};

} // namespace cj02

#endif // CJ02_IMU_H
