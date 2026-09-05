/*!
 * \file  example.cpp
 * \brief CJ02-IMU C++ SDK example — read and print IMU data.
 *
 * Build:  mkdir build && cd build && cmake .. && make
 * Usage:  ./cj02_example /dev/ttyUSB0     (Linux)
 *         ./cj02_example COM11            (Windows)
 */

#include "cj02_imu.h"
#include <cstdio>
#include <csignal>

static volatile bool g_running = true;

void signalHandler(int signum) {
    (void)signum;
    g_running = false;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("Usage: %s <serial_port> [baud]\n", argv[0]);
        printf("  Linux: %s /dev/ttyUSB0\n", argv[0]);
        printf("  Win:   %s COM11\n", argv[0]);
        return 1;
    }

    const char* port = argv[1];
    int baud = (argc > 2) ? atoi(argv[2]) : 460800;

    signal(SIGINT, signalHandler);

    cj02::CJ02IMU imu;

    // Register callbacks (use ASSIGNMENT, not function-call syntax)
    imu.onAttitude = [](const cj02::AttitudeFrame& f) {
        printf("\rAttitude: R=%7.2f  P=%7.2f  Y=%7.2f  mode=%s  "
               "acc=%d zaru=%d static=%d    ",
               f.roll, f.pitch, f.yaw, f.modeName(),
               (int)f.accelUsed(), (int)f.zaruUsed(), (int)f.isStatic());
        fflush(stdout);
    };

    imu.onRaw = [](const cj02::RawFrame& f) {
        // Print raw data every 100th frame (8 Hz) to avoid flooding
        static int count = 0;
        if (++count % 100 == 0) {
            printf("\nRaw: acc=(%.1f, %.1f, %.1f)mg  gyr=(%.2f, %.2f, %.2f)dps  seq=%u  ",
                   f.accX_mg(), f.accY_mg(), f.accZ_mg(),
                   f.gyrX_dps(), f.gyrY_dps(), f.gyrZ_dps(), (unsigned)f.seq);
        }
    };

    imu.onSyncEvent = [](const cj02::SyncEventFrame& f) {
        printf("\n*** SYNC EVENT: trigger_seq=%u  R=%.1f P=%.1f Y=%.1f ***\n",
               (unsigned)f.triggerSeq, f.roll, f.pitch, f.yaw);
    };

    // Open serial port
    if (!imu.open(port, baud)) {
        printf("Error: cannot open %s @ %d baud\n", port, baud);
        return 1;
    }
    printf("Connected to CJ02-IMU on %s @ %d baud\n", port, baud);
    printf("Press Ctrl+C to quit\n\n");

    // Read config
    imu.getConfig();

    // Run in a simple loop (run() is blocking, but we can use feed() for custom I/O)
    // For this simple example, we use a non-blocking approach:
    uint8_t buf[4096];
    while (g_running) {
        int n = imu.getSerialPort().read(buf, sizeof(buf));
        if (n > 0) {
            imu.feed(buf, n);
        }
        // Small sleep to avoid busy-waiting
#ifdef _WIN32
        Sleep(1);
#else
        usleep(1000);
#endif
    }

    imu.close();

    printf("\n\nGood frames: %u  Bad frames: %u\n", imu.goodFrames(), imu.badFrames());
    return 0;
}
