// leap_mavlink_bridge: talks to the leap_ros firmware over its MAVLink UART,
// without micro-ROS. Prints odometry and link statistics once per second and
// can drive the robot at a fixed velocity. With --csv every odometry arrival is
// logged (host time and the firmware's own timestamp) for offline analysis.
//
// One thread, one epoll set: the serial port, two timerfds (command stream,
// report) and a signalfd for Ctrl-C. Nothing blocks except epoll_wait.

#include "leap_bridge/mavlink_link.h"
#include "leap_bridge/rate_stats.h"
#include "leap_bridge/serial_port.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <getopt.h>
#include <memory>
#include <string>
#include <vector>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>

using leap_bridge::MavlinkLink;
using leap_bridge::RateStats;
using leap_bridge::SerialPort;

namespace {

// The firmware stops after 500 ms without a command (kCommandTimeoutMs);
// 20 Hz leaves room for a few late or lost frames.
constexpr int kCommandPeriodMs = 50;
constexpr int kReportPeriodMs = 1000;
constexpr int kSilentReportsBeforeWarning = 3;

struct Options {
    std::string device = "/dev/ttyUSB0";
    int baud = 230400;  // kMavlinkUartBaudRate
    bool drive = false;
    float vx = 0.0f;
    float wz = 0.0f;
    double duration_s = 0.0;
    std::string csv_path;
};

// Closes the CSV log on every exit path.
using File = std::unique_ptr<FILE, int (*)(FILE *)>;

struct Telemetry {
    float x = 0.0f;
    float y = 0.0f;
    float yaw_rad = 0.0f;
    float vx = 0.0f;
    float wz = 0.0f;
    uint64_t heartbeats = 0;
    RateStats odom_rate;
    // Every odometry inter-arrival of the run, for the percentiles at exit.
    int64_t last_odom_ns = -1;
    std::vector<float> odom_gaps_ms;
};

void Usage(const char *argv0) {
    std::fprintf(stderr,
        "usage: %s [-d DEVICE] [-b BAUD] [--vx M_S] [--wz RAD_S] [-t SECONDS] [--csv FILE]\n"
        "  -d, --device    serial device (default /dev/ttyUSB0)\n"
        "  -b, --baud      baud rate (default 230400)\n"
        "      --vx        drive forward at this speed, m/s\n"
        "      --wz        turn at this rate, rad/s (counter-clockwise positive)\n"
        "  -t, --duration  stop driving and exit after this many seconds\n"
        "      --csv       log every odometry arrival to FILE\n"
        "Without --vx/--wz the robot is only listened to, never commanded.\n",
        argv0);
}

bool ParseOptions(int argc, char **argv, Options *opt) {
    enum { kOptVx = 1000, kOptWz, kOptCsv };
    static const option kLongOptions[] = {
        {"device", required_argument, nullptr, 'd'},
        {"baud", required_argument, nullptr, 'b'},
        {"vx", required_argument, nullptr, kOptVx},
        {"wz", required_argument, nullptr, kOptWz},
        {"duration", required_argument, nullptr, 't'},
        {"csv", required_argument, nullptr, kOptCsv},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0},
    };

    int c;
    while ((c = getopt_long(argc, argv, "d:b:t:h", kLongOptions, nullptr)) != -1) {
        switch (c) {
            case 'd': opt->device = optarg; break;
            case 'b': opt->baud = std::atoi(optarg); break;
            case 't': opt->duration_s = std::atof(optarg); break;
            case kOptVx: opt->vx = std::strtof(optarg, nullptr); opt->drive = true; break;
            case kOptWz: opt->wz = std::strtof(optarg, nullptr); opt->drive = true; break;
            case kOptCsv: opt->csv_path = optarg; break;
            default: return false;
        }
    }
    return optind == argc;
}

int64_t NowNs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int MakePeriodicTimer(int period_ms) {
    const int fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (fd < 0) {
        return -1;
    }
    itimerspec spec = {};
    spec.it_interval.tv_sec = period_ms / 1000;
    spec.it_interval.tv_nsec = (period_ms % 1000) * 1000000L;
    spec.it_value = spec.it_interval;
    if (timerfd_settime(fd, 0, &spec, nullptr) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

// SIGINT/SIGTERM arrive as readable events instead of interrupting the loop,
// so the shutdown path (send a stop command) runs in normal code.
int MakeSignalFd() {
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    if (sigprocmask(SIG_BLOCK, &mask, nullptr) != 0) {
        return -1;
    }
    return signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);
}

bool WatchReadable(int epoll_fd, int fd) {
    epoll_event ev = {};
    ev.events = EPOLLIN;
    ev.data.fd = fd;
    return epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev) == 0;
}

// A timerfd or signalfd stays readable until drained.
void Drain(int fd, size_t size) {
    uint8_t buf[sizeof(signalfd_siginfo)];
    while (read(fd, buf, size) > 0) {
    }
}

bool SendVelocity(SerialPort &port, float vx, float wz) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const size_t len = MavlinkLink::PackVelocityCommand(vx, wz, buf);
    return port.WriteAll(buf, len);
}

void HandleMessage(const mavlink_message_t &msg, int64_t start_ns, const leap_bridge::LinkStats &s,
                   FILE *csv, Telemetry *t) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_ODOMETRY: {
            mavlink_odometry_t odom;
            mavlink_msg_odometry_decode(&msg, &odom);
            const int64_t now_ns = NowNs();
            t->odom_rate.Add(now_ns);
            if (t->last_odom_ns >= 0) {
                t->odom_gaps_ms.push_back(static_cast<float>((now_ns - t->last_odom_ns) / 1e6));
            }
            t->last_odom_ns = now_ns;
            if (csv != nullptr) {
                // time_usec is the firmware tick count, so it moves in tick steps.
                std::fprintf(csv, "%.6f,%llu,%.4f,%.4f,%llu,%llu,%llu\n", (now_ns - start_ns) / 1e9,
                             static_cast<unsigned long long>(odom.time_usec / 1000), odom.vx, odom.yawspeed,
                             static_cast<unsigned long long>(s.frames),
                             static_cast<unsigned long long>(s.lost_frames),
                             static_cast<unsigned long long>(s.crc_errors));
            }
            t->x = odom.x;
            t->y = odom.y;
            t->vx = odom.vx;
            t->wz = odom.yawspeed;
            // q is w, x, y, z; a ground robot only needs the yaw out of it.
            const float *q = odom.q;
            t->yaw_rad = std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                                    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT:
            ++t->heartbeats;
            break;
        default:
            break;
    }
}

void Report(const Telemetry &t, const leap_bridge::LinkStats &s) {
    std::printf(
        "odom %5.1f Hz [%5.1f/%5.1f/%5.1f ms]  x=%7.3f y=%7.3f yaw=%7.1f deg  "
        "vx=%6.3f wz=%6.3f | frames %llu lost %llu crc %llu hb %llu\n",
        t.odom_rate.rate_hz(), t.odom_rate.min_ms(), t.odom_rate.mean_ms(), t.odom_rate.max_ms(),
        t.x, t.y, t.yaw_rad * 180.0f / static_cast<float>(M_PI), t.vx, t.wz,
        static_cast<unsigned long long>(s.frames),
        static_cast<unsigned long long>(s.lost_frames),
        static_cast<unsigned long long>(s.crc_errors),
        static_cast<unsigned long long>(t.heartbeats));
    std::fflush(stdout);
}

// Nearest-rank percentile of an already sorted sample.
float Percentile(const std::vector<float> &sorted, double p) {
    const size_t i = static_cast<size_t>(p * (sorted.size() - 1) + 0.5);
    return sorted[std::min(i, sorted.size() - 1)];
}

void PrintGapSummary(std::vector<float> gaps) {
    if (gaps.empty()) {
        return;
    }
    std::sort(gaps.begin(), gaps.end());
    std::printf("odom gaps: n %zu  p50 %.1f  p95 %.1f  p99 %.1f  max %.1f ms\n", gaps.size(),
                Percentile(gaps, 0.50), Percentile(gaps, 0.95), Percentile(gaps, 0.99), gaps.back());
}

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    if (!ParseOptions(argc, argv, &opt)) {
        Usage(argv[0]);
        return 2;
    }

    SerialPort port;
    if (!port.Open(opt.device, opt.baud)) {
        std::fprintf(stderr, "%s\n", port.error().c_str());
        return 1;
    }

    const int signal_fd = MakeSignalFd();
    const int report_timer = MakePeriodicTimer(kReportPeriodMs);
    const int command_timer = opt.drive ? MakePeriodicTimer(kCommandPeriodMs) : -1;
    const int epoll_fd = epoll_create1(EPOLL_CLOEXEC);
    if (signal_fd < 0 || report_timer < 0 || (opt.drive && command_timer < 0) || epoll_fd < 0 ||
        !WatchReadable(epoll_fd, port.fd()) || !WatchReadable(epoll_fd, signal_fd) ||
        !WatchReadable(epoll_fd, report_timer) ||
        (opt.drive && !WatchReadable(epoll_fd, command_timer))) {
        std::perror("setting up the event loop");
        return 1;
    }

    File csv(nullptr, &std::fclose);
    if (!opt.csv_path.empty()) {
        csv.reset(std::fopen(opt.csv_path.c_str(), "w"));
        if (!csv) {
            std::perror(opt.csv_path.c_str());
            return 1;
        }
        std::fprintf(csv.get(), "host_s,mcu_ms,vx_mps,wz_rads,frames,lost,crc\n");
    }

    const int64_t start_ns = NowNs();
    Telemetry telemetry;
    MavlinkLink link;
    link.SetHandler([&](const mavlink_message_t &msg) {
        HandleMessage(msg, start_ns, link.stats(), csv.get(), &telemetry);
    });

    std::printf("listening on %s at %d baud%s\n", opt.device.c_str(), opt.baud,
                opt.drive ? ", driving" : " (listen only)");

    uint64_t frames_at_last_report = 0;
    int silent_reports = 0;
    int exit_code = 0;
    bool running = true;

    while (running) {
        epoll_event events[4];
        const int n = epoll_wait(epoll_fd, events, 4, -1);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::perror("epoll_wait");
            exit_code = 1;
            break;
        }

        for (int i = 0; i < n && running; ++i) {
            const int fd = events[i].data.fd;

            if (fd == port.fd()) {
                if (events[i].events & (EPOLLERR | EPOLLHUP)) {
                    std::fprintf(stderr, "%s: device gone (unplugged?)\n", opt.device.c_str());
                    exit_code = 1;
                    running = false;
                    break;
                }
                uint8_t buf[512];
                ssize_t got;
                while ((got = port.Read(buf, sizeof(buf))) > 0) {
                    link.Feed(buf, static_cast<size_t>(got));
                }
                if (got < 0) {
                    std::perror("read");
                    exit_code = 1;
                    running = false;
                }
            } else if (fd == command_timer) {
                Drain(command_timer, sizeof(uint64_t));
                if (opt.duration_s > 0 && (NowNs() - start_ns) / 1e9 >= opt.duration_s) {
                    running = false;
                } else if (!SendVelocity(port, opt.vx, opt.wz)) {
                    std::fprintf(stderr, "%s\n", port.error().c_str());
                }
            } else if (fd == report_timer) {
                Drain(report_timer, sizeof(uint64_t));
                Report(telemetry, link.stats());
                telemetry.odom_rate.ResetWindow();

                silent_reports = link.stats().frames == frames_at_last_report ? silent_reports + 1 : 0;
                frames_at_last_report = link.stats().frames;
                if (silent_reports == kSilentReportsBeforeWarning) {
                    std::fprintf(stderr,
                        "no MAVLink frames for %d s: is the robot in MAVLink UART mode, "
                        "and is this its UART0 port?\n", kSilentReportsBeforeWarning);
                }
                if (!opt.drive && opt.duration_s > 0 &&
                    (NowNs() - start_ns) / 1e9 >= opt.duration_s) {
                    running = false;
                }
            } else if (fd == signal_fd) {
                Drain(signal_fd, sizeof(signalfd_siginfo));
                running = false;
            }
        }
    }

    // Stop explicitly rather than waiting out the firmware's 500 ms timeout.
    if (opt.drive && !SendVelocity(port, 0.0f, 0.0f)) {
        std::fprintf(stderr, "failed to send stop: %s\n", port.error().c_str());
    }
    const leap_bridge::LinkStats &s = link.stats();
    std::printf("total: %.1f s, %llu bytes, frames %llu, lost %llu, crc %llu\n",
                (NowNs() - start_ns) / 1e9,
                static_cast<unsigned long long>(s.bytes),
                static_cast<unsigned long long>(s.frames),
                static_cast<unsigned long long>(s.lost_frames),
                static_cast<unsigned long long>(s.crc_errors));
    PrintGapSummary(telemetry.odom_gaps_ms);

    close(epoll_fd);
    close(report_timer);
    if (command_timer >= 0) {
        close(command_timer);
    }
    close(signal_fd);
    return exit_code;
}
