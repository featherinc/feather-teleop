// Bring up vcan0, run two virtual Robstride neck motors on it, and talk to them through the
// emulated Teensy. Builds the neck the same way the backend does, then commands a pose and checks
// that feedback comes back.
//
// Build and run from backend/: make neck_vcan
// The process re-execs into a user, network, and mount namespace so it can create vcan0 without
// root. Sysfs is remounted so the emulator's /sys/class/net scan sees that interface, and loopback
// is brought up so the emulated Teensy is reachable at 127.0.0.1.

#include "src/backend.hpp"
#include "src/robot/actuators/sim/virtual_robstride.hpp"
#include "logging.hpp"
#include "run.hpp"
#include "system.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstdlib>
#include <unistd.h>

namespace {

constexpr char kIface[] = "vcan0";
constexpr int kPitchId = 0x30;
constexpr int kYawId = 0x31;
constexpr double kPitch = -0.15;
constexpr double kYaw = 0.40;

int run_ip(const std::vector<std::string>& args, std::string* err) {
    const int rc = ::run(args, 3, nullptr, nullptr, err);
    if (rc == 0) return 0;
    std::vector<std::string> sudo_args{"sudo", "-n"};
    sudo_args.insert(sudo_args.end(), args.begin(), args.end());
    std::string sudo_err;
    const int sudo_rc = ::run(sudo_args, 3, nullptr, nullptr, &sudo_err);
    if (sudo_rc != 0 && err != nullptr) *err = sudo_err.empty() ? *err : sudo_err;
    return sudo_rc;
}

// Re-exec under unshare when this process cannot create network interfaces. The child inherits
// NECK_VCAN_NS and skips this. Returns only when already inside that namespace.
void enter_namespace(int argc, char** argv) {
    if (std::getenv("NECK_VCAN_NS") != nullptr) return;
    if (setenv("NECK_VCAN_NS", "1", 1) != 0) {
        throw std::runtime_error("setenv failed");
    }
    std::vector<char*> args{
        const_cast<char*>("unshare"), const_cast<char*>("--user"), const_cast<char*>("--map-root-user"),
        const_cast<char*>("--net"), const_cast<char*>("--mount")
    };
    for (int i = 0; i < argc; ++i) args.push_back(argv[i]);
    args.push_back(nullptr);
    execvp(args[0], args.data());
    throw std::runtime_error(std::string("unshare failed: ") + std::strerror(errno));
}

void ensure_vcan(const std::string& name) {
    std::string err;
    // The host sysfs does not show interfaces created in this namespace. Remount it, then bring
    // loopback up. The emulator listens on 127.0.0.1 and discovers CAN from /sys/class/net.
    if (std::getenv("NECK_VCAN_NS") != nullptr) {
        if (::run({"mount", "-t", "sysfs", "sysfs", "/sys"}, 3, nullptr, nullptr, &err) != 0) {
            throw std::runtime_error("could not remount sysfs: " + err);
        }
        if (::run({"ip", "link", "set", "lo", "up"}, 3, nullptr, nullptr, &err) != 0) {
            throw std::runtime_error("could not bring up loopback: " + err);
        }
    }
    if (::run({"ip", "link", "show", name}, 3, nullptr, nullptr, &err) != 0) {
        ::run({"modprobe", "vcan"}, 3);
        if (run_ip({"ip", "link", "add", "dev", name, "type", "vcan"}, &err) != 0) {
            throw std::runtime_error("could not create " + name + ": " + err);
        }
    }
    if (run_ip({"ip", "link", "set", name, "up"}, &err) != 0) {
        throw std::runtime_error("could not bring up " + name + ": " + err);
    }
}

int open_can(const char* name) {
    const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) throw std::runtime_error(std::string("CAN socket failed: ") + std::strerror(errno));

    const int own = 0;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own, sizeof own) < 0) {
        close(fd);
        throw std::runtime_error(std::string("CAN_RAW_RECV_OWN_MSGS failed: ") + std::strerror(errno));
    }

    ifreq ifr{};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", name);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        close(fd);
        throw std::runtime_error(std::string("no such CAN interface: ") + name);
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
        close(fd);
        throw std::runtime_error(std::string("CAN bind failed: ") + std::strerror(errno));
    }
    return fd;
}

void send_frame(int fd, const feather::robot::sim::SimCanFrame& frame) {
    can_frame out{};
    out.can_id = frame.id | CAN_EFF_FLAG;
    out.can_dlc = frame.len;
    std::memcpy(out.data, frame.data, 8);
    const ssize_t n = write(fd, &out, sizeof out);
    if (n != static_cast<ssize_t>(sizeof out)) {
        throw std::runtime_error(std::string("CAN write failed: ") + std::strerror(errno));
    }
}

// Both neck motors on one vcan socket. The emulated Teensy discovers this interface and probes it.
class VcanMotors {
public:
    VcanMotors(const char* iface, int pitch_id, int yaw_id, double pitch_min, double pitch_max, double yaw_min, double yaw_max)
        : fd_(open_can(iface)),
          pitch_(pitch_id, ROB_VERSION_RS05, pitch_min, pitch_max),
          yaw_(yaw_id, ROB_VERSION_RS05, yaw_min, yaw_max) {
        io_ = std::thread([this] { io_main(); });
        sim_ = std::thread([this] { sim_main(); });
    }

    ~VcanMotors() {
        stop_.store(true);
        if (io_.joinable()) io_.join();
        if (sim_.joinable()) sim_.join();
        if (fd_ >= 0) close(fd_);
    }

    VcanMotors(const VcanMotors&) = delete;
    VcanMotors& operator=(const VcanMotors&) = delete;

private:
    int fd_;
    feather::robot::sim::VirtualRobstride pitch_;
    feather::robot::sim::VirtualRobstride yaw_;
    std::mutex mu_;
    std::atomic<bool> stop_{false};
    std::thread io_;
    std::thread sim_;

    void flush() {
        feather::robot::sim::SimCanFrame frame;
        while (pitch_.pop_frame(frame)) send_frame(fd_, frame);
        while (yaw_.pop_frame(frame)) send_frame(fd_, frame);
    }

    void io_main() {
        while (!stop_.load()) {
            pollfd pfd{};
            pfd.fd = fd_;
            pfd.events = POLLIN;
            if (poll(&pfd, 1, 20) <= 0) continue;
            can_frame in{};
            if (read(fd_, &in, sizeof in) != static_cast<ssize_t>(sizeof in)) continue;
            if ((in.can_id & CAN_EFF_FLAG) == 0) continue;

            const uint32_t id = in.can_id & CAN_EFF_MASK;
            std::lock_guard<std::mutex> lock(mu_);
            try {
                pitch_.handle_frame(id, in.data, in.can_dlc);
                yaw_.handle_frame(id, in.data, in.can_dlc);
                flush();
            } catch (const std::exception& ex) {
                std::fprintf(stderr, "virtual motor rejected a frame: %s\n", ex.what());
            }
        }
    }

    void sim_main() {
        constexpr double kDt = 0.005;
        while (!stop_.load()) {
            {
                std::lock_guard<std::mutex> lock(mu_);
                try {
                    pitch_.step(kDt);
                    yaw_.step(kDt);
                    flush();
                } catch (const std::exception& ex) {
                    std::fprintf(stderr, "virtual motor step failed: %s\n", ex.what());
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
};

feather::robot::RobstrideMotorConfig motor_cfg(
    const char* name, int id, double angle_min, double angle_max
) {
    feather::robot::RobstrideMotorConfig cfg;
    cfg.name = name;
    cfg.id = id;
    cfg.version = ROB_VERSION_RS05;
    cfg.direction = 1;
    cfg.max_velocity = 10;
    cfg.max_torque = 5.5;
    cfg.max_acceleration = 50;
    cfg.angle_min = angle_min;
    cfg.angle_max = angle_max;
    cfg.loc_kp = 80;
    cfg.message_block_timeout = 1.0;
    return cfg;
}

bool wait_for_pose(feather::Backend& backend, double pitch, double yaw, double tol, double seconds) {
    const feather_time_t deadline = feather::time::now() + feather::time::from_seconds(seconds);
    while (feather::time::now() < deadline) {
        double got_pitch = 0;
        double got_yaw = 0;
        if (backend.neck.pitch_motor.try_latest(ROB_PARAM_CODE_FB_POSITION, &got_pitch) &&
            backend.neck.yaw_motor.try_latest(ROB_PARAM_CODE_FB_POSITION, &got_yaw) &&
            std::fabs(got_pitch - pitch) < tol && std::fabs(got_yaw - yaw) < tol) {
            std::printf("pitch %.3f rad, yaw %.3f rad\n", got_pitch, got_yaw);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    double got_pitch = 0;
    double got_yaw = 0;
    backend.neck.pitch_motor.try_latest(ROB_PARAM_CODE_FB_POSITION, &got_pitch);
    backend.neck.yaw_motor.try_latest(ROB_PARAM_CODE_FB_POSITION, &got_yaw);
    std::fprintf(stderr, "pose timeout: pitch %.3f (want %.3f), yaw %.3f (want %.3f)\n", got_pitch, pitch, got_yaw, yaw);
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    feather::log::Config log_cfg;
    log_cfg.console_enabled = true;
    log_cfg.file_enabled = false;
    feather::log::init(log_cfg);

    try {
        enter_namespace(argc, argv);
        ensure_vcan(kIface);
        // Motors have to be listening before the emulator discovers vcan0 and the neck probes it.
        VcanMotors motors(kIface, kPitchId, kYawId, -2.20, 0.10, -1.66, 1.66);

        feather::BackendConfig cfg;
        cfg.name = "neck-vcan";
        cfg.comms.teensy.use_emulated_teensy = true;
        cfg.neck.pitch_motor = motor_cfg("neck_pitch", kPitchId, -2.20, 0.10);
        cfg.neck.yaw_motor = motor_cfg("neck_yaw", kYawId, -1.66, 1.66);

        feather::Backend backend(cfg);
        std::printf(
            "neck initialized: pitch bus %d, yaw bus %d\n",
            backend.neck.pitch_motor.config.endpoint, backend.neck.yaw_motor.config.endpoint
        );
        if (!wait_for_pose(backend, 0, 0, 0.05, 2)) return 1;

        backend.neck.set_position(kPitch, kYaw);
        if (!wait_for_pose(backend, kPitch, kYaw, 0.05, 8)) return 1;
        std::printf("commanded pose reached\n");
        return 0;
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "%s\n", ex.what());
        return 1;
    }
}
