// Time one INFO log to a file, and one INFO log to stdout, for four messages:
// a single character, a long string with nothing to format, one pre-generated
// system-time value, and a long string with many pre-generated random values.
//
// init() sticks for the life of the process, so each target runs in its own child.
// Console bytes are discarded, but stdout keeps the buffering it had at startup:
// a terminal is line-buffered, a pipe is fully buffered.
//
// Build and run: make log_write_speed

#include "../../src/utils/logging.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <random>
#include <sys/wait.h>
#include <unistd.h>

static constexpr int kWarmup = 30000;
static constexpr int kIters = 500000;
static constexpr int kLong = 512;
static constexpr int kPregen = 4096;
static constexpr int kValues = 16;
static const char* kLogPath = "/tmp/feather_log_write_speed.log";

#define A16 "AAAAAAAAAAAAAAAA"
#define A64 A16 A16 A16 A16
#define A256 A64 A64 A64 A64
#define A512 A256 A256
#define NVALS "%lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld"

enum class Target { File, Console };

struct Samples {
    char long_msg[kLong + 1];
    std::int64_t times[kPregen];
    std::int64_t randoms[kPregen];
};

static Samples make_samples() {
    Samples s;
    const volatile char fill = 'A';
    for (int i = 0; i < kLong; ++i) s.long_msg[i] = fill;
    s.long_msg[kLong] = '\0';

    for (int i = 0; i < kPregen; ++i) {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        s.times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    }
    std::mt19937_64 rng(1);
    for (int i = 0; i < kPregen; ++i) s.randoms[i] = static_cast<std::int64_t>(rng());
    return s;
}

// The long message is already text. Logging it through the format parameter copies it
// and does not substitute any values.
static void info_text(const char* msg) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    INFO(msg);
#pragma GCC diagnostic pop
}

template <void (*fn)(const Samples&, int)>
static double time_iters(const Samples& samples) {
    for (int i = 0; i < kWarmup; ++i) fn(samples, i);
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kIters; ++i) fn(samples, i);
    const auto t1 = std::chrono::steady_clock::now();
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    return static_cast<double>(ns) / static_cast<double>(kIters);
}

static void log_char(const Samples&, int) { INFO("x"); }

static void log_long(const Samples& s, int) { info_text(s.long_msg); }

static void log_decimal(const Samples& s, int i) { INFO("%lld", static_cast<long long>(s.times[i & (kPregen - 1)])); }

static void log_values(const Samples& s, int i) {
    const std::int64_t* r = s.randoms + ((i & ((kPregen / kValues) - 1)) * kValues);
    INFO(A512 " " NVALS,
        static_cast<long long>(r[0]), static_cast<long long>(r[1]), static_cast<long long>(r[2]),
        static_cast<long long>(r[3]), static_cast<long long>(r[4]), static_cast<long long>(r[5]),
        static_cast<long long>(r[6]), static_cast<long long>(r[7]), static_cast<long long>(r[8]),
        static_cast<long long>(r[9]), static_cast<long long>(r[10]), static_cast<long long>(r[11]),
        static_cast<long long>(r[12]), static_cast<long long>(r[13]), static_cast<long long>(r[14]),
        static_cast<long long>(r[15]));
}

static const char* kCaseNames[] = {
    "single character",
    "long string",
    "one decimal",
    "long string with values",
};
static constexpr int kNumCases = 4;

struct Result { double ns[kNumCases]; };

static Result bench(Target target) {
    Result out;
    for (double& ns : out.ns) ns = -1.0;

    if (target == Target::Console) {
        if (isatty(STDOUT_FILENO)) std::setvbuf(stdout, nullptr, _IOLBF, 0);
        const int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            ::dup2(devnull, STDOUT_FILENO);
            ::close(devnull);
        }
    }

    feather::log::Config config;
    config.console_enabled = target == Target::Console;
    config.file_enabled = target == Target::File;
    config.console_level = feather::log::Level::Info;
    config.file_level = feather::log::Level::Info;
    config.file_path = kLogPath;
    if (!feather::log::init(config)) return out;

    const Samples samples = make_samples();
    out.ns[0] = time_iters<log_char>(samples);
    out.ns[1] = time_iters<log_long>(samples);
    out.ns[2] = time_iters<log_decimal>(samples);
    out.ns[3] = time_iters<log_values>(samples);
    return out;
}

static Result run_child(Target target) {
    int fds[2];
    Result out;
    for (double& ns : out.ns) ns = -1.0;
    if (::pipe(fds) != 0) return out;
    const pid_t pid = ::fork();
    if (pid < 0) return out;
    if (pid == 0) {
        ::close(fds[0]);
        const Result result = bench(target);
        const ssize_t n = ::write(fds[1], &result, sizeof result);
        ::_exit(n == static_cast<ssize_t>(sizeof result) ? 0 : 1);
    }
    ::close(fds[1]);
    const ssize_t n = ::read(fds[0], &out, sizeof out);
    ::close(fds[0]);
    int status = 0;
    ::waitpid(pid, &status, 0);
    if (n != static_cast<ssize_t>(sizeof out)) {
        for (double& ns : out.ns) ns = -1.0;
    }
    return out;
}

int main() {
    ::unlink(kLogPath);
    const bool console_is_tty = isatty(STDOUT_FILENO) != 0;
    const Result file = run_child(Target::File);
    const Result console = run_child(Target::Console);
    ::unlink(kLogPath);

    std::printf("log write speed, %d messages after %d warmup\n", kIters, kWarmup);
    bool ok = true;
    for (int c = 0; c < kNumCases; ++c) {
        std::printf("\n%s\n", kCaseNames[c]);
        std::printf("  file:     %8.1f ns/msg\n", file.ns[c]);
        std::printf("  console:  %8.1f ns/msg (%s)\n", console.ns[c], console_is_tty ? "line buffered" : "fully buffered");
        ok = ok && file.ns[c] >= 0.0 && console.ns[c] >= 0.0;
    }
    return ok ? 0 : 1;
}
