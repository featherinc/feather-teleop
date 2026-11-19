// Fastest echo rate that still gets every reply back.
//
// Echoes are pipelined. The in-flight window doubles until a larger window
// drops a reply or stops raising throughput. The timed run uses that window.
// Latency is host send to host receive. A missing or mismatched reply fails the run.
//
// Build and run: make teensy_comms_speed
// Uses the connected Teensy 4.1.

#include "src/robot/comms/teensy_manager.hpp"
#include "logging.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <exception>
#include <numeric>
#include <optional>
#include <string_view>
#include <vector>

static constexpr double kProbeSeconds = 0.3;
static constexpr double kMeasureSeconds = 2.0;
static constexpr int kMaxWindow = 512;
static constexpr int kReplyTimeoutMs = 500;

struct Run {
    bool ok = false;
    int completed = 0;
    double seconds = 0;
    std::vector<std::uint64_t> latency_ns;
};

static bool echo_ok(
    const std::optional<feather::comms::TeensyResponse>& resp,
    const std::uint8_t* got, const std::uint8_t* sent
) {
    return resp && resp->opcode == TEENSY_COMMAND_OPCODE_RESP_ECHO &&
        resp->payload_size == ECHO_PAYLOAD_SIZE &&
        resp->received_ts >= resp->inserted_ts &&
        std::memcmp(got, sent, ECHO_PAYLOAD_SIZE) == 0;
}

// Take outstanding replies so the next window is not sharing a full buffer.
static void drain(
    feather::comms::TeensyManager& mgr, std::deque<feather::comms::SentTeensyMessage>& inflight
) {
    std::array<std::uint8_t, ECHO_PAYLOAD_SIZE> ignore{};
    while (!inflight.empty()) {
        if (!mgr.get_response(inflight.front(), feather::time::from_millis(kReplyTimeoutMs),
                feather::comms::payload_as(ignore))) {
            break;
        }
        inflight.pop_front();
    }
}

// Send echoes for `seconds`, keeping at most `window` replies outstanding.
// Round trips are stored when `record` is set.
static Run run_window(
    feather::comms::TeensyManager& mgr, int window, double seconds,
    const std::string_view payload, const std::uint8_t* sent, bool record
) {
    Run run;
    std::array<std::uint8_t, ECHO_PAYLOAD_SIZE> got{};
    std::deque<feather::comms::SentTeensyMessage> inflight;
    const auto t0 = std::chrono::steady_clock::now();
    const auto finish = [&](bool ok) {
        if (!ok) drain(mgr, inflight);
        run.ok = ok && run.completed > 0 && !mgr.poll_err();
        run.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return run;
    };

    while (true) {
        const bool sending =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds;

        while (sending && static_cast<int>(inflight.size()) < window) {
            const auto msg = mgr.send_message(TEENSY_COMMAND_OPCODE_CMD_ECHO, payload);
            if (msg.msg_id == 0) return finish(false);
            inflight.push_back(msg);
        }

        if (inflight.empty()) break;

        const auto resp = mgr.get_response(
            inflight.front(), feather::time::from_millis(kReplyTimeoutMs), feather::comms::payload_as(got)
        );
        if (!echo_ok(resp, got.data(), sent)) {
            if (resp) inflight.pop_front();
            return finish(false);
        }
        if (record) run.latency_ns.push_back(resp->received_ts - resp->inserted_ts);
        inflight.pop_front();
        ++run.completed;
    }

    return finish(true);
}

static std::uint64_t percentile(const std::vector<std::uint64_t>& sorted, double p) {
    const std::size_t idx = static_cast<std::size_t>(std::ceil(p * static_cast<double>(sorted.size()))) - 1;
    return sorted[idx];
}

int main() {
    feather::log::Config log_cfg;
    log_cfg.console_enabled = false;
    log_cfg.file_enabled = false;
    feather::log::init(log_cfg);

    try {
        feather::comms::TeensyConfig cfg;
        cfg.use_emulated_teensy = false;
        feather::comms::TeensyManager mgr(cfg);

        std::array<std::uint8_t, ECHO_PAYLOAD_SIZE> sent{};
        for (std::uint8_t i = 0; i < ECHO_PAYLOAD_SIZE; ++i) sent[i] = static_cast<std::uint8_t>(0xA0u + i);
        const std::string_view payload(reinterpret_cast<const char*>(sent.data()), sent.size());

        const Run warmup = run_window(mgr, 1, 0.1, payload, sent.data(), false);
        if (!warmup.ok) {
            std::fprintf(stderr, "warmup echo failed\n");
            return 1;
        }

        int best = 1;
        double best_rate = 0;
        for (int window = 1; window <= kMaxWindow; window *= 2) {
            const Run probe = run_window(mgr, window, kProbeSeconds, payload, sent.data(), false);
            if (!probe.ok || mgr.poll_err()) {
                if (window == 1 || mgr.poll_err()) {
                    std::fprintf(stderr, "echo failed at window %d\n", window);
                    return 1;
                }
                break;
            }
            const double rate = static_cast<double>(probe.completed) / probe.seconds;
            if (best_rate > 0 && rate < best_rate * 1.05) break;
            best = window;
            best_rate = rate;
        }

        Run measured = run_window(mgr, best, kMeasureSeconds, payload, sent.data(), true);
        if (!measured.ok || measured.latency_ns.empty() || mgr.poll_err()) {
            std::fprintf(stderr, "dropped an echo during the measured run (window %d, %d replies)\n",
                best, measured.completed);
            return 1;
        }

        std::vector<std::uint64_t>& lat = measured.latency_ns;
        const double sum = static_cast<double>(
            std::accumulate(lat.begin(), lat.end(), static_cast<std::uint64_t>(0))
        );
        std::sort(lat.begin(), lat.end());
        const double rate = static_cast<double>(measured.completed) / measured.seconds;

        std::printf("teensy echo, window %d, %d messages in %.3f s\n", best, measured.completed, measured.seconds);
        std::printf("  messages/s  %8.1f\n", rate);
        std::printf("  latency us  min %8.1f  mean %8.1f  p95 %8.1f  p99 %8.1f  max %8.1f\n",
            static_cast<double>(lat.front()) / 1000.0,
            sum / static_cast<double>(lat.size()) / 1000.0,
            static_cast<double>(percentile(lat, 0.95)) / 1000.0,
            static_cast<double>(percentile(lat, 0.99)) / 1000.0,
            static_cast<double>(lat.back()) / 1000.0);
        return 0;
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "%s\n", ex.what());
        return 1;
    }
}
