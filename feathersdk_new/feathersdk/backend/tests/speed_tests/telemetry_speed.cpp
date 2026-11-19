// Speed of Telemetry::update_telemetry.
//
// One iteration is one call. n_updates is the batch size. Each update copies
// n_elems f32 values into that channel's ring. Payloads are built first.
// Warmup is not timed.
//
// Build and run: make telemetry_speed

#include "../../src/robot/telemetry.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

static constexpr std::uint32_t kSeed = 1;
static constexpr int kIterations = 200000;
static constexpr int kWarmupPct = 10;
static constexpr int kWarmup = kIterations * kWarmupPct / 100;

static constexpr int kBatch = 12;
static constexpr int kRandomMaxBatch = 26;
static constexpr int kMaxElems = 20;
static constexpr int kChannels = kRandomMaxBatch;
static constexpr std::uint64_t kCapacity = 150; // larger than kMaxElems, so every element is copied
static_assert(kCapacity > kMaxElems, "a full update must fit in the ring");

struct Batch {
    int n_updates;
    std::vector<std::uint8_t> payload;
};

static std::uint32_t next_u32(std::uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return state;
}

static int next_range(std::uint32_t& state, int lo, int hi) {
    return lo + static_cast<int>(next_u32(state) % static_cast<std::uint32_t>(hi - lo + 1));
}

// n_updates records. Record i writes n_elems f32 values into channel i.
static std::vector<std::uint8_t> make_payload(int n_updates, int n_elems) {
    const std::size_t stride = 4 + static_cast<std::size_t>(n_elems) * 4;
    std::vector<std::uint8_t> payload(static_cast<std::size_t>(n_updates) * stride);
    for (int i = 0; i < n_updates; ++i) {
        std::uint8_t* rec = payload.data() + static_cast<std::size_t>(i) * stride;
        const unsigned channel = static_cast<unsigned>(i % kChannels);
        rec[0] = static_cast<std::uint8_t>(channel);
        rec[1] = static_cast<std::uint8_t>(channel >> 8);
        rec[2] = static_cast<std::uint8_t>(n_elems);
        rec[3] = static_cast<std::uint8_t>(n_elems >> 8);
    }
    return payload;
}

static bool setup(feather::robot::Telemetry& telem) {
    for (int i = 0; i < kChannels; ++i) {
        if (telem.add_telemetry(kCapacity, DATA_TYPE_F32) < 0) return false;
    }
    return telem.initialize();
}

static double bench(feather::robot::Telemetry& telem, int n_updates, const std::vector<std::uint8_t>& payload) {
    const auto* data = payload.data();
    const std::size_t size = payload.size();
    for (int i = 0; i < kWarmup; ++i) {
        if (!telem.update_telemetry(n_updates, data, size)) return -1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        if (!telem.update_telemetry(n_updates, data, size)) return -1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::nano>(t1 - t0).count();
}

static double bench_random(feather::robot::Telemetry& telem, const std::vector<Batch>& batches) {
    for (int i = 0; i < kWarmup; ++i) {
        const Batch& batch = batches[static_cast<std::size_t>(i)];
        if (!telem.update_telemetry(batch.n_updates, batch.payload.data(), batch.payload.size())) return -1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        const Batch& batch = batches[static_cast<std::size_t>(kWarmup + i)];
        if (!telem.update_telemetry(batch.n_updates, batch.payload.data(), batch.payload.size())) return -1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::nano>(t1 - t0).count();
}

static void report(const char* name, double ns, std::int64_t updates) {
    std::printf("%s\n", name);
    std::printf("  per-batch   %8.1f ns\n", ns / kIterations);
    std::printf("  per-update  %8.1f ns\n\n", ns / static_cast<double>(updates));
}

int main() {
    feather::robot::Telemetry telem;
    if (!setup(telem)) {
        std::fprintf(stderr, "telemetry setup failed\n");
        return 1;
    }

    const int fixed[][2] = {{1, 1}, {kBatch, 1}, {kBatch, kMaxElems}};
    const char* names[] = {
        "batches of 1, 1 update each",
        "batches of 12, 1 update each",
        "batches of 12, 20 updates each",
    };
    double fixed_ns[3];
    std::vector<std::uint8_t> payloads[3];
    for (int i = 0; i < 3; ++i) {
        payloads[i] = make_payload(fixed[i][0], fixed[i][1]);
        fixed_ns[i] = bench(telem, fixed[i][0], payloads[i]);
        if (fixed_ns[i] < 0) {
            std::fprintf(stderr, "%s failed\n", names[i]);
            return 1;
        }
    }

    std::vector<Batch> random_batches(static_cast<std::size_t>(kWarmup + kIterations));
    std::uint32_t state = kSeed;
    std::int64_t random_updates = 0;
    for (int i = 0; i < kWarmup + kIterations; ++i) {
        const int n_updates = next_range(state, 1, kRandomMaxBatch);
        const int n_elems = next_range(state, 1, kMaxElems);
        random_batches[static_cast<std::size_t>(i)] = Batch{n_updates, make_payload(n_updates, n_elems)};
        if (i >= kWarmup) random_updates += n_updates;
    }
    const double random_ns = bench_random(telem, random_batches);
    if (random_ns < 0) {
        std::fprintf(stderr, "random batches failed\n");
        return 1;
    }

    std::printf("telemetry speed, %d iterations, %d%% warmup, seed %u\n\n", kIterations, kWarmupPct, kSeed);
    for (int i = 0; i < 3; ++i) {
        report(names[i], fixed_ns[i], static_cast<std::int64_t>(kIterations) * fixed[i][0]);
    }
    report("random batches 1-26, random updates 1-20", random_ns, random_updates);
    return 0;
}
