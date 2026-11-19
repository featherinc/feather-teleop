// Speed of DataMonitorFileWriter.
//
// Candump is fixed-size CandumpTelemetry records: 1 per call, 32 per call, or a
// random 1-32 per call. Telemetry is one variable-size item per batch, with the
// same batch shapes as telemetry_speed. Each update is a u16 channel, a u16
// count, and that many f32s. Frame bytes and f32 bytes come from the shared LCG
// and are built before the timer starts.
//
// The file is large enough that these writes do not wrap or seal. Warmup and
// close are not timed. Counts, seed, and LCG match tests/speed_tests/monitor_write_speed.py.
//
// Build and run: make monitor_write_speed

#include "../../src/robot/services/data_monitor.hpp"
#include "qdefs/telemetry_qdef.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <cerrno>
#include <sys/stat.h>

static constexpr std::uint32_t kSeed = 1;
static constexpr int kIterations = 100000;
static constexpr int kWarmupPct = 10;
static constexpr int kWarmup = kIterations * kWarmupPct / 100;
static constexpr int kCalls = kWarmup + kIterations;

static constexpr int kCandumpMax = 32;
static constexpr int kBatch = 12;
static constexpr int kRandomMaxBatch = 26;
static constexpr int kMaxElems = 20;
static constexpr int kChannels = kRandomMaxBatch;

static constexpr const char* kDir = "/tmp/feather_cpp_monitor_write_speed";

static_assert(sizeof(CandumpTelemetry) == 21, "candump frame");
static_assert(sizeof(FthrFileHeader) == 256, "fthr file header");
static_assert(sizeof(FthrBlockHeader) == 128, "fthr block header");

using feather::robot::DataMonitorFileWriter;
using feather::robot::DataMonitorFileWriterConfig;

struct Batch {
    std::vector<std::uint8_t> payload;
};

struct TempFile {
    char path[256];

    explicit TempFile(const char* name) {
        std::snprintf(path, sizeof path, "%s/%s", kDir, name);
    }

    ~TempFile() { std::remove(path); }
};

struct Result {
    const char* name;
    double ns;
    std::int64_t updates;
};

static std::uint32_t next_u32(std::uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return state;
}

static int next_range(std::uint32_t& state, int lo, int hi) {
    return lo + static_cast<int>(next_u32(state) % static_cast<std::uint32_t>(hi - lo + 1));
}

static void fill_random(std::uint8_t* dst, std::size_t n, std::uint32_t& state) {
    std::size_t i = 0;
    for (; i + 4 <= n; i += 4) {
        const std::uint32_t word = next_u32(state);
        std::memcpy(dst + i, &word, 4);
    }
    if (i < n) {
        const std::uint32_t word = next_u32(state);
        std::memcpy(dst + i, &word, n - i);
    }
}

static std::vector<std::uint8_t> make_frames(int n, std::uint32_t& state) {
    std::vector<std::uint8_t> buf(static_cast<std::size_t>(n) * sizeof(CandumpTelemetry));
    fill_random(buf.data(), buf.size(), state);
    return buf;
}

// n_updates records. Record i writes n_elems f32 values into channel i.
static std::vector<std::uint8_t> make_telemetry(int n_updates, int n_elems, std::uint32_t& state) {
    const std::size_t stride = 4 + static_cast<std::size_t>(n_elems) * 4;
    std::vector<std::uint8_t> payload(static_cast<std::size_t>(n_updates) * stride);
    for (int i = 0; i < n_updates; ++i) {
        std::uint8_t* rec = payload.data() + static_cast<std::size_t>(i) * stride;
        const auto channel = static_cast<std::uint16_t>(i % kChannels);
        const auto count = static_cast<std::uint16_t>(n_elems);
        std::memcpy(rec, &channel, sizeof channel);
        std::memcpy(rec + 2, &count, sizeof count);
        fill_random(rec + 4, static_cast<std::size_t>(n_elems) * 4, state);
    }
    return payload;
}

static DataMonitorFileWriterConfig fixed_cfg(const char* path, std::uint64_t nbytes) {
    DataMonitorFileWriterConfig cfg;
    cfg.path = path;
    cfg.data_name = "candump";
    cfg.format = FTHR_FILE_FORMAT_FIXED;
    cfg.chunk_size = sizeof(CandumpTelemetry);
    // One spare record so the timed writes do not wrap.
    cfg.max_file_size = sizeof(FthrFileHeader) + nbytes + sizeof(CandumpTelemetry);
    return cfg;
}

static DataMonitorFileWriterConfig variable_cfg(const char* path, std::uint64_t payload_bytes, int n_calls) {
    const std::uint64_t items = payload_bytes + static_cast<std::uint64_t>(n_calls) * sizeof(std::uint64_t);
    DataMonitorFileWriterConfig cfg;
    cfg.path = path;
    cfg.data_name = "telemetry";
    cfg.format = FTHR_FILE_FORMAT_VARIABLE;
    // One spare byte so the timed writes do not fill the block and seal it.
    cfg.chunk_size = sizeof(FthrBlockHeader) + items + 1;
    cfg.max_file_size = sizeof(FthrFileHeader) + cfg.chunk_size;
    return cfg;
}

static double time_fixed(DataMonitorFileWriter& writer, const std::vector<std::uint8_t>& payload) {
    const auto* data = payload.data();
    const std::size_t size = payload.size();
    for (int i = 0; i < kWarmup; ++i) {
        if (!writer.write(data, size)) return -1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        if (!writer.write(data, size)) return -1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::nano>(t1 - t0).count();
}

static double time_random(DataMonitorFileWriter& writer, const std::vector<Batch>& batches) {
    for (int i = 0; i < kWarmup; ++i) {
        const Batch& batch = batches[static_cast<std::size_t>(i)];
        if (!writer.write(batch.payload.data(), batch.payload.size())) return -1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        const Batch& batch = batches[static_cast<std::size_t>(kWarmup + i)];
        if (!writer.write(batch.payload.data(), batch.payload.size())) return -1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::nano>(t1 - t0).count();
}

static bool expect_size(const char* path, std::uint64_t expect) {
    struct stat st {};
    if (stat(path, &st) != 0) {
        std::perror(path);
        return false;
    }
    if (static_cast<std::uint64_t>(st.st_size) != expect) {
        std::fprintf(stderr, "%s size %lld, expected %llu\n", path, static_cast<long long>(st.st_size),
                     static_cast<unsigned long long>(expect));
        return false;
    }
    return true;
}

static void report(const Result& result) {
    std::printf("%s\n", result.name);
    std::printf("  per-batch   %8.1f ns\n", result.ns / kIterations);
    std::printf("  per-update  %8.1f ns\n\n", result.ns / static_cast<double>(result.updates));
}

static int run_fixed(std::vector<Result>& results, const char* name, const char* file_name,
                     const std::vector<std::uint8_t>& payload, std::int64_t updates, std::uint64_t nbytes) {
    TempFile file(file_name);
    double ns = 0;
    {
        DataMonitorFileWriter writer(fixed_cfg(file.path, nbytes));
        ns = time_fixed(writer, payload);
    }
    if (ns < 0) {
        std::fprintf(stderr, "%s failed\n", name);
        return 1;
    }
    if (!expect_size(file.path, sizeof(FthrFileHeader) + nbytes)) return 1;
    results.push_back(Result{name, ns, updates});
    return 0;
}

static int run_random_fixed(std::vector<Result>& results, const char* name, const char* file_name,
                            const std::vector<Batch>& batches, std::int64_t updates, std::uint64_t nbytes) {
    TempFile file(file_name);
    double ns = 0;
    {
        DataMonitorFileWriter writer(fixed_cfg(file.path, nbytes));
        ns = time_random(writer, batches);
    }
    if (ns < 0) {
        std::fprintf(stderr, "%s failed\n", name);
        return 1;
    }
    if (!expect_size(file.path, sizeof(FthrFileHeader) + nbytes)) return 1;
    results.push_back(Result{name, ns, updates});
    return 0;
}

static int run_variable(std::vector<Result>& results, const char* name, const char* file_name,
                        const std::vector<std::uint8_t>& payload, std::int64_t updates, std::uint64_t payload_bytes) {
    TempFile file(file_name);
    double ns = 0;
    {
        DataMonitorFileWriter writer(variable_cfg(file.path, payload_bytes, kCalls));
        ns = time_fixed(writer, payload);
    }
    if (ns < 0) {
        std::fprintf(stderr, "%s failed\n", name);
        return 1;
    }
    const std::uint64_t items = payload_bytes + static_cast<std::uint64_t>(kCalls) * sizeof(std::uint64_t);
    if (!expect_size(file.path, sizeof(FthrFileHeader) + sizeof(FthrBlockHeader) + items)) return 1;
    results.push_back(Result{name, ns, updates});
    return 0;
}

static int run_random_variable(std::vector<Result>& results, const char* name, const char* file_name,
                               const std::vector<Batch>& batches, std::int64_t updates, std::uint64_t payload_bytes) {
    TempFile file(file_name);
    double ns = 0;
    {
        DataMonitorFileWriter writer(variable_cfg(file.path, payload_bytes, kCalls));
        ns = time_random(writer, batches);
    }
    if (ns < 0) {
        std::fprintf(stderr, "%s failed\n", name);
        return 1;
    }
    const std::uint64_t items = payload_bytes + static_cast<std::uint64_t>(kCalls) * sizeof(std::uint64_t);
    if (!expect_size(file.path, sizeof(FthrFileHeader) + sizeof(FthrBlockHeader) + items)) return 1;
    results.push_back(Result{name, ns, updates});
    return 0;
}

int main() {
    if (mkdir(kDir, 0755) != 0 && errno != EEXIST) {
        std::perror(kDir);
        return 1;
    }

    try {
        std::vector<Result> results;
        results.reserve(7);

        const int candump_items[] = {1, kCandumpMax};
        const char* candump_names[] = {"candump, 1 item per call", "candump, 32 items per call"};
        const char* candump_files[] = {"candump_1.fthr", "candump_32.fthr"};
        for (int i = 0; i < 2; ++i) {
            std::uint32_t state = kSeed;
            const std::vector<std::uint8_t> payload = make_frames(candump_items[i], state);
            const std::uint64_t nbytes = static_cast<std::uint64_t>(kCalls) * payload.size();
            const auto updates = static_cast<std::int64_t>(kIterations) * candump_items[i];
            if (run_fixed(results, candump_names[i], candump_files[i], payload, updates, nbytes) != 0) return 1;
        }

        {
            std::vector<Batch> batches(static_cast<std::size_t>(kCalls));
            std::uint32_t state = kSeed;
            std::uint64_t nbytes = 0;
            std::int64_t updates = 0;
            for (int i = 0; i < kCalls; ++i) {
                const int n = next_range(state, 1, kCandumpMax);
                batches[static_cast<std::size_t>(i)].payload = make_frames(n, state);
                nbytes += batches[static_cast<std::size_t>(i)].payload.size();
                if (i >= kWarmup) updates += n;
            }
            if (run_random_fixed(results, "candump, random 1-32 items per call", "candump_random.fthr", batches,
                                 updates, nbytes) != 0) {
                return 1;
            }
        }

        const int telem_shape[][2] = {{1, 1}, {kBatch, 1}, {kBatch, kMaxElems}};
        const char* telem_names[] = {
            "batches of 1, 1 update each",
            "batches of 12, 1 update each",
            "batches of 12, 20 updates each",
        };
        const char* telem_files[] = {"telem_1.fthr", "telem_12_1.fthr", "telem_12_20.fthr"};
        for (int i = 0; i < 3; ++i) {
            std::uint32_t state = kSeed;
            const std::vector<std::uint8_t> payload = make_telemetry(telem_shape[i][0], telem_shape[i][1], state);
            const std::uint64_t payload_bytes = static_cast<std::uint64_t>(kCalls) * payload.size();
            const auto updates = static_cast<std::int64_t>(kIterations) * telem_shape[i][0];
            if (run_variable(results, telem_names[i], telem_files[i], payload, updates, payload_bytes) != 0) return 1;
        }

        {
            std::vector<Batch> batches(static_cast<std::size_t>(kCalls));
            std::uint32_t state = kSeed;
            std::uint64_t payload_bytes = 0;
            std::int64_t updates = 0;
            for (int i = 0; i < kCalls; ++i) {
                const int n_updates = next_range(state, 1, kRandomMaxBatch);
                const int n_elems = next_range(state, 1, kMaxElems);
                batches[static_cast<std::size_t>(i)].payload = make_telemetry(n_updates, n_elems, state);
                payload_bytes += batches[static_cast<std::size_t>(i)].payload.size();
                if (i >= kWarmup) updates += n_updates;
            }
            if (run_random_variable(results, "random batches 1-26, random updates 1-20", "telem_random.fthr", batches,
                                    updates, payload_bytes) != 0) {
                return 1;
            }
        }

        std::printf("monitor file write speed, %d iterations, %d%% warmup, seed %u\n\n", kIterations, kWarmupPct, kSeed);
        for (const Result& result : results) report(result);
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "monitor write speed: %s\n", ex.what());
        return 1;
    }
    return 0;
}
