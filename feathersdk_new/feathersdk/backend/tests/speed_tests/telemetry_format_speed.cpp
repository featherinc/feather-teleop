// Speed of storing Teensy telemetry updates into the Jetson-side telemetry rings, for two wire formats.
//
// Standalone on purpose: the ring storage below is a copy of feather::robot::Telemetry (src/robot/telemetry.cpp) as of
// writing, so this keeps compiling and measuring the same thing after the real code changes.
//
// Formats (the payload is everything after the update count in the telemetry response):
//
//   grouped (current):  the Teensy sorts samples by telemetry before sending. One record per telemetry:
//                         [u16le telem_idx][u16le n_elems][n_elems * itemsize bytes]
//
//   ordered (new):      the Teensy stores samples one at a time in the order they arrive. One record per sample:
//                         [u16le telem_idx][itemsize bytes]
//                       The item size is not sent; the Jetson already knows it from the channel's dtype.
//
// Each format is timed twice:
//   "grouped (current)"    the current write_ring, which uses `%` (a 64-bit divide) to move start/end.
//   "grouped, no modulo"   `%` replaced with compare and subtract, the same trick the ordered path uses, so the format
//                          comparison is not just a comparison of ring bookkeeping.
//   "ordered, memcpy"      each value copied with a runtime-sized memcpy, like write_ring does.
//   "ordered, sized copy"  switches on the item size so each copy is a fixed 1/2/4/8 byte move.
//
// Comparisons ("~Nx faster/slower") are per-batch against "grouped (current)".
//
// Every scenario sends the same samples in every format. Samples arrive round-robin across telemetries (t0 s0, t1 s0,
// ..., t0 s1, t1 s1, ...), which is how they would come in from polling motors in turn. Before timing, the rings that
// each format produces are checked to hold the same values in the same order.
//
// Fairness:
//   - All payloads are built before timing. Values are random bit patterns, not zeros.
//   - Each format writes into its own fresh Telemetry with identical channels.
//   - Every timed run has an untimed warmup of kWarmupPct of its iterations.
//   - Each scenario runs kRounds rounds. The format order rotates every round, and the reported time is the median.
//   - The update functions are noinline, and a checksum of the rings is printed so stores cannot be dropped.
//
// "per-update" divides by the number of telemetry updates (telemetries with new data), the same unit as
// telemetry_speed.cpp, so the formats are compared on equal work. "per-sample" divides by the number of values.
//
// Build and run (from backend/):
//   g++ -std=c++17 -O3 -march=native -Wall -Wextra -Wshadow -o /tmp/telemetry_format_speed tests/speed_tests/telemetry_format_speed.cpp
//   /tmp/telemetry_format_speed

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

using u8 = std::uint8_t;
using u64 = std::uint64_t;

static constexpr std::uint32_t kSeed = 1;
static constexpr int kIterations = 200000;
static constexpr int kWarmupPct = 10;
static constexpr int kWarmup = kIterations * kWarmupPct / 100;
static constexpr int kRounds = 5;

static constexpr int kBatch = 12;
static constexpr int kRandomMaxBatch = 26;
static constexpr int kMaxElems = 20;
static constexpr int kChannels = kRandomMaxBatch;
static constexpr u64 kCapacity = 150;  // larger than kMaxElems, so every element is copied
static constexpr u64 kItemSize = 4;    // f32, same as telemetry_speed.cpp
static_assert(kCapacity > kMaxElems, "a full update must fit in the ring");

static constexpr u64 kFnvPrime = 1099511628211ull;

// -------------------------------------------------------------------------------------------------------------------
// Jetson-side storage. Same layout as feather::robot::Telemetry: one flat byte buffer, and per-channel offset,
// itemsize, capacity, start, end, and count arrays.
// -------------------------------------------------------------------------------------------------------------------

class Telemetry {
public:
    // kChannels identical channels of kCapacity items of kItemSize bytes.
    Telemetry()
        : data(kChannels * kCapacity * kItemSize, 0), offsets(kChannels), itemsizes(kChannels, kItemSize),
          capacities(kChannels, kCapacity), starts(kChannels, 0), ends(kChannels, 0), counts(kChannels, 0) {
        for (u64 i = 0; i < kChannels; ++i) offsets[i] = i * kCapacity * kItemSize;
    }

    // Current format. Same as Telemetry::update_telemetry. Mod selects write_ring's `%` or compare-and-subtract.
    template <bool Mod>
    __attribute__((noinline)) bool update_grouped(int n_updates, const u8* payload, std::size_t size) noexcept {
        if (n_updates < 0 || payload == nullptr) return fail("bad args");
        std::size_t pos = 0;
        for (int i = 0; i < n_updates; i++) {
            if (size - pos < 4) return fail("short header");
            const unsigned tidx = read_u16(payload + pos);
            const unsigned nelem = read_u16(payload + pos + 2);
            if (tidx >= itemsizes.size()) return fail("tidx out of range");
            const u64 nbytes = static_cast<u64>(nelem) * itemsizes[tidx];
            if (nbytes > static_cast<u64>(size - (pos + 4))) return fail("short data");
            write_ring<Mod>(tidx, nelem, payload + pos + 4);
            pos += 4 + static_cast<std::size_t>(nbytes);
        }
        return true;
    }

    // New format. Sized selects a fixed-size copy per item size instead of a runtime-sized memcpy.
    template <bool Sized>
    __attribute__((noinline)) bool update_ordered(int n_updates, const u8* payload, std::size_t size) noexcept {
        if (n_updates < 0 || payload == nullptr) return fail("bad args");
        std::size_t pos = 0;
        for (int i = 0; i < n_updates; i++) {
            if (size - pos < 2) return fail("short header");
            const unsigned tidx = read_u16(payload + pos);
            if (tidx >= itemsizes.size()) return fail("tidx out of range");
            const u64 item = itemsizes[tidx];
            if (item > static_cast<u64>(size - (pos + 2))) return fail("short data");
            const u64 end = ends[tidx];
            u8* dst = data.data() + offsets[tidx] + end * item;
            const u8* src = payload + pos + 2;
            if constexpr (Sized) {
                switch (item) {
                    case 1: *dst = *src; break;
                    case 2: std::memcpy(dst, src, 2); break;
                    case 4: std::memcpy(dst, src, 4); break;
                    case 8: std::memcpy(dst, src, 8); break;
                    default: std::memcpy(dst, src, static_cast<std::size_t>(item)); break;
                }
            } else {
                std::memcpy(dst, src, static_cast<std::size_t>(item));
            }
            push_one(tidx, end);
            pos += 2 + static_cast<std::size_t>(item);
        }
        return true;
    }

    // Channel `index` oldest to newest.
    std::vector<u8> read_channel(std::size_t index) const {
        const u64 item = itemsizes[index];
        std::vector<u8> out(static_cast<std::size_t>(counts[index] * item));
        for (u64 k = 0; k < counts[index]; ++k) {
            const u64 slot = (starts[index] + k) % capacities[index];
            std::memcpy(out.data() + k * item, data.data() + offsets[index] + slot * item, static_cast<std::size_t>(item));
        }
        return out;
    }

    u64 checksum() const {
        u64 h = 1469598103934665603ull;
        for (u8 b : data) h = (h ^ b) * kFnvPrime;
        for (u64 c : counts) h = (h ^ c) * kFnvPrime;
        return h;
    }

private:
    std::vector<u8> data;
    std::vector<u64> offsets, itemsizes, capacities, starts, ends, counts;

    static unsigned read_u16(const u8* p) { return p[0] | (static_cast<unsigned>(p[1]) << 8); }

    static bool fail(const char* why) {
        std::fprintf(stderr, "update failed: %s\n", why);
        return false;
    }

    // x mod capacity. Without Mod, x must be below 2 * capacity, which write_ring guarantees since n_elems < capacity.
    template <bool Mod>
    static u64 wrap(u64 x, u64 capacity) noexcept {
        if constexpr (Mod) return x % capacity;
        return x >= capacity ? x - capacity : x;
    }

    // Same as Telemetry::write_ring, with Mod = true.
    template <bool Mod>
    void write_ring(std::size_t index, u64 n_elems, const u8* src) noexcept {
        const u64 item = itemsizes[index];
        const u64 capacity = capacities[index];
        u8* dst = data.data() + offsets[index];

        // Keep only the last `capacity` elements when the batch fills the ring.
        if (n_elems >= capacity) {
            const u64 nbytes = capacity * item;
            if (nbytes != 0) {
                src += (n_elems - capacity) * item;
                std::memcpy(dst, src, static_cast<std::size_t>(nbytes));
            }
            starts[index] = 0;
            ends[index] = 0;
            counts[index] = capacity;
            return;
        }

        // Copy up to the end of the ring, then the rest at the beginning.
        const u64 end_idx = ends[index];
        const u64 space = capacity - end_idx;
        const u64 first = n_elems < space ? n_elems : space;
        const u64 nbytes_first = first * item;
        if (nbytes_first != 0) std::memcpy(dst + end_idx * item, src, static_cast<std::size_t>(nbytes_first));
        if (first < n_elems) {
            const u64 nbytes_rest = (n_elems - first) * item;
            if (nbytes_rest != 0) std::memcpy(dst, src + nbytes_first, static_cast<std::size_t>(nbytes_rest));
        }

        const u64 new_count = counts[index] + n_elems;
        if (new_count > capacity) {
            starts[index] = wrap<Mod>(starts[index] + (new_count - capacity), capacity);
            counts[index] = capacity;
        } else {
            counts[index] = new_count;
        }
        ends[index] = wrap<Mod>(end_idx + n_elems, capacity);
    }

    // Ring bookkeeping for one sample already written at slot `end`.
    void push_one(std::size_t index, u64 end) noexcept {
        const u64 capacity = capacities[index];
        ends[index] = wrap<false>(end + 1, capacity);
        if (counts[index] == capacity) {
            starts[index] = wrap<false>(starts[index] + 1, capacity);
        } else {
            counts[index] += 1;
        }
    }
};

// -------------------------------------------------------------------------------------------------------------------
// Payloads
// -------------------------------------------------------------------------------------------------------------------

enum Format { GROUPED, GROUPED_NOMOD, ORDERED_MEMCPY, ORDERED_SIZED, NUM_FORMATS };
static const char* const kFormatNames[NUM_FORMATS] = {
    "grouped (current)", "grouped, no modulo", "ordered, memcpy", "ordered, sized copy"};

struct Payload {
    int n_updates;  // records in the payload
    std::vector<u8> bytes;
};

// One batch: n_telems telemetries, each with n_elems new samples, encoded in both formats.
struct Batch {
    Payload grouped, ordered;
    int n_telems, n_elems;
};

static std::uint32_t next_u32(std::uint32_t& state) { return state = state * 1664525u + 1013904223u; }

static int next_range(std::uint32_t& state, int lo, int hi) {
    return lo + static_cast<int>(next_u32(state) % static_cast<std::uint32_t>(hi - lo + 1));
}

static u8* put_u16(u8* p, unsigned v) {
    p[0] = static_cast<u8>(v);
    p[1] = static_cast<u8>(v >> 8);
    return p + 2;
}

static Batch make_batch(int n_telems, int n_elems, std::uint32_t& state) {
    // values[t * n_elems + k] is sample k of telemetry t, as it arrived on the Teensy.
    std::vector<std::uint32_t> values(static_cast<std::size_t>(n_telems) * n_elems);
    for (auto& v : values) v = next_u32(state);

    Batch b{{n_telems, std::vector<u8>(n_telems * (4 + n_elems * kItemSize))},
            {n_telems * n_elems, std::vector<u8>(n_telems * n_elems * (2 + kItemSize))},
            n_telems, n_elems};

    u8* p = b.grouped.bytes.data();
    for (int t = 0; t < n_telems; ++t) {
        p = put_u16(put_u16(p, t % kChannels), n_elems);
        for (int k = 0; k < n_elems; ++k, p += kItemSize) std::memcpy(p, &values[t * n_elems + k], kItemSize);
    }

    p = b.ordered.bytes.data();
    for (int k = 0; k < n_elems; ++k) {
        for (int t = 0; t < n_telems; ++t, p += kItemSize) {
            p = put_u16(p, t % kChannels);
            std::memcpy(p, &values[t * n_elems + k], kItemSize);
        }
    }
    return b;
}

// -------------------------------------------------------------------------------------------------------------------
// Benchmark
// -------------------------------------------------------------------------------------------------------------------

static bool apply(Telemetry& telem, Format f, const Batch& b) {
    const Payload& p = f == GROUPED || f == GROUPED_NOMOD ? b.grouped : b.ordered;
    switch (f) {
        case GROUPED: return telem.update_grouped<true>(p.n_updates, p.bytes.data(), p.bytes.size());
        case GROUPED_NOMOD: return telem.update_grouped<false>(p.n_updates, p.bytes.data(), p.bytes.size());
        case ORDERED_MEMCPY: return telem.update_ordered<false>(p.n_updates, p.bytes.data(), p.bytes.size());
        case ORDERED_SIZED: return telem.update_ordered<true>(p.n_updates, p.bytes.data(), p.bytes.size());
        default: return false;
    }
}

// Batch i of a run is batches[i % batches.size()]. Warmup uses the first kWarmup, timing the next kIterations.
static const Batch& batch_at(const std::vector<Batch>& batches, int i) {
    return batches[static_cast<std::size_t>(i) % batches.size()];
}

static double run_once(Telemetry& telem, Format f, const std::vector<Batch>& batches) {
    for (int i = 0; i < kWarmup; ++i) {
        if (!apply(telem, f, batch_at(batches, i))) return -1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = kWarmup; i < kWarmup + kIterations; ++i) {
        if (!apply(telem, f, batch_at(batches, i))) return -1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::nano>(t1 - t0).count();
}

// Applies every batch with each format into fresh rings and checks that every channel holds the same samples.
static bool formats_agree(const std::vector<Batch>& batches) {
    Telemetry telems[NUM_FORMATS];
    for (int f = 0; f < NUM_FORMATS; ++f) {
        for (const Batch& b : batches) {
            if (!apply(telems[f], static_cast<Format>(f), b)) return false;
        }
    }
    for (int c = 0; c < kChannels; ++c) {
        const std::vector<u8> want = telems[GROUPED].read_channel(c);
        for (int f = 1; f < NUM_FORMATS; ++f) {
            if (telems[f].read_channel(c) != want) {
                std::fprintf(stderr, "%s disagrees with grouped on channel %d\n", kFormatNames[f], c);
                return false;
            }
        }
    }
    return true;
}

struct Result {
    double ns[NUM_FORMATS];     // median total over kIterations
    double bytes[NUM_FORMATS];  // mean payload bytes per batch
    std::int64_t updates = 0;   // telemetry updates in the timed iterations
    std::int64_t samples = 0;   // samples in the timed iterations
};

static u64 g_checksum = 0;

static bool bench(const std::vector<Batch>& batches, Result& r) {
    if (!formats_agree(batches)) return false;

    std::vector<double> times[NUM_FORMATS];
    for (int round = 0; round < kRounds; ++round) {
        for (int k = 0; k < NUM_FORMATS; ++k) {
            const Format f = static_cast<Format>((round + k) % NUM_FORMATS);
            Telemetry telem;
            const double ns = run_once(telem, f, batches);
            if (ns < 0) return false;
            times[f].push_back(ns);
            g_checksum = g_checksum * kFnvPrime + telem.checksum();
        }
    }

    double grouped_bytes = 0, ordered_bytes = 0;
    for (int i = kWarmup; i < kWarmup + kIterations; ++i) {
        const Batch& b = batch_at(batches, i);
        r.updates += b.n_telems;
        r.samples += static_cast<std::int64_t>(b.n_telems) * b.n_elems;
        grouped_bytes += static_cast<double>(b.grouped.bytes.size());
        ordered_bytes += static_cast<double>(b.ordered.bytes.size());
    }
    for (int f = 0; f < NUM_FORMATS; ++f) {
        std::sort(times[f].begin(), times[f].end());
        r.ns[f] = times[f][kRounds / 2];
        r.bytes[f] = (f == GROUPED || f == GROUPED_NOMOD ? grouped_bytes : ordered_bytes) / kIterations;
    }
    return true;
}

static void report(const char* name, const Result& r) {
    std::printf("%s\n", name);
    for (int f = 0; f < NUM_FORMATS; ++f) {
        const double base = r.ns[GROUPED], v = r.ns[f];
        char cmp[32] = "";
        if (f == GROUPED) {}
        else if (v == base) std::snprintf(cmp, sizeof(cmp), "equal");
        else if (v < base) std::snprintf(cmp, sizeof(cmp), "~%.2fx faster", base / v);
        else std::snprintf(cmp, sizeof(cmp), "~%.2fx slower", v / base);

        std::printf("  %s\n", kFormatNames[f]);
        std::printf("    per-batch   %8.1f ns  %s\n", v / kIterations, cmp);
        std::printf("    per-update  %8.1f ns\n", v / static_cast<double>(r.updates));
        std::printf("    per-sample  %8.1f ns\n", v / static_cast<double>(r.samples));
        std::printf("    payload     %8.1f bytes/batch\n", r.bytes[f]);
    }
    std::printf("\n");
}

int main() {
    struct Scenario {
        const char* name;
        int n_telems, n_elems;  // 0 for random per batch
    };
    const Scenario scenarios[] = {
        {"batches of 1, 1 update each", 1, 1},
        {"batches of 12, 1 update each", kBatch, 1},
        {"batches of 12, 20 updates each", kBatch, kMaxElems},
        {"random batches 1-26, random updates 1-20", 0, 0},
    };
    constexpr int kScenarios = sizeof(scenarios) / sizeof(scenarios[0]);

    std::uint32_t state = kSeed;
    Result results[kScenarios];
    for (int i = 0; i < kScenarios; ++i) {
        const Scenario& s = scenarios[i];
        std::vector<Batch> batches;
        if (s.n_telems > 0) {
            // A handful of distinct batches so the values change, cycled through. Shape is fixed.
            for (int j = 0; j < 64; ++j) batches.push_back(make_batch(s.n_telems, s.n_elems, state));
        } else {
            batches.reserve(kWarmup + kIterations);
            for (int j = 0; j < kWarmup + kIterations; ++j) {
                const int n_telems = next_range(state, 1, kRandomMaxBatch);
                const int n_elems = next_range(state, 1, kMaxElems);
                batches.push_back(make_batch(n_telems, n_elems, state));
            }
        }
        if (!bench(batches, results[i])) {
            std::fprintf(stderr, "%s failed\n", s.name);
            return 1;
        }
    }

    std::printf("telemetry format speed, %d iterations, %d%% warmup, median of %d rounds, seed %u\n\n", kIterations,
                kWarmupPct, kRounds, kSeed);
    for (int i = 0; i < kScenarios; ++i) report(scenarios[i].name, results[i]);
    std::printf("checksum %016llx\n", static_cast<unsigned long long>(g_checksum));
    return 0;
}
