#include "telemetry.hpp"

#include "../utils/logging.hpp"
#include "../utils/system.hpp"

#include <cstring>

#define MAX_TELEMETRY_CHANNELS 2048
#define MAX_TELEMETRY_CHANNEL_CAPACITY 1024

namespace feather::robot {

// Index updates are stored after the samples they cover, and stored whole: Python readers view these arrays from
// other threads without a lock (numpy over the same memory), so on ARM a plain store could be seen before the data.
static inline void release_store(std::uint64_t& dst, std::uint64_t value) noexcept {
    __atomic_store_n(&dst, value, __ATOMIC_RELEASE);
}

int Telemetry::add_telemetry(std::uint64_t capacity, DataType dtype) noexcept {
    ENSURE(!initialized(), -1, "Telemetry already initialized");

    const std::size_t item = dtype_size(dtype);
    if (item == 0) return -1;

    channels_.push_back(Channel{capacity, item, dtype});
    return static_cast<int>(channels_.size() - 1);
}

bool Telemetry::initialize() noexcept {
    const std::size_t n = channels_.size();
    ENSURE(!initialized() && !channels_.empty(), false, "Telemetry already initialized or no channels to initialize");
    ENSURE(n <= MAX_TELEMETRY_CHANNELS, false, "Too many telem channels: %zu (max: %d)", n, MAX_TELEMETRY_CHANNELS);

    // Fill all the metadata vectors with the correct values to exact sizes.
    offsets.assign(n, 0);
    itemsizes.assign(n, 0);
    capacities.assign(n, 0);
    starts.assign(n, 0);
    ends.assign(n, 0);
    counts.assign(n, 0);
    dtypes.assign(n, DATA_TYPE_U8);

    // Fill in and validate all the metadata vectors. Also keep track of the total size of the data buffer.
    std::uint64_t total = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const std::uint64_t cap = channels_[i].capacity;
        const std::uint64_t item = channels_[i].itemsize;
        const std::uint64_t bytes = cap * item;

        ENSURE(item != 0 && item <= dtype_size(DATA_TYPE_U64), false, "Channel %zu has invalid itemsize: %zu", i, item);
        ENSURE(cap <= MAX_TELEMETRY_CHANNEL_CAPACITY, false,
            "Channel %zu has capacity %zu which is too large (max: %d)", i, cap, MAX_TELEMETRY_CHANNEL_CAPACITY);
        
        offsets[i] = total;
        itemsizes[i] = item;
        capacities[i] = cap;
        dtypes[i] = channels_[i].dtype;
        total += bytes;
    }

    data.assign(static_cast<std::size_t>(total), 0);

    channels_.clear();
    initialized_.store(true, std::memory_order_release);
    return true;
}

void Telemetry::write_ring(std::size_t index, std::uint64_t n_elems, const std::uint8_t* src) noexcept {
    const std::uint64_t item = itemsizes[index];
    const std::uint64_t capacity = capacities[index];
    std::uint8_t* dst = data.data() + offsets[index];

    // Keep only the last `capacity` elements when the batch fills the ring.
    // We don't care about position here, just write into the entire ring and move start/end accordingly.
    if (n_elems >= capacity) {
        const std::uint64_t nbytes = capacity * item;
        if (nbytes != 0) {
            src += (n_elems - capacity) * item;
            std::memcpy(dst, src, static_cast<std::size_t>(nbytes));
        }

        release_store(starts[index], 0);
        release_store(ends[index], 0);
        release_store(counts[index], capacity);
        return;
    }

    const std::uint64_t end_idx = ends[index];
    const std::uint64_t space = capacity - end_idx;
    const std::uint64_t first = n_elems < space ? n_elems : space;
    const std::uint64_t nbytes_first = first * item;

    // Copy first part of data to the ring (if there is any), then the second chunk at the beginning (if there is any).
    if (nbytes_first != 0) std::memcpy(dst + end_idx * item, src, static_cast<std::size_t>(nbytes_first));
    if (first < n_elems) {
        const std::uint64_t nbytes_rest = (n_elems - first) * item;
        if (nbytes_rest != 0) std::memcpy(dst, src + nbytes_first, static_cast<std::size_t>(nbytes_rest));
    }

    // Update the count and start/end indices.
    const std::uint64_t new_count = counts[index] + n_elems;
    if (new_count > capacity) {
        release_store(starts[index], (starts[index] + (new_count - capacity)) % capacity);
        release_store(counts[index], capacity);
    } else {
        release_store(counts[index], new_count);
    }
    release_store(ends[index], (end_idx + n_elems) % capacity);
}

bool Telemetry::update_telemetry(int n_updates, const std::uint8_t* payload, std::size_t size) noexcept {
    ENSURE(initialized(), false, "Telemetry not initialized");
    ENSURE(n_updates >= 0, false, "n_updates is negative: %d", n_updates);
    ENSURE(payload != nullptr, false, "payload is nullptr");

    if (n_updates == 0) return true;

    // Walk through each update and write the data to the ring, validating each input. On err, return immediately.
    std::size_t pos = 0;
    for (int i = 0; i < n_updates; i++) {
        ENSURE(size - pos >= 4, false, "Not enough bytes to read header: %zu < 4", size - pos);

        const unsigned tidx = payload[pos] | (static_cast<unsigned>(payload[pos + 1]) << 8);
        const unsigned nelem = payload[pos + 2] | (static_cast<unsigned>(payload[pos + 3]) << 8);
        ENSURE(tidx < itemsizes.size(), false, "tidx is out of range: %d (max: %zu)", tidx, itemsizes.size());

        const std::uint64_t nbytes = static_cast<std::uint64_t>(nelem) * itemsizes[tidx];
        ENSURE(nbytes <= static_cast<std::uint64_t>(size - (pos + 4)), false, 
            "Not enough bytes to read data: %zu < %zu", nbytes, size - (pos + 4));

        write_ring(tidx, nelem, payload + pos + 4);
        pos += 4 + static_cast<std::size_t>(nbytes);
    }

    return true;
}

bool Telemetry::latest(int index, void* out, std::size_t size) const noexcept {
    if (!initialized() || out == nullptr || index < 0 || static_cast<std::size_t>(index) >= counts.size()) {
        return false;
    }
    if (counts[index] == 0 || size != itemsizes[index] || capacities[index] == 0) return false;

    const std::uint64_t end = ends[index];
    const std::uint64_t slot = end == 0 ? capacities[index] - 1 : end - 1;
    std::memcpy(out, data.data() + offsets[index] + slot * itemsizes[index], size);
    return true;
}

}  // namespace feather::robot
