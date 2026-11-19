#pragma once

// Ring storage for actuator telemetry. Same byte layout as the Python FeatherTelemetry.
//
// After initialize(), `data` and the uint64 arrays do not move, so a numpy array can
// view them. This only stores. Reading is done from those buffers on the Python side.
//
// Each update in the payload is:
//   [u16le telem_idx][u16le n_elems][n_elems * itemsize bytes]

#include <atomic>
#include <cstdint>
#include <vector>

#include "qdefs/data_type_qdef.hpp"

namespace feather::robot {

struct TelemetryAccess;

class Telemetry {
public:

    // Adds a new telemetry channel with the given capacity and data type, and returns the index that you should use to
    // access it. Returns -1 on error (e.g. if already initialized or dtype is unknown).
    int add_telemetry(std::uint64_t capacity, DataType dtype) noexcept;

    // Initializes the telemetry system. You will no longer be able to add channels after this, and telemetry can now
    // start being recorded/read. Returns false if already initialized, or no channels were added.
    bool initialize() noexcept;

    // Updates the telemetry system with the given payload. The payload is everything after the num_updates count in
    // a teensy telemetry update message. Returns false if not initialized, or a header, index, or value run past the
    // payload. Updates before that bad record are kept.
    bool update_telemetry(int n_updates, const std::uint8_t* payload, std::size_t size) noexcept;

    // Copies the newest sample. Returns false if the channel is empty or `size` is not the item size.
    bool latest(int index, void* out, std::size_t size) const noexcept;

    bool initialized() const noexcept { return initialized_.load(std::memory_order_acquire); }

    // The arena and per-channel arrays, for a reader that views them in place (the Python SDK's numpy arrays). Valid
    // from initialize() for the object's life; null pointers before.
    struct Buffers {
        std::uint8_t* data;
        std::size_t data_size;
        std::uint64_t* starts;
        std::uint64_t* ends;
        std::uint64_t* counts;
        std::size_t n_channels;
    };
    Buffers buffers() noexcept {
        if (!initialized()) return {nullptr, 0, nullptr, nullptr, nullptr, 0};
        return {data.data(), data.size(), starts.data(), ends.data(), counts.data(), counts.size()};
    }

private:
    // Stable after initialize(). Numpy views these directly.
    std::vector<std::uint8_t> data;
    std::vector<std::uint64_t> offsets;
    std::vector<std::uint64_t> itemsizes;
    std::vector<std::uint64_t> capacities;
    std::vector<std::uint64_t> starts;
    std::vector<std::uint64_t> ends;
    std::vector<std::uint64_t> counts;
    std::vector<DataType> dtypes;

    // A single channel of telemetry data. One parameter's telemetry on one motor.
    struct Channel {
        std::uint64_t capacity;
        std::uint64_t itemsize;
        DataType dtype;
    };

    std::vector<Channel> channels_;

    // The Teensy poll thread reads this while initialize() writes it.
    std::atomic<bool> initialized_{false};

    // Writes the given number of elements to the ring at the given index.
    void write_ring(std::size_t index, std::uint64_t n_elems, const std::uint8_t* src) noexcept;

    friend struct TelemetryAccess;
};

}  // namespace feather::robot
