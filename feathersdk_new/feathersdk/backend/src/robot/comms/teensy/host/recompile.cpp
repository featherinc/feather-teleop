#include <algorithm>
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "recompile.hpp"
#include "qdefs/teensy_commands_qdef.hpp"
#include "src/utils/run.hpp"
#include "src/utils/system.hpp"
#include "src/utils/logging.hpp"

namespace feather::teensy {
namespace detail {

const std::filesystem::path source_dirs[] = {
    (__PARENT_DIR__ / "../emulator").lexically_normal(),
    (__PARENT_DIR__ / "../src").lexically_normal(),
    (__PARENT_DIR__ / "../../../../../qdefs").lexically_normal(),
};

const std::filesystem::path source_hash_path = (__PARENT_DIR__ / "../src/teensy_source_hash.hpp").lexically_normal();

const std::string src_hash_start = "// DON'T MODIFY. Stores hash of source code, auto-added on recompile \n" \
    "static const char* teensy_source_hash_str __attribute__((used)) = \"";
const std::string src_hash_end = "\";";
constexpr std::string_view src_hash_marker_start = SRC_HASH_START;
constexpr std::string_view src_hash_marker_end = SRC_HASH_END;

const std::filesystem::path teensy_bin_path = (__PARENT_DIR__ / "../bin/teensy.ino.elf").lexically_normal();
const std::filesystem::path emulated_bin_path = (__PARENT_DIR__ / "../bin/emulated_teensy").lexically_normal();

// FNV-1a. Multiplying after each byte makes character order part of the hash.
constexpr std::uint64_t fnv_offset = 14695981039346656037ull;
constexpr std::uint64_t fnv_prime = 1099511628211ull;

// Same default as TEENSY_COMPILE_TIMEOUT in the Python recompilation helper.
constexpr double compile_timeout_s = 180.0;

std::uint64_t source_hash() {
    // Collect all source files and sort them so hash is always in the same order.
    std::vector<std::filesystem::path> files;
    for (const auto& dir : detail::source_dirs) {
        for (const auto& ent : std::filesystem::recursive_directory_iterator(dir)) {
            // Don't include the source hash file itself.
            if (!ent.is_regular_file() || ent.path().lexically_normal() == source_hash_path) continue;
            files.push_back(ent.path());
        }
    }
    std::sort(files.begin(), files.end());

    // Perform FNV-1a hash on all files. The value fits in the decimal uint64 the host parser reads.
    std::uint64_t hash = detail::fnv_offset;
    for (const auto& path : files) {
        std::ifstream in(path, std::ios::binary);
        if (!in) throw std::runtime_error("failed to read " + path.string());
        std::string bytes(std::istreambuf_iterator<char>(in), {});
        for (unsigned char byte : bytes) hash = (hash ^ byte) * detail::fnv_prime;
        hash = (hash ^ 0) * detail::fnv_prime;
    }
    return hash;
}

// Searches for the start/end hash markers in the given binary, and parses the hash value between them. Returns 0 and
// logs an error if it fails.
std::uint64_t read_bin_hash(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        INFO("file does not exist: %s", path.c_str());
        return 0;
    }

    const std::string bytes(std::istreambuf_iterator<char>(in), {});
    const auto start = bytes.find(src_hash_marker_start);
    if (start == std::string::npos) {
        INFO("source hash not found in %s - missing start marker", path.c_str());
        return 0;
    }
    const auto hash_begin = start + src_hash_marker_start.size();
    const auto end = bytes.find(src_hash_marker_end, hash_begin);
    if (end == std::string::npos || end == hash_begin) {
        INFO("source hash not found in %s - missing end marker", path.c_str());
        return 0;
    }

    const std::string_view text = std::string_view(bytes).substr(hash_begin, end - hash_begin);
    std::uint64_t hash = 0;
    const auto [ptr, ec] = std::from_chars(text.begin(), text.end(), hash);
    if (ec != std::errc{} || ptr != text.end()) {
        INFO("source hash not found in %s - invalid hash value", path.c_str());
        return 0;
    }
    return hash;
}

// Runs `make compile` (or `make emulated`) in the directory that holds the Makefile.
// Returns true only when make exits 0.
bool compile(bool emulated) {
    const char* target = emulated ? "emulated" : "compile";
    const std::filesystem::path dir = (__PARENT_DIR__ / "..").lexically_normal();
    const int rc = run({"make", target}, compile_timeout_s, dir.c_str());
    ENSURE(rc == 0, false, "make %s failed with exit code %d", target, rc);
    return true;
}

} // namespace detail


RecompileResult recompile_if_needed(bool emulated) {
    std::filesystem::path bin_path = emulated ? detail::emulated_bin_path : detail::teensy_bin_path;
    std::uint64_t bin_hash = detail::read_bin_hash(bin_path);
    std::uint64_t src_hash = detail::source_hash();

    // If the source hash does not match the binary hash, or the hash is 0, then we need to recompile. Write the source
    // hash to the source hash file and perform the compilation.
    if (bin_hash != src_hash || bin_hash == 0 || src_hash == 0) {
        INFO("recompiling - source hash != binary hash, or hash == 0: %016lx != %016lx", bin_hash, src_hash);
        std::ofstream out(detail::source_hash_path);
        if (!out) throw std::runtime_error("failed to write " + detail::source_hash_path.string());
        out << detail::src_hash_start << detail::src_hash_marker_start << src_hash
            << detail::src_hash_marker_end << detail::src_hash_end;
        out.close();  // compile reads this file; a buffered write would still look empty
        if (!out) throw std::runtime_error("failed to write " + detail::source_hash_path.string());
        if (!detail::compile(emulated)) throw std::runtime_error("teensy recompile failed");
    }

    return RecompileResult{bin_path, src_hash};
}

}  // namespace feather::teensy
