#pragma once

#include <string>
#include <cstdint>

namespace feather::teensy {

struct RecompileResult {
    std::string binary;
    std::uint64_t hash;
};

// Checks if the source code has changed since the last compilation, and recompiles if needed. Returns a RecompileResult
// object containing the path to the compiled binary, and its integer hash.
RecompileResult recompile_if_needed(bool emulated);

}  // namespace feather::teensy
