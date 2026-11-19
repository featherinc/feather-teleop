#pragma once

#include "qdefs/teensy_commands_qdef.hpp"

#include <string>

namespace feather::teensy {

struct FlashConfig {
    int port = TEENSY_PORT;
    double timeout_s = 120.0;
    double reboot_wait_s = 60.0;
};

void flash_ota(const std::string& hex_path, const FlashConfig& cfg);

}  // namespace feather::teensy
