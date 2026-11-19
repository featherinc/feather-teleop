#pragma once

#include "FlashTxx.hpp"

#ifdef COMPILE_FOR_TEENSY
    namespace qindesign { namespace network { class EthernetClient; } }
    using qindesign::network::EthernetClient;
#else
    typedef struct EthernetClient EthernetClient;
#endif

bool prepare_teensy_flash(EthernetClient* client, flash_elem_t* buf_addr, size_t buf_size);
void flash_noreturn(flash_elem_t* buf_addr);
