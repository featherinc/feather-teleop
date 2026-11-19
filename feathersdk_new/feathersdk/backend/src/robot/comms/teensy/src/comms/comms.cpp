#include "comms.hpp"

#include <stdint.h>
#include <string.h>
#include <stdalign.h>
#include "../system.hpp"
#include "assert.h"

CommsConfig comms_config = DEFAULT_COMMS_CONFIG;

/**
 * Sets the communication configuration.
 * 
 * @param comms_config The communication configuration to set.
 */
void set_comms_config(const CommsConfig* cfg) {
    memcpy(&comms_config, cfg, sizeof(CommsConfig));
}
